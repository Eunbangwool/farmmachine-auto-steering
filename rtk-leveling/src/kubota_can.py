"""
kubota_can.py
구보다 트랙터 CAN 통신 골격

물리 계층 (확정 - Kubota L3560~L6060 WSM 전기 섹션):
- 속도: 500 kbps  ← 구보다 내부 CAN. 스니핑/연결 시 이 값 먼저!
- 방식: 표준 CAN 2.0 (차동 2선), NRZ + 비트스터핑
- 배선: CAN-H = 노랑(Y), CAN-L = 초록(G)

PGN 정의 (ISO 11783-7 / SAE J1939-71 표준):
- 표준 ISOBUS면 아래 PGN으로 즉시 해독 가능
- 구보다 독자면 0xFF00~0xFFFF (Proprietary B) 영역 -> 캡처로 발견

★ 현장 CAN 캡처 전까지 확정 불가한 부분은 모두 [CAPTURE] 주석으로 표시.
  캡처 후 그 부분만 채우면 완성된다.
"""

from dataclasses import dataclass
from enum import IntEnum
from typing import Optional
import struct


# ─────────────────────────────────────────────────────────────
# 물리 계층 상수 (확정)
# ─────────────────────────────────────────────────────────────
KUBOTA_CAN_BITRATE = 500_000     # bps. 구보다 내부 CAN 확정값.
KUBOTA_CAN_FALLBACK = 250_000    # 외부 작업기 포트가 다를 경우 2순위 시도

# 배선 색상 (현장 핀 식별용)
WIRE_CAN_H = "Yellow (Y)"
WIRE_CAN_L = "Green (G)"
WIRE_12V   = "Red (R / R/Y)"
WIRE_5V    = "R/W"
WIRE_GND   = "Black (B)"


# ─────────────────────────────────────────────────────────────
# ISOBUS / J1939 표준 PGN (먼저 이걸로 매칭 시도)
# ─────────────────────────────────────────────────────────────
class PGN(IntEnum):
    # --- 작업기 제어 핵심 (레벨러 후보) ---
    REAR_HITCH_STATE      = 0xFE46   # 65094: 후방 히치 위치/명령 (SPN 1868/1869)
    FRONT_HITCH_STATE     = 0xFE44   # 65092: 전방 히치
    # 보조밸브 추정흐름 0~15 (0xFE48~0xFE57)
    AUX_VALVE_FLOW_0      = 0xFE48   # 65096: Valve 0 추정흐름 (SPN 1878/1879)
    # 보조밸브 명령 0~15 (0xFE58~0xFE67) ← 상승/하강 명령 1순위 후보
    AUX_VALVE_CMD_0       = 0xFE58   # 65112: Valve 0 명령
    # 보조밸브 측정위치 0~15 (0xFE68~0xFE77)
    AUX_VALVE_POS_0       = 0xFE68   # 65128: Valve 0 측정 스풀 위치
    # --- 참고 (상태 판정용) ---
    REAR_PTO_STATE        = 0xFE45   # 65093
    SELECTED_SPEED        = 0xFE47   # 65095: 지면 기준 속도
    ENGINE_HOURS          = 0xFEE5   # 65253
    FUEL_ECONOMY          = 0xFEF2   # 65266


# 보조밸브 상태 enum (SPN 1879, 4-bit)
class AuxValveState(IntEnum):
    BLOCKED = 0   # 차단 (= HOLD)
    EXTEND  = 1   # 신장 (= 한쪽 방향)
    RETRACT = 2   # 수축 (= 반대 방향)
    FLOAT   = 3   # 플로트


# 히치 in-work (SPN 1870, 2-bit)
class HitchInWork(IntEnum):
    NOT_IN_WORK = 0
    IN_WORK     = 1
    ERROR       = 2
    NA          = 3


# ─────────────────────────────────────────────────────────────
# CAN 프레임
# ─────────────────────────────────────────────────────────────
@dataclass
class CanFrame:
    can_id: int          # 29-bit 확장 ID
    data: bytes          # 최대 8바이트
    timestamp: float = 0.0
    is_extended: bool = True

    @property
    def pgn(self) -> int:
        """29-bit ID에서 18-bit PGN 추출 (J1939)."""
        # ID = Priority(3) | Reserved(1) | DataPage(1) | PDUFormat(8) | PDUSpecific(8) | SA(8)
        pf = (self.can_id >> 16) & 0xFF
        ps = (self.can_id >> 8) & 0xFF
        if pf < 240:   # PDU1 (목적지 지정)
            return (self.can_id >> 8) & 0x3FF00
        else:          # PDU2 (브로드캐스트)
            return (self.can_id >> 8) & 0x3FFFF

    @property
    def source_address(self) -> int:
        return self.can_id & 0xFF

    @property
    def priority(self) -> int:
        return (self.can_id >> 26) & 0x7

    def hex(self) -> str:
        return f"0x{self.can_id:08X}  " + " ".join(f"{b:02X}" for b in self.data)


# ─────────────────────────────────────────────────────────────
# 표준 PGN 디코더 (표준 ISOBUS일 경우 즉시 해독)
# ─────────────────────────────────────────────────────────────
class IsobusDecoder:
    @staticmethod
    def decode_rear_hitch(data: bytes) -> dict:
        """PGN 65094 후방 히치. SPN 1868 위치, SPN 1870 in-work."""
        if len(data) < 3:
            return {}
        position_pct = data[0] * 0.4         # SPN 1868: 0.4%/bit
        command_pct  = data[1] * 0.4         # SPN 1869: set point
        in_work = HitchInWork(data[2] & 0x03)  # SPN 1870
        return {
            "hitch_position_pct": position_pct,
            "hitch_command_pct": command_pct,
            "in_work": in_work.name,
        }

    @staticmethod
    def decode_aux_valve_flow(data: bytes) -> dict:
        """PGN 65096~ 보조밸브 추정흐름. SPN 1878 흐름, SPN 1879 상태."""
        if len(data) < 2:
            return {}
        flow_pct = data[0] * 0.4 - 100.0     # SPN 1878: 0.4%/bit, -100 offset
        state = AuxValveState(data[1] & 0x0F)  # SPN 1879
        return {
            "flow_pct": flow_pct,              # -100(수축) ~ +100(신장)
            "valve_state": state.name,
        }


# ─────────────────────────────────────────────────────────────
# 구보다 레벨러 어댑터 (★ 캡처 후 완성)
# ─────────────────────────────────────────────────────────────
class KubotaLevelerCAN:
    """
    leveler_core.ValveCommand(RAISE/HOLD/LOWER) 를 실제 CAN 프레임으로 변환.

    ★★★ 이 클래스의 send_* 메서드 본문은 현장 CAN 캡처 결과로 채운다 ★★★
    캡처 절차는 capture_checklist.md 참조.
    """

    def __init__(self, bus=None):
        self.bus = bus  # python-can Bus 객체 (PCAN-USB FD)
        self._captured = False   # 캡처/검증 완료 플래그

        # [CAPTURE] 아래 값들은 캡처로 확정해야 한다 ---------------
        self.LEVELER_TX_ID: Optional[int] = None   # 상승/하강 명령 보낼 ID
        self.LEVELER_RX_ID: Optional[int] = None   # 높이/상태 받을 ID
        self.RAISE_BYTES: Optional[bytes] = None   # 상승 명령 페이로드
        self.LOWER_BYTES: Optional[bytes] = None   # 하강 명령 페이로드
        self.HOLD_BYTES: Optional[bytes]  = None   # 유지 명령 페이로드
        # ----------------------------------------------------------

    def is_ready(self) -> bool:
        """캡처로 ID/바이트가 다 채워졌는지."""
        return all([
            self.LEVELER_TX_ID is not None,
            self.RAISE_BYTES is not None,
            self.LOWER_BYTES is not None,
            self.HOLD_BYTES is not None,
        ])

    def send_command(self, command, pwm_duty: int = 100):
        """
        leveler_core 의 ValveCommand 를 받아 CAN 송신.
        command: ValveCommand (RAISE=1, HOLD=0, LOWER=-1)
        """
        if not self.is_ready():
            raise RuntimeError(
                "CAN 매핑 미완성. 현장 캡처 후 LEVELER_TX_ID/RAISE_BYTES 등을 채워야 함. "
                "capture_checklist.md 참조."
            )
        # command.value: 1=RAISE, 0=HOLD, -1=LOWER
        if command.value == 1:
            payload = self.RAISE_BYTES
        elif command.value == -1:
            payload = self.LOWER_BYTES
        else:
            payload = self.HOLD_BYTES

        # [CAPTURE] 비례밸브면 pwm_duty 를 특정 바이트에 삽입해야 할 수 있음
        # 예: payload = bytes([..., pwm_duty, ...])

        frame = CanFrame(can_id=self.LEVELER_TX_ID, data=payload)
        if self.bus:
            self._raw_send(frame)
        return frame

    def _raw_send(self, frame: CanFrame):
        """python-can 송신 래퍼."""
        try:
            import can
            msg = can.Message(
                arbitration_id=frame.can_id,
                data=frame.data,
                is_extended_id=frame.is_extended,
            )
            self.bus.send(msg)
        except ImportError:
            raise RuntimeError("python-can 미설치. pip install python-can")


# ─────────────────────────────────────────────────────────────
# 스니핑 도우미 (캡처 단계에서 사용)
# ─────────────────────────────────────────────────────────────
class CanSniffer:
    """
    레이저 모드 OFF/ON 차분 분석용. PCAN 연결 후 사용.
    1) baseline = sniff(레이저 OFF, 10초)
    2) active   = sniff(레이저 ON,  10초)
    3) diff_ids(baseline, active) -> 새로 나타난 ID = 레벨러 관련 후보
    """

    def __init__(self, bus=None):
        self.bus = bus

    def sniff(self, duration_s: float = 10.0) -> dict:
        """duration 동안 수집. {can_id: [data_samples]} 반환."""
        import time as _t
        captured: dict = {}
        if self.bus is None:
            return captured
        end = _t.time() + duration_s
        while _t.time() < end:
            msg = self.bus.recv(timeout=1.0)
            if msg is None:
                continue
            cid = msg.arbitration_id
            captured.setdefault(cid, []).append(bytes(msg.data))
        return captured

    @staticmethod
    def diff_ids(baseline: dict, active: dict) -> set:
        """active 에만 있는 ID (레이저 ON에서 새로 등장)."""
        return set(active.keys()) - set(baseline.keys())

    @staticmethod
    def changed_bytes(samples_before: list, samples_after: list) -> list:
        """
        같은 ID에서, 동작(상승버튼 등) 전/후 바뀐 바이트 위치 찾기.
        상승/하강 명령 바이트 식별의 핵심.
        """
        if not samples_before or not samples_after:
            return []
        b = samples_before[-1]
        a = samples_after[-1]
        changed = []
        for i in range(min(len(b), len(a))):
            if b[i] != a[i]:
                changed.append((i, b[i], a[i]))   # (바이트위치, 전, 후)
        return changed


# ─────────────────────────────────────────────────────────────
# 표준 PGN 매칭 시도 (캡처 직후 1차 자동 분석)
# ─────────────────────────────────────────────────────────────
def try_match_standard(captured_ids: set) -> dict:
    """캡처된 ID들 중 표준 ISOBUS 레벨러 관련 PGN이 있는지 매칭."""
    matches = {}
    candidate_pgns = {
        PGN.REAR_HITCH_STATE: "후방 히치 (상승/하강 위치 명령 가능)",
        PGN.AUX_VALVE_CMD_0:  "보조밸브 명령 (상승/하강 직접 명령)",
        PGN.AUX_VALVE_FLOW_0: "보조밸브 흐름/상태",
        PGN.AUX_VALVE_POS_0:  "보조밸브 측정위치",
    }
    for cid in captured_ids:
        f = CanFrame(can_id=cid, data=b"")
        for pgn, desc in candidate_pgns.items():
            if f.pgn == int(pgn) or (f.pgn & 0xFF00) == (int(pgn) & 0xFF00):
                matches[cid] = (hex(int(pgn)), desc)
    return matches


if __name__ == "__main__":
    # 디코더 자체 테스트 (가상 데이터)
    print("=== 물리 계층 ===")
    print(f"속도: {KUBOTA_CAN_BITRATE} bps (2순위 {KUBOTA_CAN_FALLBACK})")
    print(f"CAN-H={WIRE_CAN_H}, CAN-L={WIRE_CAN_L}")
    print()

    print("=== 표준 PGN 후보 ===")
    for p in [PGN.REAR_HITCH_STATE, PGN.AUX_VALVE_CMD_0, PGN.AUX_VALVE_FLOW_0]:
        print(f"  {p.name}: 0x{int(p):04X} ({int(p)})")
    print()

    print("=== 히치 디코드 예시 ===")
    # 가상: 위치 50%(125*0.4), 명령 60%(150*0.4), in-work
    fake = bytes([125, 150, 0x01, 0, 0, 0, 0, 0])
    print(" ", IsobusDecoder.decode_rear_hitch(fake))

    print("=== 보조밸브 디코드 예시 ===")
    # 가상: 흐름 +20%((300? no) 0.4*x-100), 상태 EXTEND
    fake2 = bytes([300 & 0xFF, 0x01, 0, 0, 0, 0, 0, 0])
    print(" ", IsobusDecoder.decode_aux_valve_flow(fake2))

    print("=== CAN ID -> PGN 추출 예시 ===")
    # 후방 히치 from TECU: 0x18FE4600
    f = CanFrame(can_id=0x18FE4600, data=b"")
    print(f"  0x18FE4600 -> PGN 0x{f.pgn:04X}, SA 0x{f.source_address:02X}, prio {f.priority}")
