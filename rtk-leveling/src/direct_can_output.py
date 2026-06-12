"""
direct_can_output.py
구보다 순정 CAN 직접 출력단 (STM32 중계 없음)

기존 leveler_core.py 는 건드리지 않는다. 이 파일은 그 위에 얹는 add-on.

기존 구조:
  leveler_core.LevelerOutput (ABC)
    ├─ LaserConnectorOutput  : Apollo → CAN → STM32 → GPIO → (7)커넥터
    ├─ PowerPackOutput       : Apollo → CAN → 밸브드라이버 → 유압팩
    └─ DirectCanOutput (이 파일) : Apollo → CAN → 구보다 순정 버스 직결  ★신규

설계 변경 근거:
  워크샵 매뉴얼 분석으로 "구보다 순정 CAN(500kbps)에 직접 붙는다"가
  확정됨. 기존 STM32 중계 전제를 직결로 대체.
  단, leveler_core 의 추상화(LevelerOutput) 덕분에 제어로직(L1~L3)은
  그대로 두고 출력단만 이 클래스로 교체하면 된다.

★ 현장 CAN 캡처 전까지 실제 메시지 ID/바이트는 미확정.
  [CAPTURE] 부분을 캡처 후 채운다. (capture_checklist.md 참조)
"""

from __future__ import annotations
import time
import logging
from typing import Optional

# 기존 코어에서 가져옴 (같은 패키지 rtk-leveling/src/)
from leveler_core import LevelerOutput, Direction, CanBus

log = logging.getLogger("leveler.directcan")


# ─────────────────────────────────────────────────────────────
# 물리 계층 (확정 - Kubota L3560~L6060 WSM)
# ─────────────────────────────────────────────────────────────
KUBOTA_CAN_BITRATE = 500_000      # bps. 구보다 내부 CAN 확정값.
KUBOTA_CAN_FALLBACK = 250_000     # 외부 작업기 포트가 다를 경우


# ─────────────────────────────────────────────────────────────
# python-can 버스 래퍼 (실제 하드웨어 - CANable/PCAN 공통)
# ─────────────────────────────────────────────────────────────
class PythonCanBus(CanBus):
    """
    python-can 기반 실제 CAN 버스.
    CANable(slcan/socketcan), PCAN(pcan) 등 인터페이스 무관.

    예:
      # CANable (Linux socketcan)
      bus = PythonCanBus(interface="socketcan", channel="can0")
      # CANable (Windows/안드로이드 slcan)
      bus = PythonCanBus(interface="slcan", channel="/dev/ttyACM0")
      # PCAN
      bus = PythonCanBus(interface="pcan", channel="PCAN_USBBUS1")
    """
    def __init__(self, interface: str = "socketcan",
                 channel: str = "can0",
                 bitrate: int = KUBOTA_CAN_BITRATE):
        self.interface = interface
        self.channel = channel
        self.bitrate = bitrate
        self._bus = None

    def start(self):
        try:
            import can
        except ImportError:
            raise RuntimeError("python-can 미설치. pip install python-can")
        self._bus = can.Bus(
            interface=self.interface,
            channel=self.channel,
            bitrate=self.bitrate,
        )
        log.info(f"CAN 버스 시작: {self.interface}/{self.channel} @ {self.bitrate}bps")

    def stop(self):
        if self._bus is not None:
            self._bus.shutdown()
            self._bus = None
        log.info("CAN 버스 종료")

    def send(self, can_id: int, data: bytes, now: float = None):
        if self._bus is None:
            raise RuntimeError("버스 미시작. start() 먼저 호출.")
        import can
        msg = can.Message(
            arbitration_id=can_id,
            data=data,
            is_extended_id=(can_id > 0x7FF),   # 29-bit이면 확장 ID
        )
        self._bus.send(msg)

    def recv(self, timeout: float = 1.0):
        """스니핑/피드백 수신용."""
        if self._bus is None:
            return None
        return self._bus.recv(timeout=timeout)


# ─────────────────────────────────────────────────────────────
# 구보다 직결 출력단 (★ 캡처 후 완성)
# ─────────────────────────────────────────────────────────────
class DirectCanOutput(LevelerOutput):
    """
    방식 D — 구보다 순정 CAN 직결.

    LevelerSystem 의 제어 로직이 (mode, direction, pulse_ms) 를 주면,
    그걸 구보다가 알아듣는 실제 CAN 프레임으로 변환해 송신한다.

    ★★★ 변환 규칙(어느 ID, 어느 바이트)은 현장 캡처로 확정 ★★★
    """

    def __init__(self, bus: CanBus):
        self.bus = bus

        # [CAPTURE] 캡처로 확정할 값 ───────────────────────────
        self.LEVELER_TX_ID: Optional[int] = None   # 상승/하강 보낼 ID
        self.RAISE_BYTES: Optional[bytes] = None   # UP 페이로드
        self.LOWER_BYTES: Optional[bytes] = None   # DOWN 페이로드
        self.HOLD_BYTES: Optional[bytes]  = None   # HOLD/NEUTRAL 페이로드
        # 비례밸브면 강도를 특정 바이트에 넣어야 할 수 있음
        self.PWM_BYTE_INDEX: Optional[int] = None  # duty 넣을 바이트 위치
        # ──────────────────────────────────────────────────────

    def is_ready(self) -> bool:
        return all([
            self.LEVELER_TX_ID is not None,
            self.RAISE_BYTES is not None,
            self.LOWER_BYTES is not None,
            self.HOLD_BYTES is not None,
        ])

    def start(self):
        self.bus.start()

    def stop(self):
        self.bus.stop()

    def send_command(self, mode, direction: Direction,
                     pulse_ms: float, heartbeat: int, now: float = None):
        if not self.is_ready():
            raise RuntimeError(
                "DirectCanOutput 미완성. 현장 캡처 후 LEVELER_TX_ID/RAISE_BYTES 등 "
                "을 채워야 함. capture_checklist.md 참조."
            )

        # 방향 → 페이로드 선택
        if direction == Direction.UP:
            payload = bytearray(self.RAISE_BYTES)
        elif direction == Direction.DOWN:
            payload = bytearray(self.LOWER_BYTES)
        else:
            payload = bytearray(self.HOLD_BYTES)

        # [CAPTURE] 비례밸브면 pulse_ms를 duty로 변환해 삽입
        if self.PWM_BYTE_INDEX is not None and direction != Direction.NEUTRAL:
            duty = 255 if pulse_ms >= 255 else max(0, min(254, int(pulse_ms)))
            payload[self.PWM_BYTE_INDEX] = duty

        self.bus.send(self.LEVELER_TX_ID, bytes(payload), now=now)


def make_direct_output(interface: str = "socketcan",
                       channel: str = "can0",
                       bitrate: int = KUBOTA_CAN_BITRATE) -> DirectCanOutput:
    """구보다 직결 출력단 생성 팩토리."""
    bus = PythonCanBus(interface=interface, channel=channel, bitrate=bitrate)
    return DirectCanOutput(bus)


if __name__ == "__main__":
    # 미완성 상태 확인 테스트 (하드웨어 없이)
    print("=== DirectCanOutput 상태 ===")
    print(f"물리계층: {KUBOTA_CAN_BITRATE}bps (fallback {KUBOTA_CAN_FALLBACK})")

    # 가짜 버스로 ready 체크
    class DummyBus(CanBus):
        def start(self): pass
        def stop(self): pass
        def send(self, can_id, data, now=None): pass

    out = DirectCanOutput(DummyBus())
    print(f"캡처 완료? {out.is_ready()}  (False 정상 - 아직 캡처 전)")
    print()
    print("캡처 후 채울 것:")
    print("  out.LEVELER_TX_ID = 0x________")
    print("  out.RAISE_BYTES   = bytes([...])")
    print("  out.LOWER_BYTES   = bytes([...])")
    print("  out.HOLD_BYTES    = bytes([...])")
    print()
    print("그 후 LevelerSystem 에 이 출력단을 주입:")
    print("  system = LevelerSystem(params, output=out)")
