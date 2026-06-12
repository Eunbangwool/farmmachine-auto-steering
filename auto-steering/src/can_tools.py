"""
can_tools.py
===========
CAN 버스 역공학 도구 — 모터 CAN 문서가 오기 전에 미리 준비.

CLAUDE.md 우선순위 #2(CanSpec 채우기) 준비:
  문서가 없어도 실차 CAN 버스를 떠서
    - 어떤 CAN ID 들이 흐르는지 (주기/DLC/바이트 변화)
    - 어느 ID 가 앵글센서인지 (운전대를 좌우로 돌릴 때 값이 따라 변함)
    - 앵글값이 어느 바이트에 int16 으로 들어있는지 (외부 신호와 상관분석)
  를 알아내 CanSpec.SENSOR_ANGLE_ID / 바이트 오프셋을 역추적한다.

autosteer_core.CanInterface(Mock/Apollo) 와 연동. 하드웨어 없이 Mock 으로 테스트 가능.

candump 호환 텍스트로 로그 저장/로드:  "<t> <id_hex> <data_hex>"
"""

from __future__ import annotations
import time
import math
import struct
import logging
from dataclasses import dataclass, field
from typing import List, Dict, Optional, Tuple

from autosteer_core import CanInterface, CanSpec

log = logging.getLogger("can_tools")


@dataclass
class CanFrame:
    t: float
    can_id: int
    data: bytes


def decode_int(data: bytes, hi: int, lo: int,
               signed: bool = True, big_endian: bool = True) -> Optional[int]:
    """data[hi]·data[lo] 두 바이트를 int16 으로 디코드. 범위 밖이면 None."""
    n = max(hi, lo)
    if n >= len(data):
        return None
    b = bytes([data[hi], data[lo]]) if big_endian else bytes([data[lo], data[hi]])
    return int.from_bytes(b, "big", signed=signed)


# ═══════════════════════════════════════════════════════════════
#  버스 분석
# ═══════════════════════════════════════════════════════════════

@dataclass
class _IdStat:
    count: int = 0
    first_t: float = 0.0
    last_t: float = 0.0
    dlc: set = field(default_factory=set)
    last_data: bytes = b""
    byte_min: List[int] = field(default_factory=lambda: [255] * 8)
    byte_max: List[int] = field(default_factory=lambda: [0] * 8)
    byte_distinct: List[set] = field(default_factory=lambda: [set() for _ in range(8)])
    changes: int = 0

    @property
    def rate_hz(self) -> float:
        dur = self.last_t - self.first_t
        return (self.count - 1) / dur if dur > 1e-6 and self.count > 1 else 0.0

    def variability(self) -> int:
        """바이트별 distinct 값 합 — 연속 변화하는(센서) ID 일수록 큼."""
        return sum(len(s) for s in self.byte_distinct)


class CanBusAnalyzer:
    """프레임을 먹여 ID 별 통계를 만든다. 앵글센서 후보 추정에 사용."""
    def __init__(self):
        self.ids: Dict[int, _IdStat] = {}

    def feed(self, frame: CanFrame):
        st = self.ids.get(frame.can_id)
        if st is None:
            st = _IdStat(first_t=frame.t)
            self.ids[frame.can_id] = st
        st.count += 1
        st.last_t = frame.t
        st.dlc.add(len(frame.data))
        if frame.data != st.last_data:
            st.changes += 1
        st.last_data = frame.data
        for i, b in enumerate(frame.data[:8]):
            st.byte_min[i] = min(st.byte_min[i], b)
            st.byte_max[i] = max(st.byte_max[i], b)
            st.byte_distinct[i].add(b)

    def feed_all(self, frames: List[CanFrame]):
        for f in frames:
            self.feed(f)

    def angle_sensor_candidates(self) -> List[Tuple[int, int]]:
        """변동성 큰 순으로 (can_id, variability) 정렬. 앵글센서일 확률 높은 순."""
        return sorted(((cid, st.variability()) for cid, st in self.ids.items()),
                      key=lambda kv: kv[1], reverse=True)

    def report(self) -> str:
        L = [f"{'CAN ID':>10}  {'count':>6}  {'rate':>7}  {'dlc':>4}  "
             f"{'changes':>7}  {'변동성':>6}  last_data"]
        L.append("-" * 78)
        for cid, st in sorted(self.ids.items()):
            dlc = ",".join(str(d) for d in sorted(st.dlc))
            L.append(f"0x{cid:08X}  {st.count:>6}  {st.rate_hz:>6.1f}H  "
                     f"{dlc:>4}  {st.changes:>7}  {st.variability():>6}  "
                     f"{st.last_data.hex()}")
        cands = self.angle_sensor_candidates()
        if cands:
            top = ", ".join(f"0x{c:X}(v={v})" for c, v in cands[:3])
            L.append("-" * 78)
            L.append(f"앵글센서 후보(변동성 상위): {top}")
        return "\n".join(L)


# ═══════════════════════════════════════════════════════════════
#  외부 신호 상관분석 — 앵글센서 ID + 바이트 오프셋 + 부호 찾기
# ═══════════════════════════════════════════════════════════════

def _interp(ts: List[float], vs: List[float], t: float) -> float:
    """(ts,vs) 시계열을 t 에서 선형보간."""
    if t <= ts[0]:
        return vs[0]
    if t >= ts[-1]:
        return vs[-1]
    lo, hi = 0, len(ts) - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if ts[mid] <= t:
            lo = mid
        else:
            hi = mid
    span = ts[hi] - ts[lo]
    if span <= 1e-12:
        return vs[lo]
    f = (t - ts[lo]) / span
    return vs[lo] + (vs[hi] - vs[lo]) * f


def _pearson(a: List[float], b: List[float]) -> float:
    n = len(a)
    if n < 3:
        return 0.0
    ma, mb = sum(a) / n, sum(b) / n
    da = [x - ma for x in a]
    db = [y - mb for y in b]
    num = sum(x * y for x, y in zip(da, db))
    den = math.sqrt(sum(x * x for x in da) * sum(y * y for y in db))
    return num / den if den > 1e-12 else 0.0


@dataclass
class CorrelationHit:
    can_id: int
    hi: int
    lo: int
    big_endian: bool
    signed: bool
    corr: float          # 부호 포함 상관계수 (+면 동상, −면 역상=부호반전)

    def describe(self) -> str:
        en = "BE" if self.big_endian else "LE"
        sg = "signed" if self.signed else "unsigned"
        return (f"0x{self.can_id:X} byte[{self.hi},{self.lo}] {en}/{sg} "
                f"corr={self.corr:+.3f}")


def correlate_with_signal(frames: List[CanFrame],
                          signal: List[Tuple[float, float]],
                          min_corr: float = 0.8) -> List[CorrelationHit]:
    """
    프레임 스트림과 외부 신호(예: 운전대를 손으로 좌우로 돌린 각도/방향)를
    상관분석해 앵글센서 인코딩을 찾는다.

    frames : 버스에서 캡처한 모든 프레임
    signal : [(t, value)] — 같은 시간축의 기준 신호(조향 방향/각)
    반환   : |corr|≥min_corr 인 (id,byte,endian,sign) 후보, 상관 큰 순
    """
    if len(signal) < 3:
        return []
    sts = [t for t, _ in signal]
    svs = [v for _, v in signal]

    by_id: Dict[int, List[CanFrame]] = {}
    for f in frames:
        by_id.setdefault(f.can_id, []).append(f)

    hits: List[CorrelationHit] = []
    for cid, fl in by_id.items():
        if len(fl) < 5:
            continue
        ref = [_interp(sts, svs, f.t) for f in fl]   # 프레임 시각에 맞춘 기준신호
        maxdlc = max(len(f.data) for f in fl)
        for be in (True, False):
            for hi in range(maxdlc - 1):
                lo = hi + 1
                series = []
                ok = True
                for f in fl:
                    v = decode_int(f.data, hi, lo, signed=True, big_endian=be)
                    if v is None:
                        ok = False
                        break
                    series.append(float(v))
                if not ok or len(set(series)) < 3:
                    continue
                r = _pearson(series, ref)
                if abs(r) >= min_corr:
                    hits.append(CorrelationHit(cid, hi, lo, be, True, r))
    hits.sort(key=lambda h: abs(h.corr), reverse=True)
    return hits


# ═══════════════════════════════════════════════════════════════
#  로거 — 실차 캡처 / 저장 / 로드
# ═══════════════════════════════════════════════════════════════

class CanLogger:
    """CanInterface 에서 프레임을 받아 타임스탬프와 함께 수집."""
    def __init__(self, can: CanInterface):
        self.can = can
        self.frames: List[CanFrame] = []

    def poll(self) -> int:
        """현재 수신 큐를 비워 프레임으로 적재. 적재 개수 반환."""
        n = 0
        msg = self.can.recv()
        while msg:
            cid, data = msg
            self.frames.append(CanFrame(time.time(), cid, bytes(data)))
            n += 1
            msg = self.can.recv()
        return n

    def run(self, duration: float, poll_dt: float = 0.002):
        """duration 초 동안 폴링 캡처 (실차에서 사용)."""
        t0 = time.time()
        while time.time() - t0 < duration:
            if self.poll() == 0:
                time.sleep(poll_dt)

    def save(self, path: str):
        with open(path, "w") as f:
            for fr in self.frames:
                f.write(f"{fr.t:.6f} {fr.can_id:X} {fr.data.hex()}\n")

    @staticmethod
    def load(path: str) -> List[CanFrame]:
        out = []
        with open(path) as f:
            for line in f:
                p = line.split()
                if len(p) >= 3:
                    out.append(CanFrame(float(p[0]), int(p[1], 16),
                                        bytes.fromhex(p[2])))
        return out


# ═══════════════════════════════════════════════════════════════
#  모터 스텝 응답 계측 — 실모터의 servo (max_rate, tau) 추정
#  → sitl_sim.ServoCanInterface 파라미터로 꽂아 tuning.py 재탐색
# ═══════════════════════════════════════════════════════════════

@dataclass
class ServoResponse:
    max_rate_deg_s: float       # 전륜 조향 최대 각속도
    tau_s: float                # 1차 지연 시정수
    n_samples: int
    trace: List[Tuple[float, float]]      # (t, angle_deg)
    note: str = ""

    def as_servo_params(self) -> dict:
        """sitl_sim.ServoCanInterface / build_system 에 바로 쓰는 형태."""
        return {"servo_rate_deg_s": round(self.max_rate_deg_s, 1),
                "servo_tau": round(self.tau_s, 3)}


class MotorResponseProbe:
    """
    모터에 조향각 스텝을 주고 앵글센서 응답을 기록해 servo 특성을 추정.
    - 큰 스텝 → 각속도 한계(max_rate) 측정
    - 작은 스텝 → 1차 지연(tau, 63.2% 도달시간) 측정
    실모터(ApolloCanInterface)와 시뮬(ServoCanInterface) 양쪽에서 동일 동작.

    ⚠ 안전: 반드시 바퀴를 들거나(잭업) 빈 농지 정지 상태에서. 사람 접근 금지.
    """
    def __init__(self, can: CanInterface, ctrl_dt: Optional[float] = None,
                 live: bool = False):
        self.can = can
        self.dt = ctrl_dt or CanSpec.MOTOR_CMD_PERIOD
        self.live = live            # True 면 실모터: 송신 간 dt 만큼 대기

    def _send_target(self, target_deg: float):
        data = bytearray(8)
        data[CanSpec.MOTOR_BYTE_MODE] = CanSpec.MOTOR_MODE_ANGLE
        raw = max(-32768, min(32767, int(target_deg * CanSpec.MOTOR_ANGLE_SCALE)))
        struct.pack_into(">h", data, CanSpec.MOTOR_BYTE_CMD_HI, raw)
        self.can.send(CanSpec.MOTOR_CMD_ID, bytes(data))

    def _read_angle(self) -> Optional[float]:
        last = None
        msg = self.can.recv()
        while msg:
            cid, d = msg
            if cid == CanSpec.SENSOR_ANGLE_ID and len(d) >= 2:
                raw = int.from_bytes(
                    d[CanSpec.SENSOR_BYTE_HI:CanSpec.SENSOR_BYTE_LO + 1],
                    "big", signed=CanSpec.SENSOR_SIGNED)
                last = raw / CanSpec.SENSOR_ANGLE_SCALE - CanSpec.SENSOR_ANGLE_OFFSET
            msg = self.can.recv()
        return last

    def _drive_to(self, target_deg: float, ticks: int):
        for _ in range(ticks):
            self._send_target(target_deg)
            self._read_angle()
            if self.live:
                time.sleep(self.dt)

    def step_response(self, target_deg: float, duration: float) -> List[Tuple[float, float]]:
        trace, t = [], 0.0
        for _ in range(max(2, int(duration / self.dt))):
            self._send_target(target_deg)
            ang = self._read_angle()
            if ang is None:
                ang = trace[-1][1] if trace else 0.0
            trace.append((t, ang))
            t += self.dt
            if self.live:
                time.sleep(self.dt)
        return trace

    def measure(self, slew_step: float = 20.0, tau_step: float = 2.0,
                duration: float = 2.0) -> ServoResponse:
        self._drive_to(0.0, 25)                          # 영점
        big = self.step_response(slew_step, duration)    # 큰 스텝 → 각속도
        max_rate = max(abs(big[i][1] - big[i - 1][1]) / self.dt
                       for i in range(1, len(big)))
        self._drive_to(0.0, 50)                          # 재영점
        small = self.step_response(tau_step, duration)   # 작은 스텝 → tau
        tau = self._estimate_tau(small)
        note = "" if max_rate > 1 else "응답 없음 — CAN ID/배선/모터 활성화 확인"
        return ServoResponse(max_rate, tau, len(big) + len(small), big + small, note)

    @staticmethod
    def _estimate_tau(trace: List[Tuple[float, float]]) -> float:
        final = trace[-1][1]
        if abs(final) < 1e-6:
            return 0.0
        thr = 0.632 * final
        for t, a in trace:
            if (final > 0 and a >= thr) or (final < 0 and a <= thr):
                return max(t, 1e-3)
        return trace[-1][0]


# ── 자체 테스트: 합성 버스(앵글센서 + 모터 + 하트비트) 역추적 ─────────
if __name__ == "__main__":
    import random
    print("=" * 78)
    print("can_tools — CAN 버스 역공학 (앵글센서 ID/바이트 자동 탐색) 테스트")
    print("=" * 78)

    ANGLE_ID, MOTOR_ID, HB_ID = 0x18FF51E5, 0x18FF50E5, 0x701
    dt = 0.01
    frames: List[CanFrame] = []
    signal: List[Tuple[float, float]] = []     # 운전대 좌우 흔든 기준 신호

    t = 0.0
    for k in range(800):
        t += dt
        # 사람이 운전대를 좌우로 흔드는 동작 (기준 신호)
        steer_deg = 20.0 * math.sin(k * 0.05)
        signal.append((t, steer_deg))

        # 앵글센서: 조향각을 0.1도 단위 int16(BE)로 byte[2,3]에 인코딩
        raw = int(steer_deg * 10) + random.randint(-1, 1)
        d = bytearray(8)
        d[2:4] = int(raw & 0xFFFF).to_bytes(2, "big")
        frames.append(CanFrame(t, ANGLE_ID, bytes(d)))

        # 모터 명령: 거의 일정 (상관 없음)
        dm = bytearray(8); dm[0] = 1; dm[1] = 100
        frames.append(CanFrame(t, MOTOR_ID, bytes(dm)))

        # 하트비트: 완전 고정
        if k % 10 == 0:
            frames.append(CanFrame(t, HB_ID, bytes([0xAA, 0x55, 0, 0, 0, 0, 0, 0])))

    an = CanBusAnalyzer()
    an.feed_all(frames)
    print("\n" + an.report())

    print("\n외부 신호(운전대 좌우)와 상관분석 → 앵글센서 인코딩 탐색:")
    hits = correlate_with_signal(frames, signal, min_corr=0.8)
    for h in hits[:5]:
        print("  " + h.describe())

    best = hits[0]
    assert best.can_id == ANGLE_ID, f"앵글센서 ID 오탐: 0x{best.can_id:X}"
    assert (best.hi, best.lo) == (2, 3), f"바이트 오프셋 오탐: {best.hi},{best.lo}"
    assert best.big_endian and best.corr > 0.95
    cands = an.angle_sensor_candidates()
    assert cands[0][0] == ANGLE_ID, "변동성 후보 1순위가 앵글센서가 아님"
    print(f"\n  ✓ 앵글센서 ID 0x{best.can_id:X}, byte[{best.hi},{best.lo}] BE 자동 식별")
    print("  → CanSpec.SENSOR_ANGLE_ID / SENSOR_BYTE_HI/LO 채울 값 확보")
    print("  → 모터 ID 는 '내가 명령 보낼 때만 변하는 ID' 로 같은 방식 식별")

    # ── 모터 스텝 응답 계측 (알려진 서보를 정확히 복원하는지 검증) ──────
    print("\n" + "=" * 78)
    print("모터 스텝 응답 계측 — servo(max_rate, tau) 추정 → tuning 입력")
    print("=" * 78)
    from sitl_sim import ServoCanInterface          # 실모터 대역(시뮬)
    TRUE_RATE, TRUE_TAU = 35.0, 0.08
    servo = ServoCanInterface(max_rate_deg_s=TRUE_RATE, tau=TRUE_TAU)
    probe = MotorResponseProbe(servo)
    resp = probe.measure(slew_step=20.0, tau_step=2.0, duration=2.0)
    print(f"  계측: max_rate={resp.max_rate_deg_s:.1f}°/s (실제 {TRUE_RATE})  "
          f"tau={resp.tau_s:.3f}s (실제 {TRUE_TAU})")
    print(f"  → ServoCanInterface 파라미터: {resp.as_servo_params()}")
    assert abs(resp.max_rate_deg_s - TRUE_RATE) < 5.0, resp.max_rate_deg_s
    assert abs(resp.tau_s - TRUE_TAU) < 0.05, resp.tau_s
    print("  ✓ 스텝응답으로 실모터 servo 특성 복원 → tuning.py 재탐색에 투입 가능")
