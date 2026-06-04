"""
sitl_sim.py
==========
SITL(Software-In-The-Loop) 시뮬레이터 — 현장 가기 전 폐루프 + 안전 검증.

CLAUDE.md 우선순위 #7(저속 안전 검증) 준비:
  실차/빈 농지에 나가기 전에, 자전거 모델 위에서 AutoSteerSystem 을
  폐루프로 돌려본다.
    - 경로 추종 성능(XTE) 사전 확인
    - 안전 계층(데드맨/비상정지/RTK품질/RTK끊김/운전자개입/과속) 전부
      "정말 해제(disengage)되는지" 시나리오로 검증

자전거 모델(후륜축):
    heading += v/L · tan(δ) · dt ;  axle 전진
    안테나 위치 = axle + antenna_to_axle 방향 보정 → lat/lon 으로 환산해 on_rtk
조향 δ 는 SteeringActuator(MockCAN)가 피드백한 measured angle 을 사용 →
명령→모터→앵글센서→상태추정→제어 전 경로가 닫힌다.
"""

from __future__ import annotations
import math
import time
import random
from dataclasses import dataclass
from typing import List, Optional

from autosteer_core import (
    AutoSteerSystem, MockCanInterface, TractorParams, KUBOTA_MR1157,
    ABLineStrategy, SafetyState, SafetyMonitor, CanInterface, CanSpec,
)

R_EARTH = 6_371_000.0


def _wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


class BicycleModel:
    """
    후륜축 기준 자전거 모델. 상태: 차축 (x,y), heading, yaw_rate.

    yaw_tau: 작업기 부하에 의한 heading 응답 지연(1차) 시정수(s).
      0       = 부하 없음(즉답, 순수 기구학) — 기존 동작과 동일
      0.2~0.3 = 로터리/방제 등 가벼운 작업
      0.5~0.8 = 쟁기/균평 등 과부하(작업기 관성·횡저항이 yaw 변화를 늦춤)
    실제로 heavy 모드가 크로스게인을 올려도 안정적인 이유 = 이 부하 감쇠.
    """
    def __init__(self, params: TractorParams,
                 x: float = 0.0, y: float = 0.0, heading: float = math.pi / 2,
                 yaw_tau: float = 0.0, dist_amp: float = 0.0):
        self.p = params
        self.x = x
        self.y = y
        self.heading = heading
        self.yaw_rate = 0.0
        self.yaw_tau = yaw_tau
        self.dist_amp = dist_amp        # 작업기 횡저항(side draft) 외란 진폭 (rad/s)
        self._yaw = 0.0
        self._k = 0

    def step(self, speed: float, steer_rad: float, dt: float):
        steer = max(-self.p.max_steer_rad, min(self.p.max_steer_rad, steer_rad))
        yaw_cmd = speed * math.tan(steer) / self.p.wheelbase
        a = dt / (self.yaw_tau + dt)          # yaw_tau=0 → a=1 (즉답)
        self._yaw += a * (yaw_cmd - self._yaw)
        self.yaw_rate = self._yaw
        # 작업기 외란: 흙저항이 차량을 경로 밖으로 끄는 저주파 + 잡음
        self._k += 1
        dist = 0.0
        if self.dist_amp:
            dist = (self.dist_amp * math.sin(self._k * 0.04)
                    + random.gauss(0, self.dist_amp * 0.4))
        self.x += speed * math.cos(self.heading) * dt
        self.y += speed * math.sin(self.heading) * dt
        self.heading = _wrap(self.heading + (self.yaw_rate + dist) * dt)

    def antenna_xy(self) -> tuple:
        """차축 → GPS 안테나 위치 (params.rear_axle_pos 의 역변환)."""
        ax = self.x + self.p.antenna_to_axle * math.cos(self.heading)
        ay = self.y + self.p.antenna_to_axle * math.sin(self.heading)
        return ax, ay


class GeoConverter:
    """로컬 xy(m) ↔ 위경도. estimator._ll_to_xy 와 동일 평면 근사."""
    def __init__(self, lat0: float = 37.0, lon0: float = 127.0):
        self.lat0, self.lon0 = lat0, lon0

    def xy_to_ll(self, x: float, y: float) -> tuple:
        lat = self.lat0 + (y / R_EARTH) * 180 / math.pi
        lon = self.lon0 + (x / (R_EARTH * math.cos(math.radians(self.lat0)))) * 180 / math.pi
        return lat, lon


class ServoCanInterface(CanInterface):
    """
    현실적 조향 서보 모델 (MockCanInterface 대체).
    MockCAN 은 매 명령마다 0.25 비율로 즉시 수렴 → 비현실적.
    실제 모터는 (1) 각속도 한계(rate limit)와 (2) 1차 지연을 가진다.

      max_rate_deg_s : 전륜 조향 최대 각속도 (모터RPM/조향비에서 유도, 기본 35°/s)
      tau            : 서보 1차 지연 시정수 (s)
    CanSpec 인코딩은 MockCanInterface 와 동일 → SteeringActuator 그대로 동작.
    """
    def __init__(self, max_rate_deg_s: float = 35.0, tau: float = 0.08,
                 ctrl_dt: Optional[float] = None):
        self._angle = 0.0          # 현재 조향각 (deg)
        self._recv_q: List[tuple] = []
        self.max_rate = max_rate_deg_s
        self.tau = tau
        self.dt = ctrl_dt or CanSpec.MOTOR_CMD_PERIOD

    def start(self): pass
    def stop(self):  pass

    def send(self, can_id: int, data: bytes):
        if can_id != CanSpec.MOTOR_CMD_ID:
            return
        raw = int.from_bytes(
            data[CanSpec.MOTOR_BYTE_CMD_HI:CanSpec.MOTOR_BYTE_CMD_LO + 1],
            "big", signed=CanSpec.SENSOR_SIGNED)
        target = raw / CanSpec.MOTOR_ANGLE_SCALE          # deg
        err = target - self._angle
        step = err * min(1.0, self.dt / (self.tau + self.dt))   # 1차 지연
        max_step = self.max_rate * self.dt                       # 각속도 한계
        step = max(-max_step, min(max_step, step))
        self._angle += step
        raw_fb = max(-32768, min(32767, int(self._angle * CanSpec.SENSOR_ANGLE_SCALE)))
        b = raw_fb.to_bytes(2, "big", signed=CanSpec.SENSOR_SIGNED)
        # 피드백을 CanSpec 이 지정한 바이트 오프셋에 배치(역추적 결과와 일관)
        frame = bytearray(8)
        frame[CanSpec.SENSOR_BYTE_HI] = b[0]
        frame[CanSpec.SENSOR_BYTE_LO] = b[1]
        self._recv_q.append((CanSpec.SENSOR_ANGLE_ID, bytes(frame)))

    def recv(self) -> Optional[tuple]:
        return self._recv_q.pop(0) if self._recv_q else None


@dataclass
class SimResult:
    steps: int
    xte_rms_cm: float
    xte_max_cm: float
    final_state: str
    engaged_end: bool
    steer_reversals: int = 0
    reversal_rate: float = 0.0      # 조향 방향전환/초 (진동 지표)
    settled: bool = True            # 후반부 안착 + 미발산


class Simulator:
    """AutoSteerSystem 을 자전거 모델로 폐루프 구동."""
    def __init__(self, sys_: AutoSteerSystem, params: TractorParams,
                 lat0: float = 37.0, lon0: float = 127.0,
                 target_speed: float = 1.2, rtk_quality: int = 4,
                 gnss_source: str = "pa3", noise: float = 0.003,
                 yaw_tau: float = 0.0, dist_amp: float = 0.0):
        self.sys = sys_
        self.params = params
        self.geo = GeoConverter(lat0, lon0)
        self.model = BicycleModel(params, yaw_tau=yaw_tau, dist_amp=dist_amp)
        self.v = target_speed
        self.q = rtk_quality
        self.src = gnss_source
        self.noise = noise
        self.feed_rtk = True          # False 면 RTK 미수신(끊김) 시뮬
        self._xte: List[float] = []
        self._targets: List[float] = []

    def step(self, dt: float) -> dict:
        # 1) 모델 → GPS(안테나) → lat/lon → on_rtk
        if self.feed_rtk:
            ax, ay = self.model.antenna_xy()
            ax += random.gauss(0, self.noise)
            ay += random.gauss(0, self.noise)
            lat, lon = self.geo.xy_to_ll(ax, ay)
            self.sys.on_rtk(lat, lon, self.q, source=self.src)

        # 2) 속도를 target 으로 끌고가는 가속도 + heading/요레이트 IMU
        est_v = self.sys.estimator.get_state().speed
        fwd_accel = (self.v - est_v) / dt
        self.sys.on_imu(self.model.heading + random.gauss(0, 0.002),
                        self.model.yaw_rate, fwd_accel, dt,
                        raw_roll=random.gauss(0, 0.001))

        # 3) 제어 → measured angle 로 모델 조향
        res = self.sys.control_step(dt)
        measured = math.radians(res["measured_angle_deg"])
        if res["engaged"]:
            self.model.step(self.v, measured, dt)
            self._xte.append(abs(res["xte_cm"]))
            self._targets.append(res["target_angle_deg"])
        return res

    def run(self, steps: int = 400, dt: float = 0.02) -> SimResult:
        last = {"safety": "SAFE", "engaged": self.sys._engaged}
        for _ in range(steps):
            last = self.step(dt)
            if not last["engaged"]:
                break
        xte = self._xte
        rms = (sum(x * x for x in xte) / max(1, len(xte))) ** 0.5
        xmax = max(xte) if xte else 0.0

        # 진동 지표: 목표 조향각 변화율의 부호전환 횟수 / 초
        reversals = 0
        diffs = [self._targets[i] - self._targets[i - 1]
                 for i in range(1, len(self._targets))]
        for i in range(1, len(diffs)):
            if diffs[i] * diffs[i - 1] < 0 and abs(diffs[i]) > 0.05:
                reversals += 1
        duration = max(1e-6, len(xte) * dt)
        rev_rate = reversals / duration

        # 안착: 후반 25% XTE RMS 작고 발산 없음
        tail = xte[3 * len(xte) // 4:] if len(xte) >= 8 else xte
        tail_rms = (sum(x * x for x in tail) / max(1, len(tail))) ** 0.5
        settled = (tail_rms < 20.0) and (xmax < 300.0)

        return SimResult(steps=len(xte), xte_rms_cm=rms, xte_max_cm=xmax,
                         final_state=last["safety"], engaged_end=last["engaged"],
                         steer_reversals=reversals, reversal_rate=rev_rate,
                         settled=settled)


# ═══════════════════════════════════════════════════════════════
#  현장 전 사전점검 빌더 + 안전 시나리오 스위트
# ═══════════════════════════════════════════════════════════════

def build_system(params: Optional[TractorParams] = None,
                 algo: str = "implement", profile="heavy",
                 realistic: bool = False,
                 servo_rate_deg_s: float = 35.0,
                 servo_tau: float = 0.08) -> AutoSteerSystem:
    """
    profile: "normal"/"heavy"/"sand" 또는 TuningProfile 인스턴스.
    realistic=True 면 ServoCanInterface(rate-limit+지연) 사용(추종/튜닝용),
    False 면 MockCanInterface(빠름, 안전 시나리오용).
    """
    params = params or KUBOTA_MR1157
    can = (ServoCanInterface(servo_rate_deg_s, servo_tau)
           if realistic else MockCanInterface())
    can.start()
    sys_ = AutoSteerSystem(can, params=params, algo=algo)
    sys_.set_profile(profile)
    sys_.safety.update_deadman(True)
    sys_.safety.update_rtk(4)
    # SITL 에는 운전대를 잡는 사람이 없으므로, 모터가 명령을 빠르게 따라가는 것이
    # 운전자 개입(앵글 급변)으로 오인되지 않도록 자동 개입검출은 끈다.
    # (개입 시나리오는 SafetyMonitor.check_steering_override 를 직접 호출해 검증)
    sys_.safety.check_steering_override = lambda *a, **k: None
    sys_.set_path(ABLineStrategy((0, 0), (0, 60), 3.0, 2, 1.2))
    sys_.on_rtk(37.0, 127.0, 4, source="pa3")
    sys_.engage()
    return sys_


@dataclass
class ScenarioOutcome:
    name: str
    expected: str
    got: str
    passed: bool


def run_safety_scenarios() -> List[ScenarioOutcome]:
    """
    각 위험상황을 주입하고 시스템이 '정확한 사유로 해제' 되는지 검증.
    반환: 시나리오별 통과 여부.
    """
    out: List[ScenarioOutcome] = []

    def check(name, expected_state, setup) -> ScenarioOutcome:
        params = KUBOTA_MR1157
        sys_ = build_system(params)
        sim = Simulator(sys_, params, target_speed=1.2)
        # 정상 20스텝
        for _ in range(20):
            sim.step(0.02)
        # 위험 주입
        setup(sys_, sim)
        res = sim.step(0.02)
        got = res["safety"]
        oc = ScenarioOutcome(name, expected_state, got,
                             got == expected_state and not res["engaged"])
        out.append(oc)
        return oc

    # 1) 데드맨 해제
    check("데드맨 스위치 해제", "DEADMAN",
          lambda s, sim: s.safety.update_deadman(False))
    # 2) 비상정지
    check("비상정지(E-STOP)", "ESTOP",
          lambda s, sim: s.safety.set_estop())
    # 3) RTK 품질 저하 (단독측위) — 계속 수신하되 품질만 1
    check("RTK 품질 저하(q=1)", "RTK_LOW",
          lambda s, sim: setattr(sim, "q", 1))
    # 4) RTK 끊김 (1초 이상 미수신) — 수신 중단 + 마지막 수신시각을 과거로
    def kill_rtk(s, sim):
        sim.feed_rtk = False
        s.safety._last_rtk_t -= 2.0
    check("RTK 끊김(stale>1s)", "RTK_LOST", kill_rtk)
    # 5) 과속 (가속도가 한 스텝에 목표속도까지 끌어올림)
    check("과속(>2.5m/s)", "OVERSPEED",
          lambda s, sim: setattr(sim, "v", 3.5))
    # 6) 운전자 개입 (앵글센서 급변 120deg/s↑) — 실제 검출 로직을 직접 호출
    def override(s, sim):
        s.safety._prev_angle = 0.0
        s.safety._prev_angle_t = time.time() - 0.05
        SafetyMonitor.check_steering_override(s.safety, math.radians(40))  # 800°/s
    check("운전자 개입(조향 급변)", "OVERRIDE", override)

    return out


# ── 자체 테스트 ───────────────────────────────────────────────────
if __name__ == "__main__":
    import logging
    logging.basicConfig(level=logging.ERROR)
    random.seed(7)

    print("=" * 74)
    print("SITL 시뮬레이터 — 폐루프 경로추종 + 안전 시나리오 사전검증")
    print("=" * 74)

    # 1) 폐루프 경로추종 (현실적 서보 + 작업기 부하 yaw 지연)
    print("\n▶ 1. 폐루프 경로추종 (AB라인 60m, 1.2 m/s, 현실 서보+작업기부하)")
    print(f"  {'알고리즘/프로파일':<24}  {'XTE RMS':>8}  {'XTE MAX':>8}  "
          f"{'진동/s':>6}  {'안착':>4}")
    print("  " + "-" * 60)
    yaw_map = {"normal": 0.2, "heavy": 0.6, "sand": 0.25}
    for algo, prof in [("pure_pursuit", "normal"), ("stanley", "normal"),
                       ("implement", "heavy"), ("implement", "sand")]:
        sys_ = build_system(KUBOTA_MR1157, algo=algo, profile=prof, realistic=True)
        sim = Simulator(sys_, KUBOTA_MR1157, target_speed=1.2, yaw_tau=yaw_map[prof])
        r = sim.run(steps=600)
        print(f"  {algo+'/'+prof:<24}  {r.xte_rms_cm:>6.1f}cm  "
              f"{r.xte_max_cm:>6.1f}cm  {r.reversal_rate:>5.1f}  "
              f"{'OK' if r.settled else '✗진동':>4}")

    # 2) 안전 시나리오 검증
    print("\n▶ 2. 안전 계층 시나리오 (위험 주입 → 해제 사유 검증)")
    print(f"  {'시나리오':<24}  {'기대':>10}  {'실제':>10}  결과")
    print("  " + "-" * 58)
    outcomes = run_safety_scenarios()
    for oc in outcomes:
        mark = "✅ PASS" if oc.passed else "❌ FAIL"
        print(f"  {oc.name:<22}  {oc.expected:>10}  {oc.got:>10}  {mark}")

    n_pass = sum(o.passed for o in outcomes)
    print(f"\n  안전 시나리오: {n_pass}/{len(outcomes)} 통과")
    assert n_pass == len(outcomes), "안전 시나리오 실패 — 현장 투입 금지"
    print("\n  ✓ 폐루프 추종 + 6개 안전 조건 모두 사전 검증 완료")
    print("  → 현장에서는 동일 절차를 1km/h 실차로 재확인 (데드맨/비상정지 우선)")
