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
    ABLineStrategy, SafetyState, SafetyMonitor,
)

R_EARTH = 6_371_000.0


def _wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


class BicycleModel:
    """후륜축 기준 자전거 모델. 상태: 차축 (x,y), heading, yaw_rate."""
    def __init__(self, params: TractorParams,
                 x: float = 0.0, y: float = 0.0, heading: float = math.pi / 2):
        self.p = params
        self.x = x
        self.y = y
        self.heading = heading
        self.yaw_rate = 0.0

    def step(self, speed: float, steer_rad: float, dt: float):
        steer = max(-self.p.max_steer_rad, min(self.p.max_steer_rad, steer_rad))
        self.yaw_rate = speed * math.tan(steer) / self.p.wheelbase
        self.x += speed * math.cos(self.heading) * dt
        self.y += speed * math.sin(self.heading) * dt
        self.heading = _wrap(self.heading + self.yaw_rate * dt)

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


@dataclass
class SimResult:
    steps: int
    xte_rms_cm: float
    xte_max_cm: float
    final_state: str
    engaged_end: bool


class Simulator:
    """AutoSteerSystem 을 자전거 모델로 폐루프 구동."""
    def __init__(self, sys_: AutoSteerSystem, params: TractorParams,
                 lat0: float = 37.0, lon0: float = 127.0,
                 target_speed: float = 1.2, rtk_quality: int = 4,
                 gnss_source: str = "pa3", noise: float = 0.003):
        self.sys = sys_
        self.params = params
        self.geo = GeoConverter(lat0, lon0)
        self.model = BicycleModel(params)
        self.v = target_speed
        self.q = rtk_quality
        self.src = gnss_source
        self.noise = noise
        self.feed_rtk = True          # False 면 RTK 미수신(끊김) 시뮬
        self._xte: List[float] = []

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
        return res

    def run(self, steps: int = 400, dt: float = 0.02) -> SimResult:
        last = {"safety": "SAFE", "engaged": self.sys._engaged}
        for _ in range(steps):
            last = self.step(dt)
            if not last["engaged"]:
                break
        rms = (sum(x * x for x in self._xte) / max(1, len(self._xte))) ** 0.5
        return SimResult(steps=len(self._xte),
                         xte_rms_cm=rms,
                         xte_max_cm=max(self._xte) if self._xte else 0.0,
                         final_state=last["safety"],
                         engaged_end=last["engaged"])


# ═══════════════════════════════════════════════════════════════
#  현장 전 사전점검 빌더 + 안전 시나리오 스위트
# ═══════════════════════════════════════════════════════════════

def build_system(params: Optional[TractorParams] = None,
                 algo: str = "implement", profile: str = "heavy") -> AutoSteerSystem:
    params = params or KUBOTA_MR1157
    can = MockCanInterface(); can.start()
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

    # 1) 폐루프 경로추종 (알고리즘/프로파일별)
    print("\n▶ 1. 폐루프 경로추종 (AB라인 60m, 1.2 m/s)")
    print(f"  {'알고리즘/프로파일':<26}  {'XTE RMS':>9}  {'XTE MAX':>9}  {'종료상태':>10}")
    print("  " + "-" * 60)
    for algo, prof in [("pure_pursuit", "normal"), ("stanley", "normal"),
                       ("implement", "heavy"), ("implement", "sand")]:
        sys_ = build_system(KUBOTA_MR1157, algo=algo, profile=prof)
        sim = Simulator(sys_, KUBOTA_MR1157, target_speed=1.2)
        r = sim.run(steps=600)
        print(f"  {algo+'/'+prof:<26}  {r.xte_rms_cm:>7.1f}cm  "
              f"{r.xte_max_cm:>7.1f}cm  {r.final_state:>10}")

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
