"""
calibration.py
==============
주행 데이터(RTK + 조향각 + heading)로 트랙터 기구학 파라미터를 자동 추정.

CLAUDE.md 우선순위 #1(실측) 부담을 줄인다 — 줄자 대신 "빈 농지에서 저속으로
원/사인 패턴 주행"만 하면 wheelbase 와 안테나 전후 오프셋을 역산.

이론 (자전거 모델, 후륜축 기준):
  요레이트:      ω = v · tan(δ) / L
                 → L = (v·tan δ) / ω          [WheelbaseEstimator]
  안테나 오프셋: 후륜축에서 전후로 d 떨어진 점의 속도 방향(COG)은
                 heading 과 atan2(ω·d, v) 만큼 차이 →
                 d = (COG − heading) · v / ω   (소각 근사)  [LeverArmEstimator]
                 d>0: 안테나가 차축 앞 / d<0: 뒤 (= antenna_to_axle)

원점 통과 최소제곱(robust) 으로 추정하며, 저속·저요레이트 샘플은 버린다.
numpy 불필요(순수 파이썬). 추정 후 field_config 로 JSON 에 기록하면 끝.
"""

from __future__ import annotations
import math
from dataclasses import dataclass
from typing import List, Optional


def _wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


@dataclass
class Estimate:
    value: float
    n_samples: int
    r2: float                 # 적합도 (0~1)
    ok: bool                  # 신뢰 가능 여부
    note: str = ""


def _fit_through_origin(xs: List[float], ys: List[float]) -> tuple:
    """y = a·x (원점 통과) 최소제곱. 반환 (a, r2)."""
    sxx = sum(x * x for x in xs)
    sxy = sum(x * y for x, y in zip(xs, ys))
    if sxx <= 1e-12:
        return 0.0, 0.0
    a = sxy / sxx
    # R² (원점 통과 모델)
    ss_res = sum((y - a * x) ** 2 for x, y in zip(xs, ys))
    ss_tot = sum(y * y for y in ys)
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 1e-12 else 0.0
    return a, max(0.0, r2)


class WheelbaseEstimator:
    """
    v·tan(δ) = L·ω  →  원점통과 회귀 기울기 = L (wheelbase).
    add_sample(speed, steer_rad, yaw_rate) 를 회전 구간에서 누적.
    """
    def __init__(self, min_speed: float = 0.3,
                 min_yaw_rate: float = 0.03,
                 max_steer_rad: float = math.radians(35)):
        self.min_speed = min_speed
        self.min_yaw_rate = min_yaw_rate
        self.max_steer = max_steer_rad
        self._x: List[float] = []   # ω
        self._y: List[float] = []   # v·tan(δ)

    def add_sample(self, speed: float, steer_rad: float, yaw_rate: float):
        if speed < self.min_speed:
            return
        if abs(yaw_rate) < self.min_yaw_rate:
            return
        if abs(steer_rad) < math.radians(1) or abs(steer_rad) > self.max_steer:
            return
        self._x.append(yaw_rate)
        self._y.append(speed * math.tan(steer_rad))

    def estimate(self) -> Estimate:
        n = len(self._x)
        if n < 20:
            return Estimate(0.0, n, 0.0, False, "샘플 부족(회전 구간 더 주행)")
        L, r2 = _fit_through_origin(self._x, self._y)
        ok = (0.8 <= L <= 5.0) and r2 > 0.9
        note = "" if ok else "신뢰도 낮음 — 일정 조향각으로 저속 원주행 권장"
        return Estimate(L, n, r2, ok, note)


class LeverArmEstimator:
    """
    안테나 전후 오프셋(antenna_to_axle) 추정.
    회전 중 GPS 진행방향(COG)과 heading 의 차이 = atan2(ω·d, v).
      (COG − heading) ≈ ω·d / v  →  v·(COG−heading) = d·ω
    원점통과 회귀 기울기 = d. d>0 앞, d<0 뒤.

    ★ COG 는 RTK 위치차로 구하므로, 짧은 구간에선 위치잡음이 각도잡음으로 증폭된다.
      → min_baseline(기본 0.5m) 이상 이동했을 때만 한 샘플 생성하고, 그 구간의
        heading/yaw_rate/speed 는 평균을 쓴다 (각도잡음 ∝ 잡음/baseline 으로 감소).
    """
    def __init__(self, min_speed: float = 0.4, min_yaw_rate: float = 0.05,
                 min_baseline: float = 0.5):
        self.min_speed = min_speed
        self.min_yaw_rate = min_yaw_rate
        self.min_baseline = min_baseline
        self._anchor: Optional[tuple] = None
        self._reset_acc()
        self._x: List[float] = []   # ω
        self._y: List[float] = []   # v·(COG−heading)

    def _reset_acc(self):
        self._s_sin = self._s_cos = self._s_yaw = self._s_v = 0.0
        self._cnt = 0

    def add_sample(self, x: float, y: float,
                   heading: float, yaw_rate: float, speed: float):
        if self._anchor is None:
            self._anchor = (x, y)
            self._reset_acc()
            return
        # 구간 누적
        self._s_sin += math.sin(heading)
        self._s_cos += math.cos(heading)
        self._s_yaw += yaw_rate
        self._s_v += speed
        self._cnt += 1
        dx, dy = x - self._anchor[0], y - self._anchor[1]
        if math.hypot(dx, dy) < self.min_baseline:
            return
        avg_h   = math.atan2(self._s_sin, self._s_cos)
        avg_yaw = self._s_yaw / self._cnt
        avg_v   = self._s_v / self._cnt
        self._anchor = (x, y)
        self._reset_acc()
        if avg_v < self.min_speed or abs(avg_yaw) < self.min_yaw_rate:
            return
        diff = _wrap(math.atan2(dy, dx) - avg_h)
        if abs(diff) > math.radians(45):        # 이상치 제거
            return
        self._x.append(avg_yaw)
        self._y.append(avg_v * diff)

    def estimate(self) -> Estimate:
        n = len(self._x)
        if n < 20:
            return Estimate(0.0, n, 0.0, False, "샘플 부족(회전 구간 더 주행)")
        d, r2 = _fit_through_origin(self._x, self._y)
        ok = (abs(d) <= 5.0) and r2 > 0.6
        side = "앞(+)" if d > 0 else "뒤(−)"
        note = f"안테나가 차축 {side}" if ok else "신뢰도 낮음 — 좌우 번갈아 회전 권장"
        return Estimate(d, n, r2, ok, note)


class HeadingCalibrator:
    """
    듀얼안테나(ver1) 헤딩 바이어스 캘리브 — 직선 자동주행(≈20m)으로 측정.

    원리: (안테나 보고 heading) vs (연속 GPS 위치의 진행방향=진로각) 의 원형평균 차이.
      = 안테나 베이스라인이 차체 종축과 어긋난 고정 yaw 바이어스(→ '중심 치우침' 원인).
    부호: bias = circular_mean(reported - course). on_heading 에서 reported - bias 로 보정.
    각도 단위: 나침반 진북기준(deg, 북=0, 시계방향) — AutoSteerSystem.on_heading 입력과 동일.
    """
    def __init__(self, min_distance_m: float = 15.0, min_samples: int = 80):
        self.min_distance = min_distance_m
        self.min_samples = min_samples
        self._sin = 0.0
        self._cos = 0.0
        self._n = 0
        self._last: Optional[tuple] = None
        self._dist = 0.0

    def add_sample(self, east: float, north: float, reported_heading_deg: float):
        if self._last is not None:
            de = east - self._last[0]; dn = north - self._last[1]
            step = math.hypot(de, dn)
            if step > 0.05:                              # 충분히 이동(정지 노이즈 제외)
                course = math.degrees(math.atan2(de, dn))    # 나침반(북=0,CW)
                diff = _wrap(math.radians(reported_heading_deg - course))
                self._sin += math.sin(diff); self._cos += math.cos(diff)
                self._n += 1; self._dist += step
        self._last = (east, north)

    @property
    def progress(self) -> float:
        return min(1.0, min(self._dist / self.min_distance, self._n / self.min_samples))

    def ready(self) -> bool:
        return self._dist >= self.min_distance and self._n >= self.min_samples

    def finish(self) -> Estimate:
        if self._n == 0:
            return Estimate(0.0, 0, 0.0, False, "샘플 없음")
        bias = math.degrees(math.atan2(self._sin, self._cos))
        R = math.hypot(self._sin, self._cos) / self._n   # 집중도(0~1) = 직진 안정성
        ok = self.ready() and R > 0.9
        return Estimate(bias, self._n, R, ok,
                        f"heading bias {bias:+.2f}° (dist {self._dist:.1f}m, R={R:.3f})")


class RollPitchEstimator:
    """
    가속도 기반 roll 추정 + (있으면) 자이로 보완필터 — 경사 보정(파라미터 2) 정밀화.

    원심가속 오염 배제: 차가 직진·저가속(|선가속도|·|yaw rate| 작음)일 때만 가속도로
    중력방향 roll 을 보정하고, 그 외 구간은 자이로(roll rate) 적분으로 유지.
    IMU 원시 가속도가 있을 때만 의미가 있다(없으면 베이스라인 틸트만 사용).
    """
    def __init__(self, alpha: float = 0.98,
                 max_lin_acc: float = 0.5, max_yaw_rate: float = 0.1):
        self.alpha = alpha            # 자이로:가속도 가중(고레이트 신뢰)
        self.max_lin_acc = max_lin_acc
        self.max_yaw_rate = max_yaw_rate
        self.roll = 0.0

    def update(self, ay: float, az: float, roll_rate: float = 0.0,
               dt: float = 0.02, lin_acc: float = 0.0,
               yaw_rate: float = 0.0) -> float:
        # 1) 자이로 적분(고레이트 평활)
        self.roll = _wrap(self.roll + roll_rate * dt)
        # 2) 정적(직진·저가속) 구간에서만 가속도 중력방향으로 보정
        if abs(lin_acc) <= self.max_lin_acc and abs(yaw_rate) <= self.max_yaw_rate:
            acc_roll = math.atan2(ay, az)
            self.roll = _wrap(self.alpha * self.roll + (1.0 - self.alpha) * acc_roll)
        return self.roll


def estimate_from_log(samples: List[dict]) -> dict:
    """
    주행 로그 일괄 추정 헬퍼.
    samples: [{x,y,heading,yaw_rate,speed,steer_rad}, ...] (시간순)
    반환: {"wheelbase": Estimate, "antenna_to_axle": Estimate}
    """
    wb = WheelbaseEstimator()
    la = LeverArmEstimator()
    for s in samples:
        wb.add_sample(s["speed"], s["steer_rad"], s["yaw_rate"])
        la.add_sample(s["x"], s["y"], s["heading"], s["yaw_rate"], s["speed"])
    return {"wheelbase": wb.estimate(), "antenna_to_axle": la.estimate()}


# ── 자체 테스트: 알려진 L, d 로 합성 주행 → 추정 정확도 검증 ──────────
if __name__ == "__main__":
    print("=" * 70)
    print("calibration — 자전거 모델 합성 주행으로 파라미터 역추정 검증")
    print("=" * 70)

    TRUE_L = 2.55          # 실제 wheelbase
    TRUE_D = -0.42         # 실제 antenna_to_axle (뒤쪽)
    dt = 0.05
    speed = 1.2

    # 좌우로 번갈아 조향하며 사인 패턴 주행 (회전 성분 확보)
    samples = []
    x = y = 0.0
    heading = math.pi / 2
    for k in range(1500):
        steer = math.radians(20) * math.sin(k * 0.02)
        yaw_rate = speed * math.tan(steer) / TRUE_L
        # 후륜축 전진
        x += speed * math.cos(heading) * dt
        y += speed * math.sin(heading) * dt
        heading = _wrap(heading + yaw_rate * dt)
        # 안테나(=차축에서 전후 d) 위치 + 약간의 RTK 노이즈
        import random
        ax = x + TRUE_D * math.cos(heading) + random.gauss(0, 0.003)
        ay = y + TRUE_D * math.sin(heading) + random.gauss(0, 0.003)
        samples.append(dict(x=ax, y=ay,
                            heading=heading + random.gauss(0, 0.002),
                            yaw_rate=yaw_rate, speed=speed, steer_rad=steer))

    res = estimate_from_log(samples)
    wb, la = res["wheelbase"], res["antenna_to_axle"]
    print(f"\nwheelbase     : 추정 {wb.value:.3f} m  (실제 {TRUE_L})  "
          f"n={wb.n_samples} R²={wb.r2:.3f} {'OK' if wb.ok else '✗'} {wb.note}")
    print(f"antenna_to_axle: 추정 {la.value:+.3f} m  (실제 {TRUE_D})  "
          f"n={la.n_samples} R²={la.r2:.3f} {'OK' if la.ok else '✗'} {la.note}")

    assert wb.ok and abs(wb.value - TRUE_L) < 0.1, f"wheelbase 오차 {wb.value}"
    assert la.ok and abs(la.value - TRUE_D) < 0.15, f"lever-arm 오차 {la.value}"
    print("\n  ✓ 저속 사인/원주행만으로 wheelbase·안테나 오프셋 자동 추정")
    print("  → field_config.save_config 로 JSON 에 기록하면 실측 대체/검산 가능")
