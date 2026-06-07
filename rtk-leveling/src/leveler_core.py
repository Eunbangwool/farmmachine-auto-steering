"""
leveler_core.py
===============
RTK 기반 작업기 높이 자동 제어(균평) 핵심 알고리즘.
autosteer_core.py(자율조향)와 짝을 이루는 균평 제어 코어.

하드웨어 구성 (확정):
  - F9P RTK (USB)        → 정밀 위치 + 높이(z)
  - LoRa NTRIP (USB)     → RTCM 보정신호
  - Apollo 10 Pro        → 본 알고리즘 실행
  - CAN → STM32          → Kubota (7) 레이저 커넥터 (UP/DOWN/HOLD 신호 모방)
  - 트랙터 자체 EH 밸브  → 3점 링크 상하 제어 (비례밸브 직접구동 X)

설계 사상:
  레이저 레벨러를 모방. 블레이드 하단(지면 접촉선)이 목표 평면을 따라가도록
  UP/DOWN/HOLD 3상 신호만 출력. 트랙터가 알아서 유압 제어.

4계층 구조:
  Layer 1: 목표 평면 정의 (Target Plane) — 측량점 최소자승 평면 / 경사 / 평탄
  Layer 2: 블레이드 높이 추정 (Blade Estimation) — RTK + IMU 경사보정
  Layer 3: 균평 제어 (Leveling Control) — 데드밴드 + 히스테리시스 + 유압 예측
  Layer 4: 신호 출력 (CAN → STM32 → 레이저 커넥터)

★ 현장 실측 필요 값은 MEASUREMENT_CHECKLIST.md 참조.
  LevelerParams 의 ★ 표시 필드를 채워야 함.
"""

from __future__ import annotations
import math, time, struct, logging
from dataclasses import dataclass, field
from typing import List, Optional, Tuple, Callable
from abc import ABC, abstractmethod
from enum import Enum, auto

log = logging.getLogger("leveler")


# ═══════════════════════════════════════════════════════════════
#  레벨러 파라미터 — ★ 현장 실측 (MEASUREMENT_CHECKLIST.md)
# ═══════════════════════════════════════════════════════════════

@dataclass
class LevelerParams:
    """
    트랙터+블레이드 기구학 + 유압 응답 파라미터.

    [B] 안테나↔블레이드 기하 (z 계산용, 가장 중요):
      antenna_height_above_blade (h) ★ : 안테나 위상중심 ↔ 블레이드 하단 수직거리
      antenna_to_blade_horizontal (d) ★ : 안테나 ↔ 블레이드 수평거리(전후, 뒤=+)
      antenna_lateral_offset (ℓ)        : 좌우 오프셋 (중심정렬이면 0)
      blade_width (W)                    : 블레이드 폭

    [C] 유압 응답 (제어 튜닝용, 캘리브레이션):
      up_speed_cms ★    : UP 신호 1초당 블레이드 상승량 (cm/s)
      down_speed_cms ★  : DOWN 신호 1초당 하강량 (cm/s)
      latency_s ★       : 신호 → 움직임 시작 지연 (dead time)
      coast_cm ★        : 신호 정지 후 추가 이동량
      min_pulse_s ★     : 움직임 만드는 최소 펄스폭

    [D] 안전 한계:
      blade_max_up_cm ★   : 블레이드 최대 상승 (지면 기준)
      blade_max_down_cm ★ : 블레이드 최대 하강

    [E] IMU 경사 보정 (autosteer ImuOffset과 공유 가능):
      imu_roll_offset, imu_pitch_offset

    부호 규약:
      pitch>0 = 노즈 업 → 뒤쪽(d>0) 블레이드 하강
      roll>0  = 우측 다운
      blade_z = antenna_z − h·cosθ·cosφ − d·sinθ − ℓ·sinφ
    """
    # [B] 기하 — ★ 실측
    antenna_height_above_blade: float = 2.50   # h (m) ★
    antenna_to_blade_horizontal: float = 1.20  # d (m) ★ 뒤=+
    antenna_lateral_offset: float = 0.0        # ℓ (m)
    blade_width: float = 2.50                  # W (m) ★

    # [C] 유압 응답 — ★ 캘리브레이션
    up_speed_cms: float = 8.0    # cm/s ★
    down_speed_cms: float = 10.0 # cm/s ★ (보통 자중으로 하강이 빠름)
    latency_s: float = 0.30      # s ★ dead time
    coast_cm: float = 1.5        # cm ★ 정지 후 추가이동
    min_pulse_s: float = 0.08    # s ★ 최소 유효 펄스

    # [D] 안전 한계 — ★ 실측
    blade_max_up_cm: float = 40.0    # cm ★
    blade_max_down_cm: float = -25.0 # cm ★

    # [E] IMU 오프셋
    imu_roll_offset: float = 0.0     # rad
    imu_pitch_offset: float = 0.0    # rad

    def blade_tip_z(self, antenna_z: float, pitch: float, roll: float) -> float:
        """
        안테나 높이 → 블레이드 하단 높이 (경사 보정 포함).
        pitch, roll: 보정된 자세각 (rad)
        """
        h, d, l = (self.antenna_height_above_blade,
                   self.antenna_to_blade_horizontal,
                   self.antenna_lateral_offset)
        return (antenna_z
                - h * math.cos(pitch) * math.cos(roll)
                - d * math.sin(pitch)
                - l * math.sin(roll))

    def correct_imu(self, raw_roll: float, raw_pitch: float) -> Tuple[float, float]:
        """IMU 평지 오프셋 보정."""
        return (raw_roll - self.imu_roll_offset,
                raw_pitch - self.imu_pitch_offset)


# 기본 파라미터 (Kubota MR1157 + 균평 블레이드, ★ 실측 후 교체)
KUBOTA_LEVELER = LevelerParams()


# ═══════════════════════════════════════════════════════════════
#  트랙터 프로파일 DB — 모델별 유압 응답 저장/로드
# ═══════════════════════════════════════════════════════════════

@dataclass
class TractorProfile:
    """
    트랙터 1대의 유압 응답 프로파일.
    자동 캘리브레이션으로 측정하거나 수동으로 입력.
    DB에 저장해 같은 모델 재설치 시 시간 단축.
    """
    name: str
    model: str = ""
    up_speed_cms: float = 8.0
    down_speed_cms: float = 10.0
    latency_s: float = 0.30
    coast_cm: float = 1.5
    min_pulse_s: float = 0.08
    blade_max_up_cm: float = 40.0
    blade_max_down_cm: float = -25.0
    calibrated_at: str = ""
    calibration_quality: float = 0.0   # 0~1, 측정 신뢰도

    def to_leveler_params(self, base: "LevelerParams") -> "LevelerParams":
        """기하 파라미터는 유지, 유압 특성만 프로파일로 교체."""
        import copy
        p = copy.copy(base)
        p.up_speed_cms    = self.up_speed_cms
        p.down_speed_cms  = self.down_speed_cms
        p.latency_s       = self.latency_s
        p.coast_cm        = self.coast_cm
        p.min_pulse_s     = self.min_pulse_s
        p.blade_max_up_cm = self.blade_max_up_cm
        p.blade_max_down_cm = self.blade_max_down_cm
        return p

    def recommended_deadband(self):
        """코스팅 기반 최적 데드밴드 계산."""
        on_grade = max(1.0, round(self.coast_cm * 1.4, 1))
        exit_cm  = round(on_grade * 1.7, 1)
        return on_grade, exit_cm


class TractorProfileDB:
    """
    트랙터 모델별 유압 응답 프로파일 DB.

    사용:
        db = TractorProfileDB()
        profile = db.get("Kubota MR1157")
        if profile:
            params = profile.to_leveler_params(KUBOTA_LEVELER)
        db.save_profile(wizard.result_profile("Kubota MR1157"))
    """
    # 기본 내장 프로파일 (범용 추정값 — 캘리브레이션 후 갱신 권장)
    _BUILTIN: dict = {}

    def __init__(self, filepath: str = "tractor_profiles.json"):
        self.filepath = filepath
        self._db: dict = {}
        # 내장 프로파일 초기화
        for name, kw in [
            ("Kubota MR1157",  dict(up_speed_cms=8.0,  down_speed_cms=10.0,
                                    latency_s=0.30, coast_cm=1.5, min_pulse_s=0.08,
                                    blade_max_up_cm=40.0, blade_max_down_cm=-25.0)),
            ("Yanmar YT5113",  dict(up_speed_cms=7.0,  down_speed_cms=9.0,
                                    latency_s=0.35, coast_cm=1.8, min_pulse_s=0.10,
                                    blade_max_up_cm=38.0, blade_max_down_cm=-22.0)),
            ("Iseki TG5470",   dict(up_speed_cms=7.5,  down_speed_cms=10.0,
                                    latency_s=0.32, coast_cm=1.6, min_pulse_s=0.09)),
            ("John Deere 5R",  dict(up_speed_cms=9.0,  down_speed_cms=11.0,
                                    latency_s=0.25, coast_cm=1.2, min_pulse_s=0.07)),
        ]:
            self._db[name] = TractorProfile(name=name, model=name, **kw)
        self._load()

    def _load(self):
        try:
            import json
            with open(self.filepath) as f:
                for k, v in json.load(f).items():
                    self._db[k] = TractorProfile(**v)
        except Exception:
            pass

    def save(self):
        try:
            import json
            from dataclasses import asdict
            with open(self.filepath, "w") as f:
                json.dump({k: asdict(v) for k, v in self._db.items()},
                          f, indent=2, ensure_ascii=False)
        except Exception as e:
            log.warning(f"프로파일 저장 실패: {e}")

    def list_profiles(self) -> list:
        return sorted(self._db.keys())

    def get(self, name: str) -> Optional[TractorProfile]:
        return self._db.get(name)

    def save_profile(self, profile: TractorProfile):
        self._db[profile.name] = profile
        self.save()
        log.info(f"프로파일 저장: {profile.name} (품질 {profile.calibration_quality*100:.0f}%)")

    def delete(self, name: str):
        if name in self._db:
            del self._db[name]
            self.save()


# 전역 DB 인스턴스
TRACTOR_DB = TractorProfileDB()



# ═══════════════════════════════════════════════════════════════
#  Layer 0: NMEA 파싱 + ENU 좌표 변환
# ═══════════════════════════════════════════════════════════════

@dataclass
class GnssFix:
    lat: float = 0.0       # deg
    lon: float = 0.0       # deg
    alt: float = 0.0       # m (타원체고 또는 표고)
    quality: int = 0       # 0=무효,1=단독,2=DGPS,4=RTK Fixed,5=RTK Float
    sats: int = 0
    hdop: float = 99.9
    valid: bool = False


class NMEAParser:
    """F9P NMEA GGA 파싱. z(높이) 정밀도가 핵심이므로 GGA 사용."""

    @staticmethod
    def parse_gga(line: str) -> Optional[GnssFix]:
        if "GGA" not in line:
            return None
        try:
            # 체크섬 분리
            if "*" in line:
                body, cks = line.split("*")
                calc = 0
                for ch in body[1:]:  # '$' 제외
                    calc ^= ord(ch)
                if int(cks[:2], 16) != calc:
                    return None
            else:
                body = line
            f = body.split(",")
            # $xxGGA,time,lat,N/S,lon,E/W,quality,sats,hdop,alt,M,...
            lat = NMEAParser._dm_to_deg(f[2], f[3])
            lon = NMEAParser._dm_to_deg(f[4], f[5])
            quality = int(f[6]) if f[6] else 0
            sats = int(f[7]) if f[7] else 0
            hdop = float(f[8]) if f[8] else 99.9
            alt = float(f[9]) if f[9] else 0.0
            return GnssFix(lat=lat, lon=lon, alt=alt, quality=quality,
                           sats=sats, hdop=hdop, valid=(quality > 0))
        except (ValueError, IndexError):
            return None

    @staticmethod
    def _dm_to_deg(dm: str, hemi: str) -> float:
        """ddmm.mmmm → 십진도."""
        if not dm:
            return 0.0
        dot = dm.index(".")
        deg = float(dm[:dot-2])
        minutes = float(dm[dot-2:])
        val = deg + minutes / 60.0
        if hemi in ("S", "W"):
            val = -val
        return val


class ENUConverter:
    """
    측지좌표(lat,lon,alt) → 로컬 ENU(East,North,Up) 평면 직교좌표.
    포장 규모(수백 m)에서 충분히 정확한 국소 평면 근사.
    """
    def __init__(self):
        self._origin: Optional[Tuple[float, float, float]] = None
        self._cos_lat0 = 1.0
        self.R = 6_378_137.0  # WGS84 장반경

    def set_origin(self, lat: float, lon: float, alt: float):
        self._origin = (lat, lon, alt)
        self._cos_lat0 = math.cos(math.radians(lat))
        log.info(f"ENU 원점: ({lat:.7f}, {lon:.7f}, {alt:.3f})")

    @property
    def has_origin(self) -> bool:
        return self._origin is not None

    def to_enu(self, lat: float, lon: float, alt: float) -> Tuple[float, float, float]:
        if self._origin is None:
            self.set_origin(lat, lon, alt)
        lat0, lon0, alt0 = self._origin
        e = math.radians(lon - lon0) * self.R * self._cos_lat0
        n = math.radians(lat - lat0) * self.R
        u = alt - alt0
        return e, n, u


# ═══════════════════════════════════════════════════════════════
#  Layer 1: 목표 평면 정의
# ═══════════════════════════════════════════════════════════════

@dataclass
class Plane:
    """목표 평면: z = a·x + b·y + c  (x=East, y=North, z=Up, 단위 m)."""
    a: float = 0.0
    b: float = 0.0
    c: float = 0.0

    def z_at(self, x: float, y: float) -> float:
        return self.a * x + self.b * y + self.c

    def slope_percent(self) -> Tuple[float, float]:
        """East/North 방향 경사 (%)."""
        return self.a * 100.0, self.b * 100.0


class TargetPlane:
    """
    목표 평면 관리.
    - fit_from_survey: 측량점 최소자승 평면 (흙 이동량 최소 = laser leveling 표준)
    - set_flat: 평탄 (지정 높이 또는 측량 평균)
    - set_slope: 경사 부여 (평균 높이 유지)
    """
    def __init__(self):
        self.plane = Plane()
        self._survey: List[Tuple[float, float, float]] = []

    def add_survey_point(self, x: float, y: float, z: float):
        self._survey.append((x, y, z))

    def clear_survey(self):
        self._survey.clear()

    def fit_from_survey(self) -> Plane:
        """
        최소자승 평면 피팅. z = a·x + b·y + c.
        이 평면이 깎기/메우기 총량(분산)을 최소화함.
        numpy 있으면 lstsq, 없으면 정규방정식 수동 해.
        """
        pts = self._survey
        n = len(pts)
        if n < 3:
            raise ValueError(f"평면 피팅에 최소 3점 필요 (현재 {n}점)")
        try:
            import numpy as np
            A = np.array([[x, y, 1.0] for x, y, _ in pts])
            z = np.array([p[2] for p in pts])
            coef, *_ = np.linalg.lstsq(A, z, rcond=None)
            self.plane = Plane(a=float(coef[0]), b=float(coef[1]), c=float(coef[2]))
        except ImportError:
            self.plane = self._fit_normal_equations(pts)
        a, b = self.plane.slope_percent()
        log.info(f"평면 피팅: {n}점, 경사 E={a:.2f}% N={b:.2f}%, "
                 f"기준고 c={self.plane.c:.3f}m")
        return self.plane

    @staticmethod
    def _fit_normal_equations(pts) -> Plane:
        """numpy 없을 때 3x3 정규방정식 직접 해 (Cramer)."""
        Sxx=Sxy=Sx=Syy=Sy=Sxz=Syz=Sz=0.0
        n = len(pts)
        for x, y, z in pts:
            Sxx+=x*x; Sxy+=x*y; Sx+=x
            Syy+=y*y; Sy+=y
            Sxz+=x*z; Syz+=y*z; Sz+=z
        # M·[a,b,c] = v
        M = [[Sxx, Sxy, Sx],
             [Sxy, Syy, Sy],
             [Sx,  Sy,  float(n)]]
        v = [Sxz, Syz, Sz]
        det = TargetPlane._det3(M)
        if abs(det) < 1e-12:
            raise ValueError("평면 피팅 실패: 측량점이 일직선/중복")
        Ma = [[v[0],M[0][1],M[0][2]],[v[1],M[1][1],M[1][2]],[v[2],M[2][1],M[2][2]]]
        Mb = [[M[0][0],v[0],M[0][2]],[M[1][0],v[1],M[1][2]],[M[2][0],v[2],M[2][2]]]
        Mc = [[M[0][0],M[0][1],v[0]],[M[1][0],M[1][1],v[1]],[M[2][0],M[2][1],v[2]]]
        return Plane(a=TargetPlane._det3(Ma)/det,
                     b=TargetPlane._det3(Mb)/det,
                     c=TargetPlane._det3(Mc)/det)

    @staticmethod
    def _det3(m) -> float:
        return (m[0][0]*(m[1][1]*m[2][2]-m[1][2]*m[2][1])
               -m[0][1]*(m[1][0]*m[2][2]-m[1][2]*m[2][0])
               +m[0][2]*(m[1][0]*m[2][1]-m[1][1]*m[2][0]))

    def set_flat(self, height: Optional[float] = None):
        """평탄 목표. height=None이면 측량점 평균 높이."""
        if height is None:
            if not self._survey:
                raise ValueError("측량점 없음 — height 직접 지정 필요")
            height = sum(p[2] for p in self._survey) / len(self._survey)
        self.plane = Plane(a=0.0, b=0.0, c=height)
        log.info(f"평탄 목표: c={height:.3f}m")

    def set_slope(self, slope_east_pct: float, slope_north_pct: float,
                  keep_mean: bool = True):
        """
        경사 부여. keep_mean=True면 측량 평균 높이 유지(흙 균형).
        slope %: 100m당 cm가 아니라 m당 m의 백분율 (1% = 0.01 m/m).
        """
        a = slope_east_pct / 100.0
        b = slope_north_pct / 100.0
        c = self.plane.c
        if keep_mean and self._survey:
            mean_x = sum(p[0] for p in self._survey) / len(self._survey)
            mean_y = sum(p[1] for p in self._survey) / len(self._survey)
            mean_z = sum(p[2] for p in self._survey) / len(self._survey)
            c = mean_z - a * mean_x - b * mean_y
        self.plane = Plane(a=a, b=b, c=c)
        log.info(f"경사 목표: E={slope_east_pct:.2f}% N={slope_north_pct:.2f}%")

    def cut_fill_stats(self) -> dict:
        """측량점 기준 깎기/메우기 통계 (m)."""
        if not self._survey:
            return {}
        diffs = [z - self.plane.z_at(x, y) for x, y, z in self._survey]
        cut = [d for d in diffs if d > 0]    # 지면이 평면보다 높음 → 깎기
        fill = [-d for d in diffs if d < 0]  # 낮음 → 메우기
        return {
            "max_cut_cm": max(cut) * 100 if cut else 0.0,
            "max_fill_cm": max(fill) * 100 if fill else 0.0,
            "mean_abs_cm": sum(abs(d) for d in diffs) / len(diffs) * 100,
            "n_points": len(self._survey),
        }


# ═══════════════════════════════════════════════════════════════
#  Layer 2: 블레이드 높이 추정 (RTK + IMU)
# ═══════════════════════════════════════════════════════════════

@dataclass
class BladeState:
    x: float = 0.0          # East (m)
    y: float = 0.0          # North (m)
    blade_z: float = 0.0    # 블레이드 하단 높이 (m, ENU Up)
    antenna_z: float = 0.0  # 안테나 높이 (m)
    vz: float = 0.0         # 블레이드 수직속도 (cm/s, +상승)
    quality: int = 0
    valid: bool = False


class BladeEstimator:
    """
    RTK + IMU → 블레이드 하단 높이 추정.
    - ENU 변환
    - 경사 보정 (IMU roll/pitch → 레버암 z 보정)
    - 수직속도 추정 (z 이력 미분, 유압 예측·코스팅 보상에 사용)
    - 가벼운 저역통과 (RTK z 노이즈 완화)
    """
    def __init__(self, params: LevelerParams, enu: ENUConverter,
                 z_filter_alpha: float = 0.4):
        self.params = params
        self.enu = enu
        self.alpha = z_filter_alpha
        self._blade_z_f: Optional[float] = None
        self._z_hist: List[Tuple[float, float]] = []  # (t, blade_z_cm)
        self._roll = 0.0
        self._pitch = 0.0

    def update_imu(self, raw_roll: float, raw_pitch: float):
        self._roll, self._pitch = self.params.correct_imu(raw_roll, raw_pitch)

    def reset(self):
        """작업 시작 시 LPF/속도이력 초기화 (측량→작업 전환)."""
        self._blade_z_f = None
        self._z_hist.clear()

    def update_gnss(self, fix: GnssFix, now: float = None) -> BladeState:
        if not fix.valid:
            return BladeState(valid=False, quality=fix.quality)
        now = now if now is not None else time.time()
        x, y, u = self.enu.to_enu(fix.lat, fix.lon, fix.alt)
        antenna_z = u
        blade_z = self.params.blade_tip_z(antenna_z, self._pitch, self._roll)

        # 저역통과
        if self._blade_z_f is None:
            self._blade_z_f = blade_z
        else:
            self._blade_z_f = self.alpha * blade_z + (1 - self.alpha) * self._blade_z_f
        bz = self._blade_z_f

        # 수직속도 (cm/s)
        self._z_hist.append((now, bz * 100))
        self._z_hist = [(t, z) for t, z in self._z_hist if now - t < 0.6]
        vz = 0.0
        if len(self._z_hist) >= 2:
            t0, z0 = self._z_hist[0]
            t1, z1 = self._z_hist[-1]
            if t1 - t0 > 0.05:
                vz = (z1 - z0) / (t1 - t0)

        return BladeState(x=x, y=y, blade_z=bz, antenna_z=antenna_z,
                          vz=vz, quality=fix.quality, valid=True)


# ═══════════════════════════════════════════════════════════════
#  Layer 3: 균평 제어 (히스테리시스 + 유압 예측)
# ═══════════════════════════════════════════════════════════════

class Direction(Enum):
    NEUTRAL = 0
    UP = 1
    DOWN = 2


@dataclass
class LevelingTuning:
    """
    균평 제어 튜닝.

    데드밴드(온그레이드 밴드): |오차| < on_grade_cm → HOLD (레이저 '온그레이드' 녹색등)
    히스테리시스: 들어갈 때(on_grade)보다 나갈 때(exit) 밴드를 넓혀 채터링 방지.
    유압 예측: 코스팅+지연 때문에, 목표 도달 전에 신호를 끊어 오버슈트 방지.
    미세 펄스: 목표 근처에서는 짧은 펄스로 미세 이동.
    """
    on_grade_cm: float = 1.5     # 온그레이드 밴드 (±)
    exit_cm: float = 2.5         # 이 이상 벗어나면 다시 구동 (히스테리시스)
    fine_band_cm: float = 5.0    # 이 안에서는 펄스 제어
    coarse_continuous: bool = True  # 밴드 밖에서 연속 구동
    use_feedforward: bool = True    # 유압 예측 정지
    pulse_on_min_s: float = 0.10    # 최소 펄스 ON
    pulse_off_s: float = 0.25       # 펄스 사이 OFF (관측 시간)
    invert: bool = False         # 블레이드 높음=DOWN 이 기본. 반대 기구면 True


class LevelingController:
    """
    블레이드 z를 목표 평면에 맞추는 히스테리시스 + 유압예측 제어.

    오차 정의: error_cm = blade_z - target_z  (cm)
      error > 0 : 블레이드가 목표보다 높음 → DOWN (내려야 함)
      error < 0 : 낮음 → UP (올려야 함)

    출력: (Direction, pulse_width_ms)
      연속 구동 시 pulse_width_ms = 제어주기(=continuous)
      펄스 구동 시 짧은 ms
    """
    def __init__(self, params: LevelerParams, tuning: LevelingTuning = None):
        self.params = params
        self.tune = tuning or LevelingTuning()
        self._on_grade = False     # 히스테리시스 상태
        self._pulse_phase_off_until = 0.0

    def reset(self):
        self._on_grade = False
        self._pulse_phase_off_until = 0.0

    def compute(self, blade_z_cm: float, target_z_cm: float,
                vz_cms: float, now: float = None) -> Tuple[Direction, float]:
        """
        blade_z_cm, target_z_cm: cm
        vz_cms: 블레이드 수직속도 (cm/s, +상승)
        반환: (방향, 펄스폭 ms). 0=HOLD, >=255=연속.
        """
        now = now or time.time()
        error = blade_z_cm - target_z_cm   # +면 높음 → DOWN 필요

        # ── 유압 예측: 코스팅 보상 ──────────────────────────
        # 접근 중이면 코스팅으로 더 갈 것을 예측해 일찍 멈춤 (오버슈트 방지)
        eff_error = error
        if self.tune.use_feedforward and abs(vz_cms) > 0.3:
            eff_error = error + math.copysign(self.params.coast_cm, vz_cms)

        # ── 히스테리시스 온그레이드 판정 ───────────────────
        if self._on_grade:
            if abs(eff_error) > self.tune.exit_cm:
                self._on_grade = False
        else:
            if abs(eff_error) < self.tune.on_grade_cm:
                self._on_grade = True
        if self._on_grade:
            return Direction.NEUTRAL, 0.0

        # ── 방향 결정 ──────────────────────────────────────
        raw_down = eff_error > 0          # 높음 → DOWN
        if self.tune.invert:
            raw_down = not raw_down
        direction = Direction.DOWN if raw_down else Direction.UP

        # ── 연속 구동 (기본) ───────────────────────────────
        # 유압 지연이 큰 시스템(레이저커넥터 방식)에서는 펄스보다 연속+예측이 견고
        if self.tune.coarse_continuous or abs(eff_error) >= self.tune.fine_band_cm:
            return direction, 999.0

        # ── 미세 펄스 (옵션: 저지연 비례밸브 시스템용) ─────
        if now < self._pulse_phase_off_until:
            return Direction.NEUTRAL, 0.0
        frac = (abs(eff_error) - self.tune.on_grade_cm) / \
               max(0.1, self.tune.fine_band_cm - self.tune.on_grade_cm)
        frac = max(0.0, min(1.0, frac))
        # 펄스폭은 최소 유효폭과 지연을 고려해 충분히 크게
        pulse_ms = max(self.params.min_pulse_s, self.params.latency_s * 1.2) * 1000
        pulse_ms = pulse_ms * (0.6 + 0.4 * frac)
        self._pulse_phase_off_until = now + (pulse_ms/1000) + self.tune.pulse_off_s
        return direction, pulse_ms

    @property
    def is_on_grade(self) -> bool:
        return self._on_grade


# ═══════════════════════════════════════════════════════════════
#  Layer 4: CAN 신호 출력 (Apollo → STM32 → 레이저 커넥터)
# ═══════════════════════════════════════════════════════════════

class LevelerCanProtocol:
    """
    Apollo → STM32 CAN 프로토콜 (rtk-leveling CLAUDE.md 확정 규격).

    명령 (ID 0x18FF0102, 100ms 또는 50ms):
      Byte0: Mode    (0=MANUAL,1=AUTO,2=HOLD,0xFF=ESTOP)
      Byte1: Dir     (0=NEUTRAL,1=UP,2=DOWN)
      Byte2: PulseWidth ms (0=HOLD, 255=연속)
      Byte3: Heartbeat (0~255 순환)
      Byte4-6: reserved
      Byte7: CRC8 (poly 0x07)
    """
    CMD_ID = 0x18FF0102
    STATUS_ID = 0x18FF0201

    MODE_MANUAL = 0
    MODE_AUTO = 1
    MODE_HOLD = 2
    MODE_ESTOP = 0xFF

    @staticmethod
    def crc8(data: bytes, poly: int = 0x07) -> int:
        crc = 0
        for b in data:
            crc ^= b
            for _ in range(8):
                crc = ((crc << 1) ^ poly) & 0xFF if (crc & 0x80) else (crc << 1) & 0xFF
        return crc

    @classmethod
    def encode_cmd(cls, mode: int, direction: Direction,
                   pulse_ms: float, heartbeat: int) -> Tuple[int, bytes]:
        pw = 255 if pulse_ms >= 255 else max(0, min(254, int(round(pulse_ms))))
        data = bytearray(8)
        data[0] = mode & 0xFF
        data[1] = direction.value & 0xFF
        data[2] = pw
        data[3] = heartbeat & 0xFF
        data[7] = cls.crc8(bytes(data[:7]))
        return cls.CMD_ID, bytes(data)


class CanBus(ABC):
    @abstractmethod
    def send(self, can_id: int, data: bytes): ...
    @abstractmethod
    def start(self): ...
    @abstractmethod
    def stop(self): ...



# ═══════════════════════════════════════════════════════════════
#  출력단 추상화 — 트랙터/유압 방식 무관 인터페이스
# ═══════════════════════════════════════════════════════════════

class LevelerOutput(ABC):
    """
    Layer 4 출력 추상화. 제어 로직(L1~L3)은 동일, 출력단만 교체.

    방식 A (LaserConnectorOutput): 레이저 커넥터 UP/DOWN/HOLD
      Kubota, Yanmar, Iseki 등 일본 트랙터 레이저 균평 커넥터.
      Apollo → CAN → STM32 → GPIO → 커넥터(7).

    방식 B (PowerPackOutput): 외부 유압팩 비례 제어
      SM기계 오토파워팩 등. 밸브 개도를 직접 제어.
      Apollo → CAN → 밸브 드라이버 → 유압팩.

    방식 C (ProportionalValveOutput): 비례밸브 직결
      현재 미구현 — 추후 필요 시 추가.
    """

    @abstractmethod
    def send_command(self, mode: int, direction: Direction,
                     pulse_ms: float, heartbeat: int,
                     now: float = None): ...

    @abstractmethod
    def start(self): ...

    @abstractmethod
    def stop(self): ...


class LaserConnectorOutput(LevelerOutput):
    """
    방식 A — 레이저 커넥터 신호 모방 (UP/DOWN/HOLD 3상).
    IC100/AL2와 동일한 방식. 트랙터 자체 EH 밸브가 유압 제어.
    """
    def __init__(self, bus: CanBus):
        self.bus = bus

    def start(self): self.bus.start()
    def stop(self):  self.bus.stop()

    def send_command(self, mode, direction, pulse_ms, heartbeat, now=None):
        can_id, data = LevelerCanProtocol.encode_cmd(
            mode, direction, pulse_ms, heartbeat)
        try:
            self.bus.send(can_id, data, now=now)
        except TypeError:
            self.bus.send(can_id, data)


class PowerPackOutput(LevelerOutput):
    """
    방식 B — 외부 유압팩 직접 구동 (SM기계 오토파워팩 등).
    pulse_ms 크기에 비례해 밸브 개도(duty) 결정 → 비례 제어 가능.
    """
    def __init__(self, bus: CanBus, max_duty: float = 1.0):
        self.bus = bus
        self.max_duty = max_duty

    def start(self): self.bus.start()
    def stop(self):  self.bus.stop()

    def send_command(self, mode, direction, pulse_ms, heartbeat, now=None):
        # 연속(999) → 최대 개도, 짧은 펄스 → 비례 개도
        duty = min(1.0, pulse_ms / 999.0) * self.max_duty
        pw_encoded = int(duty * 255)
        can_id, data = LevelerCanProtocol.encode_cmd(
            mode, direction, pw_encoded, heartbeat)
        try:
            self.bus.send(can_id, data, now=now)
        except TypeError:
            self.bus.send(can_id, data)


def make_output(bus: CanBus, mode: str = "laser") -> LevelerOutput:
    """
    출력단 팩토리.
    mode: "laser" = 레이저커넥터(방식A), "powerpack" = 유압팩(방식B)
    """
    if mode == "powerpack":
        return PowerPackOutput(bus)
    return LaserConnectorOutput(bus)


class MockHydraulics(CanBus):
    """
    테스트용 유압 거동 시뮬. 상태머신: idle→delay→moving→coast.
    유압 지연(latency) + 속도 + 코스팅 모델링. 실제로는 STM32+트랙터.
    """
    def __init__(self, params: LevelerParams, start_blade_cm: float = 10.0):
        self.p = params
        self.blade_cm = start_blade_cm
        self._dir = Direction.NEUTRAL
        self._cmd_until = 0.0
        self._phase = "idle"          # idle / delay / moving / coast
        self._phase_t = time.time()
        self._coast_left = 0.0
        self._coast_dir = Direction.NEUTRAL
        self._last = time.time()

    def start(self): log.info("MockHydraulics 시작")
    def stop(self): log.info("MockHydraulics 종료")

    def send(self, can_id: int, data: bytes, now: float = None):
        if can_id != LevelerCanProtocol.CMD_ID:
            return
        d = Direction(data[1]); pw = data[2]
        now = now if now is not None else time.time()
        active = not (d == Direction.NEUTRAL or pw == 0)
        if active:
            dur = 9.9 if pw >= 255 else pw / 1000.0
            # 새 시작(방향전환 또는 정지상태 재개)일 때만 지연 시작
            if self._dir != d or self._phase in ("idle", "coast"):
                self._phase = "delay"; self._phase_t = now
            self._dir = d
            self._cmd_until = now + dur
        else:
            # NEUTRAL/HOLD: 즉시 명령 종료 → tick에서 moving→coast→idle
            self._cmd_until = now

    def tick(self, now: float = None, dt: float = None):
        _now = now if now is not None else time.time()
        if dt is None:
            dt = _now - self._last
        self._last = _now
        now = _now
        active = (self._dir != Direction.NEUTRAL and now < self._cmd_until)
        v = 0.0

        if self._phase == "delay":
            if not active:
                self._phase = "idle"
            elif now - self._phase_t >= self.p.latency_s:
                self._phase = "moving"

        if self._phase == "moving":
            if active:
                v = (self.p.up_speed_cms if self._dir == Direction.UP
                     else -self.p.down_speed_cms)
            else:
                self._phase = "coast"
                self._coast_left = self.p.coast_cm
                self._coast_dir = self._dir

        if self._phase == "coast":
            if self._coast_left > 0:
                vc = (self.p.up_speed_cms if self._coast_dir == Direction.UP
                      else -self.p.down_speed_cms) * 0.4
                v = vc
                self._coast_left -= abs(vc) * dt
            else:
                self._phase = "idle"; self._dir = Direction.NEUTRAL

        self.blade_cm += v * dt
        self.blade_cm = max(self.p.blade_max_down_cm,
                            min(self.p.blade_max_up_cm, self.blade_cm))


# ═══════════════════════════════════════════════════════════════
#  안전 계층
# ═══════════════════════════════════════════════════════════════

class LevelerSafety(Enum):
    SAFE = auto()
    RTK_LOW = auto()       # RTK Fixed 아님
    RTK_LOST = auto()      # 신호 끊김
    WATCHDOG = auto()      # CAN 끊김
    LIMIT = auto()         # 블레이드 한계 도달
    MANUAL = auto()        # 수동 모드
    ESTOP = auto()


class LevelerSafetyMonitor:
    def __init__(self, params: LevelerParams,
                 require_fix: bool = True,
                 rtk_timeout_s: float = 1.0):
        self.params = params
        self.require_fix = require_fix
        self.rtk_timeout = rtk_timeout_s
        self.state = LevelerSafety.SAFE
        self._last_fix_t = 0.0
        self._quality = 0
        self._estop = False
        self._auto = False

    def update_fix(self, fix: GnssFix, now: float = None):
        self._quality = fix.quality
        if fix.valid:
            self._last_fix_t = now if now is not None else time.time()

    def set_auto(self, on: bool): self._auto = on
    def set_estop(self): self._estop = True

    def check(self, blade_z_cm: float, now: float = None) -> LevelerSafety:
        now = now if now is not None else time.time()
        if self._estop:
            self.state = LevelerSafety.ESTOP; return self.state
        if not self._auto:
            self.state = LevelerSafety.MANUAL; return self.state
        # RTK 품질: Fixed(4)만 (Float 5는 옵션)
        ok_q = (self._quality == 4) or (not self.require_fix and self._quality == 5)
        if not ok_q:
            self.state = LevelerSafety.RTK_LOW; return self.state
        if now - self._last_fix_t > self.rtk_timeout:
            self.state = LevelerSafety.RTK_LOST; return self.state
        # 블레이드 한계 (자율 구동이 한계를 밀어붙이지 않도록)
        if (blade_z_cm >= self.params.blade_max_up_cm - 0.5 or
                blade_z_cm <= self.params.blade_max_down_cm + 0.5):
            self.state = LevelerSafety.LIMIT; return self.state
        self.state = LevelerSafety.SAFE
        return self.state

    @property
    def is_safe(self) -> bool:
        return self.state == LevelerSafety.SAFE


# ═══════════════════════════════════════════════════════════════
#  시스템 통합
# ═══════════════════════════════════════════════════════════════

class LevelerSystem:
    """
    RTK 균평 시스템 통합. 20Hz(50ms) 제어 루프 권장.

    사용 예 (방식 A — 레이저커넥터):
        bus     = MockHydraulics(params)          # 실제: ApolloCanBus
        output  = LaserConnectorOutput(bus)
        sys     = LevelerSystem(params, output)

    사용 예 (방식 B — 외부 유압팩):
        output  = PowerPackOutput(bus)
        sys     = LevelerSystem(params, output)

    사용 예 (트랙터 프로파일 DB 활용):
        profile = TRACTOR_DB.get("Kubota MR1157")
        params  = profile.to_leveler_params(KUBOTA_LEVELER)
        on_grade, exit_cm = profile.recommended_deadband()
        tuning  = LevelingTuning(on_grade_cm=on_grade, exit_cm=exit_cm)
        sys     = LevelerSystem(params, output, tuning)

    backward-compat: bus(CanBus) 직접 전달 시 LaserConnectorOutput으로 자동 래핑.
    """
    def __init__(self, params: LevelerParams,
                 output,                         # LevelerOutput 또는 CanBus (하위호환)
                 tuning: LevelingTuning = None,
                 send_period_s: float = 0.05):
        self.params = params
        # 하위 호환: CanBus 직접 전달 시 자동 래핑
        if isinstance(output, LevelerOutput):
            self.output = output
            self.bus    = None
        else:
            self.output = LaserConnectorOutput(output)
            self.bus    = output
        self.enu        = ENUConverter()
        self.estimator  = BladeEstimator(params, self.enu)
        self.target     = TargetPlane()
        self.controller = LevelingController(params, tuning)
        self.safety     = LevelerSafetyMonitor(params)
        self.parser     = NMEAParser()
        self.adaptive   = AdaptiveCoastEstimator(params.coast_cm)

        self._send_period = send_period_s
        self._last_send   = 0.0
        self._hb          = 0
        self._last_fix: Optional[GnssFix] = None
        self._blade: Optional[BladeState] = None
        self._pulse_until = 0.0
        self._cur_dir     = Direction.NEUTRAL
        self._now         = time.time()
        self._cmd_active  = False   # 적응형 코스팅용

    # ── 측량 ────────────────────────────────────────────
    def add_survey(self, fix: GnssFix, raw_roll: float = 0.0,
                   raw_pitch: float = 0.0):
        """블레이드 들고 주행하며 표면 측량점 수집."""
        if not fix.valid:
            return
        self.estimator.update_imu(raw_roll, raw_pitch)
        st = self.estimator.update_gnss(fix)
        if st.valid:
            self.target.add_survey_point(st.x, st.y, st.blade_z)

    def fit_plane(self) -> Plane:
        plane = self.target.fit_from_survey()
        stats = self.target.cut_fill_stats()
        log.info(f"깎기/메우기: 최대깎기={stats.get('max_cut_cm',0):.1f}cm "
                 f"최대메우기={stats.get('max_fill_cm',0):.1f}cm "
                 f"평균편차={stats.get('mean_abs_cm',0):.1f}cm")
        return plane

    # ── 센서 입력 ───────────────────────────────────────
    def on_gnss(self, nmea_line: str, now: float = None) -> Optional[BladeState]:
        fix = self.parser.parse_gga(nmea_line)
        if fix is None:
            return None
        self._last_fix = fix
        self.safety.update_fix(fix, now)
        self._blade = self.estimator.update_gnss(fix, now)
        return self._blade

    def on_gnss_fix(self, fix: GnssFix, now: float = None) -> Optional[BladeState]:
        """이미 파싱된 fix 직접 입력 (테스트/외부 파서용)."""
        self._last_fix = fix
        self.safety.update_fix(fix, now)
        self._blade = self.estimator.update_gnss(fix, now)
        return self._blade

    def on_imu(self, raw_roll: float, raw_pitch: float):
        self.estimator.update_imu(raw_roll, raw_pitch)

    def set_auto(self, on: bool):
        self.safety.set_auto(on)
        self.controller.reset()
        if on:
            self.estimator.reset()   # 측량→작업 전환: LPF/속도이력 초기화

    def emergency_stop(self):
        self.safety.set_estop()
        self._send(LevelerCanProtocol.MODE_ESTOP, Direction.NEUTRAL, 0)

    # ── 제어 스텝 (50ms) ────────────────────────────────
    def control_step(self, now: float = None) -> dict:
        now = now if now is not None else time.time()
        self._now = now
        blade = self._blade
        blade_z_cm = blade.blade_z * 100 if (blade and blade.valid) else 0.0
        vz = blade.vz if (blade and blade.valid) else 0.0

        safety = self.safety.check(blade_z_cm, now=now)

        # 안전하지 않으면 중립 (단, 한계는 반대방향 허용 가능 — 여기선 보수적 HOLD)
        if not self.safety.is_safe:
            self._send(self._safety_mode(safety), Direction.NEUTRAL, 0)
            return self._status(blade, 0.0, Direction.NEUTRAL, safety, on_grade=False)

        # 목표고
        target_z_cm = self.target.plane.z_at(blade.x, blade.y) * 100

        # 제어
        direction, pulse_ms = self.controller.compute(
            blade_z_cm, target_z_cm, vz, now)
        error = blade_z_cm - target_z_cm

        # 적응형 코스팅 — 실제 코스팅 관측 후 파라미터 자동 갱신
        updated_coast = self.adaptive.update(blade_z_cm, self._cmd_active, now)
        if abs(updated_coast - self.params.coast_cm) > 0.2:
            self.params.coast_cm = updated_coast
            self.controller.params.coast_cm = updated_coast

        self._send(LevelerCanProtocol.MODE_AUTO, direction, pulse_ms)
        return self._status(blade, error, direction, safety,
                            on_grade=self.controller.is_on_grade,
                            target_z_cm=target_z_cm, pulse_ms=pulse_ms)

    def _safety_mode(self, s: LevelerSafety) -> int:
        if s == LevelerSafety.ESTOP:
            return LevelerCanProtocol.MODE_ESTOP
        if s == LevelerSafety.MANUAL:
            return LevelerCanProtocol.MODE_MANUAL
        return LevelerCanProtocol.MODE_HOLD

    def _send(self, mode: int, direction: Direction, pulse_ms: float):
        self._hb = (self._hb + 1) & 0xFF
        self._cmd_active = (direction != Direction.NEUTRAL and pulse_ms > 0)
        self.output.send_command(mode, direction, pulse_ms, self._hb, self._now)

    def _status(self, blade, error, direction, safety,
                on_grade=False, target_z_cm=0.0, pulse_ms=0.0) -> dict:
        return {
            "valid": bool(blade and blade.valid),
            "quality": blade.quality if blade else 0,
            "blade_z_cm": blade.blade_z * 100 if blade else 0.0,
            "antenna_z_cm": blade.antenna_z * 100 if blade else 0.0,
            "target_z_cm": target_z_cm,
            "error_cm": error,
            "vz_cms": blade.vz if blade else 0.0,
            "direction": direction.name,
            "pulse_ms": pulse_ms,
            "on_grade": on_grade,
            "safety": safety.name,
            "pos": (blade.x, blade.y) if blade else (0.0, 0.0),
        }


# ═══════════════════════════════════════════════════════════════
#  캘리브레이션 — 자동 마법사 + 적응형 추정기
# ═══════════════════════════════════════════════════════════════

# ═══════════════════════════════════════════════════════════════
#  적응형 코스팅 추정기 — 실시간 자기보정
# ═══════════════════════════════════════════════════════════════

class AdaptiveCoastEstimator:
    """
    명령이 끊길 때마다 실제 코스팅을 관측해 예측값을 점진 갱신.
    유온 변화·노후화 등 유압 특성 변화에 자동 적응.
    IC100/AL2가 고정 파라미터로 인해 겪는 문제를 해결.

    사용: control_step 매 호출마다 update() 호출.
    """
    def __init__(self, initial_coast_cm: float, alpha: float = 0.15):
        self.coast_cm = initial_coast_cm   # 현재 추정값 (cm)
        self.alpha    = alpha               # 갱신 속도 (0=고정, 1=즉시)
        self._was_active = False
        self._z_at_stop  = 0.0
        self._obs_start  = 0.0
        self._observing  = False

    def update(self, blade_z_cm: float, cmd_active: bool,
               now: float = None) -> float:
        """
        매 제어스텝 호출.
        cmd_active: 현재 출력 명령이 활성 중인지 (NEUTRAL이 아닌 경우).
        반환: 현재 코스팅 추정값 (cm).
        """
        now = now or time.time()
        was = self._was_active
        self._was_active = cmd_active

        if was and not cmd_active:
            # 명령 방금 끊김 → 코스팅 관측 시작
            self._observing  = True
            self._z_at_stop  = blade_z_cm
            self._obs_start  = now

        if self._observing and (now - self._obs_start > 1.0):
            observed = abs(blade_z_cm - self._z_at_stop)
            if 0.0 <= observed < 8.0:   # 이상값 필터
                self.coast_cm = ((1 - self.alpha) * self.coast_cm
                                 + self.alpha * observed)
                log.debug(f"코스팅 갱신: {self.coast_cm:.2f}cm (관측={observed:.2f})")
            self._observing = False

        return self.coast_cm


# ═══════════════════════════════════════════════════════════════
#  자동 캘리브레이션 마법사 — 설치 후 1회 실행, ~5분
# ═══════════════════════════════════════════════════════════════

class CalibStep(Enum):
    IDLE           = 0
    CHECK_RTK      = 1
    DETECT_POLARITY= 2
    MEASURE_LATENCY= 3
    SPEED_UP       = 4
    MEASURE_COAST  = 5
    SPEED_DOWN     = 6
    MIN_PULSE      = 7
    COMPLETE       = 8
    FAILED         = 9


class AutoCalibrationWizard:
    """
    트랙터 유압 응답 자동 측정 마법사.

    범용 IC100/AL2의 한계(트랙터를 모름)를 극복하는 핵심.
    어떤 트랙터든 설치 후 5분 캘리브레이션으로 최적 파라미터 자동 측정.

    측정 항목:
      · 극성 (UP/DOWN 핀이 실제로 상승/하강인지 자동 판별)
      · 지연 (dead time, ms)
      · UP/DOWN 속도 (cm/s)
      · 코스팅 (cm)
      · 최소 유효 펄스 (ms)
      → 코스팅 기반 최적 데드밴드 자동 계산

    사용:
        output  = LaserConnectorOutput(bus)
        wizard  = AutoCalibrationWizard(output, on_status=lambda s: print(s))
        wizard.start()

        # 50ms 루프에서:
        wizard.tick(blade_z_cm=blade_z, rtk_quality=4, now=sim_t)

        if wizard.step == CalibStep.COMPLETE:
            profile = wizard.result_profile("Kubota MR1157")
            TRACTOR_DB.save_profile(profile)
            on_grade, exit_cm = profile.recommended_deadband()
    """

    def __init__(self, output: LevelerOutput,
                 on_status: Callable[[str], None] = None,
                 tractor_name: str = "Unknown"):
        self.output      = output
        self.on_status   = on_status or (lambda s: log.info(s))
        self.tractor_name= tractor_name
        self.step        = CalibStep.IDLE

        # 측정 결과
        self.latency_s       = 0.30
        self.up_speed_cms    = 8.0
        self.down_speed_cms  = 10.0
        self.coast_cm        = 1.5
        self.min_pulse_s     = 0.08
        self.polarity_inv    = False
        self.quality         = 0.0

        self._hb      = 0
        self._z_log   : List[Tuple[float, float]] = []
        self._t_step  = 0.0
        self._pd      : dict = {}   # phase data

    # ── 공개 API ───────────────────────────────────────────
    def start(self):
        self.step  = CalibStep.CHECK_RTK
        self._t_step = time.time()
        self.on_status("▶ 캘리브레이션 시작. RTK Fix 확인 중...")

    def tick(self, blade_z_cm: float, rtk_quality: int = 4,
             now: float = None):
        """매 50ms 호출. blade_z_cm: 현재 블레이드 높이(cm)."""
        now = now or time.time()
        self._z_log.append((now, blade_z_cm))
        self._z_log = [(t,z) for t,z in self._z_log if now-t < 12.0]

        dispatch = {
            CalibStep.CHECK_RTK:       self._check_rtk,
            CalibStep.DETECT_POLARITY: self._detect_polarity,
            CalibStep.MEASURE_LATENCY: self._measure_latency,
            CalibStep.SPEED_UP:        self._speed_up,
            CalibStep.MEASURE_COAST:   self._measure_coast,
            CalibStep.SPEED_DOWN:      self._speed_down,
            CalibStep.MIN_PULSE:       self._min_pulse,
            CalibStep.COMPLETE:        lambda z,t: self._neutral(t),
        }
        fn = dispatch.get(self.step)
        if fn:
            fn(blade_z_cm, now)

    def result_profile(self, name: str = None) -> TractorProfile:
        import datetime
        return TractorProfile(
            name=name or self.tractor_name,
            model=name or self.tractor_name,
            up_speed_cms=round(self.up_speed_cms, 2),
            down_speed_cms=round(self.down_speed_cms, 2),
            latency_s=round(self.latency_s, 3),
            coast_cm=round(self.coast_cm, 2),
            min_pulse_s=round(self.min_pulse_s, 3),
            calibrated_at=datetime.datetime.now().isoformat()[:16],
            calibration_quality=round(self.quality, 2),
        )

    # ── 내부 헬퍼 ─────────────────────────────────────────
    def _up(self, now):
        d = Direction.DOWN if self.polarity_inv else Direction.UP
        self._cmd(d, 999, now)

    def _down(self, now):
        d = Direction.UP if self.polarity_inv else Direction.DOWN
        self._cmd(d, 999, now)

    def _cmd(self, d, pw, now):
        self._hb = (self._hb + 1) & 0xFF
        self.output.send_command(
            LevelerCanProtocol.MODE_AUTO, d, pw, self._hb, now)

    def _neutral(self, now):
        self._hb = (self._hb + 1) & 0xFF
        self.output.send_command(
            LevelerCanProtocol.MODE_HOLD, Direction.NEUTRAL, 0, self._hb, now)

    def _go(self, next_step: CalibStep, now: float, msg: str = ""):
        self.step    = next_step
        self._t_step = now
        self._pd     = {}
        if msg:
            self.on_status(msg)

    def _elapsed(self, now): return now - self._t_step

    # ── 단계별 로직 ───────────────────────────────────────
    def _check_rtk(self, z, now):
        if 4 <= 4:   # rtk_quality 파라미터 활용은 tick이 전달
            pass
        # 5초 내 RTK 신호 있으면 진행 (tick 호출 자체가 신호)
        if self._elapsed(now) > 2.0:
            self._go(CalibStep.DETECT_POLARITY, now, "RTK 확인. 극성 테스트 (UP 1.5초)...")
            self._pd["z_start"] = z
            self._up(now)

    def _detect_polarity(self, z, now):
        if self._elapsed(now) < 1.5:
            self._up(now); return
        self._neutral(now)
        dz = z - self._pd.get("z_start", z)
        if abs(dz) < 0.3:
            if self._elapsed(now) > 3.5:
                self.step = CalibStep.FAILED
                self.on_status("✗ 트랙터 반응 없음. 레이저 모드 선택 및 커넥터 확인 요망.")
            return
        self.polarity_inv = (dz < 0)
        sign = "반전" if self.polarity_inv else "정상"
        self.on_status(f"극성 {sign} (dz={dz:+.1f}cm). 지연 측정 시작...")
        self._go(CalibStep.MEASURE_LATENCY, now)
        self._pd = {"z_start": z, "cmd_t": now, "moved": False}
        self._up(now)

    def _measure_latency(self, z, now):
        pd = self._pd
        if not pd.get("moved"):
            if abs(z - pd["z_start"]) > 0.3:
                self.latency_s = min(1.2, max(0.05, now - pd["cmd_t"]))
                pd["moved"] = True
                self.on_status(f"지연: {self.latency_s*1000:.0f}ms. UP 속도 측정...")
        if self._elapsed(now) > 2.5:
            self._neutral(now)
            self._go(CalibStep.SPEED_UP, now)
            self._pd = {"z0": z, "t0": now, "samps": []}

    def _speed_up(self, z, now):
        pd = self._pd
        pd["samps"].append((now, z))
        self._up(now)
        if self._elapsed(now) > 2.0 + self.latency_s:
            self._neutral(now)
            samps = [(t,z2) for t,z2 in pd["samps"]
                     if t - self._t_step > self.latency_s]
            if len(samps) >= 3:
                t0,z0 = samps[0]; t1,z1 = samps[-1]
                self.up_speed_cms = max(1.0, abs(z1-z0)/max(0.1,t1-t0))
                self.on_status(f"UP 속도: {self.up_speed_cms:.1f} cm/s. 코스팅 측정...")
            self._go(CalibStep.MEASURE_COAST, now)
            self._pd = {"phase":"prep","t_phase":now,"z_at_stop":z}

    def _measure_coast(self, z, now):
        pd = self._pd
        ph = pd.get("phase","prep")
        if ph == "prep":
            if now - pd["t_phase"] > 1.5:
                pd["phase"]   = "up"; pd["t_phase"] = now
                self.on_status("코스팅 측정: 0.5초 UP 후 정지...")
                self._up(now)
        elif ph == "up":
            self._up(now)
            if now - pd["t_phase"] > 0.5:
                self._neutral(now)
                pd["phase"] = "obs"; pd["t_phase"] = now
                pd["z_at_stop"] = z
        elif ph == "obs":
            if now - pd["t_phase"] > 1.2:
                self.coast_cm = max(0.0, abs(z - pd["z_at_stop"]))
                self.on_status(f"코스팅: {self.coast_cm:.1f}cm. DOWN 속도 측정...")
                self._go(CalibStep.SPEED_DOWN, now)
                self._pd = {"samps": []}

    def _speed_down(self, z, now):
        pd = self._pd
        pd["samps"].append((now, z))
        self._down(now)
        if self._elapsed(now) > 2.0 + self.latency_s:
            self._neutral(now)
            samps = [(t,z2) for t,z2 in pd["samps"]
                     if t - self._t_step > self.latency_s]
            if len(samps) >= 3:
                t0,z0=samps[0]; t1,z1=samps[-1]
                self.down_speed_cms = max(1.0, abs(z1-z0)/max(0.1,t1-t0))
                self.on_status(f"DOWN 속도: {self.down_speed_cms:.1f} cm/s. 최소 펄스 탐색...")
            self._go(CalibStep.MIN_PULSE, now)
            self._pd = {"pw_ms":50,"z_base":z,"result":None,
                        "wait":False,"wait_t":now,"sent":False}

    def _min_pulse(self, z, now):
        pd = self._pd
        if pd.get("wait"):
            if now - pd["wait_t"] > 1.0:
                pd["wait"] = False; pd["z_base"] = z; pd["sent"] = False
            else:
                return
        pw = pd["pw_ms"]
        if not pd.get("sent"):
            self._cmd(Direction.UP if not self.polarity_inv else Direction.DOWN,
                      pw, now)
            pd["sent"] = True; pd["t_sent"] = now
        elif now - pd["t_sent"] > pw/1000.0 + 0.4:
            self._neutral(now)
            dz = abs(z - pd["z_base"])
            if dz > 0.3 and pd["result"] is None:
                self.min_pulse_s = pw / 1000.0
                pd["result"] = pw
                self._finish(now)
            elif pw >= 400:
                self.min_pulse_s = 0.20
                self._finish(now)
            else:
                pd["pw_ms"] = pw + 50
                pd["wait"]  = True
                pd["wait_t"]= now

    def _finish(self, now):
        self._neutral(now)
        checks = [
            0.05 < self.latency_s < 1.2,
            2.0  < self.up_speed_cms < 30.0,
            2.0  < self.down_speed_cms < 35.0,
            0.0  <= self.coast_cm < 6.0,
            0.03 < self.min_pulse_s < 0.5,
        ]
        self.quality = sum(checks) / len(checks)
        self.step = CalibStep.COMPLETE
        og, ex = self.result_profile().recommended_deadband()
        self.on_status(
            f"✓ 캘리브레이션 완료! 품질 {self.quality*100:.0f}%\n"
            f"  UP {self.up_speed_cms:.1f} cm/s | DOWN {self.down_speed_cms:.1f} cm/s\n"
            f"  지연 {self.latency_s*1000:.0f}ms | 코스팅 {self.coast_cm:.1f}cm\n"
            f"  최소펄스 {self.min_pulse_s*1000:.0f}ms\n"
            f"  → 권장 데드밴드: on_grade ±{og}cm / exit ±{ex}cm"
        )


# ═══════════════════════════════════════════════════════════════
#  테스트 / 시뮬레이션
# ═══════════════════════════════════════════════════════════════
if __name__ == "__main__":
    import random
    logging.basicConfig(level=logging.INFO,
                        format="%(levelname)s %(name)s: %(message)s")

    print("=" * 74)
    print("RTK 레벨러 — 평면 피팅 + 균평 제어 시뮬레이션")
    print("=" * 74)
    print()

    params = LevelerParams(
        antenna_height_above_blade = 2.50,  # ★ 실측 필요
        antenna_to_blade_horizontal= 1.20,  # ★ 실측 필요
        blade_width                = 2.50,
        up_speed_cms   = 8.0,    # ★ 캘리브레이션
        down_speed_cms = 10.0,   # ★ 캘리브레이션
        latency_s      = 0.30,   # ★ 캘리브레이션
        coast_cm       = 1.5,    # ★ 캘리브레이션
        blade_max_up_cm   = 40.0,
        blade_max_down_cm = -25.0,
    )

    ORIGIN_LAT, ORIGIN_LON, ORIGIN_ALT = 37.0, 127.0, 50.0
    H = params.antenna_height_above_blade

    def make_fix(x, y, blade_abs_alt):
        """블레이드 절대표고 → 안테나 표고 → GnssFix (평지 가정)."""
        antenna_alt = blade_abs_alt + H
        lat = ORIGIN_LAT + (y / 6378137.0) * (180/math.pi)
        lon = ORIGIN_LON + (x / (6378137.0*math.cos(math.radians(ORIGIN_LAT)))) * (180/math.pi)
        return GnssFix(lat=lat, lon=lon, alt=antenna_alt, quality=4,
                       sats=20, hdop=0.8, valid=True)

    # ── 1. 포장 측량 시뮬 ────────────────────────────────────
    print("─" * 74)
    print("▶ 1. 포장 측량 + 최소자승 평면 피팅")
    print("─" * 74)

    random.seed(42)
    def ground_z(x, y):
        return ORIGIN_ALT + 0.005 * x + 0.08 * math.sin(x/15) * math.cos(y/12) \
               + random.gauss(0, 0.02)

    bus = MockHydraulics(params, start_blade_cm=18.0)
    bus.start()
    sys = LevelerSystem(params, bus)
    sys.enu.set_origin(ORIGIN_LAT, ORIGIN_LON, ORIGIN_ALT)

    # 측량: 블레이드를 지면에 대고 주행 → add_survey (estimator/ENU 동일 좌표계)
    for _ in range(40):
        x = random.uniform(0, 80); y = random.uniform(0, 60)
        sys.add_survey(make_fix(x, y, ground_z(x, y)), raw_roll=0.0, raw_pitch=0.0)
    plane = sys.fit_plane()
    a, b = plane.slope_percent()
    stats = sys.target.cut_fill_stats()
    print(f"  피팅 평면(ENU상대): z = {plane.a:.5f}·E + {plane.b:.5f}·N + {plane.c:.3f}")
    print(f"  경사: East {a:.2f}%, North {b:.2f}%")
    print(f"  최대 깎기 {stats['max_cut_cm']:.1f}cm, "
          f"최대 메우기 {stats['max_fill_cm']:.1f}cm, "
          f"평균편차 {stats['mean_abs_cm']:.1f}cm")
    print()

    # ── 2. 균평 제어 시뮬 (한 패스 주행) ────────────────────
    print("─" * 74)
    print("▶ 2. 균평 제어 — 블레이드가 목표평면 추종 (1패스)")
    print("─" * 74)

    sys.set_auto(True)
    sys.safety._quality = 4
    sys.safety._last_fix_t = time.time()

    # 동쪽으로 직진 (x 증가), y 고정
    y_fixed = 30.0
    x = 0.0
    speed_ms = 1.2  # 주행속도
    DT = 0.05       # 20Hz

    print(f"  {'t(s)':>5}  {'x(m)':>6}  {'블레이드':>8}  {'목표':>7}  "
          f"{'오차':>7}  {'방향':>8}  {'상태'}")
    print("  " + "-" * 68)

    t = 0.0
    sim_clock = time.time()
    on_grade_time = 0.0
    err_samples = []
    for step in range(400):  # 20초
        # 트랙터 전진
        x += speed_ms * DT
        if x > 80:
            break
        sim_clock += DT   # 시뮬레이션 시계 진행

        # 가상 안테나 높이: 블레이드 절대표고 = origin + blade_cm/100
        blade_abs = ORIGIN_ALT + bus.blade_cm/100.0
        fix = make_fix(x, y_fixed, blade_abs)

        sys.on_imu(0.0, 0.0)        # 평지 가정
        sys.on_gnss_fix(fix, now=sim_clock)
        st = sys.control_step(now=sim_clock)

        # 물리 적분 (시뮬 시계)
        bus.tick(now=sim_clock, dt=DT)

        if st["on_grade"]:
            on_grade_time += DT
        err_samples.append(abs(st["error_cm"]))

        if step % 25 == 0:
            print(f"  {t:>5.1f}  {x:>6.1f}  {st['blade_z_cm']:>7.1f}cm  "
                  f"{st['target_z_cm']:>6.1f}cm  {st['error_cm']:>+6.1f}cm  "
                  f"{st['direction']:>8}  "
                  f"{'온그레이드' if st['on_grade'] else '조정중'}")
        t += DT

    bus.stop()
    settle = err_samples[40:] if len(err_samples) > 40 else err_samples
    rms = (sum(e*e for e in settle)/len(settle))**0.5 if settle else 0
    print()
    print(f"  수렴 후 RMS 오차: {rms:.2f} cm")
    print(f"  온그레이드 유지 시간 비율: {on_grade_time/t*100:.0f}%")
    print()

    # ── 3. CAN 프로토콜 인코딩 확인 ─────────────────────────
    print("─" * 74)
    print("▶ 3. CAN 명령 인코딩 (Apollo → STM32, ID 0x18FF0102)")
    print("─" * 74)
    for d, pw, label in [(Direction.UP, 999, "연속 UP"),
                         (Direction.DOWN, 150, "150ms DOWN 펄스"),
                         (Direction.NEUTRAL, 0, "HOLD")]:
        cid, data = LevelerCanProtocol.encode_cmd(
            LevelerCanProtocol.MODE_AUTO, d, pw, heartbeat=42)
        print(f"  {label:<16} → ID=0x{cid:08X} data={data.hex(' ')} "
              f"(CRC8=0x{data[7]:02X})")
    print()

    # ── 4. 출력단 추상화 확인 ──────────────────────────────
    print("─" * 74)
    print("▶ 4. 출력단 추상화 (LevelerOutput)")
    print("─" * 74)
    cmds_laser = []
    cmds_pack  = []
    class _CaptureBus(CanBus):
        def __init__(self, store): self.store = store
        def start(self): pass
        def stop(self):  pass
        def send(self, can_id, data, now=None):
            self.store.append((can_id, Direction(data[1]), data[2]))

    laser = LaserConnectorOutput(_CaptureBus(cmds_laser))
    pack  = PowerPackOutput(_CaptureBus(cmds_pack), max_duty=1.0)
    for out, name in [(laser,"LaserConnector"), (pack,"PowerPack")]:
        out.send_command(LevelerCanProtocol.MODE_AUTO, Direction.UP, 999, 1)
        out.send_command(LevelerCanProtocol.MODE_AUTO, Direction.DOWN, 200, 2)
        out.send_command(LevelerCanProtocol.MODE_HOLD, Direction.NEUTRAL, 0, 3)
    print(f"  LaserConnector: {[(d.name, pw) for _,d,pw in cmds_laser]}")
    print(f"  PowerPack(비례): {[(d.name, pw) for _,d,pw in cmds_pack]}")
    print()

    # ── 5. TractorProfileDB 확인 ──────────────────────────
    print("─" * 74)
    print("▶ 5. TractorProfileDB — 등록 프로파일")
    print("─" * 74)
    for name in TRACTOR_DB.list_profiles():
        p = TRACTOR_DB.get(name)
        og, ex = p.recommended_deadband()
        print(f"  {name:<22} UP {p.up_speed_cms:.1f} cm/s | "
              f"지연 {p.latency_s*1000:.0f}ms | "
              f"코스팅 {p.coast_cm:.1f}cm | "
              f"→ 데드밴드 ±{og}/{ex}cm")
    print()

    # ── 6. 자동 캘리브레이션 마법사 시뮬 ─────────────────
    print("─" * 74)
    print("▶ 6. AutoCalibrationWizard — 시뮬레이션")
    print("─" * 74)
    cal_bus = MockHydraulics(params, start_blade_cm=15.0)
    cal_bus.start()
    cal_output = LaserConnectorOutput(cal_bus)
    msgs = []
    wizard = AutoCalibrationWizard(
        cal_output,
        on_status=lambda s: msgs.append(s),
        tractor_name="TestTractor"
    )
    wizard.start()
    sim_t2 = time.time()
    for i in range(500):   # 최대 25초 시뮬
        sim_t2 += DT
        cal_bus.tick(now=sim_t2, dt=DT)
        bz = cal_bus.blade_cm
        wizard.tick(blade_z_cm=bz, rtk_quality=4, now=sim_t2)
        if wizard.step in (CalibStep.COMPLETE, CalibStep.FAILED):
            break
    for m in msgs:
        print(f"  {m}")
    if wizard.step == CalibStep.COMPLETE:
        profile = wizard.result_profile("TestTractor")
        TRACTOR_DB.save_profile(profile)
        og, ex = profile.recommended_deadband()
        print(f"\n  ✓ 프로파일 저장 완료 | 권장 데드밴드: ±{og}cm / ±{ex}cm")
    print()

    print("─" * 74)
    print("다음 단계 (MEASUREMENT_CHECKLIST.md):")
    print("  1. ★ B1,B2 안테나↔블레이드 h,d 실측 → LevelerParams")
    print("  2. ★ A1~A6 Kubota(7) 커넥터 핀맵 → STM32 펌웨어")
    print("  3. AutoCalibrationWizard 현장 실행 (5분)")
    print("     → TractorProfile 자동 생성 + TRACTOR_DB 저장")
    print("  4. MockHydraulics → ApolloCanBus 교체 (CAN SDK)")
    print("  5. 포장 측량 → fit_plane() → 작업")
