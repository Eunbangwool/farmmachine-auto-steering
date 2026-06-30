"""
z_stabilizer.py — VRS(네트워크 RTK) Z축 동적오차 방어 안정화 계층 (add-on)
============================================================================
leveler_core.py 미수정 add-on. 원시 GNSS(NMEA) + IMU 를 받아 **블레이드 Z(고도)**
를 1~2cm 급으로 안정화한 추정값과 유압제어 게이트를 산출한다. 기존 BladeEstimator
(틸트 삼각보정 blade_tip_z)·proportional_valve(칼만/데드밴드)와 중복 없이 그 위에 얹는다.

방어 대상(VRS 고질 약점): 기지국 거리가 멀어질 때 ① Z축 오차 증대 ② Fix→Float 풀림.
하드웨어 추가 없이 SW 필터/퓨전만으로 방어.

4대 메커니즘:
  1) 위성 고도각 마스크(15°) + Multi-GNSS(BeiDou/Galileo) 가용위성 감시 → 적응형 측정분산 R_z
     (GSV 고도각·구성위성수, GST 고도표준편차, GGA quality/HDOP 융합).
  2) GNSS+IMU EKF(3상태 z,vz,accel_bias) 수직퓨전 + 틸트 삼각보정 + RTK Bridge 데드레코닝
     (Float/끊김 시 IMU 로 10~20초 cm급 유지).
  3) 디젤 고주파 진동 vs 지면 저주파 분리 — 2차 Butterworth LPF(바이쿼드) 대역분리.
  4) 유압 밸브 데드밴드/게인 댐핑 — 품질 적응형(품질 저하 시 데드밴드 확대·게인 감쇠).

페일세이프 FSM: TRACK(Fix) → BRIDGE(Float/끊김, IMU DR) → HOLD(밸브 정지) → STOP(중립/estop).

★출처/근거: 기존 EKF(autosteer_core 적응형R·게이팅)·CHCNAV §8(칼만+MA, 데드존) 철학 계승.
  실차 의존값(IMU 부호/축, kalman_r, min_speed, 진동 컷오프)은 ★현장 보정.

의존성: 표준 라이브러리 + leveler_core(LevelerParams.blade_tip_z 재사용). numpy 불필요.
    cd rtk-leveling/src && python z_stabilizer.py
"""
from __future__ import annotations

import math
import time
import logging
from collections import deque
from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, List, Optional, Tuple

from leveler_core import GnssFix, NMEAParser, LevelerParams

log = logging.getLogger("leveler.zstab")

G0 = 9.80665   # 표준중력 (m/s²)


# ═══════════════════════════════════════════════════════════════
#  메커니즘 1 — 고도각 마스크 + Multi-GNSS 가용위성 품질 감시
# ═══════════════════════════════════════════════════════════════
# GSV talker → 구성위성. RTCM3.2/MSM5 마운트로 BeiDou/Galileo 대거 확보 시
# 고도각 15° 상향에도 위성수 결핍을 상쇄(요구 1). 여기선 "감시 + 적응형 R" 만 담당
# (수신기 추적 마스크 자체는 수신기 설정 — Configuration Guide 참조).
_TALKER_CONST = {"GP": "GPS", "GL": "GLONASS", "GA": "Galileo",
                 "GB": "BeiDou", "BD": "BeiDou", "GQ": "QZSS", "GN": "Mixed"}


@dataclass
class GnssQuality:
    quality: int = 0            # GGA: 4=Fix,5=Float,…
    sats_used: int = 0          # GGA 사용위성
    hdop: float = 99.9
    alt_std_m: float = 0.0      # GST 고도 표준편차(있으면) — 적응형 R 의 1순위 근거
    sats_above_mask: int = 0    # GSV 고도각≥마스크 위성수 합
    by_const: Dict[str, int] = field(default_factory=dict)  # 구성별 마스크통과 수
    age_s: float = 99.9         # 마지막 유효 GGA 후 경과


class GnssQualityMonitor:
    """NMEA(GGA/GSV/GST) 스트림 → 고도각 마스크 적용 가용위성 + 적응형 측정분산 R_z."""
    def __init__(self, elevation_mask_deg: float = 15.0,
                 sigma_fix_m: float = 0.012, sigma_float_m: float = 0.25):
        self.mask = elevation_mask_deg
        self.sigma_fix = sigma_fix_m       # Fix 시 기본 고도 σ (1~2cm)
        self.sigma_float = sigma_float_m   # Float 시 기본 고도 σ
        self._q = GnssQuality()
        self._gsv_acc: Dict[str, Dict[int, float]] = {}  # const → {prn: elev}
        self._last_gga_t = 0.0

    # ── NMEA 입력 ─────────────────────────────────────────────
    def feed(self, line: str, now: float = None) -> Optional[GnssFix]:
        now = now if now is not None else time.time()
        if "GGA" in line:
            fix = NMEAParser.parse_gga(line)
            if fix:
                self._q.quality = fix.quality
                self._q.sats_used = fix.sats
                self._q.hdop = fix.hdop
                if fix.valid:
                    self._last_gga_t = now
            return fix
        if "GSV" in line:
            self._parse_gsv(line)
        elif "GST" in line:
            self._parse_gst(line)
        return None

    def _parse_gsv(self, line: str):
        try:
            body = line.split("*")[0]
            f = body.split(",")
            talker = f[0][1:3]
            const = _TALKER_CONST.get(talker, "GPS")
            msgnum = int(f[2]) if f[2] else 1
            if msgnum == 1:
                self._gsv_acc[const] = {}        # 첫 메시지에서 누적 초기화
            acc = self._gsv_acc.setdefault(const, {})
            # 위성 블록 4개: prn,elev,azim,snr
            for i in range(4, len(f) - 3, 4):
                prn = f[i]
                elev = f[i + 1]
                if prn and elev:
                    acc[int(prn)] = float(elev)
        except (ValueError, IndexError):
            pass

    def _parse_gst(self, line: str):
        # $..GST,time,rms,stdMajor,stdMinor,orient,stdLat,stdLon,stdAlt*cks
        try:
            f = line.split("*")[0].split(",")
            if f[8]:
                self._q.alt_std_m = float(f[8])
        except (ValueError, IndexError):
            pass

    # ── 품질 산출 ─────────────────────────────────────────────
    def snapshot(self, now: float = None) -> GnssQuality:
        now = now if now is not None else time.time()
        by_const, total = {}, 0
        for const, sats in self._gsv_acc.items():
            n = sum(1 for e in sats.values() if e >= self.mask)
            if n:
                by_const[const] = n
                total += n
        self._q.by_const = by_const
        self._q.sats_above_mask = total
        self._q.age_s = now - self._last_gga_t if self._last_gga_t else 99.9
        return self._q

    def sigma_z(self, now: float = None) -> float:
        """적응형 측정분산용 고도 표준편차(m). GST > 기하/품질 휴리스틱 순."""
        q = self.snapshot(now)
        # 1순위: GST 고도 표준편차(수신기 내부 추정) — 가장 신뢰
        if q.alt_std_m > 0:
            base = q.alt_std_m
        else:
            base = self.sigma_fix if q.quality == 4 else \
                   self.sigma_float if q.quality == 5 else 5.0
        # Float 면 가중, HDOP·마스크통과위성 반영(멀어질수록 σ 증가 방어)
        if q.quality == 5:
            base = max(base, self.sigma_float)
        dop_factor = max(1.0, q.hdop / 1.0)               # HDOP 1.0 기준
        sat_factor = 1.0 if q.sats_above_mask >= 30 else \
                     1.0 + (30 - max(q.sats_above_mask, 6)) * 0.05
        return base * dop_factor * sat_factor

    @property
    def is_fixed(self) -> bool:
        return self._q.quality == 4

    @property
    def is_float(self) -> bool:
        return self._q.quality == 5


# ═══════════════════════════════════════════════════════════════
#  메커니즘 3 — 2차 Butterworth LPF(바이쿼드): 디젤 고주파 vs 지면 저주파 분리
# ═══════════════════════════════════════════════════════════════
class Biquad:
    """2차 IIR 저역통과(Butterworth, Q=1/√2). bilinear 변환 계수. numpy 불필요."""
    def __init__(self, fc_hz: float, fs_hz: float):
        self.design(fc_hz, fs_hz)
        self._x1 = self._x2 = self._y1 = self._y2 = 0.0
        self._init = False

    def design(self, fc_hz: float, fs_hz: float):
        fc = max(1e-3, min(fc_hz, fs_hz * 0.49))
        w0 = 2 * math.pi * fc / fs_hz
        cw, sw = math.cos(w0), math.sin(w0)
        Q = 1 / math.sqrt(2)
        alpha = sw / (2 * Q)
        b0 = (1 - cw) / 2; b1 = 1 - cw; b2 = (1 - cw) / 2
        a0 = 1 + alpha;    a1 = -2 * cw; a2 = 1 - alpha
        self.b0, self.b1, self.b2 = b0 / a0, b1 / a0, b2 / a0
        self.a1, self.a2 = a1 / a0, a2 / a0

    def reset(self, x0: float = 0.0):
        self._x1 = self._x2 = x0
        self._y1 = self._y2 = x0
        self._init = True

    def step(self, x: float) -> float:
        if not self._init:
            self.reset(x)
        y = (self.b0 * x + self.b1 * self._x1 + self.b2 * self._x2
             - self.a1 * self._y1 - self.a2 * self._y2)
        self._x2, self._x1 = self._x1, x
        self._y2, self._y1 = self._y1, y
        return y


# ═══════════════════════════════════════════════════════════════
#  메커니즘 2 — 수직 EKF (z, vz, accel_bias) + IMU 데드레코닝
# ═══════════════════════════════════════════════════════════════
def _mat3_mul(A, B):
    return [[sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)]
            for i in range(3)]

def _mat3_T(A):
    return [[A[j][i] for j in range(3)] for i in range(3)]

def _mat3_add(A, B):
    return [[A[i][j] + B[i][j] for j in range(3)] for i in range(3)]


class VerticalEKF:
    """
    1차원 수직 운동 EKF. 상태 x=[z(m), vz(m/s), accel_bias(m/s²)].
    예측: IMU 수직가속도(중력·바이어스 보상) 적분 → 끊김 시 데드레코닝.
    갱신: 틸트보정 GNSS 고도(스칼라), R=적응형(품질 모니터).
    """
    def __init__(self, q_z=5e-7, q_v=1e-6, q_b=1e-9):
        # q_v ← 고정밀 IMU 수직가속도 잡음 (σ_a≈1mg → q_v≈(0.01·dt)²급). ★현장 보정.
        self.x = [0.0, 0.0, 0.0]
        self.P = [[1.0, 0, 0], [0, 1.0, 0], [0, 0, 0.1]]
        self.Q = [[q_z, 0, 0], [0, q_v, 0], [0, 0, q_b]]
        self._init = False

    def init_z(self, z0: float):
        self.x = [z0, 0.0, 0.0]
        self.P = [[self.Q[0][0], 0, 0], [0, 0.04, 0], [0, 0, 0.01]]  # vz σ=0.2m/s 초기
        self._init = True

    def predict(self, a_up: float, dt: float):
        if not self._init or dt <= 0:
            return
        z, vz, ba = self.x
        a = a_up - ba                       # 바이어스 보상 수직가속도
        self.x = [z + vz * dt + 0.5 * a * dt * dt, vz + a * dt, ba]
        F = [[1, dt, -0.5 * dt * dt],
             [0, 1, -dt],
             [0, 0, 1]]
        self.P = _mat3_add(_mat3_mul(_mat3_mul(F, self.P), _mat3_T(F)), self.Q)

    def update_z(self, z_meas: float, sigma_z: float):
        """스칼라 측정 갱신 (H=[1,0,0]). 역행렬 불필요."""
        if not self._init:
            self.init_z(z_meas); return
        R = max(1e-6, sigma_z * sigma_z)
        S = self.P[0][0] + R
        K = [self.P[i][0] / S for i in range(3)]     # 칼만이득(열0)
        y = z_meas - self.x[0]
        for i in range(3):
            self.x[i] += K[i] * y
        # P = (I - K H) P  (H=[1,0,0] → P[i][j] -= K[i]*P[0][j])
        P0 = list(self.P[0])
        self.P = [[self.P[i][j] - K[i] * P0[j] for j in range(3)] for i in range(3)]

    def update_vz(self, vz_meas: float, sigma_vz: float):
        """수직속도 의사측정 갱신(H=[0,1,0]). ZUPT: vz≈0 으로 브리지 드리프트 억제."""
        if not self._init:
            return
        R = max(1e-9, sigma_vz * sigma_vz)
        S = self.P[1][1] + R
        K = [self.P[i][1] / S for i in range(3)]
        y = vz_meas - self.x[1]
        for i in range(3):
            self.x[i] += K[i] * y
        P1 = list(self.P[1])
        self.P = [[self.P[i][j] - K[i] * P1[j] for j in range(3)] for i in range(3)]

    @property
    def z(self) -> float: return self.x[0]
    @property
    def vz(self) -> float: return self.x[1]
    @property
    def sigma_z_est(self) -> float: return math.sqrt(max(0.0, self.P[0][0]))


def imu_vertical_accel(ax: float, ay: float, az: float,
                       roll: float, pitch: float) -> float:
    """
    메커니즘 2 삼각함수 좌표매핑: 차체 가속도(ax전,ay우,az상) + roll/pitch
    → 월드 수직(Up) 가속도(중력 제거). 마스트 흔들림/지면기울기(≤30°) 보정.
    ★ 부호/축 규약은 실 IMU 로 검증(현재 ENU-ish: pitch>0 노즈업, roll>0 우down).
    """
    a_up = (az * math.cos(roll) * math.cos(pitch)
            + ax * math.sin(pitch)
            - ay * math.sin(roll) * math.cos(pitch))
    return a_up - G0


# ═══════════════════════════════════════════════════════════════
#  메커니즘 4 + 페일세이프 — Z 안정화 오케스트레이터
# ═══════════════════════════════════════════════════════════════
class ZState(Enum):
    INIT = 0
    TRACK = 1       # RTK Fix — 정상 추종
    BRIDGE = 2      # Float/끊김 — IMU 데드레코닝(밸브 계속, 데드밴드 확대)
    HOLD = 3        # 브리지 한계 초과 — 밸브 정지(블레이드 유지)
    STOP = 4        # 안전정지(중립/estop 권고)


@dataclass
class ZEstimate:
    blade_z_m: float
    vz_cms: float
    state: ZState
    control_enabled: bool       # 유압 제어 허용?
    deadband_cm: float          # 품질 적응형 데드밴드 권고
    gain_scale: float           # 품질 적응형 게인 감쇠(0~1)
    sigma_z_cm: float           # 현재 Z 추정 표준편차
    bridge_remaining_s: float   # 브리지 잔여 허용시간
    quality: GnssQuality


@dataclass
class ZStabilizerConfig:
    elevation_mask_deg: float = 15.0
    nmea_rate_hz: float = 1.0
    # 메커니즘 3 대역분리 — 두 컷오프:
    accel_lpf_hz: float = 2.5        # 가속도 입력: 디젤 고주파(>5Hz) 차단, 지면 응답대역 통과
    output_lpf_hz: float = 0.35      # 제어 출력: 1Hz GNSS 잡음 톱니 평활(밸브 요동 방지)
    ctrl_rate_hz: float = 20.0       # 제어 스텝(IMU/예측) 주기
    bridge_max_s: float = 15.0       # RTK Bridge 데드레코닝 최대 유지(10~20s)
    bridge_sigma_limit_cm: float = 5.0   # 브리지 중 σ_z 이 값 초과 → HOLD
    # ZUPT(수직 정지속도 보정): 필터링 수직가속도가 이 값 미만이면 vz≈0 의사측정 →
    #   1Hz 위치측정의 약한 vz 관측성을 보완, 브리지 드리프트를 IMU 바이어스 수준으로 억제.
    zupt_accel_thresh: float = 0.10  # m/s² (이하면 '수직 정지 중'으로 간주)
    zupt_sigma_vz: float = 0.02      # m/s — ZUPT 의사측정 표준편차(작을수록 강한 구속)
    base_deadband_cm: float = 1.0    # Fix 시 데드밴드(±)
    float_deadband_cm: float = 3.0   # Float/브리지 시 데드밴드 확대
    fix_required_to_resume: bool = True  # STOP 해제는 Fix 복귀 필요


class ZStabilizer:
    """
    on_gnss(NMEA) + on_imu(가속도/자이로/자세) 를 받아 step(dt)→ZEstimate.
    출력(blade_z, control_enabled, deadband, gain)을 기존 LevelingController/
    proportional_valve 가 소비한다(틸트보정은 LevelerParams.blade_tip_z 재사용).
    """
    def __init__(self, params: LevelerParams, cfg: ZStabilizerConfig = None,
                 enu_to_up=None):
        self.params = params
        self.cfg = cfg or ZStabilizerConfig()
        self.mon = GnssQualityMonitor(self.cfg.elevation_mask_deg)
        self.ekf = VerticalEKF()
        self.lpf = Biquad(self.cfg.output_lpf_hz, self.cfg.ctrl_rate_hz)       # 출력 평활(저주파)
        self.lpf_acc = Biquad(self.cfg.accel_lpf_hz, self.cfg.ctrl_rate_hz)    # 입력 진동제거
        self._enu_to_up = enu_to_up      # (lat,lon,alt)->up(m) 변환기(없으면 alt 사용)
        self.state = ZState.INIT
        self._roll = self._pitch = 0.0
        self._a_up = 0.0
        self._bridge_t0 = 0.0
        self._last_meas_t = 0.0
        self._z_lpf = None

    # ── 센서 입력 ─────────────────────────────────────────────
    def on_imu(self, ax, ay, az, roll, pitch, now: float = None):
        """차체 가속도(m/s²) + 자세(rad). EKF 예측 입력 + 틸트각 보관."""
        self._roll, self._pitch = float(roll), float(pitch)
        # 메커니즘 3: 디젤 고주파 진동을 예측 입력단에서 제거(지면 저주파만 통과)
        raw_a_up = imu_vertical_accel(float(ax), float(ay), float(az),
                                      self._roll, self._pitch)
        self._a_up = self.lpf_acc.step(raw_a_up)

    def on_gnss(self, line: str, now: float = None) -> bool:
        """NMEA 한 줄 입력. GGA 면 틸트보정 후 EKF 갱신. 반환=Fix/Float 유효 갱신 여부."""
        now = now if now is not None else time.time()
        fix = self.mon.feed(line, now)
        if fix is None or not fix.valid:
            return False
        # 안테나고 → ENU Up
        up = self._enu_to_up(fix.lat, fix.lon, fix.alt) if self._enu_to_up else fix.alt
        # 틸트 삼각보정(blade_tip_z 재사용): 안테나 흔들림→블레이드 Z 매핑
        blade_z = self.params.blade_tip_z(up, self._pitch, self._roll)
        sigma = self.mon.sigma_z(now)
        # Float 도 갱신엔 사용하되 σ 가 커서 가중 낮음(자연스런 IMU 우위 = 브리지)
        if not self.ekf._init:
            self.ekf.init_z(blade_z)
        self.ekf.update_z(blade_z, sigma)
        self._last_meas_t = now
        return True

    # ── 제어 스텝(ctrl_rate) ──────────────────────────────────
    def step(self, dt: float, now: float = None) -> ZEstimate:
        now = now if now is not None else time.time()
        # 1) EKF 예측(IMU 데드레코닝) — GNSS 없어도 진행
        self.ekf.predict(self._a_up, dt)
        # 1b) ZUPT — 수직가속도가 작으면(블레이드 높이 유지 중) vz≈0 구속
        #     → vz 불확실성 억제 → 브리지(GNSS 끊김) 드리프트를 cm급으로 유지
        if abs(self._a_up) < self.cfg.zupt_accel_thresh:
            self.ekf.update_vz(0.0, self.cfg.zupt_sigma_vz)
        q = self.mon.snapshot(now)

        # 2) 페일세이프 FSM 전이
        sigma_cm = self.ekf.sigma_z_est * 100
        self._fsm(q, sigma_cm, now)

        # 3) 출력 평활(디젤 고주파 제거 LPF) — 제어신호 요동 방지
        z_filt = self.lpf.step(self.ekf.z)

        # 4) 품질 적응형 데드밴드/게인
        if self.state == ZState.TRACK:
            deadband = self.cfg.base_deadband_cm
            gain = 1.0
        elif self.state == ZState.BRIDGE:
            deadband = self.cfg.float_deadband_cm
            # σ 커질수록 게인 감쇠(과민반응 억제)
            gain = max(0.3, 1.0 - sigma_cm / max(0.1, self.cfg.bridge_sigma_limit_cm))
        else:                                  # HOLD/STOP/INIT
            deadband = self.cfg.float_deadband_cm
            gain = 0.0
        control = self.state in (ZState.TRACK, ZState.BRIDGE)

        remain = (max(0.0, self.cfg.bridge_max_s - (now - self._bridge_t0))
                  if self.state == ZState.BRIDGE else
                  (self.cfg.bridge_max_s if self.state == ZState.TRACK else 0.0))

        return ZEstimate(
            blade_z_m=z_filt, vz_cms=self.ekf.vz * 100, state=self.state,
            control_enabled=control, deadband_cm=deadband, gain_scale=gain,
            sigma_z_cm=sigma_cm, bridge_remaining_s=remain, quality=q)

    def _fsm(self, q: GnssQuality, sigma_cm: float, now: float):
        cfg = self.cfg
        fresh = q.age_s <= max(2.0, 3.0 / cfg.nmea_rate_hz)   # GGA 신선도
        fixed = (q.quality == 4 and fresh)
        floaty = (q.quality == 5 and fresh)

        if self.state in (ZState.INIT,) and self.ekf._init and fixed:
            self.state = ZState.TRACK
        elif self.state == ZState.TRACK:
            if not fixed:                          # Fix 풀림 → 브리지 시작
                self.state = ZState.BRIDGE
                self._bridge_t0 = now
        elif self.state == ZState.BRIDGE:
            if fixed:                              # Fix 복귀
                self.state = ZState.TRACK
            elif (now - self._bridge_t0 > cfg.bridge_max_s
                  or sigma_cm > cfg.bridge_sigma_limit_cm):
                self.state = ZState.HOLD           # 브리지 한계 → 밸브 정지
        elif self.state == ZState.HOLD:
            if fixed:
                self.state = ZState.TRACK
            elif now - self._bridge_t0 > cfg.bridge_max_s * 2:
                self.state = ZState.STOP           # 장기 끊김 → 안전정지
        elif self.state == ZState.STOP:
            if fixed or (floaty and not cfg.fix_required_to_resume):
                self.state = ZState.TRACK if fixed else ZState.BRIDGE
                if not fixed:
                    self._bridge_t0 = now

    def reset(self):
        self.ekf = VerticalEKF()
        self.lpf.reset()
        self.state = ZState.INIT


# ═══════════════════════════════════════════════════════════════
#  자가검증 (HW 불필요) — 합성 시나리오
# ═══════════════════════════════════════════════════════════════
def _gga(lat, lon, alt, q=4, sats=30, hdop=0.7) -> str:
    body = (f"GPGGA,120000.00,{_to_dm(lat,True)},{_to_dm(lon,False)},"
            f"{q},{sats:02d},{hdop:.1f},{alt:.3f},M,0.0,M,,")
    cks = 0
    for c in body:
        cks ^= ord(c)
    return f"${body}*{cks:02X}"

def _to_dm(v, is_lat):
    hemi = ("N" if v >= 0 else "S") if is_lat else ("E" if v >= 0 else "W")
    v = abs(v); d = int(v); m = (v - d) * 60
    return (f"{d:02d}{m:08.5f},{hemi}" if is_lat else f"{d:03d}{m:08.5f},{hemi}")

def _gsv(talker, prns_elevs) -> str:
    n = len(prns_elevs)
    body = f"{talker}GSV,1,1,{n:02d}"
    for prn, el in prns_elevs:
        body += f",{prn:02d},{el:02d},000,45"
    cks = 0
    for c in body:
        cks ^= ord(c)
    return f"${body}*{cks:02X}"


if __name__ == "__main__":
    print("=" * 72)
    print("z_stabilizer — VRS Z축 안정화 자가검증 (HW 불필요)")
    print("=" * 72)
    params = LevelerParams(antenna_height_above_blade=2.5,
                           antenna_to_blade_horizontal=1.2)

    # ── 1. 고도각 마스크 + Multi-GNSS 가용위성 ────────────────
    print("\n▶ 1. 고도각 15° 마스크 + BeiDou/Galileo 가용위성")
    mon = GnssQualityMonitor(elevation_mask_deg=15.0)
    # 마스크 미만(저고도) 위성 섞어 넣기 — 마스크가 걸러내야
    mon.feed(_gsv("GP", [(1, 40), (2, 8), (3, 25), (4, 70)]))    # 8°<15° 제외 → 3
    mon.feed(_gsv("GB", [(11, 30), (12, 55), (13, 20), (14, 12), (15, 45)]))  # 12°제외 → 4
    mon.feed(_gsv("GA", [(21, 35), (22, 60), (23, 18)]))         # 3
    mon.feed(_gga(37.5, 127.0, 50.0, q=4, sats=28, hdop=0.7))
    snap = mon.snapshot()
    assert snap.by_const["GPS"] == 3 and snap.by_const["BeiDou"] == 4
    assert snap.sats_above_mask == 10
    print(f"  ✓ 마스크통과 {snap.sats_above_mask}위성 (GPS3/BeiDou4/Galileo3), "
          f"저고도 위성 제외 확인")
    print(f"    σ_z(Fix,HDOP0.7)={mon.sigma_z()*100:.2f}cm")

    # ── 2. Fix 정상추종: 진동+노이즈 속 Z 1~2cm 유지 ──────────
    print("\n▶ 2. Fix 추종 — 디젤진동/노이즈 속 Z 안정화")
    zs = ZStabilizer(params)
    DT = 0.05
    t = 0.0
    import random
    random.seed(1)
    # 30+ 위성 확보(BeiDou/Galileo 대거) — 고도각 마스크 상향 결핍 상쇄
    gsv_gp = _gsv("GP", [(i, 30 + i % 40) for i in range(1, 13)])     # 12
    gsv_gb = _gsv("GB", [(i, 25 + i % 50) for i in range(20, 32)])    # 12
    gsv_ga = _gsv("GA", [(i, 20 + i % 40) for i in range(40, 48)])    # 8 → 합 32
    true_blade = 50.0 - params.antenna_height_above_blade  # blade_tip_z(평지)
    errs = []
    for i in range(400):    # 20초
        t += DT
        # IMU: 평지 정지(수직가속도 ≈ 0 + 디젤 고주파 진동)
        vib = 2.0 * math.sin(2 * math.pi * 6 * t)     # 6Hz 엔진진동
        zs.on_imu(ax=0, ay=0, az=G0 + vib, roll=0.0, pitch=0.0, now=t)
        if i % int(zs.cfg.ctrl_rate_hz / zs.cfg.nmea_rate_hz) == 0:  # 1Hz GGA
            for g in (gsv_gp, gsv_gb, gsv_ga):
                zs.on_gnss(g, now=t)
            noisy_alt = 50.0 + random.gauss(0, 0.015)   # ±1.5cm RTK 잡음
            zs.on_gnss(_gga(37.5, 127.0, noisy_alt, q=4, sats=30, hdop=0.7), now=t)
        est = zs.step(DT, now=t)
        if i > 140:     # 출력 LPF(0.35Hz) 정착 후 구간만 평가
            errs.append(abs(est.blade_z_m - true_blade))
    rms = math.sqrt(sum(e * e for e in errs) / len(errs))
    assert zs.state == ZState.TRACK
    assert rms < 0.02, f"Z RMS {rms*100:.2f}cm > 2cm"
    print(f"  ✓ 상태=TRACK, Z RMS 오차 {rms*100:.2f}cm (<2cm), 진동 평활됨")

    # ── 3. 틸트 30° → Z 튐 삼각보정 ───────────────────────────
    print("\n▶ 3. 틸트 30° 삼각보정 (마스트 흔들림)")
    zs2 = ZStabilizer(params)
    pitch = math.radians(30)
    # 같은 안테나고라도 30° 기울면 blade_tip_z 가 보정해야
    for i in range(200):    # 출력 LPF(0.35Hz) 정착까지(~10s)
        zs2.on_imu(0, 0, G0, roll=0.0, pitch=pitch, now=i * DT)
        zs2.on_gnss(_gga(37.5, 127.0, 50.0, q=4, sats=30), now=i * DT)
        e = zs2.step(DT, now=i * DT)
    z_tilt = e.blade_z_m
    z_flat = params.blade_tip_z(50.0, 0.0, 0.0)
    z_tilt_exp = params.blade_tip_z(50.0, pitch, 0.0)
    assert abs(z_tilt - z_tilt_exp) < 0.03, (z_tilt, z_tilt_exp)
    assert abs(z_tilt - z_flat) > 0.1, "틸트 보정이 반영돼야(평지와 달라야)"
    print(f"  ✓ 평지 blade_z={z_flat:.3f} vs 30°보정 {z_tilt:.3f}m "
          f"(삼각보정 Δ={abs(z_tilt-z_flat)*100:.1f}cm 반영)")

    # ── 4. VRS 끊김 → RTK Bridge(데드레코닝) → HOLD ──────────
    print("\n▶ 4. VRS 끊김 — RTK Bridge 10~20s → HOLD → 복귀")
    zs3 = ZStabilizer(params, ZStabilizerConfig(bridge_max_s=15.0))
    t = 0.0
    # 4a. Fix 확립
    for i in range(40):
        t += DT
        zs3.on_imu(0, 0, G0, 0, 0, now=t)
        zs3.on_gnss(_gga(37.5, 127.0, 50.0, q=4, sats=30), now=t)
        zs3.step(DT, now=t)
    assert zs3.state == ZState.TRACK
    # 4b. VRS 끊김(GGA 중단) — IMU 만으로 브리지
    bridge_seen = False
    t_cut = t
    for i in range(int(20 / DT)):     # 20초 끊김
        t += DT
        zs3.on_imu(0, 0, G0, 0, 0, now=t)
        est = zs3.step(DT, now=t)
        if est.state == ZState.BRIDGE:
            bridge_seen = True
            assert est.control_enabled and est.deadband_cm >= zs3.cfg.float_deadband_cm
        if t - t_cut < 10:
            assert est.state in (ZState.TRACK, ZState.BRIDGE), est.state
    assert bridge_seen, "브리지 미진입"
    assert zs3.state in (ZState.HOLD, ZState.STOP), zs3.state
    print(f"  ✓ 끊김 직후 BRIDGE(밸브 유지, 데드밴드 확대) → 한계 후 {zs3.state.name}")
    # 4c. Fix 복귀 → TRACK
    for i in range(20):
        t += DT
        zs3.on_imu(0, 0, G0, 0, 0, now=t)
        zs3.on_gnss(_gga(37.5, 127.0, 50.0, q=4, sats=30), now=t)
        zs3.step(DT, now=t)
    assert zs3.state == ZState.TRACK
    print(f"  ✓ Fix 복귀 → TRACK 재개")

    # ── 5. 페일세이프: 제어 게이트 ────────────────────────────
    print("\n▶ 5. 페일세이프 제어 게이트")
    est_stop = zs3.step(DT, now=t)
    assert est_stop.control_enabled            # 지금 TRACK
    # 강제 STOP 확인
    zs3.state = ZState.STOP
    e2 = zs3.step(DT, now=t + 100)
    assert not e2.control_enabled and e2.gain_scale == 0.0
    print(f"  ✓ STOP 시 control_enabled=False, gain=0 (유압 중립 권고)")

    print("\n" + "=" * 72)
    print("z_stabilizer 자가검증 5/5 통과.")
    print("  실배포: ZStabilizer(params, enu_to_up=lambda la,lo,al: enu.to_enu(la,lo,al)[2])")
    print("  → ZEstimate.blade_z_m/control_enabled/deadband_cm/gain_scale 를")
    print("  LevelingController·proportional_valve 입력으로 연결.")
