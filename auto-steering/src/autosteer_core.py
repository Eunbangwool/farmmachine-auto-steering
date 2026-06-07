"""
autosteer_core.py
=================
NX510 하드웨어 기반 자율조향 핵심 알고리즘.

하드웨어 구성:
  - 태블릿 (Apollo 10 Pro, Android) ← 본인 앱이 실행
  - CAN 버스 → 조향 모터 (토크 제어)
  - CAN 버스 ← 앵글센서 (조향각 피드백)
  - USB → F9P RTK 수신기 (본인 보유)
  - USB → LoRa NTRIP 수신

4계층 구조:
  Layer 1: 경로 정의 (Path Definition)
  Layer 2: 상태 추정 (State Estimation, RTK+IMU)
  Layer 3: 경로 추종 (Path Following, Pure Pursuit / Stanley)
  Layer 4: 모터 제어 (Motor Control, CAN)

⚠️  CAN 규격 미확정 구간:
  CAN_MOTOR_* 와 CAN_SENSOR_* 상수를 본인 모터 프로그램 문서로 채워야 함.
  해당 구간은 ★ 표시로 명시됨.
"""

from __future__ import annotations
import math, time, struct, threading, logging
from dataclasses import dataclass, field
from typing import List, Optional, Callable
from abc import ABC, abstractmethod
from enum import Enum, auto

log = logging.getLogger("autosteer")

class SlopeCorrectionMode(str, Enum):
    """
    경사면 보정 방식 — AgNav Image 4, 7, 8: 경사면 보정 = Standard.
    OFF      : 경사 보정 없음 (평지 전용)
    STANDARD : roll만 보정 — 좌우 경사 오차 제거 (AgNav 기본)
    ENHANCED : roll + pitch 모두 보정 — 급경사 논두렁 등
    """
    OFF      = "Off"
    STANDARD = "Standard"
    ENHANCED = "Enhanced"


# ═══════════════════════════════════════════════════════════════
#  트랙터 파라미터 — AGMO가 사용하는 5개 파라미터
# ═══════════════════════════════════════════════════════════════

@dataclass
class TractorParams:
    """
    트랙터별 기구학 파라미터. 차종마다 실측해서 설정.

    AGMO의 5개 파라미터 (필수):
      1) wheelbase         = 앞차축 ↔ 뒤차축 거리 (m)
      2) antenna_height    = GPS 안테나 설치 높이 (m, 지면 기준)
      3) antenna_to_axle   = GPS 안테나 ↔ 뒤차축 전후 거리 (m, 전방=+)
      4) antenna_to_impl   = GPS 안테나 ↔ 작업기 거리 (m, 후방=+)
      5) imu_offset        = IMU 평지 기준값 (캘리브레이션 오프셋)

    CHCNAV 추가 파라미터 (사진에서 확인, 알고리즘 정밀도 향상):
      B1) hitch_to_impl    = 히치 포인트 ↔ 작업기 거리 (m)
                             B1: 작업기 견인 포인트 (사진)
                             히치가 뒤차축보다 뒤에 있으면 +
      G)  front_track_width = 전륜 폭 거리 (m)
                             G: 전륜 폭 거리 1.56m (사진)
                             Ackermann 조향각 정확한 계산에 사용
      WAS) max_was_deg     = 최대 조향각 (°)
                             최대 WAS = 25° (사진)
                             CAN 명령 클램프에 사용

    Kubota MR1157 (사진 기반 + 추정값):
      wheelbase       ≈ 2.47 m
      antenna_height  = 2.73 m  (사진 E: 안테나 높이)
      antenna_to_axle ≈ 0.50 m  (사진 D: 안테나 리어 액슬 거리, 뒤쪽)
      antenna_to_impl ≈ 1.20 m
      hitch_to_impl   ≈ 1.00 m  (사진 B1)
      front_track_width = 1.56 m (사진 G)
      max_was_deg     = 25       (사진 최대 WAS)
    """
    # 파라미터 1: 휠베이스
    wheelbase: float = 2.47

    # 파라미터 2: 안테나 높이
    antenna_height: float = 2.73        # 사진 E에서 확인

    # 파라미터 3: 안테나 ↔ 뒤차축 전후 오프셋
    # 사진: "안테나의 위치: 뒤쪽" → 안테나가 뒤차축 뒤에 있음 → 음수
    antenna_to_axle: float = -0.50      # 사진 D ≈ 0.5m, 뒤쪽이므로 -

    # 파라미터 4: 안테나 ↔ 작업기 (뒤차축 기준 후방 거리)
    antenna_to_impl: float = 1.20

    # 파라미터 5: IMU 캘리브레이션
    imu_offset: 'ImuOffset' = None

    # CHCNAV 추가 파라미터 B1: 히치 포인트 ↔ 작업기
    hitch_to_impl: float = 1.00         # 사진 B1

    # CHCNAV 추가 파라미터 G: 전륜 폭
    front_track_width: float = 1.56     # 사진 G

    # CHCNAV 추가 파라미터 WAS: 최대 조향각
    max_was_deg: float = 25.0           # 사진 최대 WAS

    # 경사면 보정 모드 (AgNav: Standard)
    slope_correction: str = "Standard"  # "Off" / "Standard" / "Enhanced"

    def __post_init__(self):
        if self.imu_offset is None:
            self.imu_offset = ImuOffset()

    @property
    def max_steer_rad(self) -> float:
        """최대 조향각 (rad). CAN 명령 클램프에 사용."""
        return math.radians(self.max_was_deg)

    def rear_axle_pos(self, gps_x: float, gps_y: float,
                      heading: float) -> tuple:
        """
        파라미터 3 적용: GPS 안테나 → 뒤차축 좌표 변환.
        antenna_to_axle가 음수이면 안테나가 뒤차축 뒤에 위치.
        """
        axle_x = gps_x - self.antenna_to_axle * math.cos(heading)
        axle_y = gps_y - self.antenna_to_axle * math.sin(heading)
        return axle_x, axle_y

    # 하위 호환성 유지
    def antenna_to_axle_pos(self, gps_x, gps_y, heading):
        return self.rear_axle_pos(gps_x, gps_y, heading)

    def height_correction(self, roll: float) -> float:
        """파라미터 2: 경사(roll) → 수평 오차."""
        return self.antenna_height * math.sin(roll)

    def implement_position(self, gps_x: float, gps_y: float,
                           heading: float) -> tuple:
        """
        파라미터 4 적용: GPS → 작업기 위치.
        작업기 위치 기준 제어 알고리즘 + 레벨러 z축 기록에 사용.
        """
        # 뒤차축 → 작업기 (후방으로 antenna_to_impl만큼)
        axle_x, axle_y = self.rear_axle_pos(gps_x, gps_y, heading)
        impl_x = axle_x - self.antenna_to_impl * math.cos(heading)
        impl_y = axle_y - self.antenna_to_impl * math.sin(heading)
        return impl_x, impl_y

    def ackermann_correction(self, delta: float) -> float:
        """
        파라미터 G(전륜 폭) 기반 Ackermann 보정.
        좌우 앞바퀴가 서로 다른 각도를 가져야 하는 원리.
        단일 앵글 명령 시 평균값 기준으로 보정.
        delta: 기준 조향각 (rad)
        반환: 보정된 조향각 (rad)
        """
        if abs(delta) < 1e-6 or self.wheelbase < 1e-6:
            return delta
        # 회전 반경
        R = self.wheelbase / math.tan(delta)
        # 좌우 바퀴 각도
        t = self.front_track_width / 2
        try:
            delta_inner = math.atan(self.wheelbase / (R - t))
            delta_outer = math.atan(self.wheelbase / (R + t))
        except (ZeroDivisionError, ValueError):
            return delta
        # 평균 → 앵글센서 명령 기준
        return (delta_inner + delta_outer) / 2.0 * math.copysign(1, delta)


@dataclass
class ImuOffset:
    """
    파라미터 5: IMU 캘리브레이션 오프셋.
    평지에서 측정한 IMU 원시값 — 이걸 빼면 진짜 자세 나옴.

    캘리브레이션 절차:
      1) 트랙터를 수평인 평지에 정차
      2) 30초 이상 IMU 값 평균
      3) 그 평균값을 아래 필드에 저장
    """
    roll: float  = 0.0   # rad, 평지에서의 IMU roll 원시값
    pitch: float = 0.0   # rad, 평지에서의 IMU pitch 원시값
    yaw: float   = 0.0   # rad, IMU yaw 기준 보정값 (북 기준)

    def correct_roll(self, raw_roll: float) -> float:
        return raw_roll - self.roll

    def correct_pitch(self, raw_pitch: float) -> float:
        return raw_pitch - self.pitch

    def correct_yaw(self, raw_yaw: float) -> float:
        """heading = IMU yaw - 기준 보정값."""
        return _wrap(raw_yaw - self.yaw)


class ImuCalibrator:
    """
    파라미터 5(IMU 오프셋) 캘리브레이션 도우미.

    절차 (CLAUDE.md 우선순위 5 — "평지에서 30초 측정 → ImuOffset 채우기"):
      1) 트랙터를 수평 평지에 정차
      2) start() 호출 후, 100Hz IMU 루프에서 raw 값을 add_sample() 로 누적
      3) ready 가 True 가 되면 finish() 로 평균 오프셋(ImuOffset) 생성
      4) 결과를 params.imu_offset 에 대입하고 KUBOTA_MR1157 기본값/설정 저장

    roll/pitch: 평지 정차 평균값 = 센서 장착 기울기 오프셋.
    yaw: 절대 방위(북) 기준이 없으면 보정 불가 → 기본 0.
         heading_ref_rad(예: RTK 듀얼안테나/이동평균 heading)를 주면
         원형 평균으로 yaw 오프셋 산출.

    사용 예:
        cal = ImuCalibrator()          # 기본 30초 / 3000샘플(100Hz)
        cal.start()
        while not cal.ready:
            cal.add_sample(raw_roll, raw_pitch, raw_yaw)   # IMU 루프에서
        params.imu_offset = cal.finish()
    """
    def __init__(self, min_samples: int = 3000, min_duration: float = 30.0):
        self.min_samples  = min_samples
        self.min_duration = min_duration
        self._roll:  List[float] = []
        self._pitch: List[float] = []
        self._yaw:   List[float] = []
        self._t0: Optional[float] = None

    def start(self):
        """샘플 버퍼 초기화 + 타이머 시작."""
        self._roll.clear(); self._pitch.clear(); self._yaw.clear()
        self._t0 = time.time()
        log.info(f"IMU 캘리브레이션 시작 — 평지 정차 유지 "
                 f"({self.min_duration:.0f}s / {self.min_samples}샘플)")

    def add_sample(self, raw_roll: float, raw_pitch: float,
                   raw_yaw: float = 0.0):
        if self._t0 is None:
            self.start()
        self._roll.append(raw_roll)
        self._pitch.append(raw_pitch)
        self._yaw.append(raw_yaw)

    @property
    def elapsed(self) -> float:
        return 0.0 if self._t0 is None else time.time() - self._t0

    @property
    def progress(self) -> float:
        """0.0~1.0 진행률 (샘플/시간 중 느린 쪽)."""
        if self._t0 is None:
            return 0.0
        return min(1.0, min(len(self._roll) / max(1, self.min_samples),
                            self.elapsed / max(1e-6, self.min_duration)))

    @property
    def ready(self) -> bool:
        return (self._t0 is not None
                and len(self._roll) >= self.min_samples
                and self.elapsed >= self.min_duration)

    def finish(self, heading_ref_rad: Optional[float] = None) -> 'ImuOffset':
        """누적 샘플 평균 → ImuOffset 생성. min 미달이어도 강제 계산 가능."""
        if not self._roll:
            raise RuntimeError("IMU 캘리브레이션 샘플 없음 — add_sample() 먼저 호출")
        mean = lambda xs: sum(xs) / len(xs)
        roll  = mean(self._roll)
        pitch = mean(self._pitch)
        if heading_ref_rad is None:
            yaw = 0.0
        else:
            # 원형 평균(circular mean) 후 기준 방위와의 차
            sy = mean([math.sin(a) for a in self._yaw])
            cy = mean([math.cos(a) for a in self._yaw])
            yaw = _wrap(math.atan2(sy, cy) - heading_ref_rad)
        log.info(f"IMU 캘리브레이션 완료: roll={roll:+.4f} pitch={pitch:+.4f} "
                 f"yaw={yaw:+.4f} rad (샘플 {len(self._roll)}개, "
                 f"{self.elapsed:.1f}s)")
        return ImuOffset(roll=roll, pitch=pitch, yaw=yaw)


# 기본 파라미터 인스턴스 (Kubota MR1157 — 사진 기반 + 일부 추정)
# ★ 파라미터 1(휠베이스), 3(안테나↔차축) 은 반드시 실측 후 교체
KUBOTA_MR1157 = TractorParams(
    wheelbase         = 2.47,   # ★ 실측 필요
    antenna_height    = 2.73,   # 사진 E에서 확인
    antenna_to_axle   = -0.50,  # 사진 D ≈ 0.5m, 뒤쪽 → 음수 (★ 실측 필요)
    antenna_to_impl   = 1.20,   # ★ 실측 필요
    hitch_to_impl     = 1.00,   # 사진 B1
    front_track_width = 1.56,   # 사진 G
    max_was_deg       = 25.0,   # 사진 최대 WAS
    slope_correction  = "Standard",
    imu_offset = ImuOffset(roll=0.0, pitch=0.0, yaw=0.0),
)


# ═══════════════════════════════════════════════════════════════
#  AgNav 3모드 튜닝 시스템
#  사진에서 읽은 실제값: 일반 / 과부하(쟁기) / 모래토양
# ═══════════════════════════════════════════════════════════════

@dataclass
class TuningProfile:
    """
    작업별 튜닝 프로파일. AgNav '종합 설정 > 드라이버 매개변수' 3모드.

    AgNav 사진(Image 4, 5, 6, 7, 8, 19, 20)에서 확인한 실제값:

    ┌────────────────────┬────────┬────────┬────────┐
    │ 파라미터            │ 일반   │ 과부하  │ 모래토양│
    ├────────────────────┼────────┼────────┼────────┤
    │ WAS Gain           │  20    │  15    │   9    │
    │ 크로스 트랙 Gain    │  35    │ 100    │  35    │
    │ 방향 감도           │ 100    │ 100    │ 100    │
    │ U 감도              │  40    │  30    │  40    │
    │ Reverse Gain       │  10    │  15    │  10    │
    │ 온라인 진행도       │ 100    │  60    │ 100    │
    │ 진입 적극성         │  70    │  30    │  70    │
    │ PTime On           │   1.0  │   1.3  │   1.0  │
    │ PTime Off          │   1.0  │   1.0  │   2.0  │
    └────────────────────┴────────┴────────┴────────┘

    AgNav → 본인 알고리즘 매핑:
      WAS Gain      → SteeringActuator.pos_kp × was_scale
      크로스 트랙 G  → ImplementReferenced k_cross 기반
      방향 감도      → k_heading
      U 감도         → SteeringActuator.vel_kp × u_scale
      PTime On      → pred_dist = ptime_on × speed  (예측 거리)
      온라인 진행도  → 경로 위에 있을 때 게인 비율
      진입 적극성    → 경로 접근 중 게인 비율
    """
    name: str = "일반"

    # ── 조향/모터 게인 ────────────────────────────────────
    was_gain:     float = 20.0   # WAS Gain: 모터 반응 (낮을수록 부드러움)
    k_cross:      float = 35.0   # 크로스 트랙 Gain (높을수록 강한 경로 복귀)
    k_heading:    float = 100.0  # 방향 감도 (heading 오차 반응)
    u_gain:       float = 40.0   # U 감도 (속도 루프 U형 반응)
    reverse_gain: float = 10.0   # 역방향 보정

    # ── 경로 진입 vs 추종 분리 ────────────────────────────
    online_progress:         float = 100.0  # 경로 위(on-line): 게인 비율 (0-100)
    approach_aggressiveness: float = 70.0   # 경로 진입 적극성 (0-100)
    strict_heading:          bool  = False  # 엄격한 방향 제어

    # ── 예측 시간 (AgNav PTime) ───────────────────────────
    ptime_on:  float = 1.0   # 예측 시간 (s) → pred_dist = ptime_on × speed
    ptime_off: float = 1.0   # 예측 해제 시간

    # ── 정규화 프로퍼티 (알고리즘 내부 사용) ─────────────
    @property
    def k_cross_algo(self) -> float:
        """AgNav 0-100 스케일 → 알고리즘 k (0-2.0)."""
        return self.k_cross / 100.0 * 2.0

    @property
    def k_heading_algo(self) -> float:
        """방향 감도 0-100 → 알고리즘 계수 0-1.0."""
        return self.k_heading / 100.0

    @property
    def was_scale(self) -> float:
        """WAS Gain → 모터 P 게인 배율 (기준 20=1.0)."""
        return self.was_gain / 20.0

    @property
    def u_scale(self) -> float:
        """U 감도 → 속도 루프 배율 (기준 40=1.0)."""
        return self.u_gain / 40.0


# 사전 정의 프로파일 (AgNav 사진값 그대로)
PROFILE_NORMAL = TuningProfile(
    name                     = "일반 (로터리/방제/시비)",
    was_gain=20.0,  k_cross=35.0,   k_heading=100.0,
    u_gain=40.0,    reverse_gain=10.0,
    online_progress=100.0,  approach_aggressiveness=70.0,
    strict_heading=False,   ptime_on=1.0,  ptime_off=1.0,
)

PROFILE_HEAVY = TuningProfile(
    name                     = "과부하 (쟁기/균평/심토파쇄)",
    was_gain=15.0,  k_cross=100.0,  k_heading=100.0,
    u_gain=30.0,    reverse_gain=15.0,
    online_progress=60.0,   approach_aggressiveness=30.0,
    strict_heading=False,   ptime_on=1.3,  ptime_off=1.0,
)

PROFILE_SAND = TuningProfile(
    name                     = "모래토양 (사질/미끄러운 지면)",
    was_gain=9.0,   k_cross=35.0,   k_heading=100.0,
    u_gain=40.0,    reverse_gain=10.0,
    online_progress=100.0,  approach_aggressiveness=70.0,
    strict_heading=False,   ptime_on=1.0,  ptime_off=2.0,
)

PROFILES: dict = {
    "normal": PROFILE_NORMAL,
    "heavy":  PROFILE_HEAVY,
    "sand":   PROFILE_SAND,
}


# ═══════════════════════════════════════════════════════════════
#  추적 매개변수 — AgNav '커브/해로우 매개변수' (Image 13, 16)
#  경로 위(on-line) vs 접근(approach) 분리
# ═══════════════════════════════════════════════════════════════

@dataclass
class TrackingParams:
    """
    AgNav '추적 매개변수' 탭 (Image 13 커브, Image 16 해로우).
    커브/해로우 공통 확인값:
      온라인 민감도:  1.5  (경로 위 감도 배수)
      접근 민감도:    2.5  (경로 진입 감도 배수)
      온라인 임계값:  2.5m (이 거리 이내 = on-line)
      커브 계수:      1.0
    """
    online_sensitivity:   float = 1.5   # 경로 위 감도 배수
    approach_sensitivity: float = 2.5   # 접근 감도 배수
    online_threshold:     float = 2.5   # on-line 판정 임계값 (m)
    curve_coefficient:    float = 1.0   # 커브 보정 계수


# ═══════════════════════════════════════════════════════════════
#  모터 보호 파라미터 — AgNav '드라이버 매개변수' (Image 11, 12, 14)
# ═══════════════════════════════════════════════════════════════

@dataclass
class MotorProtectionParams:
    """
    AgNav '드라이버 매개변수' 화면에서 확인한 모터 물리·보호 파라미터.

    Image 11 확인값:
      조향비:           17.5
      핸들조향비 오프셋: -0.2
      핸들 데드존:       20
      스티어링 데드존:    0
      최대 과부하 전류:  300 (이 이상 → 타이머 시작)
      과부하 시간:        10s (지속 시 모터 정지)

    Image 12 확인값:
      모터 피드백 유형:  홀(Hall) ← 앵글센서가 홀센서 방식
      모터 비례 이득:   600 → vel_kp 스케일 기준
      모터 필수 Gain:   400 → friction_ff 스케일 기준

    Image 14 확인값:
      제어 모드: 모드2
      P Gain:   25
      D Gain:   80
      최대 RPM: 20  → max_ang_vel 계산에 사용
      연성:    100  → smoothing filter strength
    """
    steering_ratio:           float = 17.5
    handle_offset:            float = -0.2
    handle_deadzone:          float = 20.0
    steering_deadzone_offset: float = 0.0
    max_overload_current:     float = 300.0  # 내부 단위 (★ 실제 모터 문서 확인)
    overload_time:            float = 10.0   # s
    motor_feedback_type:      str   = "hall"
    motor_p_gain:             float = 600.0  # → vel_kp 기준
    motor_d_gain:             float = 80.0   # Image 14
    motor_necessary_gain:     float = 400.0  # → friction_ff 기준
    max_rpm:                  float = 20.0   # → max_ang_vel = max_rpm×2π/60
    softness:                 float = 100.0  # 연성: 100=매우 부드러움
    p_gain_high:              float = 25.0   # 고수준 P
    d_gain_high:              float = 80.0   # 고수준 D
    torque_limit:             float = 50.0   # 토크 제한 (Image 14 추정)


# 기본 모터 보호 파라미터 (AgNav 사진값)
DEFAULT_MOTOR_PROTECTION = MotorProtectionParams()


# ═══════════════════════════════════════════════════════════════
#  ★ CAN 규격 — 본인 모터 프로그램 문서로 채울 것
# ═══════════════════════════════════════════════════════════════

class CanSpec:
    """
    Keya KY170DD01005-08G 전동 조향 모터 CAN 프로토콜 (매뉴얼 V2.4 확정)

    ┌─────────────────────────────────────────────────────────┐
    │  핀맵 (7핀 항공 플러그)                                  │
    │  Pin1=IN+(12V)  Pin2=IN-(GND)  Pin3=TX  Pin4=RX         │
    │  Pin5=GND       Pin6=CAN-H     Pin7=CAN-L               │
    ├─────────────────────────────────────────────────────────┤
    │  CAN 설정                                                │
    │  비트레이트: 250kbps (parameter 0021=2, 공장 기본값)    │
    │  ID 타입:   Extended 29-bit                              │
    │  바이트 순서: low word big-endian first                  │
    │             (heartbeat는 big-endian)                     │
    ├─────────────────────────────────────────────────────────┤
    │  CAN ID (motor_id=1 기준, parameter 0018)               │
    │  TX(명령):   0x06000001                                  │
    │  RX(응답):   0x05800001                                  │
    │  Heartbeat:  0x07000001 (20ms 주기, parameter 0034)     │
    ├─────────────────────────────────────────────────────────┤
    │  AGMO 설정 (parameter 기반)                             │
    │  0018=1  (motor_id=1)                                   │
    │  0019=2  (CAN 제어)                                     │
    │  0020=1  (속도 제어)                                    │
    │  0021=2  (250kbps)                                      │
    └─────────────────────────────────────────────────────────┘

    속도 제어 흐름:
      1) Enable  → TX: 23 0D 20 01 00 00 00 00
      2) Speed   → TX: 23 00 20 01 [value] (< 1000ms 간격 유지)
      3) Disable → TX: 23 0C 20 01 00 00 00 00

    Watchdog: 1000ms — 속도 명령을 1초 이상 전송하지 않으면 모터 정지.
    """

    # ── 버스 설정 ──────────────────────────────────────────
    CAN_BITRATE   = 250_000         # 250kbps (매뉴얼 확정)
    MOTOR_ID      = 1               # parameter 0018 기본값
    RATED_RPM     = 80              # parameter 0002 (80 RPM)
    CMD_PERIOD    = 0.020           # 50Hz 명령 주기 (20ms)
    MOTOR_CMD_PERIOD = CMD_PERIOD     # 하위호환 별칭 (50Hz 명령주기)
    WATCHDOG_MS   = 1000            # 속도 명령 워치독 (매뉴얼 확정)

    # ── CAN ID (29-bit Extended) ───────────────────────────
    # 모터 ID에 따라 변동: TX=0x06000000|id, RX=0x05800000|id, HB=0x07000000|id
    @staticmethod
    def tx_id(motor_id: int = 1) -> int:
        return 0x06000000 | motor_id

    @staticmethod
    def rx_id(motor_id: int = 1) -> int:
        return 0x05800000 | motor_id

    @staticmethod
    def heartbeat_id(motor_id: int = 1) -> int:
        return 0x07000000 | motor_id

    MOTOR_CMD_ID       = 0x06000001   # TX ID (motor_id=1)
    MOTOR_RESPONSE_ID  = 0x05800001   # RX ID (motor_id=1)
    MOTOR_HEARTBEAT_ID = 0x07000001   # 하트비트 ID (motor_id=1)

    # ── 고정 명령 바이트 시퀀스 ───────────────────────────
    CMD_ENABLE  = bytes([0x23, 0x0D, 0x20, 0x01, 0x00, 0x00, 0x00, 0x00])
    CMD_DISABLE = bytes([0x23, 0x0C, 0x20, 0x01, 0x00, 0x00, 0x00, 0x00])
    CMD_SPEED_HDR     = bytes([0x23, 0x00, 0x20, 0x01])  # 속도 명령 헤더 (+ 4바이트 값)
    CMD_POSITION_HDR  = bytes([0x23, 0x02, 0x20, 0x01])  # 위치 명령 헤더 (+ 4바이트 값)

    # 활성화 시퀀스 (Enable 후 Speed 명령 시작)
    MOTOR_ACTIVATE_SEQ: List[tuple] = [
        (0x06000001, bytes([0x23, 0x0D, 0x20, 0x01, 0x00, 0x00, 0x00, 0x00])),  # Enable
    ]

    # ── 값 인코딩 ──────────────────────────────────────────
    # KY170C 포맷: low word (big-endian) + high word (big-endian)
    # 32-bit 값 0x12345678 → bytes [0x12][0x34][0x56][0x78]
    # (low 16-bit big-endian first, then high 16-bit big-endian)
    @staticmethod
    def encode_value(value: int) -> bytes:
        """
        KY170C 32비트 값 인코딩.
        low word big-endian first, then high word big-endian.
        예) value=1000(0x000003E8) → [0x03][0xE8][0x00][0x00]
            value=-1000(0xFFFFFC18) → [0xFC][0x18][0xFF][0xFF]
        """
        v = value & 0xFFFFFFFF
        low_word  = v & 0xFFFF
        high_word = (v >> 16) & 0xFFFF
        return bytes([
            (low_word  >> 8) & 0xFF,   # DATA_L high
            low_word         & 0xFF,   # DATA_L low
            (high_word >> 8) & 0xFF,   # DATA_H high
            high_word        & 0xFF,   # DATA_H low
        ])

    @staticmethod
    def cmd_speed(speed_permille: int) -> bytes:
        """
        속도 명령 생성.
        speed_permille: -1000 ~ +1000 (rated_rpm의 ‰)
          +1000 = +80RPM (정방향)
          -1000 = -80RPM (역방향)
          +500  = +40RPM
        워치독: 1000ms 이내 재전송 필수.
        """
        v = max(-1000, min(1000, speed_permille))
        return CanSpec.CMD_SPEED_HDR + CanSpec.encode_value(v)

    @staticmethod
    def cmd_position(counts: int) -> bytes:
        """
        위치 명령 생성 (10000 counts/circle).
        counts: 양수=반시계, 음수=시계 (예: 10000=1회전)
        """
        return CanSpec.CMD_POSITION_HDR + CanSpec.encode_value(counts)

    @staticmethod
    def rpm_to_permille(rpm: float, rated_rpm: float = 80.0) -> int:
        """RPM → 속도 명령값 변환."""
        return int(max(-1000, min(1000, rpm / rated_rpm * 1000)))

    # ── 하트비트 파싱 (모터 → 태블릿) ────────────────────
    # ID: 0x07000001, 주기: 20ms, 8바이트, big-endian
    # [0][1] = 누적 각도 (360단위/회전, 0~65535에서 리셋)
    # [2][3] = 속도 RPM (부호있는 16비트)
    # [4][5] = 전류 (부호있는 16비트)
    # [6][7] = 오류 코드 (Data0=하위, Data1=상위)
    @staticmethod
    def parse_heartbeat(data: bytes) -> dict:
        """
        하트비트 파싱.
        반환:
          angle_raw: 누적 각도 카운트 (0~65535, 리셋됨)
          angle_deg: 각도 (도, 360/circle 기준)
          speed_rpm: 속도 (RPM, 부호있음)
          current:   전류 (raw, 부호있음)
          error_d0:  오류 바이트0 (모터 상태 플래그)
          error_d1:  오류 바이트1 (드라이버 상태 플래그)
          faults:    오류 목록 (문자열 리스트)
        """
        if len(data) < 8:
            return {}
        angle_raw = (data[0] << 8) | data[1]
        speed_rpm = int.from_bytes(data[2:4], 'big', signed=True)
        current   = int.from_bytes(data[4:6], 'big', signed=True)
        d0, d1    = data[6], data[7]
        # 각도: 0x168=360 → 0x168/360.0*360 = 0x168도? 아니면 직접 360단위?
        # 매뉴얼: "360°/circle" → 값이 360일 때 1바퀴
        angle_deg = angle_raw * (360.0 / 360.0)  # 단순 매핑 (현장 검증 필요)
        faults = CanSpec._parse_fault(d0, d1)
        return {
            'angle_raw':  angle_raw,
            'angle_deg':  angle_deg,
            'speed_rpm':  speed_rpm,
            'current':    current,
            'error_d0':   d0,
            'error_d1':   d1,
            'faults':     faults,
        }

    @staticmethod
    def _parse_fault(d0: int, d1: int) -> list:
        """오류 코드 → 오류 목록 변환 (매뉴얼 p.23 기준)."""
        faults = []
        # Data0 (모터/통신 오류)
        if d0 & 0x01: faults.append("Less phase")
        if d0 & 0x02: faults.append("Motor stall")
        if d0 & 0x04: faults.append("Hall failure")
        if d0 & 0x10: faults.append("232 disconnected")
        if d0 & 0x20: faults.append("CAN disconnected")
        if d0 & 0x40: faults.append("Current sensing error")
        if d0 & 0x80: faults.append("Motor stalled 2s")
        # Data1 (드라이버 보호)
        if d1 & 0x01: faults.append("Disabled")
        if d1 & 0x02: faults.append("Overvoltage")
        if d1 & 0x08: faults.append("Hardware protection")
        if d1 & 0x10: faults.append("EEPROM error")
        if d1 & 0x20: faults.append("Undervoltage")
        if d1 & 0x40: faults.append("N/A")
        if d1 & 0x80: faults.append("Mode failure")
        return faults

    # ── 앵글센서 피드백 (CHCNAV 앵글센서 별도, 모터 인코더와 구분) ──
    # AGMO는 모터 속도 제어 + 별도 WAS(Wheel Angle Sensor)로 실제 조향각 측정
    # WAS CAN ID는 ttyWK 캡처로 확인 필요 (현장 RTK Fix 후)
    SENSOR_ANGLE_ID    = 0x301      # ★ WAS CAN ID (현장 캡처로 확인 필요)
    SENSOR_BYTE_HI     = 0
    SENSOR_BYTE_LO     = 1
    SENSOR_ANGLE_SCALE = 10.0       # ★ 현장 확인 필요
    SENSOR_ANGLE_OFFSET= 0.0
    SENSOR_SIGNED      = True

    # ── 속도 제어 파라미터 (AGMO 기반 추정) ──────────────
    # 자율조향 시 일반적 속도 범위: ±200~400 permille (±16~32 RPM)
    STEER_SPEED_MAX    = 400        # 최대 조향 속도 (‰, 32RPM)
    STEER_SPEED_MIN    = 50         # 최소 유효 속도 (‰, 4RPM, 정지 데드밴드)

    # MockCanInterface 호환용 레거시 상수 (하위 호환)
    MOTOR_CMD_ID       = 0x06000001
    MOTOR_BYTE_MODE    = 0
    MOTOR_BYTE_CMD_HI  = 4
    MOTOR_BYTE_CMD_LO  = 5
    MOTOR_BYTE_SPEED_LIM = -1
    MOTOR_BYTE_CHECKSUM  = -1
    MOTOR_MODE_DISABLE   = 0x00
    MOTOR_MODE_ANGLE     = 0x01
    MOTOR_MODE_TORQUE    = 0x02    # 레거시 호환 (현 Keya 속도제어 경로에선 미사용)
    MOTOR_ANGLE_SCALE    = 1.0
    MOTOR_MAX_SPEED      = 400


# ═══════════════════════════════════════════════════════════════
#  GNSS 수신기 스펙 — EKF 튜닝 + 버스/시리얼 설정 참조
#  (CHCNAV PA-3 데이터시트 / u-blox F9P)
# ═══════════════════════════════════════════════════════════════

@dataclass(frozen=True)
class GnssReceiverSpec:
    """
    GNSS 수신기 하드웨어 스펙. EKF 측정노이즈(R) 튜닝 + CAN/시리얼 설정 참조용.
    """
    name:              str
    can_bitrate:       int      # bps (0 = CAN 미지원)
    serial_baud:       int      # bps (NMEA-0183 출력)
    nmea_rate_hz:      float    # 위치 출력 주기
    imu_rate_hz:       float    # IMU 출력 주기 (0 = 내장 IMU 없음)
    heading_acc_deg:   float    # heading 정확도 (≈1σ, 0 = heading 소스 없음)
    rollpitch_acc_deg: float
    vel_acc_mps:       float
    rtcm:              str      # 지원 차분 포맷
    # 헤딩 소스 종류 — 둘 다 동일 on_imu 경로로 EKF 에 융합(아래 참고):
    #   "ins"  : GNSS+INS 스마트안테나가 heading/자세 융합 출력 (PA-3/NX510/FJD/AGMO ver2)
    #   "dual" : 듀얼안테나 baseline heading + 별도 IMU 각속도/자세 (AGMO ver1)
    #   "none" : heading 소스 없음 (F9P 단독 — 별도 IMU/듀얼 필요)
    heading_source:    str = "none"


# ── GNSS+INS 스마트안테나 (단일 안테나, heading/자세 융합 출력) ──
# CHCNAV PA-3 (NX510 설치 안테나, 데이터시트). FJD AT2 / AGMO ver2 동급.
#   - CAN 2포트 @500kb/s, RS232 ≤115200bps, NMEA-0183 + 내장 IMU 100Hz
CHCNAV_PA3 = GnssReceiverSpec(
    name="CHCNAV PA-3", can_bitrate=500_000, serial_baud=115_200,
    nmea_rate_hz=10.0, imu_rate_hz=100.0,
    heading_acc_deg=0.3, rollpitch_acc_deg=0.1, vel_acc_mps=0.03,
    rtcm="RTCM3.2/3.3", heading_source="ins",
)

# 본인 u-blox ZED-F9P (RTK 보드, 내장 IMU/INS 없음 → 듀얼안테나 or 별도 IMU 필요)
UBLOX_F9P = GnssReceiverSpec(
    name="u-blox ZED-F9P", can_bitrate=0, serial_baud=38_400,
    nmea_rate_hz=10.0, imu_rate_hz=0.0,
    heading_acc_deg=0.0, rollpitch_acc_deg=0.0, vel_acc_mps=0.05,
    rtcm="RTCM3.x", heading_source="none",
)


# ═══════════════════════════════════════════════════════════════
#  Layer 1: 경로 정의
# ═══════════════════════════════════════════════════════════════

@dataclass
class Waypoint:
    """경로상의 한 점."""
    x: float                        # 동쪽 (m, 로컬 좌표)
    y: float                        # 북쪽 (m)
    speed: float = 1.5              # 이 구간 목표 속도 (m/s)
    implement_down: bool = True     # 작업기 내림 여부 (레벨러 연동)
    section: str = "work"           # "work" / "headland" / "approach"

class PathStrategy(ABC):
    @abstractmethod
    def generate(self) -> List[Waypoint]:
        ...

class ABLineStrategy(PathStrategy):
    """직선 AB + 평행 패스."""
    def __init__(self, point_a: tuple, point_b: tuple,
                 implement_width: float, num_passes: int,
                 speed: float = 1.5, overlap: float = 0.0):
        self.a, self.b = point_a, point_b
        self.width, self.passes = implement_width, num_passes
        self.speed, self.overlap = speed, overlap

    def generate(self) -> List[Waypoint]:
        out = []
        dx = self.b[0] - self.a[0]; dy = self.b[1] - self.a[1]
        L = math.hypot(dx, dy); ux, uy = dx/L, dy/L
        px, py = -uy, ux                # 수직 방향
        eff = self.width - self.overlap
        STEP = 0.5

        for pi in range(self.passes):
            off = pi * eff
            if pi % 2 == 0:
                s = (self.a[0]+px*off, self.a[1]+py*off)
                e = (self.b[0]+px*off, self.b[1]+py*off)
            else:
                s = (self.b[0]+px*off, self.b[1]+py*off)
                e = (self.a[0]+px*off, self.a[1]+py*off)
            n = max(1, int(math.hypot(e[0]-s[0], e[1]-s[1]) / STEP))
            for i in range(n+1):
                t = i/n
                out.append(Waypoint(
                    x=s[0]+(e[0]-s[0])*t, y=s[1]+(e[1]-s[1])*t,
                    speed=self.speed, section="work"))
            # 헤드랜드 회전
            if pi < self.passes - 1:
                nx = (pi+1) * eff
                ns = (self.b[0]+px*nx, self.b[1]+py*nx) if pi%2==0 \
                   else (self.a[0]+px*nx, self.a[1]+py*nx)
                out.extend(_bezier_turn(
                    (e[0],e[1]), (ns[0],ns[1]),
                    top=(e[1] > self.a[1]),
                    speed=max(0.6, self.speed*0.5)))
        return out

class ContourStrategy(PathStrategy):
    """본인이 직접 주행한 등고선 기반 평행 경로."""
    def __init__(self, recorded: List[tuple],
                 implement_width: float, num_passes: int,
                 speed: float = 1.2):
        self.recorded = recorded
        self.width, self.passes, self.speed = implement_width, num_passes, speed

    def generate(self) -> List[Waypoint]:
        out = []
        for pi in range(self.passes):
            row = _offset_curve(self.recorded, pi * self.width)
            seq = row if pi % 2 == 0 else row[::-1]
            for pt in seq:
                out.append(Waypoint(x=pt[0], y=pt[1],
                                    speed=self.speed, section="work"))
            if pi < self.passes - 1:
                next_row = _offset_curve(self.recorded, (pi+1)*self.width)
                ns = next_row[0] if (pi+1)%2==0 else next_row[-1]
                out.extend(_bezier_turn(
                    seq[-1], ns, top=(seq[-1][1] > seq[0][1]),
                    speed=max(0.6, self.speed*0.5)))
        return out

class CustomStrategy(PathStrategy):
    def __init__(self, waypoints: List[Waypoint]):
        self._wpts = waypoints
    def generate(self) -> List[Waypoint]:
        return self._wpts

def _offset_curve(path: List[tuple], offset: float) -> List[tuple]:
    result = []
    for i, pt in enumerate(path):
        if i == 0: dx,dy = path[1][0]-pt[0], path[1][1]-pt[1]
        elif i == len(path)-1: dx,dy = pt[0]-path[i-1][0], pt[1]-path[i-1][1]
        else: dx,dy = path[i+1][0]-path[i-1][0], path[i+1][1]-path[i-1][1]
        L = math.hypot(dx,dy) or 1
        nx, ny = -dy/L, dx/L
        result.append((pt[0]+nx*offset, pt[1]+ny*offset))
    return result

def _bezier_turn(p1: tuple, p2: tuple, top: bool,
                 speed: float = 0.7, N: int = 48) -> List[Waypoint]:
    """헤드랜드 회전 (3차 베지에)."""
    d = max(abs(p2[0]-p1[0])*0.55, 5.0)
    sign = 1 if top else -1
    c1 = (p1[0], p1[1]+d*sign)
    c2 = (p2[0], p2[1]+d*sign)
    out = []
    for i in range(1, N+1):
        t = i/N; u = 1-t
        x = u**3*p1[0] + 3*u**2*t*c1[0] + 3*u*t**2*c2[0] + t**3*p2[0]
        y = u**3*p1[1] + 3*u**2*t*c1[1] + 3*u*t**2*c2[1] + t**3*p2[1]
        out.append(Waypoint(x=x, y=y, speed=speed,
                            implement_down=False, section="headland"))
    return out


# ═══════════════════════════════════════════════════════════════
#  Layer 2: 상태 추정 (RTK + IMU 융합)
# ═══════════════════════════════════════════════════════════════

@dataclass
class VehicleState:
    x: float = 0.0          # 동쪽 (m)
    y: float = 0.0          # 북쪽 (m)
    heading: float = 0.0    # rad, 북=0, 동=π/2
    speed: float = 0.0      # m/s
    angular_vel: float = 0.0
    rtk_quality: int = 0    # 4=Fixed, 5=Float, 1=단독

class StateEstimator:
    """
    확장 칼만 필터: RTK(10Hz) + IMU(100Hz) 융합.
    상태: [x, y, heading, speed, angular_vel]

    TractorParams 반영:
      - update_rtk: 파라미터 2(높이 보정) + 파라미터 3(레버암 보정) 적용
      - update_imu: 파라미터 5(IMU 오프셋) 자동 보정
    """
    def __init__(self, params: TractorParams = None):
        import numpy as np
        self.np = np
        self.params = params or KUBOTA_MR1157
        self.x = np.zeros(5)
        self.P = np.eye(5) * 0.1
        self.Q = np.diag([0.01, 0.01, 0.001, 0.1, 0.01])
        self.R_rtk = np.diag([0.02**2, 0.02**2])
        self.R_hdg  = np.array([[0.001]])
        self._rtk_origin: Optional[tuple] = None
        self._current_roll:  float = 0.0   # roll 경사 보정용
        self._current_pitch: float = 0.0   # pitch 경사 보정용 (Enhanced 모드)

    def _ll_to_xy(self, lat: float, lon: float):
        if self._rtk_origin is None:
            self._rtk_origin = (lat, lon)
        lat0, lon0 = self._rtk_origin
        R = 6_371_000.0
        x = (lon - lon0) * math.pi/180 * R * math.cos(math.radians(lat0))
        y = (lat - lat0) * math.pi/180 * R
        return x, y

    def predict(self, dt: float):
        np = self.np
        x, y, th, v, w = self.x
        self.x[0] = x + v * math.cos(th) * dt
        self.x[1] = y + v * math.sin(th) * dt
        self.x[2] = _wrap(th + w * dt)
        F = np.eye(5)
        F[0,2] = -v*math.sin(th)*dt; F[0,3] = math.cos(th)*dt
        F[1,2] =  v*math.cos(th)*dt; F[1,3] = math.sin(th)*dt
        F[2,4] = dt
        self.P = F @ self.P @ F.T + self.Q

    def update_rtk(self, lat: float, lon: float, quality: int):
        """
        파라미터 2 + 3 적용 + 경사면 보정 모드(params.slope_correction):
          Off      : 보정 없음
          Standard : roll → 좌우 수평 오차 보정 (AgNav 기본)
          Enhanced : roll + pitch → 전후 오차까지 보정
        """
        np = self.np
        heading = self.x[2]
        gps_x, gps_y = self._ll_to_xy(lat, lon)
        mode = getattr(self.params, 'slope_correction', 'Standard')

        if mode != 'Off':
            # 파라미터 2: roll → 횡방향(좌우) 수평 오차
            lat_err = self.params.height_correction(self._current_roll)
            gps_x -= lat_err * math.sin(heading)
            gps_y += lat_err * math.cos(heading)
            if mode == 'Enhanced':
                # Enhanced: pitch → 종방향(전후) 오차도 보정
                lon_err = self.params.antenna_height * math.sin(self._current_pitch)
                gps_x += lon_err * math.cos(heading)
                gps_y += lon_err * math.sin(heading)

        # 파라미터 3: 안테나 → 뒤차축 레버암 보정
        rx, ry = self.params.antenna_to_axle_pos(gps_x, gps_y, heading)

        scale = 1 if quality==4 else 25 if quality==5 else 2500
        R = self.R_rtk * scale
        H = np.array([[1,0,0,0,0],[0,1,0,0,0]])
        z = np.array([rx, ry])
        resid = z - H @ self.x
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x += K @ resid
        self.P = (np.eye(5) - K @ H) @ self.P

    def update_imu(self, raw_heading: float, angular_vel: float,
                   fwd_accel: float, dt: float,
                   raw_roll: float = 0.0, raw_pitch: float = 0.0):
        """
        파라미터 5 적용: IMU 오프셋 보정 후 칼만 업데이트.
          - heading: yaw 보정값 사용
          - roll/pitch: 경사 보정에 활용 (파라미터 2와 연동)
        """
        np = self.np
        offset = self.params.imu_offset

        # 파라미터 5: IMU 오프셋 보정
        heading  = offset.correct_yaw(raw_heading)
        roll     = offset.correct_roll(raw_roll)
        pitch    = offset.correct_pitch(raw_pitch)

        # 보정된 roll/pitch 저장 (경사면 보정 연동)
        self._current_roll  = roll
        self._current_pitch = pitch

        H = np.array([[0,0,1,0,0]])
        resid = np.array([_wrap(heading - self.x[2])])
        S = H @ self.P @ H.T + self.R_hdg
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + (K @ resid.reshape(1,1)).flatten()
        self.P = (np.eye(5) - K @ H) @ self.P
        self.x[3] = self.x[3] + fwd_accel * dt
        self.x[4] = angular_vel

    def get_state(self) -> VehicleState:
        return VehicleState(x=self.x[0], y=self.x[1],
                            heading=self.x[2], speed=self.x[3],
                            angular_vel=self.x[4])

    def tune_for_receiver(self, spec: 'GnssReceiverSpec'):
        """
        INS 수신기(PA-3 등)의 heading 정확도로 EKF heading 측정노이즈 R 설정.
        PA-3 heading<0.3° → R_hdg=(0.3°)². F9P처럼 INS 없으면(0°) 변경 안 함.
        """
        if spec.heading_acc_deg > 0:
            sigma = math.radians(spec.heading_acc_deg)
            self.R_hdg = self.np.array([[sigma * sigma]])
            log.info(f"EKF heading R = ({spec.heading_acc_deg}°)² "
                     f"[{spec.name} INS 기준]")


# ═══════════════════════════════════════════════════════════════
#  Layer 3: 경로 추종
# ═══════════════════════════════════════════════════════════════

MAX_STEER_RAD = math.radians(35)

def _wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))

class PathFollower(ABC):
    @abstractmethod
    def compute_steering(self, state: VehicleState,
                         path: List[Waypoint]) -> float:
        """목표 조향각 반환 (rad)."""
        ...
    def reset(self): pass

class PurePursuit(PathFollower):
    """
    전방주시거리(Ld) 앞 목표점을 향해 조향.
    δ = atan2(2·L·sin(α), Ld)
    튜닝: lookahead_base (작을수록 민첩), speed_gain (속도비례 증가)
    """
    def __init__(self, wheelbase: float = 2.5,
                 lookahead_base: float = 3.0,
                 speed_gain: float = 0.5):
        self.L = wheelbase
        self.Ld_base = lookahead_base
        self.kv = speed_gain
        self._idx = 0

    def compute_steering(self, state: VehicleState,
                         path: List[Waypoint]) -> float:
        Ld = self.Ld_base + self.kv * max(0, state.speed)
        # lookahead 점 탐색
        for i in range(self._idx, len(path)):
            if math.hypot(path[i].x-state.x, path[i].y-state.y) >= Ld:
                self._idx = max(0, i-1)
                tp = path[i]; break
        else:
            tp = path[-1]
        alpha = _wrap(math.atan2(tp.y-state.y, tp.x-state.x) - state.heading)
        delta = math.atan2(2*self.L*math.sin(alpha), Ld)
        return max(-MAX_STEER_RAD, min(MAX_STEER_RAD, delta))

    def reset(self): self._idx = 0

class Stanley(PathFollower):
    """
    앞바퀴 기준 횡방향 오차 + heading 오차 동시 보정.
    δ = Δheading + atan2(k·e, v+ε)
    직선 정밀도 우수.
    튜닝: k_cross (클수록 빠른 복귀, 너무 크면 진동)
    """
    def __init__(self, wheelbase: float = 2.5,
                 k_cross: float = 0.8, k_soft: float = 1.0):
        self.L = wheelbase
        self.k = k_cross
        self.ks = k_soft

    def compute_steering(self, state: VehicleState,
                         path: List[Waypoint]) -> float:
        fx = state.x + self.L * math.cos(state.heading)
        fy = state.y + self.L * math.sin(state.heading)
        idx, cross, ph = self._nearest(fx, fy, path)
        h_err = _wrap(ph - state.heading)
        cross_term = math.atan2(self.k * cross, self.ks + state.speed)
        delta = h_err - cross_term
        return max(-MAX_STEER_RAD, min(MAX_STEER_RAD, delta))

    def _nearest(self, x, y, path):
        best, bi = 1e9, 0
        for i, p in enumerate(path):
            d = math.hypot(p.x-x, p.y-y)
            if d < best: best, bi = d, i
        p = path[bi]
        if bi < len(path)-1:
            ph = math.atan2(path[bi+1].y-p.y, path[bi+1].x-p.x)
        else:
            ph = math.atan2(p.y-path[bi-1].y, p.x-path[bi-1].x)
        dx, dy = x-p.x, y-p.y
        cross = math.sin(ph)*dx - math.cos(ph)*dy
        return bi, cross, ph


# ═══════════════════════════════════════════════════════════════
#  Layer 3-Extended: 작업기 기준 추종 알고리즘
#  AGMO S자 진동 해결 — CHCNAV "차량 후면 기준" 방식
# ═══════════════════════════════════════════════════════════════

class ImplementReferenced(PathFollower):
    """
    작업기 위치 기준 경로 추종 — AgNav 과부하 모드 원리 구현.

    AgNav 설계 원리 (사진 분석):
      1) 기준점을 GPS 안테나 → 작업기 위치로 변경 (S자 억제)
      2) 경로 진입(approach) / 경로 위(on-line) 게인 분리
         진입 시: approach_sensitivity × approach_aggressiveness
         추종 시: online_sensitivity × online_progress
      3) PTime On = 예측 시간 → pred_dist = ptime_on × speed
         AgNav 과부하 ptime_on=1.3: 1.5m/s에서 pred_dist=1.95m
         (작업기 관성이 클수록 예측 시간을 늘려 선제 조향)
      4) WAS Gain이 낮을수록 모터 P 게인 축소 → 부드러운 조향
    """
    def __init__(self, params: TractorParams,
                 profile:  'TuningProfile'  = None,
                 tracking: 'TrackingParams' = None):
        self.params   = params
        self.profile  = profile  or PROFILE_NORMAL
        self.tracking = tracking or TrackingParams()

    def set_profile(self, profile: 'TuningProfile'):
        self.profile = profile

    def compute_steering(self, state: VehicleState,
                         path: List[Waypoint]) -> float:
        # ── 1. 작업기 위치 계산 ──────────────────────────
        ix, iy = self.params.implement_position(
            state.x, state.y, state.heading)

        # ── 2. 작업기 위치 기준 횡방향 오차 ─────────────
        impl_xte, path_hdg = self._cross_track(ix, iy, path)

        # ── 3. heading 오차 ───────────────────────────────
        h_err = _wrap(path_hdg - state.heading)

        # ── 4. 접근/온라인 분리 게인 (AgNav TrackingParams) ──
        #   XTE > threshold → 접근 모드 (approach_sensitivity × aggressiveness)
        #   XTE ≤ threshold → 온라인 모드 (online_sensitivity × progress)
        abs_xte = abs(impl_xte)
        if abs_xte > self.tracking.online_threshold:
            k_eff = (self.profile.k_cross_algo
                     * self.tracking.approach_sensitivity
                     * self.profile.approach_aggressiveness / 100.0)
        else:
            k_eff = (self.profile.k_cross_algo
                     * self.tracking.online_sensitivity
                     * self.profile.online_progress / 100.0)

        # ── 5. 예측 제어 — PTime On × 속도 = 예측 거리 ──
        pred_xte = 0.0
        pred_dist = self.profile.ptime_on * max(0.3, state.speed)
        if state.speed > 0.1:
            px = ix + pred_dist * math.cos(state.heading)
            py = iy + pred_dist * math.sin(state.heading)
            pred_xte, _ = self._cross_track(px, py, path)

        # ── 6. 조향각 계산 ────────────────────────────────
        k_h    = self.profile.k_heading_algo
        k_soft = 0.8 + 0.5 * max(0, state.speed)     # 속도 softening
        cross_t = math.atan2(k_eff * impl_xte, k_soft)
        pred_t  = 0.12 * pred_xte                      # 예측 선제 조향

        delta = k_h * h_err - cross_t - pred_t

        # ── 7. Ackermann 보정 (파라미터 G) + WAS 클램프 ──
        delta = self.params.ackermann_correction(delta)
        limit = self.params.max_steer_rad
        return max(-limit, min(limit, delta))

    def _cross_track(self, x: float, y: float,
                     path: List[Waypoint]) -> tuple:
        best, bi = 1e9, 0
        for i, p in enumerate(path):
            d = math.hypot(p.x - x, p.y - y)
            if d < best: best, bi = d, i
        p = path[bi]
        if bi < len(path) - 1:
            ph = math.atan2(path[bi+1].y-p.y, path[bi+1].x-p.x)
        else:
            ph = math.atan2(p.y-path[bi-1].y, p.x-path[bi-1].x)
        cross = math.sin(ph)*(x-p.x) - math.cos(ph)*(y-p.y)
        return cross, ph

    def reset(self): pass


# ═══════════════════════════════════════════════════════════════
#  알고리즘 비교 요약
# ═══════════════════════════════════════════════════════════════
# │ 알고리즘             │ 기준점  │ 쟁기 S자 │ 직선 정밀도 │ 권장 작업 │
# ├─────────────────────┼────────┼─────────┼────────────┼──────────│
# │ PurePursuit         │ 안테나  │ 있음     │ 보통        │ 로터리   │
# │ Stanley             │ 앞바퀴  │ 있음     │ 우수        │ 시비/방제│
# │ ImplementReferenced │ 작업기  │ 감소     │ 우수        │ 쟁기/균평│
# └─────────────────────┴────────┴─────────┴────────────┴──────────┘

class CanInterface(ABC):
    """CAN 버스 추상화. 실제 구현은 플랫폼별로 분리."""
    @abstractmethod
    def send(self, can_id: int, data: bytes): ...
    @abstractmethod
    def recv(self) -> Optional[tuple]: ...   # (can_id, data) or None
    @abstractmethod
    def start(self): ...
    @abstractmethod
    def stop(self): ...

class ApolloCanInterface(CanInterface):
    """
    Apollo 10 Pro 내장 CAN 인터페이스 — Linux SocketCAN 기반 구현.

    Apollo 10 Pro는 Android(리눅스 커널) + 내장 CAN 포트(can0)를 제공한다.
    SocketCAN이 활성화된 환경에서는 표준 CAN_RAW 소켓으로 송수신 가능.

    ── 사용 전 준비 (Apollo ADB 셸 또는 부팅 스크립트) ──────────────
        ip link set can0 type can bitrate 500000
        ip link set up can0
        (★ bitrate는 CanSpec.CAN_BITRATE 와 일치시킬 것)

    구현 순서:
      1) python-can 가 설치돼 있으면 우선 사용 (가장 견고)
      2) 없으면 순수 socket(PF_CAN, CAN_RAW) 폴백
      3) 둘 다 실패(하드웨어/권한 없음)하면 available=False 로 두고
         예외를 던지지 않음 → PC 테스트에서는 MockCanInterface 사용 권장

    ★ Apollo 전용 CAN 드라이버가 SocketCAN이 아닌 별도 SDK라면,
      start/send/recv 내부만 해당 SDK 호출로 교체하면 된다.
    """
    # SocketCAN 프레임 포맷: <can_id(uint32) dlc(uint8) pad(3) data(8B)> = 16바이트
    _FRAME_FMT  = "=IB3x8s"
    _FRAME_SIZE = struct.calcsize(_FRAME_FMT)

    def __init__(self, channel: str = "can0",
                 bitrate: int = CanSpec.CAN_BITRATE,
                 use_python_can: bool = True):
        self.channel = channel
        self.bitrate = bitrate
        self.use_python_can = use_python_can
        self._sock = None          # 순수 socket 폴백
        self._bus  = None          # python-can Bus
        self._available = False
        log.info(f"Apollo CAN: {channel} @ {bitrate} bps (SocketCAN)")

    @property
    def available(self) -> bool:
        """CAN 버스가 실제로 열렸는지. False면 송수신 무시."""
        return self._available

    def start(self):
        # 1) python-can 우선
        if self.use_python_can:
            try:
                import can as pycan
                self._bus = pycan.interface.Bus(
                    channel=self.channel, bustype="socketcan")
                self._available = True
                log.info(f"ApolloCAN: python-can 으로 {self.channel} 열림")
                return
            except Exception as e:
                log.warning(f"ApolloCAN: python-can 사용 불가 ({e}) → socket 폴백")

        # 2) 순수 SocketCAN 폴백
        try:
            import socket
            s = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
            s.bind((self.channel,))
            s.setblocking(False)            # recv non-blocking
            self._sock = s
            self._available = True
            log.info(f"ApolloCAN: raw socket 으로 {self.channel} 열림")
        except Exception as e:
            self._available = False
            log.error(f"ApolloCAN.start() 실패: {e} "
                      f"— 하드웨어/SocketCAN 확인 (PC 테스트는 MockCanInterface 사용)")

    def stop(self):
        if self._bus is not None:
            try: self._bus.shutdown()
            except Exception: pass
            self._bus = None
        if self._sock is not None:
            try: self._sock.close()
            except Exception: pass
            self._sock = None
        self._available = False

    def send(self, can_id: int, data: bytes):
        if not self._available:
            return
        data = bytes(data)[:8]
        if self._bus is not None:
            import can as pycan
            self._bus.send(pycan.Message(
                arbitration_id=can_id, data=data, is_extended_id=False))
        elif self._sock is not None:
            frame = struct.pack(self._FRAME_FMT, can_id,
                                len(data), data.ljust(8, b"\x00"))
            try:
                self._sock.send(frame)
            except OSError as e:
                log.debug(f"ApolloCAN TX 실패: {e}")
        log.debug(f"CAN TX id=0x{can_id:03X} data={data.hex()}")

    def recv(self) -> Optional[tuple]:
        """non-blocking 수신. (can_id, data) 또는 None."""
        if not self._available:
            return None
        if self._bus is not None:
            msg = self._bus.recv(timeout=0.0)
            return (msg.arbitration_id, bytes(msg.data)) if msg else None
        if self._sock is not None:
            try:
                frame = self._sock.recv(self._FRAME_SIZE)
            except (BlockingIOError, OSError):
                return None
            if len(frame) < self._FRAME_SIZE:
                return None
            can_id, dlc, payload = struct.unpack(self._FRAME_FMT, frame)
            can_id &= 0x1FFFFFFF      # EFF/RTR/ERR 플래그 비트 제거
            return (can_id, payload[:dlc])
        return None

class MockCanInterface(CanInterface):
    """시뮬레이션/테스트용 더미 CAN. 즉시 사용 가능."""
    def __init__(self):
        self._angle = 0.0     # 시뮬레이션 현재 조향각
        self._recv_q = []
    def start(self): log.info("MockCAN 시작")
    def stop(self):  log.info("MockCAN 종료")
    def send(self, can_id: int, data: bytes):
        # 모터 명령을 받아 시뮬레이션 각도 갱신 (단순 1차 응답)
        if can_id == CanSpec.MOTOR_CMD_ID:
            raw = int.from_bytes(
                data[CanSpec.MOTOR_BYTE_CMD_HI:CanSpec.MOTOR_BYTE_CMD_LO+1],
                'big', signed=CanSpec.SENSOR_SIGNED)
            target = raw / CanSpec.MOTOR_ANGLE_SCALE
            self._angle += (target - self._angle) * 0.25   # 시뮬 응답
            # 응답 메시지 생성 (앵글센서 피드백 흉내)
            raw_fb = int(self._angle * CanSpec.SENSOR_ANGLE_SCALE)
            fb = raw_fb.to_bytes(2, 'big', signed=CanSpec.SENSOR_SIGNED)
            self._recv_q.append((CanSpec.SENSOR_ANGLE_ID, fb + bytes(6)))
    def recv(self) -> Optional[tuple]:
        return self._recv_q.pop(0) if self._recv_q else None

class MotorProtection:
    """
    AgNav '드라이버 매개변수' 기반 모터 보호.

    AgNav 확인값 (Image 11, 12):
      최대 과부하 전류: 300  → 초과 시 타이머 시작
      과부하 시간:     10s  → 지속 시 모터 정지
      연성(부드러움):  100  → smoothing filter 강도

    연성 필터:
      softness=100 → alpha=0.05 (매우 부드러움, 큰 관성)
      softness=0   → alpha=1.0  (즉각 반응)
    """
    def __init__(self, mp: MotorProtectionParams = None):
        self.mp         = mp or MotorProtectionParams()
        # 연성 → 1차 IIR 필터 계수
        self._alpha     = max(0.05, 1.0 - self.mp.softness / 100.0 * 0.95)
        self._cmd_prev  = 0.0
        self._ol_start: Optional[float] = None
        self._disabled  = False

    def smooth(self, cmd: float) -> float:
        """연성 필터: AgNav '연성(부드러움)' 파라미터 적용."""
        out = self._alpha * cmd + (1.0 - self._alpha) * self._cmd_prev
        self._cmd_prev = out
        return out

    def check_overload(self, cmd_magnitude: float) -> bool:
        """
        AgNav '최대 과부하 전류' + '과부하 시간' 기반 보호.
        cmd_magnitude: |명령값| (실제 전류 피드백 있으면 교체)
        반환: True = 과부하 → 모터 정지 필요
        """
        # 명령 → 추정 전류 (★ 실제 모터 전류 피드백으로 교체 권장)
        estimated_current = cmd_magnitude * (self.mp.max_overload_current / 50.0)
        if estimated_current > self.mp.max_overload_current:
            if self._ol_start is None:
                self._ol_start = time.time()
            elif time.time() - self._ol_start > self.mp.overload_time:
                self._disabled = True
                log.warning("모터 과부하 보호 동작 — 비활성화")
        else:
            self._ol_start = None
        return self._disabled

    def reset(self):
        self._disabled = False
        self._ol_start = None
        self._cmd_prev = 0.0

    @property
    def is_disabled(self) -> bool:
        return self._disabled


class SteeringActuator:
    """
    목표 조향각 → CAN 모터 명령.
    AgNav '드라이버 매개변수' 기반 이중 루프 + 모터 보호 + 연성 필터.

    외부 루프: 목표 조향각 → 목표 각속도 (P)
    내부 루프: 목표 각속도 → 모터 명령 (PI + 피드포워드)
    모터 보호: 과부하 전류 감지 + 연성 필터 (AgNav 확인값)

    AgNav 드라이버 매개변수 (Image 12, 14):
      모터 비례 이득:  600  → vel_kp 기준
      모터 필수 Gain: 400  → friction_ff 기준
      P Gain:         25   → pos_kp 기준
      D Gain:         80   → vel_ki 기준 (근사)
      최대 RPM:        20  → max_ang_vel = 20×2π/60 ≈ 2.09 rad/s
    """
    def __init__(self, can: CanInterface,
                 motor_params: MotorProtectionParams = None,
                 pos_kp:       float = 3.0,
                 vel_kp:       float = 2.0,
                 vel_ki:       float = 5.0,
                 friction_ff:  float = 8.0,
                 deadband_deg: float = 0.4):
        self.can = can
        self._base_pos_kp   = pos_kp
        self._base_vel_kp   = vel_kp
        self._base_ff       = friction_ff
        self.pos_kp         = pos_kp
        self.vel_kp         = vel_kp
        self.vel_ki         = vel_ki
        self.friction_ff    = friction_ff
        self.deadband       = math.radians(deadband_deg)
        self._vel_integral  = 0.0
        self._measured_angle= 0.0
        self._lock          = threading.Lock()

        # 조향각 피드백 소스:
        #   use_motor_encoder=False → WAS(SENSOR_ANGLE_ID)  (기본)
        #   use_motor_encoder=True  → 모터 하트비트 누적각 (AGMO 등 WAS 미사용)
        # 하트비트가 아직 안 들어오면 자동으로 WAS 로 폴백(시뮬/초기 안전).
        self.use_motor_encoder = False
        self.steer_ratio       = 17.5     # 조향비(AgNav 확인값): 모터/컬럼각 → 조향각
        self._motor_angle_zero = 0.0      # 직진(중앙) 기준 모터 누적각(deg) — 캘리브레이션
        self._hb               = {}       # 최신 모터 하트비트(angle/rpm/current/faults)
        self._hb_seen          = False
        self._last_raw         = None     # 직전 누적각 raw(0~65535) — wrap 언랩용
        self._motor_cont_deg   = 0.0      # 언랩된 연속 모터각(deg)
        # 실모터(Keya) 속도제어 모드: True 면 cmd_speed(SDO)로 직접 구동(무WAS).
        # 부호 규약(현장 확정): +permille=좌회전, -permille=우회전.
        self.speed_control     = False
        self.steer_permille_max = 400     # 조향 속도 상한(‰) — CanSpec.STEER_SPEED_MAX
        self._est_angle        = 0.0      # 하트비트 없을 때 명령속도 적분 추정 조향각(rad)

        # 모터 보호 (AgNav 과부하 전류/시간/연성)
        mp = motor_params or DEFAULT_MOTOR_PROTECTION
        self._mp         = mp
        self._protection = MotorProtection(mp)

        # AgNav max_rpm → max_ang_vel (rad/s)
        self.max_ang_vel = mp.max_rpm * 2 * math.pi / 60.0  # 20rpm → 2.09 rad/s

        # 활성화 시퀀스
        for cid, data in CanSpec.MOTOR_ACTIVATE_SEQ:
            self.can.send(cid, data)

    def apply_profile(self, profile: 'TuningProfile'):
        """
        TuningProfile의 WAS/U 게인으로 모터 게인 스케일링.
        AgNav: 과부하 모드 WAS=15 → pos_kp = base × (15/20) = 75%
        """
        self.pos_kp      = self._base_pos_kp * profile.was_scale
        self.vel_kp      = self._base_vel_kp * profile.u_scale
        self.friction_ff = self._base_ff
        self._vel_integral = 0.0  # 게인 변경 시 적분 리셋
        log.info(f"모터 게인: pos_kp={self.pos_kp:.2f} vel_kp={self.vel_kp:.2f} "
                 f"(WAS={profile.was_gain}, U={profile.u_gain})")

    # ── 조향각 피드백 수신 (WAS 또는 모터 하트비트) ──────────────
    def process_can_recv(self):
        """호출 루프에서 계속 호출: CAN 수신 처리."""
        msg = self.can.recv()
        while msg:
            can_id, data = msg
            # 1) 모터 하트비트 (Keya 0x07000001) — WAS 미사용 시 조향각 피드백 + 보호용
            if can_id == CanSpec.MOTOR_HEARTBEAT_ID and len(data) >= 8:
                hb = CanSpec.parse_heartbeat(data)
                if hb:
                    self._hb = hb
                    self._hb_seen = True
                    # 누적각 raw(0~65535) 언랩 → 연속 모터각(deg)
                    raw = hb.get('angle_raw', 0)
                    if self._last_raw is not None:
                        d = raw - self._last_raw
                        if d > 32768:   d -= 65536
                        elif d < -32768: d += 65536
                        self._motor_cont_deg += d
                    self._last_raw = raw
                    if self.use_motor_encoder:
                        # 연속 모터각 → 중앙기준 → 조향비로 나눠 조향각(rad)
                        motor_deg = self._motor_cont_deg - self._motor_angle_zero
                        steer_rad = math.radians(motor_deg / max(1e-6, self.steer_ratio))
                        with self._lock:
                            self._measured_angle = steer_rad
            # 2) WAS(SENSOR_ANGLE_ID) — WAS 모드, 또는 하트비트 아직 없을 때 폴백
            elif can_id == CanSpec.SENSOR_ANGLE_ID and len(data) >= 2:
                if (not self.use_motor_encoder) or (not self._hb_seen):
                    raw = int.from_bytes(
                        data[CanSpec.SENSOR_BYTE_HI:CanSpec.SENSOR_BYTE_LO+1],
                        'big', signed=CanSpec.SENSOR_SIGNED)
                    angle_rad = math.radians(
                        raw / CanSpec.SENSOR_ANGLE_SCALE
                        - CanSpec.SENSOR_ANGLE_OFFSET)
                    with self._lock:
                        self._measured_angle = angle_rad
            msg = self.can.recv()

    def set_motor_center(self):
        """현재 연속 모터각을 직진(중앙) 기준으로 캘리브레이션 (WAS 미사용 모드)."""
        self._motor_angle_zero = self._motor_cont_deg
        self._est_angle = 0.0
        with self._lock:
            self._measured_angle = 0.0
        log.info(f"모터 중앙 캘리브레이션: zero={self._motor_angle_zero:.1f}deg")

    def latest_heartbeat(self) -> dict:
        """최신 모터 하트비트(angle_deg/speed_rpm/current/faults). 없으면 빈 dict."""
        return dict(self._hb)

    def get_measured_angle(self) -> float:
        with self._lock:
            return self._measured_angle

    # ── 제어 루프 ──────────────────────────────────
    def update(self, target_angle: float,
               measured_ang_vel: float, dt: float) -> float:
        """
        target_angle: Layer 3 목표 조향각 (rad)
        measured_ang_vel: 앵글센서 미분값 (rad/s)
        dt: 제어 주기 (s)
        반환: 실제 전송된 명령값 (디버그용)
        """
        # 과부하 보호 체크
        if self._protection.is_disabled:
            return 0.0

        # 실모터(Keya) 속도제어 모드 — cmd_speed(SDO)로 직접 구동(무WAS)
        if self.speed_control:
            return self._update_speed(target_angle, dt)

        measured   = self.get_measured_angle()
        angle_err  = target_angle - measured

        if abs(angle_err) < self.deadband:
            self._vel_integral = 0.0
            self._protection.smooth(0.0)   # 필터 상태 유지
            self._send_motor(0.0)
            return 0.0

        # 외부 루프: 각도 오차 → 목표 각속도
        tgt_vel = self.pos_kp * angle_err
        tgt_vel = max(-self.max_ang_vel, min(self.max_ang_vel, tgt_vel))

        # 내부 루프: 각속도 오차 → 모터 명령 (PI)
        vel_err = tgt_vel - measured_ang_vel
        self._vel_integral = max(-20, min(20,
            self._vel_integral + vel_err * dt))

        cmd_raw = (self.vel_kp * vel_err +
                   self.vel_ki * self._vel_integral +
                   self.friction_ff * math.copysign(1, angle_err))

        # 연성 필터 (AgNav '연성' 파라미터 — softness=100 → 매우 부드러움)
        cmd = self._protection.smooth(cmd_raw)

        # 과부하 전류 보호 (AgNav '최대 과부하 전류 + 과부하 시간')
        if self._protection.check_overload(abs(cmd)):
            self.disable()
            return 0.0

        self._send_motor(cmd)
        return cmd

    def _update_speed(self, target_angle: float, dt: float) -> float:
        """
        Keya 속도제어 조향 (무WAS):
          조향각 오차 → 목표 조향각속도(P) → 모터 RPM(조향비) → permille → cmd_speed.
        피드백 = 모터 하트비트 누적각(use_motor_encoder). 부호: +permille=좌, -permille=우.

        ★ 안전: 모터 인코더 모드인데 하트비트 미수신이면 명령 금지(폭주 방지).
        """
        # 피드백 소스: 하트비트(실측) > 없으면 데드레커닝 추정각
        #   (Keya 내부 Hall 이 속도명령을 충실히 실행 → 명령 permille 적분으로 조향각 추정.
        #    실제 경로 오차는 GNSS 헤딩 외부루프가 보정.)
        if self.use_motor_encoder and not self._hb_seen:
            measured = self._est_angle
        else:
            measured = self.get_measured_angle()
        angle_err = target_angle - measured
        if abs(angle_err) < self.deadband:
            self._send_speed(0)
            return 0.0
        # 각도오차 → 목표 조향각속도(rad/s)
        rate = max(-self.max_ang_vel, min(self.max_ang_vel, self.pos_kp * angle_err))
        # 조향각속도 → 모터 RPM (조향비) → permille
        motor_rpm = rate * self.steer_ratio * 60.0 / (2 * math.pi)
        permille  = CanSpec.rpm_to_permille(motor_rpm, CanSpec.RATED_RPM)
        permille  = int(max(-self.steer_permille_max, min(self.steer_permille_max, permille)))
        # 과부하 보호(하트비트 전류 있으면 실값, 없으면 명령크기 추정)
        cur = abs(self._hb.get('current', 0)) or abs(permille) * (self._mp.max_overload_current / 400.0)
        if self._protection.check_overload(cur):
            self.disable()
            return 0.0
        # 하트비트 없으면: 실제 송신 permille 로 추정각 적분(포화 반영)
        if self.use_motor_encoder and not self._hb_seen:
            actual_rate = (permille / 1000.0 * CanSpec.RATED_RPM) * (2 * math.pi / 60.0) \
                          / max(1e-6, self.steer_ratio)
            # ±60° 로 클램프(추정 폭주 방지 — 물리 조향 한계 안)
            self._est_angle = max(-1.05, min(1.05, self._est_angle + actual_rate * dt))
            with self._lock:
                self._measured_angle = self._est_angle
        self._send_speed(permille)
        return float(permille)

    def _send_speed(self, permille: int):
        """Keya 속도 명령 전송 (워치독 1s 이내 재전송은 제어루프가 보장)."""
        self.can.send(CanSpec.MOTOR_CMD_ID, CanSpec.cmd_speed(int(permille)))

    def _send_motor(self, cmd_val: float):
        """
        ★ 명령값을 CAN 바이트로 인코딩 (제네릭 바이트레이아웃 placeholder).

        실모터(Keya KY170)는 속도제어 SDO 프로토콜이다 — 실차 배선 시
        `CanSpec.cmd_speed(CanSpec.rpm_to_permille(rpm))` 로 교체할 것.
        (cmd_val[rad/s 영역] → 조향비(17.5) → 모터 RPM → permille → cmd_speed)
        SITL 재검증 후 적용. 현재는 Mock/SITL 호환 placeholder 유지.
        """
        data = bytearray(8)
        data[CanSpec.MOTOR_BYTE_MODE] = CanSpec.MOTOR_MODE_ANGLE
        # cmd_val을 각도 단위로 변환 (현재 목표각으로 변환)
        measured = self.get_measured_angle()
        target_deg = math.degrees(measured) + cmd_val   # 임시, 규격에 따라 변경
        raw = int(target_deg * CanSpec.MOTOR_ANGLE_SCALE)
        raw = max(-32768, min(32767, raw))
        struct.pack_into('>h', data, CanSpec.MOTOR_BYTE_CMD_HI, raw)
        if CanSpec.MOTOR_BYTE_SPEED_LIM >= 0:
            data[CanSpec.MOTOR_BYTE_SPEED_LIM] = min(255, max(0, CanSpec.MOTOR_MAX_SPEED))
        if CanSpec.MOTOR_BYTE_CHECKSUM >= 0:
            data[CanSpec.MOTOR_BYTE_CHECKSUM] = sum(data[:CanSpec.MOTOR_BYTE_CHECKSUM]) & 0xFF
        self.can.send(CanSpec.MOTOR_CMD_ID, bytes(data))

    def disable(self):
        """모터 비활성화."""
        if self.speed_control:
            self.can.send(CanSpec.MOTOR_CMD_ID, CanSpec.cmd_speed(0))
            self.can.send(CanSpec.MOTOR_CMD_ID, CanSpec.CMD_DISABLE)
            self._vel_integral = 0.0
            return
        data = bytearray(8)
        data[CanSpec.MOTOR_BYTE_MODE] = CanSpec.MOTOR_MODE_DISABLE
        self.can.send(CanSpec.MOTOR_CMD_ID, bytes(data))
        self._vel_integral = 0.0


# ═══════════════════════════════════════════════════════════════
#  안전 계층 — 타협 불가
# ═══════════════════════════════════════════════════════════════

class SafetyState(Enum):
    SAFE    = auto()
    RTK_LOW = auto()    # RTK 품질 부족
    RTK_LOST= auto()    # RTK 신호 끊김
    OVERSPEED=auto()    # 속도 초과
    OVERRIDE= auto()    # 운전자 개입
    DEADMAN = auto()    # 데드맨 스위치 해제
    ESTOP   = auto()    # 비상정지

class SafetyMonitor:
    def __init__(self, max_speed_mps: float = 2.5):
        self.max_speed = max_speed_mps
        self.state = SafetyState.SAFE
        self._rtk_quality = 0
        self._last_rtk_t = 0.0
        self._deadman = False
        self._override = False
        self._estop = False
        self._prev_angle = 0.0
        self._prev_angle_t = time.time()   # 0이면 첫 dt가 매우 커져 오감지

    def update_rtk(self, quality: int):
        self._rtk_quality = quality
        self._last_rtk_t = time.time()

    def update_deadman(self, pressed: bool):
        self._deadman = pressed

    def set_estop(self):
        self._estop = True

    def check_steering_override(self, measured_angle: float):
        """앵글센서 급변으로 운전자 개입 감지."""
        now = time.time()
        dt = now - self._prev_angle_t
        if dt > 0 and 0 < dt < 0.5:
            rate = abs(measured_angle - self._prev_angle) / dt
            # 120 deg/s 이상 급변 = 운전자 조작 (실제값; Mock에서는 발동 안 됨)
            if rate > math.radians(120):
                self._override = True
        self._prev_angle = measured_angle
        self._prev_angle_t = now

    def clear_override(self):
        self._override = False

    def check(self, speed: float) -> SafetyState:
        if self._estop:
            self.state = SafetyState.ESTOP; return self.state
        if not self._deadman:
            self.state = SafetyState.DEADMAN; return self.state
        if self._override:
            self.state = SafetyState.OVERRIDE; return self.state
        if self._rtk_quality not in (4, 5):
            self.state = SafetyState.RTK_LOW; return self.state
        if time.time() - self._last_rtk_t > 1.0:
            self.state = SafetyState.RTK_LOST; return self.state
        if speed > self.max_speed:
            self.state = SafetyState.OVERSPEED; return self.state
        self.state = SafetyState.SAFE
        return self.state

    @property
    def is_safe(self) -> bool:
        return self.state == SafetyState.SAFE


# ═══════════════════════════════════════════════════════════════
#  GNSS 소스 중재 — PA-3(주) + F9P(백업) 이중화
# ═══════════════════════════════════════════════════════════════

class GnssArbiter:
    """
    다중 GNSS 소스 중재기 (이중화/페일오버).

    정책:
      1순위) priority 순서대로 "사용 가능"(fresh + 품질 4/5) 한 소스를 active
      2순위) 모두 사용 불가면, fresh 한 최우선 소스를 active 로
             (품질은 그대로 전달 → SafetyMonitor 가 RTK_LOW 로 거부)
      3순위) 둘 다 stale 이면 active=None (SafetyMonitor 가 RTK_LOST 처리)

    submit() 은 들어온 소스가 현재 active 일 때만 fix 를 반환 →
    EKF 는 active 소스 한 곳에서만 갱신되어 이중 카운팅이 없다.
    PA-3 가 끊기면 다음 F9P submit 에서 자동 페일오버, 복구되면 다시 PA-3.

    예 (PA-3 주 / F9P 백업):
        arb = GnssArbiter(priority=("pa3", "f9p"))
    """
    def __init__(self, priority=("pa3", "f9p"),
                 stale_timeout: float = 1.0,
                 good_quality=(4, 5)):
        self.priority = list(priority)
        self.stale_timeout = stale_timeout
        self.good_quality = set(good_quality)
        self._last: dict = {}              # source -> (lat, lon, quality, t)
        self.active: Optional[str] = None

    @property
    def primary(self) -> str:
        return self.priority[0]

    def _fresh(self, src: str, now: float) -> bool:
        d = self._last.get(src)
        return d is not None and (now - d[3]) <= self.stale_timeout

    def _usable(self, src: str, now: float) -> bool:
        d = self._last.get(src)
        return self._fresh(src, now) and d[2] in self.good_quality

    def _select(self, now: float) -> Optional[str]:
        for s in self.priority:                 # 1순위: fresh + 품질 양호
            if self._usable(s, now):
                return s
        for s in self.priority:                 # 2순위: fresh (품질 무관)
            if self._fresh(s, now):
                return s
        return None                             # 모두 stale

    def submit(self, source: str, lat: float, lon: float,
               quality: int) -> Optional[tuple]:
        """소스 갱신. 이 소스가 active 면 (lat,lon,quality,source) 반환, 아니면 None."""
        now = time.time()
        if source not in self.priority:         # 미등록 소스 → 최하 우선
            self.priority.append(source)
        self._last[source] = (lat, lon, quality, now)
        new_active = self._select(now)
        if new_active != self.active:
            log.info(f"GNSS 소스 전환: {self.active} → {new_active}")
            self.active = new_active
        if source == self.active:
            return (lat, lon, quality, source)
        return None

    def status(self) -> dict:
        now = time.time()
        return {s: {"fresh": self._fresh(s, now),
                    "quality": self._last.get(s, (0, 0, 0, 0))[2],
                    "active": s == self.active}
                for s in self.priority}


# ═══════════════════════════════════════════════════════════════
#  메인 시스템 통합
# ═══════════════════════════════════════════════════════════════

class AutoSteerSystem:
    """
    4계층 통합 + AgNav 3모드 프로파일 시스템.

    사용 예:
        params = KUBOTA_MR1157  # 또는 TractorParams(...)
        can    = ApolloCanInterface()
        sys    = AutoSteerSystem(can, params=params, algo="implement")

        # 작업별 프로파일 전환 (AgNav 3모드)
        sys.set_profile("heavy")    # 쟁기/균평: WAS↓ k_cross↑ PTime 1.3s
        sys.set_profile("normal")   # 로터리/방제
        sys.set_profile("sand")     # 사질 토양

        # 경사면 보정 모드
        sys.set_slope_correction("Enhanced")   # 급경사지
        sys.set_slope_correction("Standard")   # 일반 (기본)

        # 50Hz 루프:
        sys.on_rtk(lat, lon, quality)
        sys.on_imu(heading, ang_vel, accel, dt, raw_roll, raw_pitch)
        result = sys.control_step(dt)
        # result['profile'] → 현재 프로파일명
    """
    def __init__(self, can: CanInterface,
                 params:       TractorParams        = None,
                 algo:         str                  = "pure_pursuit",
                 motor_params: MotorProtectionParams = None,
                 max_speed_mps: float               = 2.5,
                 vendor:       str                  = None):
        self.params    = params or KUBOTA_MR1157
        self.estimator = StateEstimator(self.params)
        self.actuator  = SteeringActuator(can, motor_params=motor_params)
        self.safety    = SafetyMonitor(max_speed_mps)
        self.path: List[Waypoint] = []
        self._profile  = PROFILE_NORMAL
        self.tracking  = TrackingParams()
        self._engaged  = False
        self._target_idx = 0
        self._prev_angle   = 0.0
        self._prev_angle_t = time.time()
        self.on_disengage: Optional[Callable[[str], None]] = None

        # GNSS 이중화: PA-3(주) + F9P(백업)
        self.gnss = GnssArbiter(priority=("pa3", "f9p"))
        self._active_gnss: Optional[str] = None

        # 벤더(제조사) 프로파일 — 미선택 시 기존 동작(검증된 것으로 간주)
        self.vendor = None
        self.motor_verified = True

        self.set_algorithm(algo, self.params.wheelbase)

        if vendor:
            self.select_vendor(vendor)

    def select_vendor(self, key: str):
        """
        제조사 프로파일 활성화 (앱 시작 시 선택).
        해당 벤더의 모터 CAN(CanSpec) + GNSS 우선순위 + EKF 튜닝 + 기본 알고리즘을 적용.

        can_verified=False 인 벤더(모터 프로토콜 미확정)는 안전을 위해
        조향 출력을 비활성한다(engage 거부). GNSS/상태표시는 계속 동작.
        """
        import vendor_profiles as vp
        p = vp.apply_vendor(key)
        self.vendor = p
        self.motor_verified = p.can_verified
        # WAS 미사용 벤더(AGMO 등)는 모터 하트비트 누적각으로 조향각 피드백
        self.actuator.use_motor_encoder = not getattr(p, "uses_was", False)

        # GNSS 소스 우선순위 재구성 + 수신기 기반 EKF 튜닝
        self.gnss = GnssArbiter(priority=p.gnss_priority)
        self._active_gnss = None
        self.estimator.tune_for_receiver(p.gnss_primary)

        # 기본 알고리즘 적용
        self.set_algorithm(p.default_algo, self.params.wheelbase)

        if not p.can_verified:
            self._engaged = False
            log.warning(f"[{p.display_name}] 모터 CAN 프로토콜 ★미확정 — "
                        f"조향 출력 비활성(engage 거부). 문서 입수 후 "
                        f"vendor_profiles 의 canspec/can_verified 갱신 필요.")
        log.info(f"벤더 선택: {p.display_name} "
                 f"(모터확정={p.can_verified}, GNSS={p.gnss_primary.name}, "
                 f"우선순위={p.gnss_priority})")
        return p

    def set_algorithm(self, algo: str, wheelbase: float):
        """
        algo:
          "pure_pursuit" — 로터리/방제 등 일반 작업
          "stanley"      — 직선 정밀 중요
          "implement"    — 쟁기/균평: 작업기 기준 제어 + 3모드 프로파일 적용
        """
        if algo == "stanley":
            self.follower = Stanley(wheelbase)
        elif algo == "implement":
            self.follower = ImplementReferenced(
                self.params,
                profile=self._profile,
                tracking=self.tracking)
        else:
            self.follower = PurePursuit(wheelbase)
        log.info(f"알고리즘: {algo}, 휠베이스: {wheelbase:.2f}m, "
                 f"최대WAS: {self.params.max_was_deg:.0f}°, "
                 f"프로파일: {self._profile.name}")

    def set_profile(self, profile_name_or_obj):
        """
        AgNav 3모드 프로파일 전환.
        profile_name_or_obj: "normal"/"heavy"/"sand" 또는 TuningProfile 인스턴스

        전환 시 자동으로:
          1) 추종 알고리즘 게인 갱신
          2) 모터 P/U 게인 스케일링 (WAS/U 감도)
        """
        if isinstance(profile_name_or_obj, str):
            profile = PROFILES.get(profile_name_or_obj, PROFILE_NORMAL)
        else:
            profile = profile_name_or_obj
        self._profile = profile
        # 추종기 게인 갱신
        if hasattr(self.follower, 'set_profile'):
            self.follower.set_profile(profile)
        # 모터 게인 스케일링
        self.actuator.apply_profile(profile)
        log.info(f"프로파일 전환 → {profile.name}")

    def set_slope_correction(self, mode: str):
        """
        경사면 보정 모드 전환.
        mode: "Off" / "Standard" / "Enhanced"
        """
        self.params.slope_correction = mode
        log.info(f"경사면 보정 모드: {mode}")

    def set_path(self, strategy: PathStrategy):
        self.path = strategy.generate()
        self.follower.reset()
        self._target_idx = 0
        log.info(f"경로 설정: {len(self.path)} 웨이포인트")

    # ── 센서 입력 콜백 ──────────────────────────────
    def on_rtk(self, lat: float, lon: float, quality: int,
               source: Optional[str] = None):
        """
        GNSS fix 수신 + 다중 소스(PA-3/F9P) 이중화 중재.
          - source=None → 기본(주) 소스로 간주
          - GnssArbiter 가 active 소스 한 곳만 EKF/안전에 반영 (페일오버 자동)
        파라미터 2, 3 보정은 StateEstimator 내부에서 자동 적용.
        """
        src = source or self.gnss.primary
        sel = self.gnss.submit(src, lat, lon, quality)
        if sel is None:                       # active 소스가 아니면 무시
            return
        s_lat, s_lon, s_quality, s_src = sel
        self._active_gnss = s_src
        self.estimator.update_rtk(s_lat, s_lon, s_quality)
        self.safety.update_rtk(s_quality)

    def rtk_callback(self, source: str):
        """
        외부 GNSS 클라이언트(F9pUsbClient / ChcnavPa3SerialClient)를
        특정 source 라벨로 on_rtk 에 연결하는 3-인자 콜백 생성.
            pa3 = ChcnavPa3SerialClient(on_rtk=sys.rtk_callback("pa3"))
            f9p = F9pUsbClient(on_rtk=sys.rtk_callback("f9p"))
        """
        return lambda lat, lon, quality: self.on_rtk(lat, lon, quality,
                                                     source=source)

    def on_imu(self, raw_heading: float, ang_vel: float,
               fwd_accel: float, dt: float,
               raw_roll: float = 0.0, raw_pitch: float = 0.0):
        """
        파라미터 5(IMU 오프셋) 보정은 StateEstimator 내부에서 자동 적용.
        raw_roll, raw_pitch를 전달하면 경사 보정(파라미터 2)도 연동됨.
        """
        self.estimator.predict(dt)
        self.estimator.update_imu(raw_heading, ang_vel, fwd_accel, dt,
                                   raw_roll, raw_pitch)

    def get_implement_position(self) -> tuple:
        """
        파라미터 4: 현재 작업기 위치 반환.
        레벨러 z축 기록, 작업영역 표시에 사용.
        """
        state = self.estimator.get_state()
        origin = self.estimator._rtk_origin
        if origin is None:
            return (0.0, 0.0)
        # 로컬 좌표계에서 작업기 위치
        impl_x, impl_y = self.params.implement_position(
            state.x, state.y, state.heading)
        return (impl_x, impl_y)

    # ── 제어 스텝 (50Hz 호출) ───────────────────────
    def control_step(self, dt: float) -> dict:
        """
        반환값:
          engaged, safety_state, target_angle, measured_angle,
          motor_cmd, xte_m, progress, waypoint_section
        """
        self.actuator.process_can_recv()
        state = self.estimator.get_state()

        # 앵글센서 각속도 계산 (미분)
        measured = self.actuator.get_measured_angle()
        now = time.time()
        elapsed = now - self._prev_angle_t
        # 첫 스텝 또는 dt가 너무 작으면 각속도 0으로
        ang_vel = ((measured - self._prev_angle) / elapsed
                   if 0.001 < elapsed < 0.5 else 0.0)
        self._prev_angle = measured
        self._prev_angle_t = now
        self.safety.check_steering_override(measured)

        # 안전 확인
        safety = self.safety.check(state.speed)
        if not self.safety.is_safe:
            self._disengage(str(safety.name))
            self.actuator.disable()
            return self._status(state, 0.0, measured, 0.0, safety)

        if not self._engaged or not self.path:
            return self._status(state, 0.0, measured, 0.0, safety)

        # 목표 인덱스 진행
        while self._target_idx < len(self.path)-1:
            if math.hypot(self.path[self._target_idx].x - state.x,
                          self.path[self._target_idx].y - state.y) < 0.8:
                self._target_idx += 1
            else: break

        # 경로 완주
        if self._target_idx >= len(self.path)-1:
            last = self.path[-1]
            if math.hypot(last.x-state.x, last.y-state.y) < 0.8:
                self._disengage("경로 완료")
                return self._status(state, 0.0, measured, 0.0, safety)

        # 추종 알고리즘
        target_angle = self.follower.compute_steering(state, self.path)

        # 작업기 제어 신호 (레벨러 연동용)
        cur_wp = self.path[self._target_idx]

        # 모터 제어
        cmd = self.actuator.update(target_angle, ang_vel, dt)

        xte = self._compute_xte(state)
        return self._status(state, target_angle, measured, cmd, safety,
                            xte=xte, section=cur_wp.section,
                            implement_down=cur_wp.implement_down)

    def _compute_xte(self, state: VehicleState) -> float:
        """횡방향 오차 (m)."""
        idx = self._target_idx
        p = self.path[idx]
        if idx < len(self.path)-1:
            ph = math.atan2(self.path[idx+1].y-p.y, self.path[idx+1].x-p.x)
        else:
            ph = math.atan2(p.y-self.path[idx-1].y, p.x-self.path[idx-1].x)
        dx = state.x - p.x; dy = state.y - p.y
        return math.sin(ph)*dx - math.cos(ph)*dy

    def _status(self, state, target, measured, cmd, safety,
                xte=0.0, section="work", implement_down=True) -> dict:
        prog = (self._target_idx / max(1, len(self.path)-1)) * 100
        return {
            "engaged": self._engaged,
            "safety": safety.name,
            "profile": self._profile.name,
            "vendor": self.vendor.key if self.vendor else None,
            "vendor_name": self.vendor.display_name if self.vendor else None,
            "motor_verified": self.motor_verified,
            "active_gnss": self._active_gnss,
            "slope_correction": self.params.slope_correction,
            "target_angle_deg": math.degrees(target),
            "measured_angle_deg": math.degrees(measured),
            "motor_cmd": cmd,
            "xte_cm": xte * 100,
            "progress_pct": prog,
            "section": section,
            "implement_down": implement_down,
            "speed_mps": state.speed,
            "pos": (state.x, state.y),
            "heading_deg": math.degrees(state.heading),
        }

    # ── 모드 전환 ───────────────────────────────────
    def engage(self) -> bool:
        if not self.motor_verified:
            vn = self.vendor.display_name if self.vendor else "?"
            log.warning(f"engage 거부: [{vn}] 모터 CAN 프로토콜 미확정")
            return False
        if not self.safety.is_safe:
            log.warning(f"engage 거부: {self.safety.state.name}")
            return False
        if not self.path:
            log.warning("engage 거부: 경로 없음")
            return False
        self._engaged = True
        self.follower.reset()
        log.info("자율조향 활성화")
        return True

    def _disengage(self, reason: str = ""):
        if self._engaged:
            self._engaged = False
            self.actuator.disable()
            log.info(f"자율조향 해제: {reason}")
            if self.on_disengage:
                self.on_disengage(reason)

    def disengage(self):
        self._disengage("수동 해제")

    def emergency_stop(self):
        self.safety.set_estop()
        self._disengage("비상정지")


# ═══════════════════════════════════════════════════════════════
#  사용 예시 (테스트)
# ═══════════════════════════════════════════════════════════════
if __name__ == "__main__":
    import random
    logging.basicConfig(level=logging.WARNING,
                        format="%(levelname)s %(name)s: %(message)s")

    print("=" * 78)
    print("AgNav 3모드 시스템 + 경사면 보정 + 접근/온라인 분리 — 통합 테스트")
    print("=" * 78)
    print()

    # Kubota MR1157 — 사진 기반 파라미터
    params = TractorParams(
        wheelbase         = 2.47,        # ★ 실측 필요
        antenna_height    = 2.73,        # 사진 E
        antenna_to_axle   = -0.50,       # 사진 D, 뒤쪽 → 음수 (★ 실측 필요)
        antenna_to_impl   = 1.20,        # ★ 실측 필요
        hitch_to_impl     = 1.00,        # 사진 B1
        front_track_width = 1.56,        # 사진 G
        max_was_deg       = 25.0,        # 사진 최대 WAS
        slope_correction  = "Standard",  # AgNav 기본
        imu_offset = ImuOffset(roll=0.0, pitch=0.0, yaw=0.0),
    )

    # AgNav '드라이버 매개변수' (Image 11, 12, 14)
    motor_params = MotorProtectionParams(
        steering_ratio       = 17.5,
        handle_deadzone      = 20.0,
        max_overload_current = 300.0,
        overload_time        = 10.0,
        motor_p_gain         = 600.0,
        motor_necessary_gain = 400.0,
        max_rpm              = 20.0,
        softness             = 100.0,
    )

    print("AgNav 확인 파라미터:")
    for name, val in [
        ("A  휠베이스",      f"{params.wheelbase} m"),
        ("D  안테나↔차축",   f"{params.antenna_to_axle} m (뒤쪽=음수)"),
        ("E  안테나 높이",   f"{params.antenna_height} m"),
        ("B1 히치↔작업기",   f"{params.hitch_to_impl} m"),
        ("G  전륜 폭",       f"{params.front_track_width} m"),
        ("   최대 WAS",      f"{params.max_was_deg}°"),
        ("   경사면 보정",   params.slope_correction),
        ("   조향비",        f"{motor_params.steering_ratio}"),
        ("   과부하 전류",   f"{motor_params.max_overload_current} (내부단위)"),
        ("   과부하 시간",   f"{motor_params.overload_time}s"),
        ("   연성",          f"{motor_params.softness}%"),
        ("   최대 RPM",      f"{motor_params.max_rpm}"),
    ]:
        print(f"  {name:<18} {val}")
    print()

    # ── 테스트 함수 ────────────────────────────────────────────
    def run_test(algo: str, profile_name: str,
                 slope_mode: str = "Standard",
                 disturbance: float = 0.003) -> dict:
        can = MockCanInterface()
        can.start()
        sys_ = AutoSteerSystem(can, params=params, algo=algo,
                               motor_params=motor_params)
        sys_.set_profile(profile_name)
        sys_.set_slope_correction(slope_mode)
        sys_.safety.update_deadman(True)
        sys_.safety.update_rtk(4)
        sys_.safety.check_steering_override = lambda _: None

        sys_.set_path(ABLineStrategy(
            point_a=(0,0), point_b=(0,40),
            implement_width=3.0, num_passes=2, speed=1.5))
        sys_.on_rtk(37.0000, 127.0000, 4)
        sys_.engage()

        DT  = 0.02
        lat = 37.0000
        xte_list, impl_xte_list = [], []

        for step in range(300):
            dist = disturbance * math.sin(step * 0.15) if 20 < step < 280 else 0.0
            raw_heading = math.pi/2 + random.gauss(0, 0.002) + dist
            raw_roll    = random.gauss(0, 0.001)   # 약한 경사 노이즈
            sys_.on_imu(raw_heading, random.gauss(0, 0.002),
                        random.gauss(0.5, 0.05), DT,
                        raw_roll=raw_roll, raw_pitch=0.0)
            lat += 1.5 * DT / 111320
            sys_.on_rtk(lat, 127.0000, 4)
            sys_.safety.clear_override()
            result = sys_.control_step(DT)

            # 작업기 XTE
            ix, iy = sys_.get_implement_position()
            pi_ = sys_._target_idx
            if pi_ < len(sys_.path):
                p = sys_.path[pi_]
                n = pi_+1 if pi_+1 < len(sys_.path) else pi_-1
                ph_ = math.atan2(sys_.path[n].y-p.y, sys_.path[n].x-p.x)
                impl_e = (math.sin(ph_)*(ix-p.x) - math.cos(ph_)*(iy-p.y))*100
                impl_xte_list.append(abs(impl_e))
            xte_list.append(abs(result['xte_cm']))
            if not result['engaged']:
                break

        can.stop()
        rms  = lambda lst: (sum(x*x for x in lst)/max(1,len(lst)))**.5
        mx   = lambda lst: max(lst) if lst else 0
        return {'a_rms': rms(xte_list), 'a_max': mx(xte_list),
                'i_rms': rms(impl_xte_list), 'i_max': mx(impl_xte_list)}

    # ── 1. 3가지 알고리즘 비교 ─────────────────────────────────
    print("─" * 78)
    print("▶ 1. 알고리즘 + 프로파일 조합 비교  (외란=sin파 ±0.003 rad)")
    print("─" * 78)
    hdr = f"{'조합':<38}  {'안테나RMS':>9}  {'안테나MAX':>9}  {'작업기RMS':>9}  {'작업기MAX':>9}"
    print(hdr)
    print("─" * 78)

    combos = [
        ("pure_pursuit", "normal", "Standard", "Pure Pursuit + 일반"),
        ("stanley",      "normal", "Standard", "Stanley + 일반"),
        ("implement",    "normal", "Standard", "Implement + 일반 프로파일"),
        ("implement",    "heavy",  "Standard", "Implement + 과부하 프로파일 ★"),
        ("implement",    "sand",   "Standard", "Implement + 모래토양 프로파일"),
    ]
    for algo, prof, slope, label in combos:
        r = run_test(algo, prof, slope)
        print(f"  {label:<36}  {r['a_rms']:>8.1f}cm  {r['a_max']:>8.1f}cm"
              f"  {r['i_rms']:>8.1f}cm  {r['i_max']:>8.1f}cm")

    # ── 2. 경사면 보정 모드 비교 ───────────────────────────────
    print()
    print("─" * 78)
    print("▶ 2. 경사면 보정 모드 비교  (implement + heavy, roll 노이즈 ±0.01 rad)")
    print("─" * 78)
    print(hdr)
    print("─" * 78)
    for mode in ["Off", "Standard", "Enhanced"]:
        r = run_test("implement", "heavy", mode, disturbance=0.001)
        print(f"  경사보정: {mode:<28}  {r['a_rms']:>8.1f}cm  {r['a_max']:>8.1f}cm"
              f"  {r['i_rms']:>8.1f}cm  {r['i_max']:>8.1f}cm")

    # ── 3. 프로파일 실시간 전환 시연 ──────────────────────────
    print()
    print("─" * 78)
    print("▶ 3. 프로파일 실시간 전환 시연 (작업 중 일반→과부하→모래토양)")
    print("─" * 78)
    can3 = MockCanInterface(); can3.start()
    sys3 = AutoSteerSystem(can3, params=params, algo="implement",
                           motor_params=motor_params)
    sys3.safety.update_deadman(True)
    sys3.safety.update_rtk(4)
    sys3.safety.check_steering_override = lambda _: None
    sys3.set_path(ABLineStrategy((0,0),(0,40), 3.0, 2, 1.5))
    sys3.on_rtk(37.0000, 127.0000, 4)
    sys3.engage()
    lat3 = 37.0000

    print(f"  {'step':>4}  {'프로파일':<28}  {'XTE':>7}  {'목표각':>7}  {'경사보정'}")
    print(f"  {'':>4}  {'──────────────────────────':>28}  {'───':>7}  {'──':>7}")
    for step in range(120):
        # 전환 시뮬
        if step == 0:   sys3.set_profile("normal")
        elif step == 40: sys3.set_profile("heavy")
        elif step == 80: sys3.set_profile("sand"); sys3.set_slope_correction("Enhanced")

        sys3.on_imu(math.pi/2 + random.gauss(0, 0.002),
                    random.gauss(0, 0.002), random.gauss(0.5, 0.05), 0.02,
                    raw_roll=random.gauss(0, 0.003))
        lat3 += 1.5 * 0.02 / 111320
        sys3.on_rtk(lat3, 127.0000, 4)
        sys3.safety.clear_override()
        r3 = sys3.control_step(0.02)

        if step % 20 == 0:
            print(f"  {step:>4}  {r3['profile']:<28}  "
                  f"{r3['xte_cm']:>+6.1f}cm  "
                  f"{r3['target_angle_deg']:>+6.1f}°  "
                  f"{r3['slope_correction']}")
        if not r3['engaged']: break
    can3.stop()

    print()
    print("─" * 78)
    print("AgNav 3모드 프로파일 특성 요약:")
    for p in [PROFILE_NORMAL, PROFILE_HEAVY, PROFILE_SAND]:
        print(f"  {p.name}")
        print(f"    WAS {p.was_gain:.0f} → 모터 P게인 ×{p.was_scale:.2f}  "
              f"| k_cross {p.k_cross:.0f}({p.k_cross_algo:.2f})  "
              f"| PTime {p.ptime_on:.1f}s  "
              f"| 진입 {p.approach_aggressiveness:.0f}%  온라인 {p.online_progress:.0f}%")
    print()
    print("다음 단계:")
    print("  1. CanSpec.* 모터 문서로 채우기")
    print("  2. ApolloCanInterface 구현")
    print("  3. 파라미터 A(휠베이스), D(안테나↔차축) 실측")
    print("  4. 빈 농지에서 일반 모드 → 과부하 모드 순서로 검증")
