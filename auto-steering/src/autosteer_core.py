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
    ★ 이 섹션의 값들을 본인 모터 프로그램 문서로 교체하세요.

    확인해야 할 항목:
      1) 통신 속도: 250kbps / 500kbps / 1Mbps
      2) 모터 활성화 시퀀스 (전원 ON 후 보내야 하는 핸드셰이크)
      3) 모터 명령 CAN ID + 데이터 바이트 구조
      4) 앵글센서 CAN ID + 각도값 인코딩
    """

    # ── 버스 설정 ─────────────────────────────────────
    CAN_BITRATE = 500_000           # ★ 250000 / 500000 / 1000000

    # ── 모터 명령 (태블릿 → 모터) ────────────────────
    MOTOR_CMD_ID       = 0x201      # ★ 모터 명령 CAN ID
    MOTOR_CMD_PERIOD   = 0.020      # 명령 주기 50Hz (변경 가능)

    # 데이터 바이트 레이아웃 ★
    # 현재 가정: [0]=모드, [1][2]=목표각(int16, 0.1deg), [3]=속도제한, [4]=체크섬
    # 실제 문서와 다를 수 있음 — 반드시 교체
    MOTOR_BYTE_MODE       = 0       # ★ 제어 모드 바이트 인덱스
    MOTOR_BYTE_CMD_HI     = 1       # ★ 목표값 상위 바이트
    MOTOR_BYTE_CMD_LO     = 2       # ★ 목표값 하위 바이트
    MOTOR_BYTE_SPEED_LIM  = 3       # ★ 속도 제한 바이트
    MOTOR_BYTE_CHECKSUM   = 4       # ★ 체크섬 바이트 (-1이면 없음)

    MOTOR_MODE_DISABLE    = 0x00    # ★ 모터 비활성화 모드값
    MOTOR_MODE_ANGLE      = 0x01    # ★ 각도 제어 모드값
    MOTOR_MODE_TORQUE     = 0x02    # ★ 토크 제어 모드값 (없으면 ANGLE만 사용)
    MOTOR_ANGLE_SCALE     = 10.0    # ★ 각도→CAN값 변환 (예: 10 → 0.1도 단위)
    MOTOR_MAX_SPEED       = 50      # ★ 속도 제한 기본값 (문서 단위로)

    # 활성화 시퀀스 ★ (없으면 빈 리스트)
    MOTOR_ACTIVATE_SEQ: List[tuple] = [
        # (CAN_ID, bytes) 형식
        # (0x200, bytes([0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])),
    ]

    # ── 앵글센서 피드백 (앵글센서 → 태블릿) ─────────
    SENSOR_ANGLE_ID    = 0x301      # ★ 앵글센서 CAN ID
    SENSOR_BYTE_HI     = 0          # ★ 각도 상위 바이트
    SENSOR_BYTE_LO     = 1          # ★ 각도 하위 바이트
    SENSOR_ANGLE_SCALE = 10.0       # ★ CAN값→각도 변환 (예: 10 → 0.1도 단위)
    SENSOR_ANGLE_OFFSET= 0.0        # ★ 영점 오프셋 (캘리브레이션 후 설정)
    SENSOR_SIGNED      = True       # ★ 부호 있는 정수인지 (보통 True)


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
    Apollo 10 Pro 내장 CAN 인터페이스.
    실제 구현은 Apollo CAN SDK / android-can 라이브러리 활용.
    ★ 본인 Apollo SDK 문서에 맞게 구현 필요.
    """
    def __init__(self, channel: str = "can0",
                 bitrate: int = CanSpec.CAN_BITRATE):
        self.channel = channel
        self.bitrate = bitrate
        self._sock = None
        log.info(f"Apollo CAN: {channel} @ {bitrate} bps (★미구현, 교체 필요)")

    def start(self):
        # ★ Apollo CAN SDK 초기화
        # 예: self._sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        #     self._sock.bind((self.channel,))
        log.warning("ApolloCanInterface.start(): ★ 실제 SDK로 교체 필요")

    def stop(self):
        if self._sock:
            self._sock.close()

    def send(self, can_id: int, data: bytes):
        # ★ 실제 CAN 프레임 전송
        log.debug(f"CAN TX id=0x{can_id:03X} data={data.hex()}")

    def recv(self) -> Optional[tuple]:
        # ★ 실제 CAN 프레임 수신
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

    # ── 앵글센서 수신 ──────────────────────────────
    def process_can_recv(self):
        """호출 루프에서 계속 호출: CAN 수신 처리."""
        msg = self.can.recv()
        while msg:
            can_id, data = msg
            if can_id == CanSpec.SENSOR_ANGLE_ID and len(data) >= 2:
                raw = int.from_bytes(
                    data[CanSpec.SENSOR_BYTE_HI:CanSpec.SENSOR_BYTE_LO+1],
                    'big', signed=CanSpec.SENSOR_SIGNED)
                angle_rad = math.radians(
                    raw / CanSpec.SENSOR_ANGLE_SCALE
                    - CanSpec.SENSOR_ANGLE_OFFSET)
                with self._lock:
                    self._measured_angle = angle_rad
            msg = self.can.recv()

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

    def _send_motor(self, cmd_val: float):
        """★ 명령값을 CAN 바이트로 인코딩. 규격 확인 후 수정 필요."""
        data = bytearray(8)
        data[CanSpec.MOTOR_BYTE_MODE] = CanSpec.MOTOR_MODE_ANGLE
        # cmd_val을 각도 단위로 변환 (현재 목표각으로 변환)
        measured = self.get_measured_angle()
        target_deg = math.degrees(measured) + cmd_val   # 임시, 규격에 따라 변경
        raw = int(target_deg * CanSpec.MOTOR_ANGLE_SCALE)
        raw = max(-32768, min(32767, raw))
        struct.pack_into('>h', data, CanSpec.MOTOR_BYTE_CMD_HI, raw)
        data[CanSpec.MOTOR_BYTE_SPEED_LIM] = CanSpec.MOTOR_MAX_SPEED
        if CanSpec.MOTOR_BYTE_CHECKSUM >= 0:
            data[CanSpec.MOTOR_BYTE_CHECKSUM] = sum(data[:CanSpec.MOTOR_BYTE_CHECKSUM]) & 0xFF
        self.can.send(CanSpec.MOTOR_CMD_ID, bytes(data))

    def disable(self):
        """모터 비활성화."""
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
                 max_speed_mps: float               = 2.5):
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

        self.set_algorithm(algo, self.params.wheelbase)

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
    def on_rtk(self, lat: float, lon: float, quality: int):
        """파라미터 2, 3 보정은 StateEstimator 내부에서 자동 적용."""
        self.estimator.update_rtk(lat, lon, quality)
        self.safety.update_rtk(quality)

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
