"""
vendor_profiles.py — 제조사(벤더) 프로파일 레지스트리

컨셉: 이 앱은 CHCNAV / AGMO / FJDynamics 의 태블릿에 설치만 하면 그들의
하드웨어(조향모터 + GNSS + 앵글센서)를 그대로 사용한다. 앱 시작 시 제조사를
선택하면 해당 하드웨어 스택에 맞는 설정이 한 번에 활성화된다.

하나의 VendorProfile 이 묶는 것:
  - 모터 CAN 프로토콜 (CanSpec 스칼라값 dict — field_config.apply_canspec 로 반영)
  - GNSS 수신기 스펙 (주/백업) + GnssArbiter 우선순위
  - 기본 추종 알고리즘
  - can_verified: 모터 CAN 프로토콜 확정 여부
        True  → 실제 조향 출력 허용
        False → ★ 프로토콜 미확정. 안전을 위해 조향 출력 비활성(엔게이지 거부),
                GNSS/UI 는 동작. 문서/버스 캡처로 canspec 채운 뒤 True 로.

현재 확정 상태:
  - AGMO  : Keya KY170 매뉴얼 V2.4 (250k 속도제어) 확정 → can_verified=True
  - CHCNAV: PA-3 GNSS+INS 확정 / 모터 CAN ★미확정 → can_verified=False
  - FJD   : AT2 GNSS+INS / 모터 CAN ★미확정 → can_verified=False

사용:
    import vendor_profiles as vp
    vp.list_vendors()           # UI 용 목록
    p = vp.apply_vendor("agmo") # CanSpec 활성화 + 프로파일 반환
"""
from __future__ import annotations
from dataclasses import dataclass, field
from typing import Optional, List, Dict

from autosteer_core import GnssReceiverSpec, CHCNAV_PA3, UBLOX_F9P


# ═══════════════════════════════════════════════════════════════
#  벤더 GNSS 스펙 (CHCNAV_PA3 / UBLOX_F9P 는 autosteer_core 재사용)
# ═══════════════════════════════════════════════════════════════

# AGMO ver1 — 듀얼안테나 + IMU. ★ 물리구성 확인(오너 제공 사진):
#   가로 막대형 케이스, 양쪽 사이드(좌/우)에 안테나 2개 = **가로(횡) 베이스라인**,
#   케이스 **내부 중앙에 IMU**(별도 박스 아님). heading=좌우 베이스라인,
#   각속도/자세=중앙 IMU. → 가로 베이스라인이므로 두 안테나 높이차(RELPOSNED relPosD)
#   는 **roll(횡경사)** 를 준다(세로였다면 pitch). 방법4 on_heading_meas 의 roll 유도와 일치.
#   ★ base=좌측·rover=우측(진행방향 기준) → relPosHeading=차체+90°(코드: dual_baseline_offset_deg=90),
#     relPosD>0=우측하강=roll+(dual_roll_sign=+1). 부호 최종확인=실차 틸트테스트.
#   정확도값은 추정(베이스라인이 짧으면 heading σ 커짐 → accHeading 으로 적응형 R 처리).
AGMO_V1_DUAL = GnssReceiverSpec(
    name="AGMO ver1 (듀얼안테나+중앙IMU)", can_bitrate=500_000, serial_baud=115_200,
    nmea_rate_hz=10.0, imu_rate_hz=100.0,
    heading_acc_deg=0.3, rollpitch_acc_deg=0.2, vel_acc_mps=0.05,
    rtcm="RTCM3.x", heading_source="dual",
)
# AGMO ver2 — GNSS+INS 스마트안테나 (CHCNAV NX510 동급). ★ 추정값
AGMO_V2_INS = GnssReceiverSpec(
    name="AGMO ver2 (GNSS+INS 스마트안테나)", can_bitrate=500_000, serial_baud=115_200,
    nmea_rate_hz=10.0, imu_rate_hz=100.0,
    heading_acc_deg=0.4, rollpitch_acc_deg=0.2, vel_acc_mps=0.05,
    rtcm="RTCM3.x", heading_source="ins",
)

# FJDynamics AT2 — GNSS+INS 스마트 수신기 (AT2 HW 설치매뉴얼 6.1 스펙표 확인값).
#   멀티밴드: GPS L1C/A·L1C·L2P(W)·L2C·L5 / GLONASS L1·L2 / BDS B1I·B2I·B3I·B1C·B2a
#            / Galileo E1·E5a·E5b / SBAS. 9–36V, <300mA, IP66.
#   내장 IMU: 자이로 정확도 0.1°/s, 가속도 0.5mg → roll/pitch 0.2°(확인값).
#   ★ heading 정확도는 매뉴얼 미기재 → 0.4° 추정(roll/pitch 0.2°·PA-3 대비 보수적).
#   별도 자세센서(attitude sensor 5V·200Hz·분해능<0.1°·IP67)도 보유 → roll/pitch 보강 가능.
FJD_AT2 = GnssReceiverSpec(
    name="FJDynamics AT2 GNSS+INS", can_bitrate=250_000, serial_baud=115_200,
    nmea_rate_hz=10.0, imu_rate_hz=100.0,
    heading_acc_deg=0.4, rollpitch_acc_deg=0.2, vel_acc_mps=0.05,
    rtcm="RTCM3.x", heading_source="ins",
)


# ═══════════════════════════════════════════════════════════════
#  모터 CanSpec dict (field_config._CANSPEC_FIELDS 키 + MOTOR_ACTIVATE_SEQ)
# ═══════════════════════════════════════════════════════════════

# AGMO = Keya KY170DD01005-08G (매뉴얼 V2.4 확정). 250k 속도제어 SDO.
# 주: cmd_speed/parse_heartbeat 등 실제 인코딩은 CanSpec 클래스 메서드(고정).
#     여기 dict 는 버스/ID/레거시 바이트레이아웃 스칼라만 반영한다.
KEYA_CANSPEC: Dict = {
    "CAN_BITRATE":        250_000,
    "MOTOR_CMD_ID":       0x06000001,   # 29-bit Extended TX (motor_id=1)
    "MOTOR_CMD_PERIOD":   0.020,
    "MOTOR_BYTE_MODE":    0, "MOTOR_BYTE_CMD_HI": 4, "MOTOR_BYTE_CMD_LO": 5,
    "MOTOR_BYTE_SPEED_LIM": -1, "MOTOR_BYTE_CHECKSUM": -1,
    "MOTOR_MODE_DISABLE": 0x00, "MOTOR_MODE_ANGLE": 0x01, "MOTOR_MODE_TORQUE": 0x02,
    "MOTOR_ANGLE_SCALE":  1.0, "MOTOR_MAX_SPEED": 400,
    "SENSOR_ANGLE_ID":    0x301,   # ★ WAS CAN ID 현장 캡처 필요
    "SENSOR_BYTE_HI":     0, "SENSOR_BYTE_LO": 1,
    "SENSOR_ANGLE_SCALE": 10.0, "SENSOR_ANGLE_OFFSET": 0.0, "SENSOR_SIGNED": True,
    "MOTOR_ACTIVATE_SEQ": [[0x06000001, "230d200100000000"]],  # CMD_ENABLE
}


def _placeholder_canspec(bitrate: int) -> Dict:
    """★ 미확정 벤더용 안전 placeholder. can_verified=False 와 함께 쓴다."""
    return {
        "CAN_BITRATE":        bitrate,
        "MOTOR_CMD_ID":       0x201,    # ★ 미확정
        "MOTOR_CMD_PERIOD":   0.020,
        "MOTOR_BYTE_MODE":    0, "MOTOR_BYTE_CMD_HI": 1, "MOTOR_BYTE_CMD_LO": 2,
        "MOTOR_BYTE_SPEED_LIM": 3, "MOTOR_BYTE_CHECKSUM": -1,
        "MOTOR_MODE_DISABLE": 0x00, "MOTOR_MODE_ANGLE": 0x01, "MOTOR_MODE_TORQUE": 0x02,
        "MOTOR_ANGLE_SCALE":  10.0, "MOTOR_MAX_SPEED": 50,
        "SENSOR_ANGLE_ID":    0x301,   # ★ 미확정
        "SENSOR_BYTE_HI":     0, "SENSOR_BYTE_LO": 1,
        "SENSOR_ANGLE_SCALE": 10.0, "SENSOR_ANGLE_OFFSET": 0.0, "SENSOR_SIGNED": True,
        "MOTOR_ACTIVATE_SEQ": [],
    }


# ═══════════════════════════════════════════════════════════════
#  VendorProfile + 레지스트리
# ═══════════════════════════════════════════════════════════════

@dataclass(frozen=True)
class VendorProfile:
    key:           str               # "agmo" / "chcnav" / "fjd"
    display_name:  str
    tagline:       str
    can_verified:  bool              # 모터 CAN 프로토콜 확정 여부
    canspec:       Dict              # CanSpec 스칼라값 (apply_canspec 입력)
    gnss_primary:  GnssReceiverSpec
    gnss_backup:   Optional[GnssReceiverSpec]
    gnss_priority: tuple             # GnssArbiter source 우선순위
    default_algo:  str
    notes:         str
    # 앵글센서(WAS) 사용 여부. False = WAS 없이 조향(모터 인코더/GNSS 피드백).
    #   AGMO: 항상 False(WAS 미사용 알고리즘 — Keya 하트비트 누적각으로 조향각 추정)
    #   CHCNAV/FJD: WAS 장착 선택 가능하나 없어도 동작 → 기본 False
    uses_was:      bool = False
    # 대체 안테나(헤딩 소스 다른 버전). AGMO 처럼 ver1(듀얼)/ver2(INS) 둘 다 지원 시.
    gnss_alt:      Optional[GnssReceiverSpec] = None
    # 추종 튜닝 오버라이드(TrackingParams 필드 dict). None = 기본값(AgNav 문서값).
    #   CHCNAV 수준 성능 목표 — AgNav 사진 확인값. 실하드웨어 튜닝(tuning.py)으로 미세조정.
    tracking:      Optional[Dict] = None


# CHCNAV(AgNav) 문서 확인 추종 튜닝값 — 기본 TrackingParams 와 동일하나 명시 고정.
# CHCNAV 수준 성능 목표값. 실하드웨어에서 tuning.py 로 미세조정.
CHCNAV_TUNING: Dict = {
    "online_sensitivity":   1.5,   # AgNav '온라인 민감도'
    "approach_sensitivity": 2.5,   # AgNav '접근 라인 민감도'
    "online_threshold":     2.5,   # AgNav '온라인 임계값'(m)
    "curve_coefficient":    1.0,   # AgNav '커브 계수'
}

VENDOR_PROFILES: Dict[str, VendorProfile] = {
    "agmo": VendorProfile(
        key="agmo", display_name="기본",
        tagline="Apollo 10 Pro · Keya 조향모터",
        can_verified=True,
        canspec=KEYA_CANSPEC,
        gnss_primary=AGMO_V2_INS, gnss_backup=UBLOX_F9P,
        gnss_priority=("agmo", "f9p"),
        default_algo="implement",
        uses_was=False,    # AGMO = WAS 미사용(모터 인코더 피드백)
        gnss_alt=AGMO_V1_DUAL,   # ver1(듀얼안테나+IMU) — ver2(INS)와 둘 다 지원
        notes="Keya KY170 매뉴얼 V2.4 프로토콜 확정(250k 속도제어, 확장프레임). "
              "앵글센서 미사용 — 조향각은 Keya 하트비트 누적각으로 추정. "
              "안테나: ver1=듀얼안테나+IMU / ver2=GNSS+INS 스마트안테나(둘 다 지원). "
              "GNSS 스펙은 추정값 — 현장 확인.",
    ),
    "chcnav": VendorProfile(
        key="chcnav", display_name="CHCNAV",
        tagline="NX510 / AgNav · PA-3 안테나",
        can_verified=False,
        canspec=_placeholder_canspec(500_000),   # PA-3 버스 500k (데이터시트)
        gnss_primary=CHCNAV_PA3, gnss_backup=UBLOX_F9P,
        gnss_priority=("pa3", "f9p"),
        default_algo="implement",
        uses_was=False,    # WAS 장착 선택 가능, 없어도 동작 → 기본 미사용
        tracking=CHCNAV_TUNING,   # AgNav 문서 추종값 명시 적용
        notes="PA-3 GNSS+INS 확정. 앵글센서(WAS) 선택(없어도 자동조향 가능). "
              "모터 CAN 프로토콜 ★미확정 — CHCNAV OEM CAN 문서 입수 후 "
              "canspec 채울 것(현재 조향 비활성).",
    ),
    "fjd": VendorProfile(
        key="fjd", display_name="FJDynamics",
        tagline="AT2 GNSS+INS · 전동 스티어링휠",
        can_verified=False,
        canspec=_placeholder_canspec(250_000),
        gnss_primary=FJD_AT2, gnss_backup=UBLOX_F9P,
        gnss_priority=("fjd", "f9p"),
        default_algo="implement",
        uses_was=False,    # WAS 장착 선택 가능, 없어도 동작 → 기본 미사용
        notes="AT2 GNSS+INS(매뉴얼 확인: 멀티밴드, gyro 0.1°/s, roll/pitch 0.2°). "
              "조향=전동 스티어링휠(컬럼형) — 최대토크 20Nm@12V / 30Nm@24V, 12·24V, "
              "스플라인 슬리브로 핸들 결합. AGMO Keya(인라인) 와 모터 형식 자체가 다름. "
              "보정신호=UHF 라디오(410–470 / 902–928MHz). 앵글센서(WAS) 선택(없어도 동작). "
              "모터 CAN 프로토콜 ★미확정 — FJD 문서/버스 캡처(can_tools) 후 canspec "
              "채울 것(현재 조향 비활성).",
    ),
}

DEFAULT_VENDOR = "agmo"


# ═══════════════════════════════════════════════════════════════
#  API
# ═══════════════════════════════════════════════════════════════

def list_vendors() -> List[dict]:
    """UI 선택화면용 목록 (직렬화 가능한 dict 리스트)."""
    return [{
        "key":          p.key,
        "name":         p.display_name,
        "tagline":      p.tagline,
        "can_verified": p.can_verified,
        "gnss":         p.gnss_primary.name,
        "bitrate":      p.canspec.get("CAN_BITRATE"),
        "uses_was":     p.uses_was,
        "heading_source": p.gnss_primary.heading_source,
        "gnss_alt":     (p.gnss_alt.name if p.gnss_alt else None),
        "notes":        p.notes,
    } for p in VENDOR_PROFILES.values()]


def get_profile(key: str) -> VendorProfile:
    key = (key or "").lower().strip()
    if key not in VENDOR_PROFILES:
        raise KeyError(f"알 수 없는 벤더: {key!r} (가능: {list(VENDOR_PROFILES)})")
    return VENDOR_PROFILES[key]


def apply_vendor(key: str) -> VendorProfile:
    """벤더 CanSpec 을 런타임 활성화하고 프로파일을 반환한다."""
    import field_config
    p = get_profile(key)
    n = field_config.apply_canspec(p.canspec)
    return p


if __name__ == "__main__":
    # 셀프테스트: 레지스트리 일관성 + CanSpec 활성화 검증
    from autosteer_core import CanSpec
    import field_config

    print("등록 벤더:")
    for v in list_vendors():
        flag = "✅확정" if v["can_verified"] else "★미확정"
        print(f"  {v['key']:7s} {v['name']:12s} 모터={flag}  "
              f"GNSS={v['gnss']:24s} bus={v['bitrate']}")

    # 모든 프로파일 canspec 키가 field_config 가 아는 키인지
    known = set(field_config._CANSPEC_FIELDS) | {"MOTOR_ACTIVATE_SEQ"}
    for p in VENDOR_PROFILES.values():
        unknown = set(p.canspec) - known
        assert not unknown, f"{p.key}: 미지원 canspec 키 {unknown}"

    # AGMO 활성화 → 250k Keya 값 반영 확인
    p = apply_vendor("agmo")
    assert CanSpec.CAN_BITRATE == 250_000, CanSpec.CAN_BITRATE
    assert CanSpec.MOTOR_CMD_ID == 0x06000001
    assert p.can_verified is True
    print(f"\napply_vendor('agmo') → BITRATE={CanSpec.CAN_BITRATE} "
          f"CMD_ID=0x{CanSpec.MOTOR_CMD_ID:08X} ✅")

    # CHCNAV 활성화 → 500k 반영 + 미확정 플래그
    p = apply_vendor("chcnav")
    assert CanSpec.CAN_BITRATE == 500_000
    assert p.can_verified is False
    print(f"apply_vendor('chcnav') → BITRATE={CanSpec.CAN_BITRATE} "
          f"can_verified={p.can_verified} (조향 비활성) ✅")

    print("\n  ✓ vendor_profiles 셀프테스트 통과")
