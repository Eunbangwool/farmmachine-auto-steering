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

# AGMO 자율주행 키트 수신기 — ★ 실제 스펙 미확인(보수적 추정값)
AGMO_GNSS = GnssReceiverSpec(
    name="내장 GNSS (추정)", can_bitrate=500_000, serial_baud=115_200,
    nmea_rate_hz=10.0, imu_rate_hz=100.0,
    heading_acc_deg=0.5, rollpitch_acc_deg=0.2, vel_acc_mps=0.05,
    rtcm="RTCM3.x",
)

# FJDynamics AT2 dome — GNSS+INS. ★ 실제 스펙 미확인(추정값)
FJD_AT2 = GnssReceiverSpec(
    name="FJDynamics AT2 dome (추정)", can_bitrate=250_000, serial_baud=115_200,
    nmea_rate_hz=10.0, imu_rate_hz=100.0,
    heading_acc_deg=0.5, rollpitch_acc_deg=0.2, vel_acc_mps=0.05,
    rtcm="RTCM3.x",
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


VENDOR_PROFILES: Dict[str, VendorProfile] = {
    "agmo": VendorProfile(
        key="agmo", display_name="기본",
        tagline="Apollo 10 Pro · Keya 조향모터",
        can_verified=True,
        canspec=KEYA_CANSPEC,
        gnss_primary=AGMO_GNSS, gnss_backup=UBLOX_F9P,
        gnss_priority=("agmo", "f9p"),
        default_algo="implement",
        uses_was=False,    # AGMO = WAS 미사용(모터 인코더 피드백)
        notes="Keya KY170 매뉴얼 V2.4 프로토콜 확정(250k 속도제어, 확장프레임). "
              "앵글센서 미사용 — 조향각은 Keya 하트비트 누적각으로 추정. "
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
        notes="PA-3 GNSS+INS 확정. 앵글센서(WAS) 선택(없어도 자동조향 가능). "
              "모터 CAN 프로토콜 ★미확정 — CHCNAV OEM CAN 문서 입수 후 "
              "canspec 채울 것(현재 조향 비활성).",
    ),
    "fjd": VendorProfile(
        key="fjd", display_name="FJDynamics",
        tagline="AT2 dome · FJD 전동조향",
        can_verified=False,
        canspec=_placeholder_canspec(250_000),
        gnss_primary=FJD_AT2, gnss_backup=UBLOX_F9P,
        gnss_priority=("fjd", "f9p"),
        default_algo="implement",
        uses_was=False,    # WAS 장착 선택 가능, 없어도 동작 → 기본 미사용
        notes="AT2 GNSS+INS(추정). 앵글센서(WAS) 선택(없어도 자동조향 가능). "
              "모터 CAN 프로토콜 ★미확정 — FJD 문서/버스 캡처(can_tools) 후 "
              "canspec 채울 것(현재 조향 비활성).",
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
