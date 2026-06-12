"""
field_config.py
===============
TractorParams + CanSpec 를 JSON 설정 파일로 외부화.

목적 (CLAUDE.md 우선순위 #1, #2 준비):
  - #1 실측값(wheelbase, antenna_to_axle …)이 들어오면 코드 수정 없이
    JSON 한 줄만 바꾸면 됨.
  - #2 모터 CAN 문서(CAN ID, 바이트맵)가 들어오면 JSON 으로 주입 →
    CanSpec 에 런타임 반영.

사용:
    from field_config import write_template, load_config, save_config
    write_template("tractor.json")          # ★ 자리 포함 템플릿 생성
    # → 현장에서 줄자/모터문서 값으로 tractor.json 편집 →
    params = load_config("tractor.json")     # TractorParams 반환 + CanSpec 적용

CanSpec 은 클래스 속성이라 apply_canspec() 가 런타임에 setattr 로 덮어쓴다.
"""

from __future__ import annotations
import json
from dataclasses import asdict
from typing import Optional, Tuple

from autosteer_core import TractorParams, ImuOffset, CanSpec, KUBOTA_MR1157

# JSON 으로 직렬화할 CanSpec 스칼라 필드 (MOTOR_ACTIVATE_SEQ 는 별도 처리)
_CANSPEC_FIELDS = [
    "CAN_BITRATE",
    "MOTOR_CMD_ID", "MOTOR_CMD_PERIOD",
    "MOTOR_BYTE_MODE", "MOTOR_BYTE_CMD_HI", "MOTOR_BYTE_CMD_LO",
    "MOTOR_BYTE_SPEED_LIM", "MOTOR_BYTE_CHECKSUM",
    "MOTOR_MODE_DISABLE", "MOTOR_MODE_ANGLE", "MOTOR_MODE_TORQUE",
    "MOTOR_ANGLE_SCALE", "MOTOR_MAX_SPEED",
    "SENSOR_ANGLE_ID", "SENSOR_BYTE_HI", "SENSOR_BYTE_LO",
    "SENSOR_ANGLE_SCALE", "SENSOR_ANGLE_OFFSET", "SENSOR_SIGNED",
]


# ── TractorParams ↔ dict ──────────────────────────────────────────
def tractor_to_dict(p: TractorParams) -> dict:
    return asdict(p)          # imu_offset(dataclass) 중첩 포함


def tractor_from_dict(d: dict) -> TractorParams:
    d = dict(d)
    imu = d.pop("imu_offset", None)
    params = TractorParams(**d)
    if imu:
        params.imu_offset = ImuOffset(**imu)
    return params


# ── CanSpec ↔ dict ────────────────────────────────────────────────
def canspec_to_dict() -> dict:
    out = {k: getattr(CanSpec, k) for k in _CANSPEC_FIELDS}
    # 활성화 시퀀스: [(id, bytes)] → [[id, "hex"]]
    out["MOTOR_ACTIVATE_SEQ"] = [[cid, data.hex()]
                                 for cid, data in CanSpec.MOTOR_ACTIVATE_SEQ]
    return out


def apply_canspec(d: dict) -> int:
    """주어진 dict 의 알려진 키만 CanSpec 클래스 속성에 반영. 반영 개수 반환."""
    n = 0
    for k in _CANSPEC_FIELDS:
        if k in d:
            setattr(CanSpec, k, d[k]); n += 1
    if "MOTOR_ACTIVATE_SEQ" in d:
        seq = [(int(cid), bytes.fromhex(hexs))
               for cid, hexs in d["MOTOR_ACTIVATE_SEQ"]]
        setattr(CanSpec, "MOTOR_ACTIVATE_SEQ", seq); n += 1
    return n


# ── 파일 입출력 ───────────────────────────────────────────────────
def save_config(path: str, params: TractorParams,
                include_canspec: bool = True):
    cfg = {"tractor": tractor_to_dict(params)}
    if include_canspec:
        cfg["canspec"] = canspec_to_dict()
    with open(path, "w", encoding="utf-8") as f:
        json.dump(cfg, f, indent=2, ensure_ascii=False)


def load_config(path: str) -> Tuple[TractorParams, int]:
    """JSON 로드 → (TractorParams, 적용된 CanSpec 필드 수)."""
    with open(path, encoding="utf-8") as f:
        cfg = json.load(f)
    params = (tractor_from_dict(cfg["tractor"])
              if "tractor" in cfg else KUBOTA_MR1157)
    applied = apply_canspec(cfg["canspec"]) if "canspec" in cfg else 0
    return params, applied


def write_template(path: str):
    """
    현장 기입용 템플릿 생성. ★ 가 붙은 값은 실측/문서로 교체 대상.
    _measure_TODO 에 무엇을 어떻게 재는지 메모를 같이 남긴다.
    """
    cfg = {
        "_measure_TODO": {
            "wheelbase":       "★ 앞차축 중심 ↔ 뒷차축 중심 (줄자, m)",
            "antenna_to_axle": "★ 뒷차축 ↔ 안테나 전후 (뒤쪽이면 음수, m). calibration.py 로 자동 추정 가능",
            "antenna_to_impl": "★ 안테나 ↔ 작업기 (m)",
            "canspec":         "★ 모터 CAN 문서: MOTOR_CMD_ID / SENSOR_ANGLE_ID / 바이트맵. can_tools.py 로 역추적 가능",
        },
        "tractor": tractor_to_dict(KUBOTA_MR1157),
        "canspec": canspec_to_dict(),
    }
    with open(path, "w", encoding="utf-8") as f:
        json.dump(cfg, f, indent=2, ensure_ascii=False)


# ── 자체 테스트 ───────────────────────────────────────────────────
if __name__ == "__main__":
    import tempfile, os
    print("=" * 70)
    print("field_config — TractorParams/CanSpec JSON 외부화 테스트")
    print("=" * 70)

    tmp = os.path.join(tempfile.gettempdir(), "tractor_test.json")
    write_template(tmp)
    print(f"\n템플릿 생성: {tmp}")

    # 실측값을 받은 것처럼 편집
    with open(tmp, encoding="utf-8") as f:
        cfg = json.load(f)
    cfg["tractor"]["wheelbase"] = 2.55          # 줄자 실측
    cfg["tractor"]["antenna_to_axle"] = -0.42   # 실측
    cfg["canspec"]["MOTOR_CMD_ID"] = 0x18FF50E5 # 모터 문서값(예)
    cfg["canspec"]["SENSOR_ANGLE_ID"] = 0x18FF51E5
    cfg["canspec"]["CAN_BITRATE"] = 250000
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(cfg, f, indent=2, ensure_ascii=False)

    params, applied = load_config(tmp)
    print(f"\n로드 결과: wheelbase={params.wheelbase}m  "
          f"antenna_to_axle={params.antenna_to_axle}m")
    print(f"CanSpec 적용 {applied}개: "
          f"MOTOR_CMD_ID=0x{CanSpec.MOTOR_CMD_ID:X}  "
          f"SENSOR_ANGLE_ID=0x{CanSpec.SENSOR_ANGLE_ID:X}  "
          f"BITRATE={CanSpec.CAN_BITRATE}")

    assert params.wheelbase == 2.55
    assert params.antenna_to_axle == -0.42
    assert CanSpec.MOTOR_CMD_ID == 0x18FF50E5
    assert CanSpec.CAN_BITRATE == 250000
    assert isinstance(params.imu_offset, ImuOffset)
    print("\n  ✓ 실측값/CAN문서를 코드 수정 없이 JSON 으로 주입 가능")
    os.remove(tmp)
