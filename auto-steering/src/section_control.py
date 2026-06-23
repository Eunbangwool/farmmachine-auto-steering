"""
section_control.py — 가변시비(VRA) + 섹션 컨트롤 (시비·방제 공통 모듈)
=====================================================================
정밀농업 작업기 제어. 자율조향(autosteer_core)이 차체를 몰면, 이 모듈은
**작업기(살포기/방제기)의 살포율과 섹션 ON/OFF** 를 위치 기반으로 제어한다.

기능 (오너 결정 2026-06-23):
  1) 가변시비(Variable Rate Application) — 처방맵(GeoJSON Rx)을 읽어
     현재 위치의 목표 살포율을 결정. 시비(입제 kg/ha)·방제(액제 L/ha) 공통.
  2) 섹션 컨트롤 — 작업기를 N개 섹션으로 나눠, 각 섹션의 지면 접점이
     ① 이미 살포된 영역(중복)·② 포장 경계 밖·③ 제외구역 안·④ 작업기 들림
     (헤드랜드)·⑤ 처방율 0 일 때 자동 OFF. 밸브 지연을 lead/lag 시간으로 보정.

출력 = **자체(커스텀) CAN 프로토콜**(ImplementCanProtocol, 아래 FMSC v1).
  오너가 직접 만드는 작업기 컨트롤러용 규약이므로 우리가 정의한다.
  ★ 안전장치: 실제 컨트롤러 HW 가 아직 없으므로 IMPLEMENT_CAN_VERIFIED=False.
    False 면 프레임을 만들고 로그/시뮬은 하되 **버스로 송신하지 않는다**
    (vendor_profiles.can_verified 와 동일 패턴 — 미검증 추측 송신 금지).
    실 컨트롤러로 버스 캡처/검증 후 True 로 바꾼다.

좌표계: autosteer_core 와 동일한 로컬 평면 (x=동 m, y=북 m), 원점=RTK 첫 fix.
  전진단위 = (cos h, sin h), 좌측단위 = (-sin h, cos h)  (코드 실제 규약과 일치).

의존성: 표준 라이브러리만 (math). numpy 불필요 → 즉시 테스트 가능:
    cd auto-steering/src && python section_control.py
"""
from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple


# ═══════════════════════════════════════════════════════════════
#  자체 CAN 프로토콜 — FMSC v1 (Farm-Machine Section Control)
# ═══════════════════════════════════════════════════════════════
# 오너가 만드는 작업기 컨트롤러 규약. 250kbps, 29-bit 확장 ID.
# ★ Keya 조향모터 ID(TX 0x06000001 / RX 0x05800001 / HB 0x07000001)와 겹치지
#   않도록 0x0A1/0x0A18 대역 사용 → 동일 물리버스 공유 가능(또는 별도 버스).
# 부호/스케일은 명시 — 매직넘버 금지 규칙(VehicleProfile 모범).

FMSC_BITRATE = 250_000

RATE_CMD_ID     = 0x0A100001    # APP → 컨트롤러: 목표 살포율 + 단위 + 지면속도
SECTION_CMD_ID  = 0x0A100002    # APP → 컨트롤러: 섹션 ON/OFF 비트마스크 + 마스터
STATUS_HB_ID    = 0x0A180001    # 컨트롤러 → APP: 실제 섹션/살포율/결함/호퍼레벨

RATE_CMD_PERIOD    = 0.10       # s — 살포율 명령 주기 (10Hz)
SECTION_CMD_PERIOD = 0.10       # s — 섹션 명령 주기 (10Hz)
STATUS_TIMEOUT     = 0.50       # s — 이 시간 HB 없으면 comm_lost

RATE_SCALE = 10.0               # 살포율 인코딩 스케일: uint16 = rate × 10 (0.1 분해능)
RATE_MAX   = 6553.5             # uint16/RATE_SCALE 상한

# 제품 모드 (RATE_CMD b0 상위니블) — 시비/방제 공통화
MODE_FERT_GRANULAR = 0          # 입제 시비 (kg/ha)
MODE_LIQUID_SPRAY  = 1          # 액제 방제 (L/ha)
MODE_SEED          = 2          # 파종 (seeds/m²)

# 단위 enum (RATE_CMD b1)
UNIT_KG_HA   = 0
UNIT_L_HA    = 1
UNIT_SEED_M2 = 2
_UNIT_NAME = {UNIT_KG_HA: "kg/ha", UNIT_L_HA: "L/ha", UNIT_SEED_M2: "seeds/m²"}
_NAME_UNIT = {v: k for k, v in _UNIT_NAME.items()}

# STATUS_HB b4 결함 비트
FAULT_LOW_HOPPER = 0x01
FAULT_VALVE      = 0x02
FAULT_OVER_RATE  = 0x04
FAULT_COMM_LOST  = 0x08

MAX_SECTIONS = 16               # 비트마스크 uint16

# ★ 실 컨트롤러 HW 미보유 → 미검증. 버스 캡처/실기검증 후 True 로.
#   (False 동안 encode 는 동작, send 만 차단 = 추측 송신 방지)
IMPLEMENT_CAN_VERIFIED = False


def _checksum(b: bytes) -> int:
    """b0..b6 XOR → b7. 단순/결정적(컨트롤러 MCU 측 구현 부담 최소)."""
    x = 0
    for v in b[:7]:
        x ^= v
    return x & 0xFF


def _clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else hi if v > hi else v


def encode_rate_cmd(target_rate: float, *, master_on: bool, rate_valid: bool,
                    mode: int = MODE_FERT_GRANULAR, unit: int = UNIT_KG_HA,
                    ground_speed_mps: float = 0.0, section_count: int = 0) -> bytes:
    """RATE_CMD 8바이트 인코딩. target_rate 단위는 `unit` 기준."""
    r = int(round(_clamp(target_rate, 0.0, RATE_MAX) * RATE_SCALE)) & 0xFFFF
    spd = int(round(_clamp(ground_speed_mps, 0.0, 65.535) * 1000)) & 0xFFFF   # mm/s
    b = bytearray(8)
    b[0] = ((1 if master_on else 0)
            | (2 if rate_valid else 0)
            | ((mode & 0x0F) << 4))
    b[1] = unit & 0xFF
    b[2] = (r >> 8) & 0xFF
    b[3] = r & 0xFF
    b[4] = (spd >> 8) & 0xFF
    b[5] = spd & 0xFF
    b[6] = section_count & 0xFF
    b[7] = _checksum(b)
    return bytes(b)


def encode_section_cmd(section_mask: int, section_count: int, *,
                       master_on: bool, applied_rate: float = 0.0) -> bytes:
    """SECTION_CMD 8바이트. bit0=섹션1(좌측 끝) … 좌→우 순서."""
    m = section_mask & 0xFFFF
    r = int(round(_clamp(applied_rate, 0.0, RATE_MAX) * RATE_SCALE)) & 0xFFFF
    b = bytearray(8)
    b[0] = (m >> 8) & 0xFF
    b[1] = m & 0xFF
    b[2] = section_count & 0xFF
    b[3] = 1 if master_on else 0
    b[4] = (r >> 8) & 0xFF
    b[5] = r & 0xFF
    b[6] = 0
    b[7] = _checksum(b)
    return bytes(b)


def decode_status(data: bytes) -> Optional[dict]:
    """STATUS_HB 디코딩. 체크섬 불일치/길이부족 → None(조용히 먹지 않고 호출측이 판단)."""
    if len(data) < 8 or _checksum(data) != data[7]:
        return None
    mask = (data[0] << 8) | data[1]
    rate = ((data[2] << 8) | data[3]) / RATE_SCALE
    return {
        "section_mask": mask,
        "actual_rate":  rate,
        "faults":       data[4],
        "hopper_pct":   data[5],
    }


# ═══════════════════════════════════════════════════════════════
#  좌표 변환 (lat/lon ↔ 로컬 평면) — autosteer 와 동일 원점 공유
# ═══════════════════════════════════════════════════════════════
# autosteer_core._ll_to_xy 와 동일 등거리(equirectangular) 근사. 같은 원점을
# 주면 처방맵(위경도)이 차체 로컬 좌표와 정확히 정합된다.
_EARTH_R = 6_371_000.0


class GeoLocal:
    def __init__(self, origin: Optional[Tuple[float, float]] = None):
        self.origin = origin           # (lat0, lon0) 또는 None(첫 점에서 설정)

    def set_origin(self, lat: float, lon: float):
        self.origin = (lat, lon)

    def to_xy(self, lat: float, lon: float) -> Tuple[float, float]:
        if self.origin is None:
            self.origin = (lat, lon)
        lat0, lon0 = self.origin
        x = math.radians(lon - lon0) * _EARTH_R * math.cos(math.radians(lat0))
        y = math.radians(lat - lat0) * _EARTH_R
        return x, y


def _point_in_ring(x: float, y: float, ring: List[Tuple[float, float]]) -> bool:
    """레이캐스팅 point-in-polygon (로컬 x/y, 링은 [(x,y), …])."""
    inside = False
    n = len(ring)
    j = n - 1
    for i in range(n):
        xi, yi = ring[i]
        xj, yj = ring[j]
        if ((yi > y) != (yj > y)) and \
           (x < (xj - xi) * (y - yi) / ((yj - yi) or 1e-12) + xi):
            inside = not inside
        j = i
    return inside


# ═══════════════════════════════════════════════════════════════
#  처방맵 (GeoJSON Rx)
# ═══════════════════════════════════════════════════════════════
# 표준 정밀농업 처방맵: FeatureCollection, 각 Feature=구역 폴리곤 +
#   properties.rate(목표 살포율). MultiPolygon/Polygon 지원(외곽 링만 사용,
#   홀은 미지원 — 농업 처방맵은 통상 단순 구역). 좌표 = [lon, lat](GeoJSON 표준).

# rate 속성 키 후보 (처방맵 생산도구마다 다름) — 순서대로 탐색.
_RATE_KEYS = ("rate", "RATE", "Rate", "rate_kg_ha", "rate_l_ha", "target_rate", "VRA_RATE")


@dataclass
class RxZone:
    rings: List[List[Tuple[float, float]]]   # 로컬 x/y 폴리곤(들)
    rate: float
    name: str = ""


class PrescriptionMap:
    """처방맵: 위치 → 목표 살포율. 구역 밖이면 default_rate."""
    def __init__(self, geo: GeoLocal, default_rate: float = 0.0,
                 unit: int = UNIT_KG_HA):
        self.geo = geo
        self.default_rate = default_rate
        self.unit = unit
        self.zones: List[RxZone] = []
        self.rate_min = 0.0
        self.rate_max = 0.0

    @classmethod
    def from_geojson(cls, gj, geo: GeoLocal, default_rate: float = 0.0,
                     unit: Optional[int] = None) -> "PrescriptionMap":
        """gj = dict / JSON 문자열 / 파일경로."""
        if isinstance(gj, str):
            if gj.lstrip().startswith("{"):
                gj = json.loads(gj)
            else:
                with open(gj) as f:
                    gj = json.load(f)
        feats = gj.get("features", []) if gj.get("type") == "FeatureCollection" \
            else [gj]
        # 단위: 명시 인자 > 첫 Feature properties.unit > kg/ha
        if unit is None:
            u = (feats[0].get("properties", {}).get("unit") if feats else None)
            unit = _NAME_UNIT.get(str(u), UNIT_KG_HA)
        pm = cls(geo, default_rate=default_rate, unit=unit)
        for ft in feats:
            props = ft.get("properties", {}) or {}
            rate = None
            for k in _RATE_KEYS:
                if k in props and props[k] is not None:
                    rate = float(props[k]); break
            if rate is None:
                continue                     # rate 없는 Feature 는 무시(경계 등)
            geom = ft.get("geometry", {}) or {}
            rings = pm._geom_to_local_rings(geom)
            if rings:
                pm.zones.append(RxZone(rings=rings, rate=rate,
                                       name=str(props.get("name", props.get("zone", "")))))
        rates = [z.rate for z in pm.zones]
        if rates:
            pm.rate_min, pm.rate_max = min(rates), max(rates)
        return pm

    def _geom_to_local_rings(self, geom: dict) -> List[List[Tuple[float, float]]]:
        t = geom.get("type")
        coords = geom.get("coordinates", [])
        rings: List[List[Tuple[float, float]]] = []
        polys = []
        if t == "Polygon":
            polys = [coords]
        elif t == "MultiPolygon":
            polys = coords
        else:
            return rings
        for poly in polys:
            if not poly:
                continue
            outer = poly[0]              # 외곽 링만 (홀 미지원)
            ring = [self.geo.to_xy(lat=pt[1], lon=pt[0]) for pt in outer]
            if len(ring) >= 3:
                rings.append(ring)
        return rings

    def rate_at(self, x: float, y: float) -> float:
        """로컬 좌표(x,y) 의 목표 살포율. 구역 안이면 그 구역 rate, 아니면 default."""
        for z in self.zones:
            for ring in z.rings:
                if _point_in_ring(x, y, ring):
                    return z.rate
        return self.default_rate

    def summary(self) -> dict:
        return {
            "zones": len(self.zones),
            "rate_min": self.rate_min,
            "rate_max": self.rate_max,
            "default_rate": self.default_rate,
            "unit": _UNIT_NAME.get(self.unit, "kg/ha"),
            "origin": self.geo.origin,
        }


# ═══════════════════════════════════════════════════════════════
#  커버리지 그리드 (살포 완료 영역 추적 → 중복 방지)
# ═══════════════════════════════════════════════════════════════
class CoverageGrid:
    """로컬 평면 격자. 살포된 셀을 set 으로 보관(메모리 = 작업면적/해상도²)."""
    def __init__(self, resolution_m: float = 0.5):
        self.res = float(resolution_m)
        self._cells: set = set()

    def _key(self, x: float, y: float) -> Tuple[int, int]:
        return (int(math.floor(x / self.res)), int(math.floor(y / self.res)))

    def mark(self, x: float, y: float):
        self._cells.add(self._key(x, y))

    def covered(self, x: float, y: float) -> bool:
        return self._key(x, y) in self._cells

    def clear(self):
        self._cells.clear()

    @property
    def area_m2(self) -> float:
        return len(self._cells) * self.res * self.res


# ═══════════════════════════════════════════════════════════════
#  작업기 섹션 기하
# ═══════════════════════════════════════════════════════════════
@dataclass
class SectionLayout:
    """
    작업기 폭을 N개 섹션으로 등분. 섹션 0 = 좌측 끝, N-1 = 우측 끝.
    impl_behind: 작업기 살포지점이 기준점(보통 작업기 GNSS/뒤차축)보다 뒤쪽 m.
                 (caller 가 이미 작업기 위치를 주면 0)
    """
    width_m: float = 12.0
    num_sections: int = 8
    impl_behind: float = 0.0

    def section_width(self) -> float:
        return self.width_m / max(1, self.num_sections)

    def lateral_offsets(self) -> List[float]:
        """각 섹션 중심의 좌측(+) 횡오프셋(m). 좌→우 = +W/2 → -W/2."""
        w = self.section_width()
        # 좌측 끝(+W/2) 에서 시작해 우측으로 진행 → 섹션0 = 가장 왼쪽(+)
        return [self.width_m / 2.0 - (i + 0.5) * w for i in range(self.num_sections)]


# ═══════════════════════════════════════════════════════════════
#  섹션 컨트롤러 (커버리지·경계·헤드랜드·처방 기반 ON/OFF)
# ═══════════════════════════════════════════════════════════════
@dataclass
class SectionDecision:
    mask: int
    on: List[bool]
    reasons: List[str]               # 각 섹션 OFF 사유("" = ON)
    ground_points: List[Tuple[float, float]]


class SectionController:
    """
    매 제어틱: 차체 포즈 + 속도로 각 섹션의 지면 접점을 계산하고
    ON/OFF 를 결정. 밸브 ON 선행(lead)·OFF 지연(lag)을 속도×시간 으로 보정.
    """
    def __init__(self, layout: SectionLayout,
                 coverage: Optional[CoverageGrid] = None,
                 overlap_off: bool = True,
                 lead_on_s: float = 0.4, lag_off_s: float = 0.3):
        self.layout = layout
        self.coverage = coverage or CoverageGrid()
        self.overlap_off = overlap_off       # 중복살포 시 OFF (스킵)
        self.lead_on_s = lead_on_s           # 켜짐 선행(밸브 응답 지연 보상)
        self.lag_off_s = lag_off_s           # 꺼짐 지연(잔여 살포 끝까지)
        self.boundary: Optional[List[Tuple[float, float]]] = None   # 포장 외곽
        self.exclusions: List[List[Tuple[float, float]]] = []        # 제외구역

    def set_boundary(self, ring: Optional[List[Tuple[float, float]]]):
        self.boundary = ring

    def add_exclusion(self, ring: List[Tuple[float, float]]):
        self.exclusions.append(ring)

    def compute(self, x: float, y: float, heading: float, speed: float, *,
                master_on: bool, implement_down: bool,
                rx: Optional[PrescriptionMap] = None,
                mark_coverage: bool = True) -> SectionDecision:
        fx, fy = math.cos(heading), math.sin(heading)        # 전진 단위
        lx, ly = -math.sin(heading), math.cos(heading)       # 좌측 단위
        # 살포지점(기준점에서 뒤로 impl_behind)
        bx = x - self.layout.impl_behind * fx
        by = y - self.layout.impl_behind * fy

        offs = self.layout.lateral_offsets()
        on: List[bool] = []
        reasons: List[str] = []
        pts: List[Tuple[float, float]] = []

        for off in offs:
            # 섹션 중심 지면점 + 켜짐 선행 거리만큼 전방 투영(밸브 지연 보상)
            lead = speed * self.lead_on_s
            sx = bx + off * lx + lead * fx
            sy = by + off * ly + lead * fy
            pts.append((sx, sy))

            reason = ""
            if not master_on:
                reason = "master_off"
            elif not implement_down:
                reason = "headland"               # 작업기 들림(회전)
            elif self.boundary is not None and not _point_in_ring(sx, sy, self.boundary):
                reason = "out_of_field"
            elif any(_point_in_ring(sx, sy, ex) for ex in self.exclusions):
                reason = "exclusion"
            elif rx is not None and rx.rate_at(sx, sy) <= 0.0:
                reason = "rate_zero"
            elif self.overlap_off and self.coverage.covered(sx, sy):
                reason = "overlap"
            on.append(reason == "")
            reasons.append(reason)

        # 켜진 섹션의 실제 살포지점(선행거리 제외)으로 커버리지 기록
        if mark_coverage:
            for i, off in enumerate(offs):
                if on[i]:
                    mx = bx + off * lx
                    my = by + off * ly
                    self.coverage.mark(mx, my)

        mask = 0
        for i, v in enumerate(on):
            if v:
                mask |= (1 << i)
        return SectionDecision(mask=mask, on=on, reasons=reasons, ground_points=pts)


# ═══════════════════════════════════════════════════════════════
#  살포율 컨트롤러 (처방맵 → 목표율, 변화율 제한)
# ═══════════════════════════════════════════════════════════════
class RateController:
    """
    처방맵에서 위치별 목표 살포율을 읽고, 급변을 막기 위해 변화율(slew)을
    제한해 출력. 처방맵 없으면 default_rate 고정.
    """
    def __init__(self, default_rate: float = 0.0,
                 min_rate: float = 0.0, max_rate: float = RATE_MAX,
                 slew_per_s: float = 200.0):
        self.default_rate = default_rate
        self.min_rate = min_rate
        self.max_rate = max_rate
        self.slew_per_s = slew_per_s          # 초당 최대 변화량(단위/s)
        self._cur = 0.0

    def target_at(self, x: float, y: float,
                  rx: Optional[PrescriptionMap]) -> float:
        r = rx.rate_at(x, y) if rx is not None else self.default_rate
        return _clamp(r, self.min_rate, self.max_rate)

    def step(self, target: float, dt: float) -> float:
        """변화율 제한 적용해 현재 출력값을 target 쪽으로 이동."""
        if self.slew_per_s > 0 and dt > 0:
            max_step = self.slew_per_s * dt
            d = _clamp(target - self._cur, -max_step, max_step)
            self._cur += d
        else:
            self._cur = target
        return self._cur

    @property
    def current(self) -> float:
        return self._cur


# ═══════════════════════════════════════════════════════════════
#  통합 컨트롤러 (조정자) — 포즈 → 섹션/살포율 → CAN
# ═══════════════════════════════════════════════════════════════
class ApplicationController:
    """
    autosteer 제어루프가 매 틱 update() 를 호출.
      - 위치별 목표 살포율(처방맵) 결정
      - 섹션 ON/OFF 결정(커버리지·경계·헤드랜드·처방0)
      - FMSC CAN 프레임 생성 → (검증 시) 버스 송신
      - 살포명령 로컬 기록(약관 §8: 명령/개입 기록 유지)
      - 상태 dict 반환(UI/status_json)

    bus: autosteer_core.CanInterface 호환(send/recv). None 이면 송신 생략(시뮬).
    can_verified: 기본 IMPLEMENT_CAN_VERIFIED. False 면 프레임 생성하되 미송신.
    """
    def __init__(self, layout: Optional[SectionLayout] = None,
                 mode: int = MODE_FERT_GRANULAR, unit: int = UNIT_KG_HA,
                 default_rate: float = 0.0, bus=None,
                 can_verified: Optional[bool] = None,
                 coverage_res_m: float = 0.5):
        self.layout = layout or SectionLayout()
        self.mode = mode
        self.unit = unit
        self.geo = GeoLocal()
        self.coverage = CoverageGrid(coverage_res_m)
        self.sections = SectionController(self.layout, self.coverage)
        self.rates = RateController(default_rate=default_rate)
        self.rx: Optional[PrescriptionMap] = None
        self.bus = bus
        self.can_verified = (IMPLEMENT_CAN_VERIFIED if can_verified is None
                             else bool(can_verified))
        self.master_on = False
        # 로컬 명령 기록(약관 §8) — 링버퍼
        self.log: List[dict] = []
        self._log_max = 5000
        self._last_tx = 0.0
        self._last_status: dict = {}
        self.last_error: str = ""               # 실패 사유 기록(조용히 먹지 않음)
        # 컨트롤러 피드백(STATUS_HB)
        self.hb: Optional[dict] = None
        self._hb_t = 0.0

    # ── 설정 ──────────────────────────────────────────────────
    def set_origin(self, lat: float, lon: float):
        """autosteer RTK 원점과 동일하게 맞춤(처방맵 정합 필수)."""
        self.geo.set_origin(lat, lon)

    def load_prescription(self, gj, default_rate: Optional[float] = None) -> dict:
        dr = self.rates.default_rate if default_rate is None else default_rate
        self.rx = PrescriptionMap.from_geojson(gj, self.geo, default_rate=dr,
                                               unit=self.unit)
        self.unit = self.rx.unit
        self.rates.default_rate = dr
        return self.rx.summary()

    def set_layout(self, width_m: float = None, num_sections: int = None,
                   impl_behind: float = None):
        if width_m is not None:
            self.layout.width_m = float(width_m)
        if num_sections is not None:
            self.layout.num_sections = max(1, min(MAX_SECTIONS, int(num_sections)))
        if impl_behind is not None:
            self.layout.impl_behind = float(impl_behind)

    def set_master(self, on: bool):
        self.master_on = bool(on)
        if not self.master_on:                  # 즉시 전 섹션 OFF 송신
            self._send(SECTION_CMD_ID,
                       encode_section_cmd(0, self.layout.num_sections,
                                          master_on=False))

    def clear_coverage(self):
        self.coverage.clear()

    # ── 메인 틱 ────────────────────────────────────────────────
    def update(self, x: float, y: float, heading: float, speed: float,
               implement_down: bool, dt: float) -> dict:
        """포즈(로컬 x/y/heading) + 속도 → 섹션/살포율 산출 + CAN 송신."""
        self._drain_status()                     # 컨트롤러 HB 수신 반영

        dec = self.sections.compute(
            x, y, heading, speed,
            master_on=self.master_on, implement_down=implement_down,
            rx=self.rx, mark_coverage=self.master_on and implement_down)

        # 살포율: 살포지점(작업기 중심)의 처방율 → 변화율 제한
        fx, fy = math.cos(heading), math.sin(heading)
        bx = x - self.layout.impl_behind * fx
        by = y - self.layout.impl_behind * fy
        target = self.rates.target_at(bx, by, self.rx)
        any_on = any(dec.on)
        applied = self.rates.step(target if (self.master_on and implement_down
                                             and any_on) else 0.0, dt)

        # ── CAN 송신(주기 제한) ──
        now = time.time()
        if now - self._last_tx >= min(RATE_CMD_PERIOD, SECTION_CMD_PERIOD):
            self._send(RATE_CMD_ID, encode_rate_cmd(
                applied, master_on=self.master_on,
                rate_valid=(self.rx is not None),
                mode=self.mode, unit=self.unit,
                ground_speed_mps=speed,
                section_count=self.layout.num_sections))
            self._send(SECTION_CMD_ID, encode_section_cmd(
                dec.mask, self.layout.num_sections,
                master_on=self.master_on, applied_rate=applied))
            self._last_tx = now
            self._record(now, dec, target, applied, speed)

        self._last_status = {
            "master_on": self.master_on,
            "can_verified": self.can_verified,
            "num_sections": self.layout.num_sections,
            "width_m": self.layout.width_m,
            "section_mask": dec.mask,
            "sections_on": dec.on,
            "section_reasons": dec.reasons,
            "target_rate": round(target, 2),
            "applied_rate": round(applied, 2),
            "unit": _UNIT_NAME.get(self.unit, "kg/ha"),
            "mode": self.mode,
            "covered_area_m2": round(self.coverage.area_m2, 1),
            "rx_loaded": self.rx is not None,
            "hopper_pct": (self.hb or {}).get("hopper_pct"),
            "faults": (self.hb or {}).get("faults", 0),
            "comm_lost": self._comm_lost(now),
            "last_error": self.last_error,
        }
        return self._last_status

    def status(self) -> dict:
        return dict(self._last_status)

    # ── CAN I/O ───────────────────────────────────────────────
    def _send(self, can_id: int, data: bytes):
        if self.bus is None:
            return
        if not self.can_verified:
            # 미검증 프로토콜 → 추측 송신 금지(안전). 사유 기록.
            self.last_error = "can_unverified"
            return
        try:
            self.bus.send(can_id, data)
        except Exception as e:
            self.last_error = f"send_fail:{e}"

    def _drain_status(self):
        """버스에서 STATUS_HB 를 수집(있으면). 다른 RX 프레임은 무시."""
        if self.bus is None or not hasattr(self.bus, "recv"):
            return
        while True:
            fr = self.bus.recv()
            if not fr:
                break
            cid, data = fr
            if cid == STATUS_HB_ID:
                st = decode_status(bytes(data))
                if st is not None:
                    self.hb = st
                    self._hb_t = time.time()

    def _comm_lost(self, now: float) -> bool:
        if self.bus is None:
            return False
        return (now - self._hb_t) > STATUS_TIMEOUT if self._hb_t else True

    # ── 기록(약관 §8) ─────────────────────────────────────────
    def _record(self, t: float, dec: SectionDecision,
                target: float, applied: float, speed: float):
        self.log.append({
            "t": round(t, 3), "mask": dec.mask,
            "target": round(target, 2), "applied": round(applied, 2),
            "speed": round(speed, 2),
        })
        if len(self.log) > self._log_max:
            del self.log[:len(self.log) - self._log_max]


# ═══════════════════════════════════════════════════════════════
#  SITL 자가검증 (하드웨어 불필요)
# ═══════════════════════════════════════════════════════════════
def _demo_geojson(lat0: float, lon0: float) -> dict:
    """원점 부근 2구역 처방맵 — 동쪽 절반=고율, 서쪽 절반=저율."""
    def ll(dx, dy):   # 로컬 m → lat/lon (역변환, GeoLocal 과 정합)
        lat = lat0 + math.degrees(dy / _EARTH_R)
        lon = lon0 + math.degrees(dx / (_EARTH_R * math.cos(math.radians(lat0))))
        return [lon, lat]
    return {
        "type": "FeatureCollection",
        "features": [
            {"type": "Feature", "properties": {"name": "west", "rate": 80, "unit": "kg/ha"},
             "geometry": {"type": "Polygon", "coordinates": [[
                 ll(-30, -5), ll(0, -5), ll(0, 60), ll(-30, 60), ll(-30, -5)]]}},
            {"type": "Feature", "properties": {"name": "east", "rate": 160, "unit": "kg/ha"},
             "geometry": {"type": "Polygon", "coordinates": [[
                 ll(0, -5), ll(30, -5), ll(30, 60), ll(0, 60), ll(0, 60), ll(0, -5)]]}},
        ],
    }


if __name__ == "__main__":
    print("=" * 70)
    print("section_control — 가변시비/섹션컨트롤 SITL 자가검증 (HW 불필요)")
    print("=" * 70)

    # ── 1. CAN 인코드/디코드 라운드트립 + 체크섬 ──────────────
    print("\n▶ 1. FMSC CAN 인코드/디코드")
    rc = encode_rate_cmd(123.4, master_on=True, rate_valid=True,
                         unit=UNIT_KG_HA, ground_speed_mps=1.5, section_count=8)
    assert len(rc) == 8 and rc[7] == _checksum(rc)
    sc = encode_section_cmd(0b10110101, 8, master_on=True, applied_rate=123.4)
    assert sc[7] == _checksum(sc) and ((sc[0] << 8) | sc[1]) == 0b10110101
    # STATUS 라운드트립
    hb = bytearray(8)
    hb[0], hb[1] = 0x00, 0b00001111
    r = int(99.9 * RATE_SCALE)
    hb[2], hb[3] = (r >> 8) & 0xFF, r & 0xFF
    hb[4], hb[5] = FAULT_LOW_HOPPER, 42
    hb[7] = _checksum(hb)
    st = decode_status(bytes(hb))
    assert st and st["section_mask"] == 0b1111 and abs(st["actual_rate"] - 99.9) < 0.05
    assert st["faults"] == FAULT_LOW_HOPPER and st["hopper_pct"] == 42
    # 체크섬 깨짐 → None
    bad = bytearray(hb); bad[7] ^= 0xFF
    assert decode_status(bytes(bad)) is None
    print(f"  ✓ RATE/SECTION/STATUS 8B 라운드트립 + 체크섬 검증")

    # ── 2. 처방맵 위치별 가변율 ───────────────────────────────
    print("\n▶ 2. 처방맵(GeoJSON Rx) 가변 살포율")
    lat0, lon0 = 37.0, 127.0
    app = ApplicationController(
        layout=SectionLayout(width_m=12.0, num_sections=8),
        default_rate=0.0)
    app.set_origin(lat0, lon0)
    summ = app.load_prescription(_demo_geojson(lat0, lon0))
    print(f"  처방맵: {summ}")
    assert summ["zones"] == 2 and summ["rate_max"] == 160
    assert abs(app.rx.rate_at(-15, 20) - 80) < 1e-6,  "서측=80"
    assert abs(app.rx.rate_at(15, 20) - 160) < 1e-6,  "동측=160"
    assert app.rx.rate_at(100, 100) == 0.0,           "구역밖=default(0)"
    print("  ✓ 서측 80 / 동측 160 / 구역밖 0")

    # ── 3. 섹션 컨트롤: 헤드랜드(작업기 들림) → 전 섹션 OFF ────
    print("\n▶ 3. 섹션 ON/OFF 규칙")
    app.set_master(True)
    # 작업기 들림 → 전 OFF
    s = app.update(0, 0, math.pi / 2, 1.5, implement_down=False, dt=0.1)
    assert s["section_mask"] == 0, "헤드랜드 전 OFF"
    assert all(r == "headland" for r in s["section_reasons"])
    # 작업기 내림 + 동측 위치 → 전 ON, 살포율 160 으로 수렴
    for _ in range(40):
        s = app.update(10, 0, math.pi / 2, 1.5, implement_down=True, dt=0.1)
    assert s["section_mask"] == 0xFF, f"동측 전 ON, got {s['section_mask']:#x}"
    assert abs(s["target_rate"] - 160) < 1e-6
    assert abs(s["applied_rate"] - 160) < 1.0, s["applied_rate"]
    print(f"  ✓ 헤드랜드 전OFF / 작업중 전ON, 살포율→{s['applied_rate']} (목표160)")

    # ── 4. 마스터 OFF → 전 OFF + 미검증 송신 차단 ─────────────
    print("\n▶ 4. 마스터 OFF / 미검증 송신 차단")
    app.set_master(False)
    s = app.update(10, 10, math.pi / 2, 1.5, implement_down=True, dt=0.1)
    assert s["section_mask"] == 0, "마스터 OFF → 즉시 전 섹션 OFF"
    for _ in range(20):     # 살포율은 변화율 제한으로 0 까지 램프다운
        s = app.update(10, 10, math.pi / 2, 1.5, implement_down=True, dt=0.1)
    assert s["applied_rate"] == 0.0, s["applied_rate"]
    assert s["can_verified"] is False
    print("  ✓ 마스터 OFF=즉시 전 OFF / 살포율 램프다운→0 / can_verified=False")

    # ── 5. 커버리지 중복 → 재주행 시 OFF (스킵) ───────────────
    print("\n▶ 5. 커버리지 중복 자동 OFF")
    app2 = ApplicationController(layout=SectionLayout(12.0, 8), default_rate=100.0)
    app2.set_origin(lat0, lon0)
    app2.set_master(True)
    # 1차 패스: heading=0(동진, 전진=+x) 로 y=0 라인 주행하며 살포(커버리지 기록)
    x = -10.0
    while x <= 10.0:
        app2.update(x, 0.0, 0.0, 1.5, implement_down=True, dt=0.1)
        x += 1.5 * 0.1
    area1 = app2.coverage.area_m2
    # 2차 패스: 같은 라인 재주행 → 이미 덮인 영역이라 대부분 OFF
    s = app2.update(0.0, 0.0, 0.0, 1.5, implement_down=True, dt=0.1)
    off_overlap = sum(1 for r in s["section_reasons"] if r == "overlap")
    assert off_overlap >= 6, f"중복 OFF 섹션 {off_overlap}/8"
    print(f"  ✓ 1차 살포면적 {area1:.0f}m² → 재주행시 {off_overlap}/8 섹션 중복 OFF")

    # ── 6. 포장 경계 밖 섹션 OFF ──────────────────────────────
    print("\n▶ 6. 포장 경계 밖 섹션 OFF")
    app3 = ApplicationController(layout=SectionLayout(12.0, 8), default_rate=100.0)
    app3.set_master(True)
    # 좁은 경계(x ∈ [-3, 3]) — 폭12m 작업기는 좌우 섹션이 경계 밖
    app3.sections.set_boundary([(-3, -50), (3, -50), (3, 50), (-3, 50)])
    s = app3.update(0, 0, math.pi / 2, 0.0, implement_down=True, dt=0.1)
    out = sum(1 for r in s["section_reasons"] if r == "out_of_field")
    assert 0 < out < 8 and s["section_mask"] not in (0, 0xFF), s["section_reasons"]
    print(f"  ✓ 경계 밖 {out}/8 섹션 OFF (중앙 섹션만 살포)")

    print("\n" + "=" * 70)
    print("section_control 자가검증 6/6 통과.")
    print("  실배포: ApplicationController(bus=ApolloCanBus(...)) +")
    print("  IMPLEMENT_CAN_VERIFIED=True (실 컨트롤러 FMSC 버스 검증 후)")
