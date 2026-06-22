"""
implement_gnss.py — 작업기(균평기) 안테나 GNSS + 레벨 그리드 (벤더 독립)

★ 균평기는 GNSS 안테나가 2개다(혼동 금지):
  - 차체 안테나 : 위치+주행(자율조향). 벤더별 포트(AGMO Ver2=/dev/ttyS4, CHCNAV=/dev/ttyS6 …).
  - 작업기 안테나: 후방 작업기에 장착, **레벨(표고) 측정 전용**. 별도 USB 시리얼 GNSS(/dev/ttyUSB*).

이 모듈은 **작업기 안테나 전용 독립 레이어**다. 차체 주행 GNSS/EKF 와 절대 간섭하지 않는다.
모든 제조사(agmo_dual/agmo_single/chcnav/fjd) 공통으로 동작(벤더 프로파일과 분리).

표고는 반드시 작업기 안테나에서 받는다(차체 안테나 표고 사용 금지). GGA 의 고도(alt) 사용.
fix=4(RTK fixed)/5(float) 추적 — fix 아니면 표고 부정확 → UI 가 신뢰도 표시/제외.

의존: pyserial(없으면 graceful 비동작). NMEA 파서는 f9p_client 헬퍼 재사용.
"""
from __future__ import annotations
import math
import threading
import time
import logging
from typing import Optional, Tuple, List, Dict

log = logging.getLogger("implement_gnss")

# NMEA 체크섬/도분 변환은 차체 GNSS 파서(f9p_client) 와 동일 구현 재사용.
from f9p_client import _nmea_checksum_ok, _dm_to_deg


# ── 작업기 안테나 설치 높이(지면→안테나) — 표고 환산용 ──────────────
#   지면표고 = GNSS표고(alt) − 안테나높이. ★ TODO(HW): 실측값을 UI 에서 입력.
DEFAULT_IMPL_ANTENNA_HEIGHT_M = 1.5

DEFAULT_CELL_SIZE_M = 0.5
# ★ TODO(HW): 작업기 GNSS 보레이트 확인(9600/115200). 후보를 순차 시도.
BAUD_CANDIDATES = (115200, 9600, 460800, 38400)
# USB 시리얼 후보(4G 모뎀 ttyUSB 와 구분: NMEA(GGA) 나오는 포트만 채택).
USB_PORT_CANDIDATES = ("/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyUSB2",
                       "/dev/ttyUSB3", "/dev/ttyACM0", "/dev/ttyACM1")


def parse_gga_alt(line: str) -> Optional[Tuple[float, float, float, int, int]]:
    """
    GGA → (lat, lon, alt_m, quality, n_sats). 실패 시 None.
    GxGGA,time,lat,N/S,lon,E/W,quality,numSV,HDOP,alt,M,geoid,M,...*CS
    """
    line = line.strip()
    if "GGA" not in line or not _nmea_checksum_ok(line):
        return None
    f = line.split("*", 1)[0].split(",")
    if len(f) < 10:
        return None
    lat_raw, lat_dir, lon_raw, lon_dir = f[2], f[3], f[4], f[5]
    if not lat_raw or not lon_raw:
        return None
    try:
        quality = int(f[6]) if f[6] else 0
        n_sats = int(f[7]) if f[7] else 0
        lat = _dm_to_deg(lat_raw)
        lon = _dm_to_deg(lon_raw)
        alt = float(f[9]) if f[9] else float("nan")
    except (ValueError, IndexError):
        return None
    if lat_dir == "S":
        lat = -lat
    if lon_dir == "W":
        lon = -lon
    return (lat, lon, alt, quality, n_sats)


class LevelerGrid:
    """
    작업기 표고 샘플 → 그리드 셀 평균 → 기준면 대비 편차(cm).
    좌표: 첫 샘플을 원점으로 한 로컬 ENU(equirectangular). 셀=cell_size_m.
    스레드 안전(주행 루프와 무관하지만 폴링/수신 스레드 분리이므로 lock).
    """
    def __init__(self, cell_size_m: float = DEFAULT_CELL_SIZE_M,
                 antenna_height_m: float = DEFAULT_IMPL_ANTENNA_HEIGHT_M,
                 start_avg_n: int = 20):
        self.cell = float(cell_size_m)
        self.antenna_h = float(antenna_height_m)
        self._lock = threading.Lock()
        self._cells: Dict[Tuple[int, int], dict] = {}
        self._lat0 = None
        self._lon0 = None
        self._coslat0 = 1.0
        self._ref = None               # 기준면 지면표고(m). None=미설정
        self._ref_mode = "start"       # "start"(시작구간 평균) / "manual"(영점 버튼)
        self._start_buf: List[float] = []
        self._start_avg_n = int(start_avg_n)
        self.last_fix = 0
        self.last_sample_t = 0.0

    def set_antenna_height(self, h: float):
        with self._lock:
            self.antenna_h = float(h)

    def _enu(self, lat, lon):
        e = math.radians(lon - self._lon0) * 6378137.0 * self._coslat0
        n = math.radians(lat - self._lat0) * 6378137.0
        return e, n

    def add_sample(self, lat, lon, alt, fix, n_sats=0):
        if alt != alt:   # NaN
            return
        with self._lock:
            self.last_fix = int(fix)
            self.last_sample_t = time.time()
            if self._lat0 is None:
                self._lat0, self._lon0 = lat, lon
                self._coslat0 = math.cos(math.radians(lat))
            ground = alt - self.antenna_h        # 지면표고 환산
            # 기준면(start 모드): 시작 구간 평균을 0 기준으로
            if self._ref is None and self._ref_mode == "start":
                self._start_buf.append(ground)
                if len(self._start_buf) >= self._start_avg_n:
                    self._ref = sum(self._start_buf) / len(self._start_buf)
            e, n = self._enu(lat, lon)
            gx = int(math.floor(e / self.cell))
            gy = int(math.floor(n / self.cell))
            c = self._cells.get((gx, gy))
            if c is None:
                c = {"sum": 0.0, "count": 0, "fix": int(fix)}
                self._cells[(gx, gy)] = c
            c["sum"] += ground
            c["count"] += 1
            c["fix"] = int(fix)          # 최신 fix 상태

    def set_reference_here(self) -> Optional[float]:
        """현재까지 마지막 셀 평균을 기준면으로(영점 버튼). manual 모드 전환."""
        with self._lock:
            self._ref_mode = "manual"
            # 가장 최근 샘플 위치의 셀 평균을 기준으로
            if not self._cells:
                return None
            # 최근 갱신 셀: count 기준 마지막 추가 대신, 전체 평균을 기준으로(간단·안정).
            tot = sum(v["sum"] for v in self._cells.values())
            cnt = sum(v["count"] for v in self._cells.values())
            self._ref = (tot / cnt) if cnt else None
            return self._ref

    def clear(self):
        with self._lock:
            self._cells.clear()
            self._lat0 = self._lon0 = None
            self._ref = None
            self._ref_mode = "start"
            self._start_buf = []

    def snapshot(self) -> dict:
        with self._lock:
            ref = self._ref
            cells = []
            if ref is not None:
                for (gx, gy), v in self._cells.items():
                    if v["count"] <= 0:
                        continue
                    mean = v["sum"] / v["count"]
                    cells.append({
                        "gx": gx, "gy": gy,
                        "dev_cm": round((mean - ref) * 100.0, 1),
                        "n": v["count"], "fix": v["fix"],
                    })
            return {
                "reference_cm": (round(ref * 100.0, 1) if ref is not None else None),
                "ref_mode": self._ref_mode,
                "cell_size_m": self.cell,
                "antenna": "implement",          # ★ 작업기 안테나 기준 명시
                "antenna_height_m": self.antenna_h,
                "origin": ({"lat": self._lat0, "lon": self._lon0}
                           if self._lat0 is not None else None),
                "cells": cells,
                "cell_count": len(self._cells),
            }


class ImplementGnss:
    """
    작업기 안테나 USB 시리얼 GNSS — 독립 스레드 수신. 차체 주행 GNSS 와 분리.
    GGA → LevelerGrid.add_sample. pyserial 없거나 포트 없으면 graceful(ok=False).
    """
    def __init__(self, grid: LevelerGrid = None):
        self.grid = grid or LevelerGrid()
        self._ser = None
        self._thread = None
        self._running = False
        self.port = None
        self.baud = None
        self.last = {"lat": None, "lon": None, "alt": None, "fix": 0, "n_sats": 0}
        self.error = ""

    def _open_port(self, port: str, baud: int):
        import serial
        s = serial.Serial(port, baud, timeout=1.0)
        return s

    def _detect(self, ports, bauds) -> Optional[Tuple[str, int, object]]:
        """후보 포트×보레이트 중 GGA 가 실제로 나오는 포트 선택(4G 모뎀 포트 배제)."""
        try:
            import serial  # noqa
        except Exception as e:
            self.error = "pyserial 없음: %s" % e
            return None
        for p in ports:
            for b in bauds:
                ser = None
                try:
                    ser = self._open_port(p, b)
                    t_end = time.time() + 1.5
                    while time.time() < t_end:
                        ln = ser.readline().decode("ascii", "ignore")
                        if parse_gga_alt(ln) is not None:
                            return (p, b, ser)     # NMEA 나오는 포트 = 작업기 GNSS
                    ser.close()
                except Exception:
                    if ser is not None:
                        try: ser.close()
                        except Exception: pass
                    continue
        self.error = "작업기 GNSS 포트 미발견(USB/모뎀 구분 실패 또는 미연결)"
        return None

    def start(self, port: str = None, baud: int = None) -> bool:
        """비동기 시작 — 포트 탐지(수십 초 가능)를 백그라운드에서. 즉시 반환(UI 프리즈 방지).
        탐지 성공 여부는 status().impl_gnss_ok / impl_gnss_state 로 폴링."""
        if self._running:
            return True
        self._req = ([port] if port else list(USB_PORT_CANDIDATES),
                     [int(baud)] if baud else list(BAUD_CANDIDATES))
        self.error = ""
        self._running = True
        self._thread = threading.Thread(target=self._run, daemon=True,
                                        name="implement-gnss")
        self._thread.start()
        return True

    def _run(self):
        ports, bauds = self._req
        found = self._detect(ports, bauds)
        if not found:
            self._running = False     # 탐지 실패 → off (error 에 사유)
            return
        self.port, self.baud, self._ser = found
        log.info("작업기 GNSS 연결 %s @%d", self.port, self.baud)
        self._loop()

    def _loop(self):
        while self._running and self._ser is not None:
            try:
                ln = self._ser.readline().decode("ascii", "ignore")
            except Exception as e:
                self.error = "읽기 실패: %s" % e
                break
            r = parse_gga_alt(ln)
            if r is None:
                continue
            lat, lon, alt, q, ns = r
            self.last = {"lat": lat, "lon": lon, "alt": alt, "fix": q, "n_sats": ns}
            # fix=4/5(RTK) 만 그리드 누적 — Single/0 표고는 부정확하므로 제외.
            if q in (4, 5):
                self.grid.add_sample(lat, lon, alt, q, ns)

    def stop(self):
        self._running = False
        if self._ser is not None:
            try: self._ser.close()
            except Exception: pass
        self._ser = None

    def status(self) -> dict:
        connected = bool(self._running and self._ser is not None)
        state = "connected" if connected else ("detecting" if self._running else "off")
        return {
            "impl_gnss_ok": connected,
            "impl_gnss_state": state,
            "impl_gnss_fix": int(self.last.get("fix", 0)),
            "impl_gnss_port": self.port,
            "impl_gnss_sats": int(self.last.get("n_sats", 0)),
            "impl_gnss_alt": self.last.get("alt"),
            "impl_gnss_error": self.error,
        }


if __name__ == "__main__":
    # 셀프테스트: 합성 GGA 주입(시리얼 없이) → 그리드/편차 검증
    logging.basicConfig(level=logging.INFO)
    g = LevelerGrid(cell_size_m=0.5, antenna_height_m=1.0, start_avg_n=5)

    def gga(lat, lon, alt, q=4, ns=20):
        body = "$GNGGA,000000,%s,N,%s,E,%d,%02d,0.8,%.3f,M,0.0,M,,"
        # lat/lon 을 도분(ddmm.mmmm)으로 — 간단히 위도 37도, 경도 127도 부근
        latdm = "3700.%04d" % int((lat - 37) * 600000) if lat >= 37 else "3700.0000"
        londm = "12700.%04d" % int((lon - 127) * 600000) if lon >= 127 else "12700.0000"
        s = body % (latdm, londm, q, ns, alt)
        cs = 0
        for ch in s[1:]:
            cs ^= ord(ch)
        return "%s*%02X" % (s, cs)

    # 기준면(시작 5샘플) ~ alt 100.0 → 지면 99.0
    base_lat, base_lon = 37.0001, 127.0001
    for i in range(5):
        r = parse_gga_alt(gga(base_lat, base_lon, 100.0))
        assert r is not None, "GGA 파싱 실패"
        g.add_sample(*r[:4])
    snap = g.snapshot()
    assert snap["reference_cm"] is not None, "기준면 미설정"
    # 다른 셀: alt 100.05 (5cm 높음 → +5cm 깎기)
    r = parse_gga_alt(gga(37.0002, 127.0002, 100.05))
    g.add_sample(*r[:4])
    snap = g.snapshot()
    devs = [c["dev_cm"] for c in snap["cells"]]
    print("기준면_cm=%s  편차들=%s  셀수=%d"
          % (snap["reference_cm"], devs, snap["cell_count"]))
    assert any(abs(d - 5.0) < 1.5 for d in devs), "5cm 편차 셀 미검출: %s" % devs
    print("antenna=%s cell=%.1fm" % (snap["antenna"], snap["cell_size_m"]))
    print("\n  ✓ implement_gnss 셀프테스트 통과 (GGA alt 파싱 + 그리드 편차)")
