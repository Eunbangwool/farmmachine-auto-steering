"""
f9p_client.py
=============
u-blox ZED-F9P RTK 수신기 USB 시리얼 클라이언트.

CLAUDE.md 우선순위 4: F9pUsbClient → AutoSteerSystem.on_rtk() 콜백 연결.

하드웨어 경로:
    F9P (USB) → Apollo 10 Pro → 이 클라이언트 → on_rtk(lat, lon, quality)

    LoRa NTRIP 보정신호(RTCM)는 rtk-lora-bridge / rtk-leveling 모듈이
    별도로 F9P 입력에 주입한다. 이 모듈은 "출력(NMEA) 파싱 + 콜백"만 담당.

NMEA GGA 메시지에서 위경도 + RTK 품질 파싱:
    품질 0 = Invalid          → SafetyMonitor 거부
         1 = GPS (단독)        → 거부
         2 = DGPS             → 거부
         4 = RTK Fixed        → 허용 (SafetyMonitor)
         5 = RTK Float        → 허용
    (autosteer_core.SafetyMonitor 의 허용값 4/5 와 일치)

사용 예:
    from autosteer_core import AutoSteerSystem, MockCanInterface
    from f9p_client import F9pUsbClient

    sys_   = AutoSteerSystem(MockCanInterface())
    client = F9pUsbClient(port="/dev/ttyACM0", baudrate=38400,
                          on_rtk=sys_.on_rtk)
    client.start()      # 백그라운드 스레드에서 GGA 파싱 → sys_.on_rtk(...)
    ...
    client.stop()

pyserial 미설치/하드웨어 없음 환경에서도 import 는 항상 성공한다.
parse_gga() 는 의존성 없이 단독 테스트 가능.
"""

from __future__ import annotations
import threading
import logging
import struct
from typing import Callable, Optional, Tuple, Dict

log = logging.getLogger("f9p")

# RTK 품질 코드 (NMEA GGA fix quality 필드)
RTK_FIX_FIXED = 4
RTK_FIX_FLOAT = 5


def _dm_to_deg(dm: str) -> float:
    """
    NMEA 도분(ddmm.mmmm / dddmm.mmmm) → 십진도.
    위도는 2자리 도, 경도는 3자리 도 → 소수점 앞 2자리를 분으로 보고 분리.
    """
    dot = dm.index(".")
    deg_len = dot - 2                      # 분은 항상 2자리(정수부) + 소수
    degrees = int(dm[:deg_len])
    minutes = float(dm[deg_len:])
    return degrees + minutes / 60.0


def _nmea_checksum_ok(sentence: str) -> bool:
    """'$....*CS' 체크섬 검증. '*' 없으면 검증 생략(True)."""
    if "*" not in sentence:
        return True
    body, _, cs = sentence.partition("*")
    body = body.lstrip("$")
    cs = cs.strip()[:2]
    if len(cs) < 2:
        return False
    calc = 0
    for ch in body:
        calc ^= ord(ch)
    try:
        return calc == int(cs, 16)
    except ValueError:
        return False


def parse_gga(line: str) -> Optional[Tuple[float, float, int]]:
    """
    GGA 문장 → (lat, lon, quality). 파싱 실패 시 None.
    GxGGA,time,lat,N/S,lon,E/W,quality,numSV,HDOP,alt,M,...*CS
    """
    line = line.strip()
    if "GGA" not in line:
        return None
    if not _nmea_checksum_ok(line):
        log.debug("GGA 체크섬 불일치 — 무시")
        return None
    body = line.split("*", 1)[0]
    f = body.split(",")
    if len(f) < 7:
        return None
    lat_raw, lat_dir = f[2], f[3]
    lon_raw, lon_dir = f[4], f[5]
    if not lat_raw or not lon_raw:
        return None
    try:
        quality = int(f[6]) if f[6] else 0
        lat = _dm_to_deg(lat_raw)
        lon = _dm_to_deg(lon_raw)
    except (ValueError, IndexError):
        return None
    if lat_dir == "S":
        lat = -lat
    if lon_dir == "W":
        lon = -lon
    return (lat, lon, quality)


def parse_hdt(line: str) -> Optional[float]:
    """
    헤딩 문장 → 진북 기준 heading(도). 실패 시 None.
    듀얼안테나(AGMO ver1)·INS 스마트안테나(ver2/NX510/FJD) 모두 진헤딩을 HDT 로 출력:
        $xxHDT,123.4,T*CS   (T=True)
    """
    line = line.strip()
    if "HDT" not in line:
        return None
    if not _nmea_checksum_ok(line):
        return None
    f = line.split("*", 1)[0].split(",")
    if len(f) < 2 or not f[1]:
        return None
    try:
        hdg = float(f[1]) % 360.0
    except ValueError:
        return None
    return hdg


def _ubx_checksum(body: bytes) -> Tuple[int, int]:
    """UBX 8-bit Fletcher 체크섬 (class+id+len+payload 구간)."""
    ck_a = ck_b = 0
    for b in body:
        ck_a = (ck_a + b) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    return ck_a, ck_b


def parse_relposned(frame: bytes) -> Optional[Dict]:
    """
    무빙베이스 RTK 헤딩 프레임(UBX-NAV-RELPOSNED, class 0x01 / id 0x3C) 파싱.

    듀얼안테나(AGMO ver1) 헤딩을 NMEA-HDT(반올림·저레이트) 대신 바이너리로 받아
    에폭마다 **헤딩 + 정확도(accHeading) + 베이스라인 길이 + fix 플래그**를 추출.
    → AutoSteerSystem.on_heading_meas() 에서 적응형 R + fix 게이팅 + 틸트보상에 사용.

    frame: 완전한 UBX 메시지(b5 62 cls id len_lo len_hi payload ck_a ck_b).
    v1 페이로드(64B, relPosHeading 포함)만 허용. v0(40B)·길이불일치·체크섬오류 → None.
    """
    if len(frame) < 8 or frame[0] != 0xB5 or frame[1] != 0x62:
        return None
    if frame[2] != 0x01 or frame[3] != 0x3C:      # NAV-RELPOSNED
        return None
    length = frame[4] | (frame[5] << 8)
    if length != 64:                               # v1(64B)만; v0(40B)엔 헤딩 없음
        return None
    if len(frame) < 6 + length + 2:
        return None
    payload = frame[6:6 + length]
    ck_a, ck_b = _ubx_checksum(frame[2:6 + length])
    if ck_a != frame[6 + length] or ck_b != frame[7 + length]:
        return None

    relPosD       = struct.unpack_from("<i", payload, 16)[0]   # cm
    relPosLength  = struct.unpack_from("<i", payload, 20)[0]   # cm
    relPosHeading = struct.unpack_from("<i", payload, 24)[0]   # 1e-5 deg
    relPosHPD     = struct.unpack_from("<b", payload, 34)[0]   # 0.1 mm
    relPosHPLen   = struct.unpack_from("<b", payload, 35)[0]   # 0.1 mm
    accHeading    = struct.unpack_from("<I", payload, 52)[0]   # 1e-5 deg
    flags         = struct.unpack_from("<I", payload, 60)[0]

    baseline_m = relPosLength * 0.01 + relPosHPLen * 0.0001
    rel_d_m    = relPosD * 0.01 + relPosHPD * 0.0001
    return {
        "heading_deg":   relPosHeading * 1e-5,
        "acc_deg":       accHeading * 1e-5,
        "baseline_m":    baseline_m,
        "rel_d_m":       rel_d_m,
        "fix_ok":        bool(flags & 0x01),         # gnssFixOK
        "valid":         bool(flags & 0x04),         # relPosValid
        "carr_soln":     (flags >> 3) & 0x03,        # 0 none/1 float/2 fixed
        "heading_valid": bool(flags & 0x100),        # relPosHeadingValid
    }


def parse_vtg(line: str) -> Optional[Tuple[float, float]]:
    """
    NMEA VTG → (진로각 course[deg, 북=0], 속도[m/s]). 실패/정지 시 None.
        $xxVTG,course,T,courseM,M,knots,N,kmh,K,mode*CS
    진로각(COG) 보조(방법 5)에 사용.
    """
    if "VTG" not in line:
        return None
    if not _nmea_checksum_ok(line):
        return None
    f = line.split("*", 1)[0].split(",")
    if len(f) < 8:
        return None
    try:
        course = float(f[1]) % 360.0 if f[1] else None
        kmh = float(f[7]) if f[7] else None
    except ValueError:
        return None
    if course is None or kmh is None:
        return None
    return (course, kmh / 3.6)


class _StreamFramer:
    """
    혼재(NMEA/UBX/RTCM3) 바이트 스트림을 프레이밍해 콜백으로 분배.
    F9pUsbClient 바이트 읽기 경로에서 사용(바이너리 UBX-RELPOSNED 수신용).
    프레이밍 규칙은 GNSS 스니퍼(SniffReport._parse)와 동일.
    """
    MAX_BUF = 8192

    def __init__(self, on_nmea: Optional[Callable[[str], None]] = None,
                 on_ubx: Optional[Callable[[bytes], None]] = None):
        self.on_nmea = on_nmea
        self.on_ubx = on_ubx
        self._buf = bytearray()

    def feed_bytes(self, data: bytes):
        self._buf.extend(data)
        buf = self._buf
        while buf:
            fmt = detect_format(bytes(buf[:2]))
            if fmt == "nmea":
                nl = buf.find(b"\n")
                if nl < 0:
                    if len(buf) > self.MAX_BUF:
                        buf.clear()
                    break
                line = bytes(buf[:nl + 1]); del buf[:nl + 1]
                if self.on_nmea:
                    try: self.on_nmea(line.decode("ascii", errors="ignore"))
                    except Exception: pass
            elif fmt == "ubx":
                if len(buf) < 6:
                    break
                length = buf[4] | (buf[5] << 8)
                frame_len = 6 + length + 2
                if frame_len > self.MAX_BUF:
                    del buf[0]; continue
                if len(buf) < frame_len:
                    break
                frame = bytes(buf[:frame_len]); del buf[:frame_len]
                if self.on_ubx:
                    try: self.on_ubx(frame)
                    except Exception: pass
            elif fmt == "rtcm3":
                if len(buf) < 3:
                    break
                length = ((buf[1] & 0x03) << 8) | buf[2]
                frame_len = 3 + length + 3
                if frame_len > self.MAX_BUF:
                    del buf[0]; continue
                if len(buf) < frame_len:
                    break
                del buf[:frame_len]
            else:
                del buf[0]                       # resync: 한 바이트 버림


class F9pUsbClient:
    """
    F9P USB 시리얼에서 NMEA를 읽어 GGA를 파싱하고 on_rtk 콜백을 호출.

    on_rtk(lat, lon, quality) 시그니처는 AutoSteerSystem.on_rtk 와 동일.
    백그라운드 스레드에서 동작하며, control loop 와 비동기로 위치를 갱신한다.
    """
    def __init__(self,
                 port: str = "/dev/ttyACM0",
                 baudrate: int = 38400,
                 on_rtk: Optional[Callable[[float, float, int], None]] = None,
                 on_fix_change: Optional[Callable[[int], None]] = None,
                 on_heading: Optional[Callable[[float], None]] = None,
                 on_heading_meas: Optional[Callable[[Dict], None]] = None,
                 on_velocity: Optional[Callable[[float, float], None]] = None,
                 read_timeout: float = 1.0,
                 source: str = "f9p"):
        self.port = port
        self.baudrate = baudrate
        self.on_rtk = on_rtk
        self.on_fix_change = on_fix_change      # 품질 변화 시 알림(옵션)
        self.on_heading = on_heading            # HDT 진헤딩(도) 콜백 — 듀얼/INS 공통
        self.on_heading_meas = on_heading_meas  # 무빙베이스 RELPOSNED dict(방법 1·3·4)
        self.on_velocity = on_velocity          # VTG (course_deg, speed_mps)(방법 5)
        self.read_timeout = read_timeout
        self.source = source                    # GnssArbiter 소스 라벨

        self._ser = None
        self._thread: Optional[threading.Thread] = None
        self._running = False
        self._last_quality: int = -1
        self._last_fix: Optional[Tuple[float, float, int]] = None

    # ── 수명주기 ────────────────────────────────────────────────
    def start(self) -> bool:
        """시리얼 포트를 열고 읽기 스레드를 시작. 실패 시 False."""
        try:
            import serial  # pyserial
        except ImportError:
            log.error("pyserial 미설치 — 'pip install pyserial' 필요")
            return False
        try:
            self._ser = serial.Serial(self.port, self.baudrate,
                                      timeout=self.read_timeout)
        except Exception as e:
            log.error(f"F9P 포트 열기 실패 ({self.port}): {e}")
            return False

        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True,
                                        name="f9p-reader")
        self._thread.start()
        log.info(f"F9P 클라이언트 시작: {self.port} @ {self.baudrate}")
        return True

    def stop(self):
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        if self._ser is not None:
            try: self._ser.close()
            except Exception: pass
            self._ser = None
        log.info("F9P 클라이언트 종료")

    def write_rtcm(self, data: bytes):
        """NTRIP 으로 받은 RTCM 보정신호를 수신기(F9P) 시리얼에 주입."""
        if self._ser is not None:
            try:
                self._ser.write(data)
            except Exception as e:
                log.debug(f"RTCM write 실패: {e}")

    # ── 읽기 루프 ───────────────────────────────────────────────
    def _loop(self):
        # 바이트 단위 read + 프레이머: NMEA(텍스트)와 UBX(바이너리 RELPOSNED)를 함께 처리.
        framer = _StreamFramer(on_nmea=self.feed, on_ubx=self._on_ubx)
        while self._running and self._ser is not None:
            try:
                n = getattr(self._ser, "in_waiting", 0) or 1
                raw = self._ser.read(n)
            except Exception as e:
                log.warning(f"F9P 읽기 오류: {e}")
                continue
            if not raw:
                continue
            framer.feed_bytes(raw)

    def _on_ubx(self, frame: bytes):
        """UBX 프레임 → RELPOSNED 면 헤딩 측정 dict 콜백(방법 1·3·4)."""
        m = parse_relposned(frame)
        if m is not None and self.on_heading_meas:
            self.on_heading_meas(m)

    def feed(self, line: str):
        """
        한 줄(NMEA 문장)을 처리. 테스트/리플레이에서 직접 호출 가능.
        하드웨어 없이도 parse_gga/parse_hdt + 콜백 경로를 검증할 수 있다.
        """
        # 진헤딩(HDT) — 듀얼안테나(ver1)·INS 스마트안테나(ver2/NX510/FJD) 공통
        if "HDT" in line:
            hdg = parse_hdt(line)
            if hdg is not None and self.on_heading:
                self.on_heading(hdg)
            return
        # 진로각(VTG) — GNSS 속도벡터 보조(방법 5)
        if "VTG" in line:
            v = parse_vtg(line)
            if v is not None and self.on_velocity:
                self.on_velocity(v[0], v[1])
            return
        fix = parse_gga(line)
        if fix is None:
            return
        lat, lon, quality = fix
        self._last_fix = fix
        if quality != self._last_quality:
            self._last_quality = quality
            if self.on_fix_change:
                self.on_fix_change(quality)
            log.info(f"RTK 품질 변경 → {quality} "
                     f"({'Fixed' if quality==RTK_FIX_FIXED else 'Float' if quality==RTK_FIX_FLOAT else '비RTK'})")
        if self.on_rtk:
            self.on_rtk(lat, lon, quality)

    # ── 상태 조회 ───────────────────────────────────────────────
    @property
    def last_fix(self) -> Optional[Tuple[float, float, int]]:
        return self._last_fix

    @property
    def has_rtk(self) -> bool:
        """현재 RTK Fixed/Float 인지 (SafetyMonitor 허용 품질)."""
        return self._last_quality in (RTK_FIX_FIXED, RTK_FIX_FLOAT)


class ChcnavPa3SerialClient(F9pUsbClient):
    """
    CHCNAV PA-3 스마트 안테나의 NMEA(RS232) 출력 클라이언트.

    PA-3도 NMEA-0183을 출력하므로 parse_gga / F9pUsbClient 로직을 그대로 재사용.
    차이는 baud(공장 기본 115200)와 소스 라벨('pa3')뿐. PA-3는 INS 융합
    위치/heading을 내보내므로 GnssArbiter에서 'pa3'(주)로 F9P보다 우선한다.

    스펙(autosteer_core.CHCNAV_PA3): CAN 500k / RS232 ≤115200 / NMEA 10Hz / IMU 100Hz.
    포트는 PA-3가 물린 시리얼(예: COM1 UART0 → /dev/ttyS0, COM2 UART1 → /dev/ttyS1)에 맞춰 지정.
    ★ CAN 출력(위치+자세)을 쓰려면 CHCNAV OEM CAN 프로토콜 문서 필요(별도).
    """
    def __init__(self,
                 port: str = "/dev/ttyS1",
                 baudrate: int = 115200,
                 on_rtk: Optional[Callable[[float, float, int], None]] = None,
                 on_fix_change: Optional[Callable[[int], None]] = None,
                 read_timeout: float = 1.0):
        super().__init__(port=port, baudrate=baudrate, on_rtk=on_rtk,
                         on_fix_change=on_fix_change, read_timeout=read_timeout,
                         source="pa3")


# ═══════════════════════════════════════════════════════════════
#  GNSS 스트림 정찰(sniff) — 포맷/보레이트/메시지 분석
#  실장비 'UART tool' 대체: 무엇이 어떤 포맷으로 나오는지 직접 확인
# ═══════════════════════════════════════════════════════════════

UBX_SYNC  = b"\xb5\x62"      # UBX 바이너리 동기 헤더 (µb)
RTCM3_PRE = 0xD3            # RTCM3 프리앰블


def detect_format(head: bytes) -> str:
    """버퍼 맨 앞으로 프레임 포맷 추정: 'nmea'/'ubx'/'rtcm3'/'unknown'."""
    if not head:
        return "unknown"
    if head[0] == 0x24:                       # '$'
        return "nmea"
    if head[:2] == UBX_SYNC:
        return "ubx"
    if head[0] == RTCM3_PRE:
        return "rtcm3"
    return "unknown"


class SniffReport:
    """
    GNSS 원시 스트림 분석기. NMEA/UBX/RTCM3 혼재 스트림을 프레이밍해
    포맷·메시지·RTK 품질 통계를 낸다. feed_bytes 로 하드웨어 없이 테스트 가능.
    """
    MAX_BUF = 8192

    def __init__(self):
        self.total_bytes  = 0
        self.formats      = set()
        self.nmea_by_type = {}
        self.talkers      = {}
        self.nmea_ok      = 0
        self.nmea_bad     = 0
        self.ubx_frames   = 0
        self.rtcm3_frames = 0
        self.dropped      = 0
        self.gga_count    = 0
        self.gga_qualities = set()
        self.last_fix     = None
        self._buf = bytearray()

    def feed_bytes(self, data: bytes):
        self.total_bytes += len(data)
        self._buf.extend(data)
        self._parse()

    def _parse(self):
        buf = self._buf
        while buf:
            fmt = detect_format(bytes(buf[:2]))
            if fmt == "nmea":
                nl = buf.find(b"\n")
                if nl < 0:                       # 미완성 문장 → 더 기다림
                    if len(buf) > self.MAX_BUF:
                        self.dropped += len(buf); buf.clear()
                    break
                line = bytes(buf[:nl + 1]); del buf[:nl + 1]
                self._on_nmea(line)
            elif fmt == "ubx":
                if len(buf) < 6:
                    break
                length = buf[4] | (buf[5] << 8)  # payload len (LE)
                frame_len = 6 + length + 2       # hdr+cls+id+len + payload + ck
                if len(buf) < frame_len:
                    if frame_len > self.MAX_BUF:
                        del buf[0]; self.dropped += 1
                    break
                del buf[:frame_len]
                self.ubx_frames += 1; self.formats.add("ubx")
            elif fmt == "rtcm3":
                if len(buf) < 3:
                    break
                length = ((buf[1] & 0x03) << 8) | buf[2]
                frame_len = 3 + length + 3       # hdr+len + payload + CRC24
                if len(buf) < frame_len:
                    if frame_len > self.MAX_BUF:
                        del buf[0]; self.dropped += 1
                    break
                del buf[:frame_len]
                self.rtcm3_frames += 1; self.formats.add("rtcm3")
            else:
                del buf[0]; self.dropped += 1    # resync: 한 바이트 버림

    def _on_nmea(self, raw: bytes):
        line = raw.decode("ascii", errors="ignore").strip()
        if not line.startswith("$") or len(line) < 6:
            return
        self.formats.add("nmea")
        tag = line[1:6]                          # 예: GNGGA
        talker, mtype = tag[:2], tag[2:5]
        self.talkers[talker]   = self.talkers.get(talker, 0) + 1
        self.nmea_by_type[mtype] = self.nmea_by_type.get(mtype, 0) + 1
        if _nmea_checksum_ok(line):
            self.nmea_ok += 1
        else:
            self.nmea_bad += 1
        if mtype == "GGA":
            fix = parse_gga(line)
            if fix:
                self.gga_count += 1
                self.gga_qualities.add(fix[2])
                self.last_fix = fix

    def summary(self) -> dict:
        return {
            "total_bytes": self.total_bytes,
            "formats": sorted(self.formats),
            "nmea_by_type": dict(self.nmea_by_type),
            "talkers": dict(self.talkers),
            "nmea_ok": self.nmea_ok, "nmea_bad": self.nmea_bad,
            "ubx_frames": self.ubx_frames, "rtcm3_frames": self.rtcm3_frames,
            "gga_count": self.gga_count,
            "gga_qualities": sorted(self.gga_qualities),
            "last_fix": self.last_fix, "dropped": self.dropped,
        }

    def format_report(self) -> str:
        s = self.summary()
        L = [f"수신 {s['total_bytes']}B | 포맷: {', '.join(s['formats']) or '없음'}"]
        if s["nmea_by_type"]:
            L.append("NMEA 문장: " + ", ".join(
                f"{k}×{v}" for k, v in sorted(s['nmea_by_type'].items())))
            L.append("NMEA talker: " + ", ".join(
                f"{k}×{v}" for k, v in s['talkers'].items()))
            L.append(f"NMEA 체크섬: OK {s['nmea_ok']} / 오류 {s['nmea_bad']}")
        if s["gga_count"]:
            q = s["gga_qualities"]
            rtk = "✅ RTK Fixed/Float" if (4 in q or 5 in q) else "⚠ RTK 아님(보정신호 확인)"
            L.append(f"GGA {s['gga_count']}건, 품질코드 {q} → {rtk}")
            if s["last_fix"]:
                la, lo, qq = s["last_fix"]
                L.append(f"마지막 위치: {la:.7f}, {lo:.7f} (q={qq})")
        if s["ubx_frames"]:
            L.append(f"⚠ UBX 바이너리 {s['ubx_frames']}프레임 — NMEA 전환 또는 UBX 파서 필요")
        if s["rtcm3_frames"]:
            L.append(f"RTCM3 {s['rtcm3_frames']}프레임 (보정신호 스트림)")
        rec = self._recommend(s)
        if rec:
            L.append(f"권고: {rec}")
        return "\n".join("  " + x for x in L)

    @staticmethod
    def _recommend(s: dict) -> str:
        if s["ubx_frames"] and s["nmea_ok"] == 0:
            return "UBX 전용 출력 → 수신기 설정에서 NMEA(GGA) 활성화 (그러면 parse_gga 사용 가능)"
        if s["nmea_ok"] and s["nmea_bad"] > s["nmea_ok"]:
            return "체크섬 오류 과다 → 보레이트 불일치 의심 (detect_baudrate 사용)"
        if s["nmea_ok"] and s["gga_count"] == 0:
            return "NMEA는 나오나 GGA 없음 → 수신기에서 GGA 문장 활성화"
        if 4 in s["gga_qualities"] or 5 in s["gga_qualities"]:
            return "정상 — GGA + RTK fix 확인. on_rtk 연결 가능"
        return ""


class GnssSniffer:
    """
    실장비 시리얼 포트를 열어 원시 바이트를 SniffReport 로 분석.
    포트/포맷/보레이트를 모를 때 가장 먼저 돌려보는 정찰 도구.
        rep = GnssSniffer("/dev/ttyS1", 115200, echo=True).run(5.0)
        print(rep.format_report())
    """
    def __init__(self, port: str = "/dev/ttyACM0",
                 baudrate: int = 115200, echo: bool = False):
        self.port = port
        self.baudrate = baudrate
        self.echo = echo

    def run(self, duration: float = 5.0) -> SniffReport:
        import time
        report = SniffReport()
        try:
            import serial
        except ImportError:
            log.error("pyserial 미설치 — 'pip install pyserial'")
            return report
        try:
            ser = serial.Serial(self.port, self.baudrate, timeout=0.2)
        except Exception as e:
            log.error(f"포트 열기 실패({self.port}@{self.baudrate}): {e}")
            return report
        t0 = time.time()
        try:
            while time.time() - t0 < duration:
                data = ser.read(256)
                if data:
                    report.feed_bytes(data)
                    if self.echo:
                        print(data.decode("ascii", "replace"), end="")
        finally:
            ser.close()
        return report


def detect_baudrate(port: str = "/dev/ttyACM0",
                    candidates=(115200, 38400, 9600, 460800, 57600, 230400),
                    window: float = 2.0):
    """
    후보 보레이트를 순회하며 가장 유효한 스트림을 찾는다.
    점수 = NMEA 체크섬 OK×2 + UBX + RTCM3 프레임 수.
    반환: (best_baud, SniffReport) 또는 (None, None).
    """
    best = (None, None, 0)
    for baud in candidates:
        rep = GnssSniffer(port, baud).run(window)
        score = rep.nmea_ok * 2 + rep.ubx_frames + rep.rtcm3_frames
        log.info(f"baud {baud}: 점수 {score} "
                 f"(NMEA_ok={rep.nmea_ok}, UBX={rep.ubx_frames}, RTCM3={rep.rtcm3_frames})")
        if score > best[2]:
            best = (baud, rep, score)
    return (best[0], best[1]) if best[2] > 0 else (None, None)


def scan_ports(ports=None,
               bauds=(115200, 38400, 460800, 9600, 57600, 230400),
               window: float = 1.5):
    """
    여러 시리얼 포트를 순회하며 GNSS(NMEA/UBX)가 나오는 포트를 자동 탐지.
    AGMO ver1 = Apollo **내부 UART** 직결이라 어느 /dev/ttySx 인지 모를 때 현장 1단계용.
    ports=None 이면 /dev/ttyS* + /dev/ttyUSB* + /dev/ttyACM* 자동 후보.
    반환 dict: {"best": {port,baud,score,...}|None, "ports": [포트별 요약...]}.
    """
    import glob as _glob
    if ports is None:
        ports = sorted(set(_glob.glob("/dev/ttyS*") +
                           _glob.glob("/dev/ttyUSB*") +
                           _glob.glob("/dev/ttyACM*")))
    results = []; best = None
    for p in ports:
        baud, rep = detect_baudrate(p, candidates=bauds, window=window)
        if rep is None:
            results.append({"port": p, "found": False, "score": 0}); continue
        s = rep.summary()
        score = s["nmea_ok"] * 2 + s["ubx_frames"] + s["rtcm3_frames"]
        entry = {"port": p, "found": score > 0, "baud": baud, "score": score,
                 "formats": s["formats"], "nmea_by_type": s["nmea_by_type"],
                 "gga_count": s["gga_count"], "gga_qualities": s["gga_qualities"],
                 "ubx_frames": s["ubx_frames"]}
        results.append(entry)
        if score > 0 and (best is None or score > best["score"]):
            best = entry
    log.info(f"GNSS 포트 스캔: best={best['port'] if best else None}")
    return {"best": best, "ports": results}


# ═══════════════════════════════════════════════════════════════
#  UBX-CFG — u-blox 설정 메시지 빌더 (무빙베이스 듀얼안테나 heading 활성)
#  레거시 CFG-MSG/CFG-RATE/CFG-CFG (F9P 하위호환 수용). 동작 사실=u-blox 프로토콜 공개표준.
# ═══════════════════════════════════════════════════════════════

def build_ubx(msg_class: int, msg_id: int, payload: bytes = b"") -> bytes:
    """UBX 프레임 = B5 62 cls id len(LE2) payload ck_a ck_b (Fletcher)."""
    body = bytes([msg_class & 0xFF, msg_id & 0xFF,
                  len(payload) & 0xFF, (len(payload) >> 8) & 0xFF]) + bytes(payload)
    cka, ckb = _ubx_checksum(body)
    return b"\xb5\x62" + body + bytes([cka, ckb])


def ubx_cfg_msg(msg_class: int, msg_id: int, rate: int = 1) -> bytes:
    """CFG-MSG(0x06 0x01): 해당 메시지를 명령 수신 포트에서 rate(0=off) 로 출력."""
    return build_ubx(0x06, 0x01, bytes([msg_class & 0xFF, msg_id & 0xFF, rate & 0xFF]))


def ubx_cfg_rate(meas_ms: int = 100, nav_rate: int = 1, time_ref: int = 1) -> bytes:
    """CFG-RATE(0x06 0x08): meas_ms(측정주기) / nav_rate(사이클) / time_ref(1=GPS)."""
    import struct
    return build_ubx(0x06, 0x08, struct.pack("<HHH", meas_ms, nav_rate, time_ref))


def ubx_cfg_save() -> bytes:
    """CFG-CFG(0x06 0x09): 현재 설정을 BBR+Flash+EEPROM 에 저장(전원유지)."""
    import struct
    return build_ubx(0x06, 0x09,
                     struct.pack("<IIIB", 0x00000000, 0x0000FFFF, 0x00000000, 0x17))


# UBX 메시지 ID (class 0x01=NAV, 0xF0=NMEA 표준) — 공개 프로토콜 확정값
_NAV = 0x01
_NMEA = 0xF0


def moving_base_heading_cfg(meas_ms: int = 100):
    """
    듀얼안테나 heading(UBX-NAV-RELPOSNED) + 위치/속도 출력 활성 + 저장하는 UBX 프레임 목록.
    farmmachine 파서 정합: RELPOSNED(헤딩)·PVT·VELNED(COG, 방법5)·NMEA GGA(위치)·VTG.
    잡음 NMEA(GLL/GSA/GSV)는 끈다.

    ★ 전제: 수신기가 **무빙베이스/로버 모드**여야 RELPOSNED 가 유효 heading 을 준다
      (AGMO ver1 돔은 공장에서 그렇게 설정됐을 가능성 높음). 적용 후에도 RELPOSNED 가
      안 나오거나 carrSoln/heading_valid 가 0 이면 → 무빙베이스 모드 미설정 → 현장 조사 필요.
    헤딩 NMEA(HDT)는 표준 ID 불확실 + UBX RELPOSNED 가 주경로라 활성하지 않음.
    """
    return [
        ubx_cfg_rate(meas_ms, 1, 1),
        ubx_cfg_msg(_NAV,  0x3C, 1),   # NAV-RELPOSNED — 듀얼안테나 heading
        ubx_cfg_msg(_NAV,  0x07, 1),   # NAV-PVT — 위치/속도
        ubx_cfg_msg(_NAV,  0x12, 1),   # NAV-VELNED — 진로각(COG, 방법5)
        ubx_cfg_msg(_NMEA, 0x00, 1),   # NMEA GGA — 위치(parse_gga)
        ubx_cfg_msg(_NMEA, 0x05, 1),   # NMEA VTG — 진로각/속도(parse_vtg)
        ubx_cfg_msg(_NMEA, 0x01, 0),   # GLL off
        ubx_cfg_msg(_NMEA, 0x02, 0),   # GSA off
        ubx_cfg_msg(_NMEA, 0x03, 0),   # GSV off
        ubx_cfg_save(),
    ]


def configure_serial(port: str, baud: int = 115200, frames=()) -> str:
    """UBX 설정 프레임들을 시리얼 포트에 순차 기록. 'ok'/'no-pyserial'/'no-port'."""
    import time as _t
    try:
        import serial
    except ImportError:
        log.error("pyserial 미설치"); return "no-pyserial"
    try:
        ser = serial.Serial(port, baud, timeout=0.5)
    except Exception as e:
        log.error(f"설정 포트 열기 실패({port}@{baud}): {e}"); return "no-port"
    try:
        for f in frames:
            ser.write(f); ser.flush(); _t.sleep(0.05)
    finally:
        ser.close()
    log.info(f"UBX 설정 {len(list(frames))}프레임 기록 → {port}@{baud}")
    return "ok"


# ═══════════════════════════════════════════════════════════════
#  단독 테스트 (하드웨어 없이 GGA 파싱 + 콜백 경로 검증)
# ═══════════════════════════════════════════════════════════════
if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO,
                        format="%(levelname)s %(name)s: %(message)s")

    # 실제 F9P 출력 예시 (체크섬 포함). quality 필드만 바꿔 RTK 상태 시뮬.
    samples = [
        # 단독측위(1) — SafetyMonitor 거부 대상
        "$GNGGA,123519.00,3722.55000,N,12701.23000,E,1,08,0.9,55.2,M,18.0,M,,*45",
        # RTK Float(5)
        "$GNGGA,123520.00,3722.55010,N,12701.23010,E,5,12,0.6,55.3,M,18.0,M,1.0,0000*61",
        # RTK Fixed(4)
        "$GNGGA,123521.00,3722.55020,N,12701.23020,E,4,14,0.5,55.3,M,18.0,M,0.8,0000*6D",
        # GGA 아님 (무시)
        "$GNRMC,123521.00,A,3722.55020,N,12701.23020,E,0.02,,010620,,,A*5F",
        # 위도/경도 비어있음 (무시)
        "$GNGGA,123522.00,,,,,0,00,99.9,,M,,M,,*44",
    ]

    received = []
    client = F9pUsbClient(on_rtk=lambda la, lo, q: received.append((la, lo, q)))

    print("=" * 70)
    print("F9pUsbClient — GGA 파싱 + on_rtk 콜백 테스트 (하드웨어 불필요)")
    print("=" * 70)
    for s in samples:
        client.feed(s)

    print(f"\nGGA 콜백 수신 {len(received)}건:")
    for la, lo, q in received:
        tag = {4: "RTK Fixed ✅", 5: "RTK Float ✅"}.get(q, f"품질 {q} ❌(거부)")
        print(f"  lat={la:.7f}  lon={lo:.7f}  quality={q}  {tag}")

    print("\n검증:")
    assert len(received) == 3, f"GGA 3건만 콜백돼야 함 (실제 {len(received)})"
    assert abs(received[0][0] - 37.3758333) < 1e-5, "위도 변환 오류"
    assert abs(received[0][1] - 127.0205) < 1e-4, "경도 변환 오류"
    assert received[1][2] == 5 and received[2][2] == 4
    assert client.has_rtk, "마지막 fix(4)는 RTK 허용 상태여야 함"
    print("  ✓ 도분→십진도 변환 정확")
    print("  ✓ 비GGA/빈좌표 문장 무시")
    print("  ✓ RTK 품질 4/5 식별")
    print("\n다음 단계: AutoSteerSystem.rtk_callback('f9p'/'pa3') 로 연결하면 끝.")

    # ── 스트림 정찰(sniff): NMEA + UBX + RTCM3 혼재 스트림 분석 ──────
    print("\n" + "=" * 70)
    print("GNSS 스트림 정찰(sniff) — NMEA/UBX/RTCM3 혼재 분석 (하드웨어 불필요)")
    print("=" * 70)

    # 합성 스트림: GGA 3건(q=1/5/4) + RMC + UBX 1프레임 + RTCM3 1프레임
    ubx_frame   = b"\xb5\x62\x01\x07\x00\x00\x00\x00"          # NAV-PVT(len0) + dummy ck
    rtcm3_frame = b"\xd3\x00\x02\xab\xcd\x00\x00\x00"          # len2 + payload2 + CRC3
    stream = (
        (samples[0] + "\r\n").encode()
        + (samples[1] + "\r\n").encode()
        + ubx_frame
        + (samples[2] + "\r\n").encode()
        + (samples[3] + "\r\n").encode()      # RMC
        + rtcm3_frame
    )

    report = SniffReport()
    # 일부러 두 조각으로 쪼개 넣어 프레임 경계 버퍼링까지 검증
    report.feed_bytes(stream[:50])
    report.feed_bytes(stream[50:])
    print(report.format_report())

    s = report.summary()
    assert "nmea" in s["formats"] and "ubx" in s["formats"] and "rtcm3" in s["formats"]
    assert s["gga_count"] == 3, s
    assert s["ubx_frames"] == 1 and s["rtcm3_frames"] == 1, s
    assert 4 in s["gga_qualities"] and 5 in s["gga_qualities"]
    assert s["nmea_bad"] == 0, "체크섬 모두 통과해야 함"
    print("\n  ✓ 포맷 자동 감지(NMEA/UBX/RTCM3)")
    print("  ✓ 프레임 경계 분할 수신 버퍼링")
    print("  ✓ GGA 추출 + RTK 품질 + UBX/RTCM3 카운트")
    print("\n실장비 사용: GnssSniffer('/dev/ttyS1',115200,echo=True).run(5) "
          "또는 detect_baudrate('/dev/ttyS1')")
