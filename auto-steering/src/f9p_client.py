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
from typing import Callable, Optional, Tuple

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
                 read_timeout: float = 1.0,
                 source: str = "f9p"):
        self.port = port
        self.baudrate = baudrate
        self.on_rtk = on_rtk
        self.on_fix_change = on_fix_change      # 품질 변화 시 알림(옵션)
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
        while self._running and self._ser is not None:
            try:
                raw = self._ser.readline()
            except Exception as e:
                log.warning(f"F9P 읽기 오류: {e}")
                continue
            if not raw:
                continue
            try:
                line = raw.decode("ascii", errors="ignore")
            except Exception:
                continue
            self.feed(line)

    def feed(self, line: str):
        """
        한 줄(NMEA 문장)을 처리. 테스트/리플레이에서 직접 호출 가능.
        하드웨어 없이도 parse_gga + 콜백 경로를 검증할 수 있다.
        """
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
