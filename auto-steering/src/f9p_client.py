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
                 read_timeout: float = 1.0):
        self.port = port
        self.baudrate = baudrate
        self.on_rtk = on_rtk
        self.on_fix_change = on_fix_change      # 품질 변화 시 알림(옵션)
        self.read_timeout = read_timeout

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
    print("\n다음 단계: AutoSteerSystem.on_rtk 를 on_rtk 콜백으로 연결하면 끝.")
