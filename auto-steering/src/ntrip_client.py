"""
ntrip_client.py — NTRIP caster 클라이언트 (RTCM 보정신호 수신)

표준 라이브러리(socket/base64/threading)만 사용 — Chaquopy 추가 의존성 없음.
NTRIP v1/v2 모두 대응(ICY 200 OK / HTTP 200). VRS caster 용 주기적 GGA 전송 지원.

흐름:
    caster(host:port/mountpoint) ── RTCM3 ──▶ on_rtcm(bytes) ──▶ GNSS 수신기(F9P 시리얼)
    수신기 NMEA GGA(위치) ──▶ submit_gga() ──▶ caster (VRS 위치 보고)

사용:
    nc = NtripClient("rtk.example.com", 2101, "AUTO", "user", "pw", on_rtcm=cb)
    nc.start()
    nc.submit_gga("$GPGGA,...")   # VRS 면 주기 전송됨
    nc.status()                   # {connected, bytes, error, ...}
    nc.stop()
"""
from __future__ import annotations
import socket, base64, threading, time, logging
from typing import Callable, Optional

log = logging.getLogger("ntrip")


class NtripClient:
    def __init__(self, host: str, port: int, mountpoint: str,
                 user: str = "", password: str = "",
                 on_rtcm: Optional[Callable[[bytes], None]] = None):
        self.host = (host or "").strip()
        self.port = int(port)
        self.mountpoint = (mountpoint or "").strip().lstrip("/")
        self.user = user or ""
        self.password = password or ""
        self.on_rtcm = on_rtcm

        self._sock: Optional[socket.socket] = None
        self._thread: Optional[threading.Thread] = None
        self._run = False
        self._gga: Optional[bytes] = None

        # 상태
        self.connected = False
        self.bytes_rx = 0
        self.last_error = ""

    # ── 수명주기 ──────────────────────────────────────────
    def start(self):
        if self._run:
            return
        self._run = True
        self._thread = threading.Thread(target=self._loop, daemon=True, name="ntrip")
        self._thread.start()

    def stop(self):
        self._run = False
        try:
            if self._sock:
                self._sock.close()
        except Exception:
            pass
        self.connected = False

    def submit_gga(self, gga_line: str):
        """VRS caster 용 위치 보고(GGA). 수신기에서 받은 GGA 문장을 넣는다."""
        if not gga_line:
            return
        if not gga_line.endswith("\r\n"):
            gga_line = gga_line.strip() + "\r\n"
        self._gga = gga_line.encode("ascii", "replace")

    # ── 내부 ──────────────────────────────────────────────
    def _build_request(self) -> bytes:
        lines = [
            f"GET /{self.mountpoint} HTTP/1.1",
            f"Host: {self.host}:{self.port}",
            "Ntrip-Version: Ntrip/2.0",
            "User-Agent: NTRIP farmmachine/1.0",
        ]
        if self.user:
            token = base64.b64encode(f"{self.user}:{self.password}".encode()).decode()
            lines.append(f"Authorization: Basic {token}")
        lines += ["Accept: */*", "Connection: close", "", ""]
        return "\r\n".join(lines).encode()

    def _read_header(self) -> str:
        hdr = b""
        self._sock.settimeout(10)
        while b"\r\n\r\n" not in hdr and b"\n\n" not in hdr:
            c = self._sock.recv(1)
            if not c:
                raise IOError("헤더 수신 중 연결 끊김")
            hdr += c
            if len(hdr) > 2048:
                break
        return hdr.decode(errors="replace")

    def _loop(self):
        backoff = 1.0
        while self._run:
            try:
                self.last_error = ""
                self._sock = socket.create_connection((self.host, self.port), timeout=10)
                self._sock.sendall(self._build_request())
                head = self._read_header()
                first = head.splitlines()[0] if head.strip() else "(빈 응답)"
                if ("200" not in first) and ("ICY 200" not in head):
                    raise IOError(f"caster 응답: {first}")

                self.connected = True
                backoff = 1.0
                log.info(f"NTRIP 연결됨 {self.host}:{self.port}/{self.mountpoint}")

                last_gga = 0.0
                self._sock.settimeout(2)
                while self._run:
                    if self._gga and (time.time() - last_gga > 10):
                        try:
                            self._sock.sendall(self._gga)
                        except Exception:
                            pass
                        last_gga = time.time()
                    try:
                        data = self._sock.recv(4096)
                    except socket.timeout:
                        continue
                    if not data:
                        raise IOError("스트림 연결 끊김")
                    self.bytes_rx += len(data)
                    if self.on_rtcm:
                        try:
                            self.on_rtcm(data)
                        except Exception as e:
                            log.warning(f"on_rtcm 콜백 오류: {e}")
            except Exception as e:
                self.connected = False
                self.last_error = str(e)
                log.warning(f"NTRIP 오류: {e} (재연결 {backoff:.0f}s)")
                t = time.time()
                while self._run and (time.time() - t < backoff):
                    time.sleep(0.2)
                backoff = min(30.0, backoff * 2)
            finally:
                try:
                    if self._sock:
                        self._sock.close()
                except Exception:
                    pass
                self.connected = False

    def status(self) -> dict:
        return {
            "connected": self.connected,
            "bytes": self.bytes_rx,
            "error": self.last_error,
            "host": self.host,
            "port": self.port,
            "mount": self.mountpoint,
        }


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    # 네트워크 없이 검증: 요청 빌드 + 상태 구조
    nc = NtripClient("rtk.example.com", 2101, "/AUTO", "user0001", "pw")
    req = nc._build_request().decode()
    assert "GET /AUTO HTTP/1.1" in req
    assert "Authorization: Basic" in req
    assert nc.status()["connected"] is False
    print("요청 헤더:\n" + req)
    print("status:", nc.status())
    print("✓ ntrip_client 셀프테스트 통과")
