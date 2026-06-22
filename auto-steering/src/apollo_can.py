"""
apollo_can.py
=============
Apollo 10 Pro 내장 CAN 버스 접근 — 백엔드 교체형 + 자동 재연결.

장비: Apollo 10 Pro (Shenzhen CPDEVICE, Android 9, IP65, MIL-STD-810).
      I/O: CAN(J1939/CANopen/ISO15765), RS-232/485, USB. 중국 농기계 자율조향 표준 단말.

이 모듈은 autosteer_core.CanInterface 와 동일한 인터페이스(start/send/recv/stop)를
구현하는 ApolloCanBus 를 제공한다. 따라서:
  - autosteer_core.AutoSteerSystem(can=ApolloCanBus(...))  → 조향 모터/앵글센서
  - leveler_core.LaserConnectorOutput(bus=ApolloCanBus(...)) → 레벨러 밸브 CAN 출력
  둘 다 같은 버스 1개를 공유할 수 있다(같은 물리 CAN 버스).

──────────────────────────────────────────────────────────────────────
아키텍처: Python 알고리즘 ↔ Android CAN
──────────────────────────────────────────────────────────────────────
결정(오너): **Kotlin 브릿지**가 주 경로.
  Apollo의 CAN은 벤더 SDK(Kotlin/JNI)로만 열리는 경우가 많아, Kotlin 포그라운드
  서비스가 CAN을 열고 localhost TCP 소켓으로 노출 → Python이 클라이언트로 접속.
  (벤더 SDK가 SocketCAN이 아니어도 동작. autosteer 알고리즘은 그대로 Python 유지)

백엔드(교체 가능):
  bridge   ★기본 — Kotlin CAN 서비스에 TCP 접속 (BridgeBackend)
  socketcan      — Apollo 커널이 can0 노출 시 PF_CAN 직접 (SocketCanBackend)
  slcan          — USB-CAN(LAWICEL/SLCAN) /dev/ttyUSB* (SlcanSerialBackend)
  mock           — 모터 응답 시뮬(테스트) (MockBackend)

ApolloCanBus 가 백엔드 위에 공통 기능을 얹는다:
  - 연결 상태 감시 + 지수 백오프 자동 재연결 (백그라운드 IO 스레드)
  - 비차단 send/recv (TX/RX 큐), 통계(tx/rx/reconnects), 상태 콜백
  - 하드웨어/브릿지 없으면 available=False 로 두고 예외 없이 재시도

브릿지 와이어 프로토콜 / Kotlin 서비스 계약은 auto-steering/APOLLO_CAN.md 참고.
"""

from __future__ import annotations
import socket
import struct
import time
import threading
import logging
from abc import ABC, abstractmethod
from collections import deque
from dataclasses import dataclass, field
from typing import Optional, Tuple, List, Callable

from autosteer_core import CanInterface, CanSpec, MockCanInterface

log = logging.getLogger("apollo_can")

CanFrameT = Tuple[int, bytes]

BRIDGE_DEFAULT_PORT = 47100          # Kotlin CAN 서비스 localhost 포트
BRIDGE_HEARTBEAT_ID = 0x7FFFFFFF     # 브릿지 keepalive 마커(실프레임 아님)
_RECORD = struct.Struct(">IB8s")     # 와이어 레코드: id(u32 BE) dlc(u8) data(8B) = 13B

# SLCAN(LAWICEL) 비트레이트 코드
_SLCAN_BITRATE = {10000: "S0", 20000: "S1", 50000: "S2", 100000: "S3",
                  125000: "S4", 250000: "S5", 500000: "S6", 800000: "S7",
                  1_000_000: "S8"}


# ═══════════════════════════════════════════════════════════════
#  백엔드 추상화
# ═══════════════════════════════════════════════════════════════

class CanBackend(ABC):
    """단일 CAN 링크. ApolloCanBus 가 이 위에 재연결/큐를 얹는다."""
    name = "base"

    @abstractmethod
    def open(self) -> bool: ...
    @abstractmethod
    def close(self): ...
    @abstractmethod
    def send(self, can_id: int, data: bytes): ...      # 실패 시 예외 → 재연결
    @abstractmethod
    def poll(self) -> List[CanFrameT]: ...             # 수신 프레임 비우기(비차단)
    def healthy(self) -> bool:                          # 링크 정상?
        return True


# ── SocketCAN (can0) ──────────────────────────────────────────────
class SocketCanBackend(CanBackend):
    name = "socketcan"
    _FMT = struct.Struct("=IB3x8s")     # 리눅스 can_frame 16B

    def __init__(self, channel: str = "can0",
                 bitrate: int = CanSpec.CAN_BITRATE, use_python_can: bool = True,
                 listen_only: bool = False):
        self.channel = channel
        self.bitrate = bitrate
        self.use_python_can = use_python_can
        # ★ Listen-Only: 송신 금지(스니핑 전용). AGMO Ver2/CHCNAV 모터 CAN ID 미확정 →
        #   확정 전 추측 프레임 송신 방지. send() 가 무시한다(버스 오염 방지).
        self.listen_only = listen_only
        self._sock = None
        self._bus = None

    def open(self) -> bool:
        if self.use_python_can:
            try:
                import can as pycan
                self._bus = pycan.interface.Bus(channel=self.channel, bustype="socketcan")
                return True
            except Exception as e:
                log.debug(f"python-can 불가({e}) → raw socket")
        try:
            s = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
            s.bind((self.channel,))
            s.setblocking(False)
            self._sock = s
            return True
        except Exception as e:
            log.debug(f"SocketCAN open 실패: {e}")
            return False

    def close(self):
        for obj, m in ((self._bus, "shutdown"), (self._sock, "close")):
            if obj is not None:
                try: getattr(obj, m)()
                except Exception: pass
        self._bus = self._sock = None

    def send(self, can_id: int, data: bytes):
        if self.listen_only:
            return     # 스니핑 전용 — 송신 금지(미확정 CAN ID 추측 송신 방지)
        data = bytes(data)[:8]
        if self._bus is not None:
            import can as pycan
            self._bus.send(pycan.Message(arbitration_id=can_id, data=data,
                                         is_extended_id=can_id > 0x7FF))
        elif self._sock is not None:
            self._sock.send(self._FMT.pack(can_id, len(data), data.ljust(8, b"\x00")))

    def poll(self) -> List[CanFrameT]:
        out = []
        if self._bus is not None:
            while True:
                m = self._bus.recv(timeout=0.0)
                if not m:
                    break
                out.append((m.arbitration_id, bytes(m.data)))
        elif self._sock is not None:
            while True:
                try:
                    frame = self._sock.recv(self._FMT.size)
                except (BlockingIOError, OSError):
                    break
                if len(frame) < self._FMT.size:
                    break
                cid, dlc, payload = self._FMT.unpack(frame)
                out.append((cid & 0x1FFFFFFF, payload[:dlc]))
        return out


# ── SLCAN (USB-CAN, LAWICEL ASCII) ───────────────────────────────
def slcan_encode(can_id: int, data: bytes) -> bytes:
    data = bytes(data)[:8]
    body = data.hex().upper()
    if can_id > 0x7FF:
        return f"T{can_id:08X}{len(data)}{body}\r".encode()
    return f"t{can_id:03X}{len(data)}{body}\r".encode()


def slcan_decode(line: str) -> Optional[CanFrameT]:
    if not line:
        return None
    k = line[0]
    try:
        if k == "t":
            cid = int(line[1:4], 16); dlc = int(line[4]); off = 5
        elif k == "T":
            cid = int(line[1:9], 16); dlc = int(line[9]); off = 10
        else:
            return None
        data = bytes.fromhex(line[off:off + 2 * dlc])
        return (cid, data)
    except (ValueError, IndexError):
        return None


class SlcanSerialBackend(CanBackend):
    name = "slcan"

    def __init__(self, port: str = "/dev/ttyUSB0",
                 bitrate: int = CanSpec.CAN_BITRATE, serial_baud: int = 115200):
        self.port = port
        self.bitrate = bitrate
        self.serial_baud = serial_baud
        self._ser = None
        self._buf = ""

    def open(self) -> bool:
        try:
            import serial
        except ImportError:
            log.error("pyserial 미설치 — SLCAN 사용 불가")
            return False
        try:
            self._ser = serial.Serial(self.port, self.serial_baud, timeout=0)
            code = _SLCAN_BITRATE.get(self.bitrate, "S6")
            self._ser.write(b"C\r")               # 닫기(혹시 열려있으면)
            self._ser.write(f"{code}\r".encode())  # 비트레이트
            self._ser.write(b"O\r")               # 열기
            self._buf = ""
            return True
        except Exception as e:
            log.debug(f"SLCAN open 실패: {e}")
            return False

    def close(self):
        if self._ser is not None:
            try:
                self._ser.write(b"C\r"); self._ser.close()
            except Exception: pass
            self._ser = None

    def send(self, can_id: int, data: bytes):
        self._ser.write(slcan_encode(can_id, data))

    def poll(self) -> List[CanFrameT]:
        out = []
        try:
            chunk = self._ser.read(4096)
        except Exception as e:
            raise ConnectionError(f"SLCAN read 오류: {e}")
        if chunk:
            self._buf += chunk.decode("ascii", "ignore")
            while "\r" in self._buf:
                line, self._buf = self._buf.split("\r", 1)
                fr = slcan_decode(line)
                if fr:
                    out.append(fr)
        return out


# ── Kotlin 브릿지 (TCP, 13바이트 레코드) ★기본 ──────────────────────
class BridgeBackend(CanBackend):
    """
    Kotlin CAN 서비스(localhost)에 TCP 접속.
    와이어: 양방향 13바이트 레코드 = id(u32 BE) | dlc(u8) | data(8B).
            id==BRIDGE_HEARTBEAT_ID 는 keepalive(수신 무시, 링크 신선도만 갱신).
    """
    name = "bridge"

    def __init__(self, host: str = "127.0.0.1", port: int = BRIDGE_DEFAULT_PORT,
                 connect_timeout: float = 1.0):
        self.host = host
        self.port = port
        self.connect_timeout = connect_timeout
        self._sock = None
        self._buf = b""

    def open(self) -> bool:
        try:
            s = socket.create_connection((self.host, self.port), self.connect_timeout)
            s.setblocking(False)
            self._sock = s
            self._buf = b""
            return True
        except OSError as e:
            log.debug(f"브릿지 접속 실패 {self.host}:{self.port} ({e})")
            return False

    def close(self):
        if self._sock is not None:
            try: self._sock.close()
            except Exception: pass
            self._sock = None

    def send(self, can_id: int, data: bytes):
        data = bytes(data)[:8]
        self._sock.sendall(_RECORD.pack(can_id, len(data), data.ljust(8, b"\x00")))

    def poll(self) -> List[CanFrameT]:
        out = []
        try:
            chunk = self._sock.recv(8192)
        except BlockingIOError:
            return out
        except OSError as e:
            raise ConnectionError(f"브릿지 recv 오류: {e}")
        if chunk == b"":
            raise ConnectionError("브릿지 연결 종료(peer closed)")
        self._buf += chunk
        while len(self._buf) >= _RECORD.size:
            rec, self._buf = self._buf[:_RECORD.size], self._buf[_RECORD.size:]
            cid, dlc, payload = _RECORD.unpack(rec)
            if cid == BRIDGE_HEARTBEAT_ID:
                continue                          # keepalive
            out.append((cid & 0x1FFFFFFF, payload[:min(dlc, 8)]))
        return out


# ── Mock (테스트: 모터 응답 시뮬 + 결함 주입) ───────────────────────
class MockBackend(CanBackend):
    """MockCanInterface(모터→앵글 피드백) 래핑 + link_down 으로 단선 시뮬."""
    name = "mock"

    def __init__(self):
        self._mock = MockCanInterface()
        self.link_down = False        # 테스트에서 토글 → 자동재연결 검증

    def open(self) -> bool:
        if self.link_down:
            return False
        self._mock.start()
        return True

    def close(self):
        try: self._mock.stop()
        except Exception: pass

    def send(self, can_id: int, data: bytes):
        if self.link_down:
            raise ConnectionError("mock link down")
        self._mock.send(can_id, data)

    def poll(self) -> List[CanFrameT]:
        if self.link_down:
            raise ConnectionError("mock link down")
        out = []
        m = self._mock.recv()
        while m:
            out.append(m); m = self._mock.recv()
        return out

    def healthy(self) -> bool:
        return not self.link_down


def make_backend(kind: str, **kw) -> CanBackend:
    kind = kind.lower()
    if kind == "bridge":    return BridgeBackend(**kw)
    if kind == "socketcan": return SocketCanBackend(**kw)
    if kind == "slcan":     return SlcanSerialBackend(**kw)
    if kind == "mock":      return MockBackend(**kw)
    raise ValueError(f"알 수 없는 backend: {kind}")


# ═══════════════════════════════════════════════════════════════
#  ApolloCanBus — 자동 재연결 슈퍼바이저 (CanInterface 호환)
# ═══════════════════════════════════════════════════════════════

@dataclass
class CanStats:
    tx: int = 0
    rx: int = 0
    reconnects: int = 0
    state: str = "init"          # init/connecting/connected/reconnecting/stopped
    last_rx_t: float = 0.0
    tx_dropped: int = 0


class ApolloCanBus(CanInterface):
    """
    백엔드 위 자동재연결 CAN 버스. MockCanInterface 와 동일 인터페이스.

    예)
        bus = ApolloCanBus(backend="bridge", host="127.0.0.1")
        bus.start()
        sys = AutoSteerSystem(bus, ...)        # 조향
        leveler_out = LaserConnectorOutput(bus) # 레벨러(같은 버스 공유)
        ...
        bus.stop()

    하드웨어/브릿지 없으면 available=False 로 두고 백그라운드에서 재시도(예외 X).
    """
    def __init__(self, backend: str = "bridge",
                 rx_timeout: Optional[float] = None,
                 reconnect_min: float = 0.5, reconnect_max: float = 5.0,
                 on_state: Optional[Callable[[str], None]] = None,
                 queue_size: int = 4000, poll_dt: float = 0.001,
                 **backend_kw):
        self._backend = make_backend(backend, **backend_kw)
        self.rx_timeout = rx_timeout          # 연결됐는데 이 시간 무수신 → 재연결(None=비활성)
        self.reconnect_min = reconnect_min
        self.reconnect_max = reconnect_max
        self.on_state = on_state
        self.poll_dt = poll_dt
        self._tx: deque = deque(maxlen=queue_size)
        self._rx: deque = deque(maxlen=queue_size)
        self._stats = CanStats()
        self._connected = False
        self._first_open = True
        self._running = False
        self._thread: Optional[threading.Thread] = None

    # ── CanInterface ──────────────────────────────────────────
    def start(self):
        if self._running:
            return
        self._running = True
        self._set_state("connecting")
        self._thread = threading.Thread(target=self._io_loop, daemon=True,
                                        name="apollo-can-io")
        self._thread.start()
        log.info(f"ApolloCanBus 시작 (backend={self._backend.name})")

    def stop(self):
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        self._backend.close()
        self._connected = False
        self._set_state("stopped")

    def switch_backend(self, backend: str, **kw):
        """런타임 백엔드 교체(예: 벤더 전환 bridge↔socketcan). 동일 ApolloCanBus 객체 유지
        → AutoSteerSystem/액추에이터가 들고 있는 참조 그대로. 실패해도 예외 없이 재연결 루프가 처리."""
        was = self._running
        try:
            if was:
                self.stop()
        except Exception:
            pass
        try:
            self._backend.close()
        except Exception:
            pass
        self._backend = make_backend(backend, **kw)
        self._connected = False
        self._first_open = True
        if was:
            self.start()
        log.info(f"CAN 백엔드 교체 → {self._backend.name}")
        return self._backend.name

    def send(self, can_id: int, data: bytes):
        if len(self._tx) == self._tx.maxlen:
            self._stats.tx_dropped += 1        # 가득 차면 가장 오래된 것 밀려남
        self._tx.append((can_id, bytes(data)))

    def recv(self) -> Optional[CanFrameT]:
        return self._rx.popleft() if self._rx else None

    # ── 상태 조회 ─────────────────────────────────────────────
    @property
    def available(self) -> bool:
        return self._connected

    @property
    def stats(self) -> CanStats:
        return self._stats

    def wait_connected(self, timeout: float = 5.0) -> bool:
        t0 = time.time()
        while time.time() - t0 < timeout:
            if self._connected:
                return True
            time.sleep(0.01)
        return False

    # ── 내부 ──────────────────────────────────────────────────
    def _set_state(self, s: str):
        if self._stats.state != s:
            self._stats.state = s
            log.info(f"ApolloCanBus 상태: {s}")
            if self.on_state:
                try: self.on_state(s)
                except Exception: pass

    def _fail(self, reason):
        log.warning(f"ApolloCanBus 링크 실패: {reason} → 재연결")
        try: self._backend.close()
        except Exception: pass
        self._connected = False
        self._set_state("reconnecting")

    def _io_loop(self):
        backoff = self.reconnect_min
        while self._running:
            # 1) 연결 보장
            if not self._connected:
                ok = False
                try:
                    ok = self._backend.open()
                except Exception as e:
                    log.debug(f"open 예외: {e}")
                if ok:
                    self._connected = True
                    if not self._first_open:
                        self._stats.reconnects += 1
                    self._first_open = False
                    self._stats.last_rx_t = time.time()
                    backoff = self.reconnect_min
                    self._set_state("connected")
                else:
                    self._set_state("reconnecting")
                    time.sleep(backoff)
                    backoff = min(self.reconnect_max, backoff * 2)
                    continue

            # 2) TX
            try:
                while self._tx:
                    cid, data = self._tx.popleft()
                    self._backend.send(cid, data)
                    self._stats.tx += 1
            except Exception as e:
                self._fail(e); continue

            # 3) RX
            try:
                frames = self._backend.poll()
            except Exception as e:
                self._fail(e); continue
            if frames:
                self._stats.last_rx_t = time.time()
                for fr in frames:
                    self._rx.append(fr); self._stats.rx += 1

            # 4) 헬스/수신 타임아웃
            if not self._backend.healthy():
                self._fail("unhealthy"); continue
            if (self.rx_timeout and
                    time.time() - self._stats.last_rx_t > self.rx_timeout):
                self._fail("rx timeout"); continue

            time.sleep(self.poll_dt)


# ═══════════════════════════════════════════════════════════════
#  브릿지 참조 서버(테스트/Kotlin 동작 참조) — TX 를 RX 로 에코
# ═══════════════════════════════════════════════════════════════

def run_loopback_bridge(port: int, stop_event: threading.Event,
                        echo: bool = True):
    """13바이트 레코드를 받아 그대로 되돌려보내는 참조 서버(Kotlin 측 동작 예시)."""
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("127.0.0.1", port)); srv.listen(1); srv.settimeout(0.2)
    while not stop_event.is_set():
        try:
            conn, _ = srv.accept()
        except socket.timeout:
            continue
        except OSError:
            break
        conn.settimeout(0.2)
        buf = b""
        while not stop_event.is_set():
            try:
                data = conn.recv(8192)
            except socket.timeout:
                continue
            except OSError:
                break
            if data == b"":
                break
            buf += data
            while len(buf) >= _RECORD.size:
                rec, buf = buf[:_RECORD.size], buf[_RECORD.size:]
                if echo:
                    try: conn.sendall(rec)
                    except OSError: break
        conn.close()
    srv.close()


# ── 자체 테스트 (하드웨어 없이) ─────────────────────────────────────
if __name__ == "__main__":
    logging.basicConfig(level=logging.WARNING,
                        format="%(levelname)s %(name)s: %(message)s")
    import struct as _s

    def wait_for(cond, timeout=3.0, dt=0.01):
        t0 = time.time()
        while time.time() - t0 < timeout:
            if cond():
                return True
            time.sleep(dt)
        return False

    print("=" * 74)
    print("apollo_can — 백엔드 + 자동재연결 테스트 (하드웨어 불필요)")
    print("=" * 74)

    # ── 1. SLCAN 인코드/디코드 라운드트립 ──────────────────────
    print("\n▶ 1. SLCAN(LAWICEL) 인코드/디코드")
    for cid, data in [(0x201, bytes([1, 0x64, 0, 0])), (0x18FF51E5, bytes([0, 0, 0x12, 0x34]))]:
        line = slcan_encode(cid, data).decode().strip()
        dec = slcan_decode(line)
        print(f"  0x{cid:X} {data.hex()} → '{line}' → {dec[0]:#x} {dec[1].hex()}")
        assert dec[0] == cid and dec[1] == data
    print("  ✓ 표준/확장 프레임 라운드트립")

    # ── 2. Mock 백엔드: 모터 명령 → 앵글 피드백 (CanInterface 호환) ──
    print("\n▶ 2. Mock 백엔드 폐루프 (send 모터 → recv 앵글)")
    bus = ApolloCanBus(backend="mock")
    bus.start()
    assert bus.wait_connected(2.0), "mock 연결 실패"
    # 모터 명령 프레임(목표각 10°)
    d = bytearray(8); d[0] = CanSpec.MOTOR_MODE_ANGLE
    _s.pack_into(">h", d, CanSpec.MOTOR_BYTE_CMD_HI, int(10.0 * CanSpec.MOTOR_ANGLE_SCALE))
    bus.send(CanSpec.MOTOR_CMD_ID, bytes(d))
    got = wait_for(lambda: bus.recv() is not None or bus.stats.rx > 0)
    assert bus.stats.tx >= 1 and bus.stats.rx >= 1, bus.stats
    print(f"  ✓ tx={bus.stats.tx} rx={bus.stats.rx} state={bus.stats.state}")

    # ── 3. 자동 재연결: 링크 끊김 → 복구 ──────────────────────
    print("\n▶ 3. 자동 재연결 (링크 단선 주입 → 복구)")
    states = []
    bus.on_state = states.append
    bus._backend.link_down = True
    assert wait_for(lambda: bus.stats.state == "reconnecting"), "단선 감지 실패"
    print(f"  단선 감지 → state={bus.stats.state}")
    bus._backend.link_down = False
    assert wait_for(lambda: bus.available and bus.stats.reconnects >= 1), "재연결 실패"
    print(f"  ✓ 복구 → state={bus.stats.state}, reconnects={bus.stats.reconnects}")
    bus.stop()
    assert bus.stats.state == "stopped"

    # ── 4. 브릿지 백엔드: 참조 서버와 TCP 송수신 ───────────────
    print("\n▶ 4. Kotlin 브릿지 경로 (참조 TCP 서버 에코)")
    port = 47199
    stop = threading.Event()
    srv_t = threading.Thread(target=run_loopback_bridge, args=(port, stop), daemon=True)
    srv_t.start()
    time.sleep(0.2)
    bridge = ApolloCanBus(backend="bridge", host="127.0.0.1", port=port)
    bridge.start()
    assert bridge.wait_connected(3.0), "브릿지 접속 실패"
    bridge.send(0x201, bytes([0xAB, 0xCD, 0x01, 0x02]))
    assert wait_for(lambda: bridge.stats.rx >= 1, 3.0), "브릿지 에코 수신 실패"
    fr = None
    for _ in range(50):
        fr = bridge.recv()
        if fr:
            break
        time.sleep(0.01)
    print(f"  ✓ 브릿지 송수신: tx={bridge.stats.tx} rx={bridge.stats.rx} echo={fr[0]:#x} {fr[1].hex()}")
    assert fr and fr[0] == 0x201
    bridge.stop()
    stop.set(); srv_t.join(timeout=2.0)

    print("\n" + "=" * 74)
    print("모든 백엔드/재연결 self-test 통과.")
    print("  실배포: backend='bridge' + Kotlin CAN 서비스(APOLLO_CAN.md 계약) 기동")
    print("  대안: backend='socketcan'(can0) / 'slcan'(/dev/ttyUSB0)")
