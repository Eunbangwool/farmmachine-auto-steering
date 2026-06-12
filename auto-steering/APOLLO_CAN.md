# Apollo 10 Pro CAN 연결 (apollo_can.py) — 설계 + Kotlin 계약

Apollo 10 Pro = Shenzhen CPDEVICE 러기드 차량탑재 안드로이드(Android 9, IP65,
MIL-STD-810). I/O: **CAN(J1939/CANopen/ISO15765)**, RS-232/485, USB.

`auto-steering/src/apollo_can.py` 의 `ApolloCanBus` 가 `autosteer_core.CanInterface`
(start/send/recv/stop)를 구현한다. 조향(AutoSteerSystem)과 레벨러
(LaserConnectorOutput)가 **같은 버스 인스턴스를 공유**한다.

## 아키텍처 — Python 알고리즘 ↔ Android CAN

```
┌─────────────────────────── Apollo 10 Pro (Android) ───────────────────────────┐
│  [Python 자율조향/레벨러]                      [Kotlin 앱 (farm-work-manager)]   │
│   AutoSteerSystem ─┐                                                            │
│   LaserConnector ──┤→ ApolloCanBus(backend="bridge")                            │
│                    │        │  TCP 127.0.0.1:47100 (13B 레코드)                 │
│                    │        ▼                                                   │
│                    │   ApolloCanBridgeService (Kotlin) ── 벤더 CAN SDK ─┐       │
│                    │                                                    ▼       │
│                    │                                             내장 CAN 포트  │
└────────────────────┴──────────────────────────────────────────────────┼───────┘
                                                                          ▼
                                                  조향모터 / 앵글센서 / 레벨러밸브
```

**결정**: 주 경로 = **Kotlin 브릿지**. 벤더 CAN SDK가 SocketCAN이 아니어도 되고,
자율조향 알고리즘은 Python 그대로 유지. (대안 백엔드: `socketcan`, `slcan`, `mock`)

| backend | 용도 | 비고 |
|---|---|---|
| `bridge` ★ | Kotlin CAN 서비스에 TCP 접속 | 기본. 벤더 SDK 무관 |
| `socketcan` | Apollo 커널이 `can0` 노출 | root/드라이버 필요 |
| `slcan` | USB-CAN(LAWICEL) `/dev/ttyUSB0` | python-can 불필요, pyserial |
| `mock` | 모터 응답 시뮬(테스트) | 하드웨어 불필요 |

ApolloCanBus 공통 기능: **연결 감시 + 지수 백오프 자동 재연결**, 비차단 TX/RX 큐,
통계(tx/rx/reconnects/state), 상태 콜백, 미연결 시 `available=False`(예외 X).

## 브릿지 와이어 프로토콜

- 전송 계층: TCP, `127.0.0.1:47100` (기본). Kotlin = 서버, Python = 클라이언트.
- **레코드 = 13바이트 고정** (양방향 동일):

  | 오프셋 | 크기 | 필드 | 설명 |
  |---|---|---|---|
  | 0 | 4 | `can_id` | uint32 **big-endian**. 11/29bit 모두. |
  | 4 | 1 | `dlc` | 0..8 |
  | 5 | 8 | `data` | 페이로드(우측 0패딩) |

- `can_id == 0x7FFFFFFF` = **heartbeat**(keepalive). 실프레임 아님 → 수신측 무시,
  링크 신선도(rx_timeout)만 갱신. 서버가 ~1s 주기로 보내면 단선 감지 빨라짐.
- 바이트오더/레이아웃은 Python `struct ">IB8s"` 와 일치.
- 참조 구현: `apollo_can.run_loopback_bridge()` (TX를 RX로 에코) — 테스트/동작 예시.

## Kotlin 서비스 스켈레톤 (참조)

`com.sangwolnongsan.farmwork` 앱에 포그라운드 서비스로 추가. 벤더 CAN SDK
호출부만 채우면 된다. (아래는 골격 — gradle 미연동, 그대로 드롭인 전 검토)

```kotlin
package com.sangwolnongsan.farmwork.can

import java.io.DataInputStream
import java.net.ServerSocket
import java.net.Socket
import kotlin.concurrent.thread

/** Python ApolloCanBus(backend="bridge") 가 접속하는 13바이트 레코드 TCP 서버. */
class ApolloCanBridge(private val port: Int = 47100) {
    @Volatile private var running = false
    private val REC = 13
    private val HEARTBEAT = 0x7FFFFFFFL

    fun start() {
        running = true
        thread(name = "apollo-can-bridge") { serve() }
    }
    fun stop() { running = false }

    private fun serve() {
        val server = ServerSocket(port).apply { reuseAddress = true }
        while (running) {
            val sock = try { server.accept() } catch (e: Exception) { continue }
            handle(sock)
        }
        server.close()
    }

    private fun handle(sock: Socket) {
        // TODO(벤더 SDK): val can = CanManager.open(channel=0, bitrate=500_000)
        val inp = DataInputStream(sock.getInputStream())
        val out = sock.getOutputStream()

        // CAN → 클라이언트 (수신 프레임을 13B 레코드로)
        val rxThread = thread {
            while (running && !sock.isClosed) {
                // TODO: val f = can.receive(timeoutMs=10) ?: continue
                // out.write(encode(f.id, f.dlc, f.data)); out.flush()
            }
        }
        // 하트비트
        val hbThread = thread {
            while (running && !sock.isClosed) {
                out.write(encode(HEARTBEAT, 0, ByteArray(8))); out.flush()
                Thread.sleep(1000)
            }
        }
        // 클라이언트 → CAN (13B 레코드를 파싱해 송신)
        val rec = ByteArray(REC)
        try {
            while (running) {
                inp.readFully(rec)
                val id  = ((rec[0].toLong() and 0xFF) shl 24) or
                          ((rec[1].toLong() and 0xFF) shl 16) or
                          ((rec[2].toLong() and 0xFF) shl 8)  or
                           (rec[3].toLong() and 0xFF)
                val dlc = rec[4].toInt() and 0xFF
                val data = rec.copyOfRange(5, 5 + dlc.coerceIn(0, 8))
                if (id != HEARTBEAT) {
                    // TODO: can.send(id, data)
                }
            }
        } catch (_: Exception) {}
        rxThread.interrupt(); hbThread.interrupt(); sock.close()
    }

    private fun encode(id: Long, dlc: Int, data: ByteArray): ByteArray {
        val b = ByteArray(REC)
        b[0] = (id ushr 24).toByte(); b[1] = (id ushr 16).toByte()
        b[2] = (id ushr 8).toByte();  b[3] = id.toByte(); b[4] = dlc.toByte()
        System.arraycopy(data, 0, b, 5, minOf(data.size, 8))
        return b
    }
}
```

## 레벨러 연동 (LaserConnectorOutput) 계약

`leveler_core.py` 의 `LaserConnectorOutput` 은 `CanInterface` 호환 버스를 받아
밸브/커넥터 CAN 프레임을 송신하면 된다(조향과 동일 버스 공유 가능):

```python
class LaserConnectorOutput:
    def __init__(self, bus):           # bus: ApolloCanBus 또는 MockCanInterface
        self.bus = bus
    def set_valve(self, up: float):    # up: -1.0(하강)~+1.0(상승)
        raw = int(max(-1, min(1, up)) * 1000)
        data = bytes([0x01]) + int(raw & 0xFFFF).to_bytes(2, "big") + bytes(5)
        self.bus.send(LEVELER_VALVE_ID, data)   # ★ LEVELER_VALVE_ID = 모터문서값
```

같은 `ApolloCanBus` 인스턴스를 `AutoSteerSystem` 과 `LaserConnectorOutput` 에
함께 넘기면 단일 CAN 버스를 공유한다.

## 배포 절차

1. Kotlin 앱에 `ApolloCanBridge` 서비스 추가 → 벤더 CAN SDK로 open/send/receive 채움
2. 앱 시작 시 `ApolloCanBridge(47100).start()` (포그라운드 서비스)
3. Python:
   ```python
   from apollo_can import ApolloCanBus
   bus = ApolloCanBus(backend="bridge", host="127.0.0.1", port=47100,
                      rx_timeout=1.0, on_state=print)
   bus.start()
   sys = AutoSteerSystem(bus, ...)        # 조향
   ```
4. `python apollo_can.py` 로 브릿지/재연결/SLCAN self-test (하드웨어 불필요).
