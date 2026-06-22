package com.farmmachine.autosteer.can

import android.os.Binder
import android.os.IBinder
import android.os.Parcel
import android.util.Log
import java.io.DataInputStream
import java.net.ServerSocket
import java.net.Socket
import java.util.concurrent.LinkedBlockingQueue
import java.util.concurrent.TimeUnit
import kotlin.concurrent.thread

/**
 * AGMO Ver2(agmo_single) CAN 출력 브리지 — BnMcuCanService binder 직접 호출.
 *
 * 배경(ADB 실측): Ver2(Apollo2_10)엔 SocketCAN 인터페이스 없음. CAN = /dev/spidev2.0 →
 *   cpdevice MCU(cpcomm_server) → CAN 모터. binder `com.cpdevice.BnMcuCanService`(service #34) 중계.
 *
 * 설계(C안): Python BridgeBackend(TCP 13바이트 레코드 계약) **불변**. 이 브리지가 ApolloCanBridge 와
 *   동일한 TCP 서버 계약을 구현하되, 내부 전송을 VanMcu 대신 BnMcuCanService binder 로 한다.
 *   모터 프레임 ID(0x06000001 cmd_speed / 0x07000001 HB)는 Python CanSpec 그대로(중복정의 없음).
 *
 * 프로토콜(VER2_BINDER_PROTOCOL.md, libcpcomm.so 역분석 확정):
 *   - 디스크립터 = "com.cpdevice.BnMcuCanService"
 *   - TX  code 7  sendCanFrame: writeInterfaceToken + writeInt(N) + writeByteArray(N×14B)
 *   - RX  code 1  registerCallback: writeInterfaceToken + writeStrongBinder(우리 콜백)
 *   - 14바이트 프레임: [0]channel [1..3]canId(**big-endian**) [4](canId&0xFF)|flags
 *                      [5]dlc [6..13]data
 *   binder 호출은 public API(Parcel/IBinder.transact). ServiceManager 만 hidden → reflection.
 *
 * ⚠ 미확정(실차/추가분석 — TODO): ① byte[4] ext 플래그 비트 위치 ② 모터 CAN 채널 번호
 *   ③ RX 콜백 IBinder transact code/디스크립터 ④ Android11 hidden API 접근 제한(reflection 폴백).
 *   구조는 확정 — 위 4개는 안전 기본값 + TODO 로 두고 실차에서 최종 확인.
 */
class CpdeviceCanBridge(private val port: Int = 47100) {

    @Volatile private var running = false
    private var serverThread: Thread? = null
    private var binder: IBinder? = null
    private val rxQueue = LinkedBlockingQueue<Pair<Int, ByteArray>>(512)

    // ★ TODO(HW): 모터가 붙은 CAN 채널 번호 — 실차 확인(0/1/2). 기본 0.
    private var channel: Int = 0

    private val REC = 13
    private val HEARTBEAT = 0x7FFFFFFFL
    private val TAG = "CpdeviceCanBridge"

    companion object {
        const val SERVICE_NAME = "com.cpdevice.BnMcuCanService"
        const val DESCRIPTOR = "com.cpdevice.BnMcuCanService"
        const val TX_SEND_CAN_FRAME = 7        // 확정(onTransact 점프테이블)
        const val TX_REGISTER_CALLBACK = 1     // 확정(registerCallback)
        // ★ TODO(HW): byte[4] 확장ID(ext) 플래그 비트 — 실차 ext 프레임 송신으로 최종확인. 잠정 0x02.
        const val EXT_FLAG = 0x02

        @Volatile var binderReady = false        // BnMcuCanService binder 획득
        @Volatile var clientConnected = false    // Python BridgeBackend 접속
        @Volatile var txCount = 0                // sendCanFrame transact 호출 수
        @Volatile var lastTxOk = false
        @Volatile var rxCount = 0                // 콜백 수신 프레임 수
        @Volatile var lastError = "init"
        @Volatile var instance: CpdeviceCanBridge? = null
    }

    fun start() {
        if (running) return
        running = true
        instance = this
        connectBinder()
        registerRxCallback()
        serverThread = thread(name = "cpdevice-can-bridge") { serve() }
        Log.i(TAG, "CpdeviceCanBridge 시작 :$port (binderReady=$binderReady)")
    }

    fun stop() {
        running = false
        clientConnected = false
        binder = null
        serverThread = null
    }

    fun setChannel(ch: Int) { channel = ch }   // 현장 진단용(채널 스윕)

    /** BnMcuCanService binder 획득 — ServiceManager 는 hidden API → reflection. 실패해도 크래시 금지. */
    private fun connectBinder() {
        try {
            val sm = Class.forName("android.os.ServiceManager")
            val getService = sm.getMethod("getService", String::class.java)
            binder = getService.invoke(null, SERVICE_NAME) as? IBinder
            binderReady = (binder != null)
            lastError = if (binderReady) "ok" else "service 없음($SERVICE_NAME)"
            Log.i(TAG, "binder 연결: ${if (binderReady) "OK" else lastError}")
        } catch (e: Throwable) {
            // ★ TODO: Android 11 hidden API 제한 시 다른 reflection 경로 필요할 수 있음.
            binder = null; binderReady = false
            lastError = "binder 예외: ${e.message}"
            Log.w(TAG, lastError)
        }
    }

    fun status(): String =
        """{"bridge":"cpdevice","binderReady":$binderReady,"connected":$clientConnected,""" +
        """"txCount":$txCount,"lastTxOk":$lastTxOk,"rxCount":$rxCount,"channel":$channel,""" +
        """"lastError":"${lastError.replace("\"","'")}"}"""

    // ── TX: 13바이트 TCP 레코드 → 14바이트 CAN 프레임 → sendCanFrame(code 7) ──────────
    private fun makeFrame14(canId: Int, dlc: Int, data: ByteArray): ByteArray {
        val ext = canId > 0x7FF                       // 29비트 확장 ID (Keya 0x06000001 등)
        val f = ByteArray(14)
        f[0] = channel.toByte()                       // ★ TODO(HW): 채널 실차확인
        f[1] = (canId ushr 24).toByte()               // canId big-endian (MSB first)
        f[2] = (canId ushr 16).toByte()
        f[3] = (canId ushr 8).toByte()
        f[4] = ((canId and 0xFF) or (if (ext) EXT_FLAG else 0)).toByte()  // ★ TODO: ext 비트 실차검증
        f[5] = dlc.coerceIn(0, 8).toByte()
        System.arraycopy(data, 0, f, 6, minOf(data.size, 8))
        return f
    }

    private fun sendCanFrame(canId: Int, dlc: Int, data: ByteArray) {
        val b = binder
        if (b == null) { lastTxOk = false; return }   // binder 없음 → 조용히 무시(추측 송신 안 함)
        val p = Parcel.obtain(); val r = Parcel.obtain()
        try {
            p.writeInterfaceToken(DESCRIPTOR)
            p.writeInt(1)                              // 프레임 개수 N=1
            p.writeByteArray(makeFrame14(canId, dlc, data))
            b.transact(TX_SEND_CAN_FRAME, p, r, 0)
            r.readException()
            lastTxOk = true; txCount++
            if (txCount % 50 == 1)
                Log.i(TAG, "TX sendCanFrame id=0x%08X dlc=%d ch=%d (#%d)".format(canId, dlc, channel, txCount))
        } catch (e: Throwable) {
            lastTxOk = false
            lastError = "transact 실패: ${e.message}"
            Log.w(TAG, lastError)
        } finally {
            p.recycle(); r.recycle()
        }
    }

    // ── RX: registerCallback(code 1) — 우리 Binder 등록, 수신 14B 프레임을 rxQueue 로 ──────
    private val rxCallback = object : Binder() {
        override fun onTransact(code: Int, data: Parcel, reply: Parcel?, flags: Int): Boolean {
            // ★ TODO: 콜백 transact code/디스크립터 미확정 → code 무시하고 14B 만 읽음(실차검증).
            try {
                val frame = data.createByteArray()
                if (frame != null && frame.size >= 6) {
                    val dlc = (frame[5].toInt() and 0xFF).coerceIn(0, 8)
                    // canId big-endian 복원. low 바이트의 ext 플래그(잠정 EXT_FLAG)는 제거(★ TODO 실차).
                    val low = (frame[4].toInt() and 0xFF) and EXT_FLAG.inv()
                    val canId = ((frame[1].toInt() and 0xFF) shl 24) or
                                ((frame[2].toInt() and 0xFF) shl 16) or
                                ((frame[3].toInt() and 0xFF) shl 8) or low
                    val end = minOf(6 + dlc, frame.size)
                    val payload = if (end > 6) frame.copyOfRange(6, end) else ByteArray(0)
                    rxQueue.offer(canId to payload); rxCount++
                }
            } catch (e: Throwable) { Log.w(TAG, "RX onTransact 파싱 실패: ${e.message}") }
            return true
        }
    }

    private fun registerRxCallback() {
        val b = binder ?: return
        val p = Parcel.obtain(); val r = Parcel.obtain()
        try {
            p.writeInterfaceToken(DESCRIPTOR)
            p.writeStrongBinder(rxCallback)
            b.transact(TX_REGISTER_CALLBACK, p, r, 0)
            r.readException()
            Log.i(TAG, "registerCallback(code 1) OK")
        } catch (e: Throwable) {
            lastError = "registerCallback 실패: ${e.message}"
            Log.w(TAG, lastError)   // RX 실패해도 TX(모터 구동)는 가능 — graceful
        } finally {
            p.recycle(); r.recycle()
        }
    }

    // ── TCP 서버 (ApolloCanBridge 와 동일 13바이트 레코드 계약) ──────────
    private fun serve() {
        val server = ServerSocket(port).apply { reuseAddress = true; soTimeout = 200 }
        while (running) {
            val sock = try { server.accept() } catch (e: Exception) { continue }
            try { handle(sock) } catch (e: Exception) { Log.w(TAG, "client 종료: ${e.message}") }
            try { sock.close() } catch (_: Exception) {}
        }
        try { server.close() } catch (_: Exception) {}
    }

    private fun handle(sock: Socket) {
        sock.tcpNoDelay = true
        clientConnected = true
        val inp = DataInputStream(sock.getInputStream())
        val out = sock.getOutputStream()

        // CAN → Python: 콜백 rxQueue 를 13B 레코드로 relay
        val rxThread = thread(name = "cpdevice-can-rx") {
            while (running && !sock.isClosed) {
                val f = try { rxQueue.poll(50, TimeUnit.MILLISECONDS) } catch (e: InterruptedException) { break } ?: continue
                try { synchronized(out) { out.write(encode(f.first.toLong() and 0xFFFFFFFFL, f.second.size, f.second)); out.flush() } }
                catch (e: Exception) { break }
            }
        }
        // keepalive (~1s) — Python rx_timeout 단선 감지용
        val hbThread = thread(name = "cpdevice-can-hb") {
            while (running && !sock.isClosed) {
                try { synchronized(out) { out.write(encode(HEARTBEAT, 0, ByteArray(8))); out.flush() }; Thread.sleep(1000) }
                catch (e: Exception) { break }
            }
        }
        // Python → CAN: 13B 레코드 파싱 → sendCanFrame
        val rec = ByteArray(REC)
        try {
            while (running) {
                inp.readFully(rec)
                val id = (((rec[0].toInt() and 0xFF) shl 24) or
                          ((rec[1].toInt() and 0xFF) shl 16) or
                          ((rec[2].toInt() and 0xFF) shl 8) or
                           (rec[3].toInt() and 0xFF))
                val dlc = (rec[4].toInt() and 0xFF).coerceIn(0, 8)
                val data = rec.copyOfRange(5, 5 + dlc)
                if ((id.toLong() and 0xFFFFFFFFL) != HEARTBEAT) sendCanFrame(id, dlc, data)
            }
        } finally {
            clientConnected = false
            rxThread.interrupt(); hbThread.interrupt()
        }
    }

    private fun encode(id: Long, dlc: Int, data: ByteArray): ByteArray {
        val b = ByteArray(REC)
        b[0] = (id ushr 24).toByte(); b[1] = (id ushr 16).toByte()
        b[2] = (id ushr 8).toByte(); b[3] = id.toByte()
        b[4] = dlc.toByte()
        System.arraycopy(data, 0, b, 5, minOf(data.size, 8))
        return b
    }
}
