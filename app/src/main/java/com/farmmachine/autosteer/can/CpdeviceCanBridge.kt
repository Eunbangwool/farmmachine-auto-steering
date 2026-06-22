package com.farmmachine.autosteer.can

import android.os.IBinder
import android.util.Log
import java.io.DataInputStream
import java.net.ServerSocket
import java.net.Socket
import java.util.concurrent.TimeUnit
import kotlin.concurrent.thread

/**
 * AGMO Ver2(agmo_single) CAN 출력 브리지 — 골격.
 *
 * 배경(ADB 실측): Ver2 태블릿(Apollo2_10)엔 SocketCAN 인터페이스가 없다(can0~2 부재).
 *   CAN 경로 = /dev/spidev2.0 → cpdevice MCU(cpcomm_server) → CAN 모터. libcpcomm.so 가
 *   binder `com.cpdevice.BnMcuCanService` 로 중계. 표준 AF_CAN 소켓 접근 불가.
 *
 * 설계(C안): Python BridgeBackend(TCP 13바이트 레코드 계약)는 **변경 없음**.
 *   이 브리지가 ApolloCanBridge 와 **동일한 localhost TCP 서버 계약**을 구현하되, 내부 전송은
 *   VanMcu(libsysmcu) 대신 BnMcuCanService binder 로 한다. vendor=agmo_single 일 때만 기동.
 *
 * ★ 모터 프레임 ID/바이트(0x06000001 cmd_speed / 0x07000001 HB)는 agmo_dual 과 동일 →
 *   Python 프레임 생성부(CanSpec)에서 그대로 옴. 여기선 바이트를 그대로 전달만 한다(중복 정의 금지).
 *
 * ⚠ 이번 단계 = **골격까지만**. BnMcuCanService 의 onTransact 트랜잭션 코드·Parcel 마샬링 포맷이
 *   **미확정**이므로 실제 transact 는 TODO 로 막는다(추측 트랜잭션/프레임 송신 금지 — 모터 안전).
 *   확정 경로: (A) Ghidra 로 libcpcomm.so BnMcuCanService onTransact 분석 → 코드/Parcel 포맷 확정,
 *             (B) libcpcomm.so + libcpbase.so 번들 + JNI 래퍼(BpMcuCanService) — AGMO 독점 .so 라
 *                 라이선스/사용 허가 확인 필요.
 */
class CpdeviceCanBridge(private val port: Int = 47100) {

    @Volatile private var running = false
    private var serverThread: Thread? = null
    private var binder: IBinder? = null

    private val REC = 13
    private val HEARTBEAT = 0x7FFFFFFFL
    private val TAG = "CpdeviceCanBridge"

    companion object {
        const val SERVICE_NAME = "com.cpdevice.BnMcuCanService"  // service list #34 (실측)
        // ★ TODO(HW): onTransact 트랜잭션 코드 — Ghidra 분석으로 확정 전까지 미사용(추측 금지).
        //   const val TX_SEND_CAN_FRAME = ?    // sendCanFrame(canId, dlc, data)
        //   const val TX_SET_BAUDRATE  = ?    // setCANBaudrate(bitrate)
        //   const val TX_SET_RX_CB     = ?    // setCanFrameRxCallback(cb)
        @Volatile var binderReady = false        // BnMcuCanService binder 획득 성공
        @Volatile var clientConnected = false    // Python BridgeBackend 접속
        @Volatile var txAttempts = 0             // TX 요청 수(현재는 TODO 라 실제 송신 0)
        @Volatile var rxCount = 0                // RX 프레임 수(콜백 연결 후)
        @Volatile var lastError = "init"
        @Volatile var instance: CpdeviceCanBridge? = null
    }

    fun start() {
        if (running) return
        running = true
        instance = this
        connectBinder()
        serverThread = thread(name = "cpdevice-can-bridge") { serve() }
        Log.i(TAG, "CpdeviceCanBridge 시작 :$port (binderReady=$binderReady)")
    }

    fun stop() {
        running = false
        clientConnected = false
        binder = null
        serverThread = null
    }

    /** BnMcuCanService binder 획득(숨김 API ServiceManager 리플렉션). 실패해도 크래시 금지. */
    private fun connectBinder() {
        try {
            val sm = Class.forName("android.os.ServiceManager")
            val getService = sm.getMethod("getService", String::class.java)
            binder = getService.invoke(null, SERVICE_NAME) as? IBinder
            binderReady = (binder != null)
            lastError = if (binderReady) "ok" else "service 없음($SERVICE_NAME)"
            Log.i(TAG, "binder 연결: ${if (binderReady) "OK" else lastError}")
        } catch (e: Throwable) {
            binder = null; binderReady = false
            lastError = "binder 예외: ${e.message}"
            Log.w(TAG, lastError)
        }
    }

    fun status(): String =
        """{"bridge":"cpdevice","binderReady":$binderReady,"connected":$clientConnected,""" +
        """"txAttempts":$txAttempts,"rxCount":$rxCount,"lastError":"${lastError.replace("\"","'")}"}"""

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

        // CAN → Python relay. ★ TODO: setCanFrameRxCallback 등록 시 수신 프레임을 여기로 push.
        //   현재는 keepalive(heartbeat)만 — Python rx_timeout 단선 감지/접속 유지용.
        val hbThread = thread(name = "cpdevice-can-hb") {
            while (running && !sock.isClosed) {
                try {
                    synchronized(out) { out.write(encode(HEARTBEAT, 0, ByteArray(8))); out.flush() }
                    Thread.sleep(1000)
                } catch (e: Exception) { break }
            }
        }

        val rec = ByteArray(REC)
        try {
            while (running) {
                inp.readFully(rec)
                val id = (((rec[0].toLong() and 0xFF) shl 24) or
                          ((rec[1].toLong() and 0xFF) shl 16) or
                          ((rec[2].toLong() and 0xFF) shl 8) or
                           (rec[3].toLong() and 0xFF))
                val dlc = (rec[4].toInt() and 0xFF).coerceIn(0, 8)
                val data = rec.copyOfRange(5, 5 + dlc)
                if (id != HEARTBEAT) sendCanFrame(id, dlc, data)
            }
        } finally {
            clientConnected = false
            hbThread.interrupt()
        }
    }

    /**
     * BnMcuCanService.sendCanFrame(canId, dlc, data) 호출 — ★ 골격(미연결).
     * 트랜잭션 코드/Parcel 마샬링 포맷이 미확정이라 **실제 transact 는 TODO**.
     * 추측 트랜잭션/프레임을 모터에 보내지 않는다(안전). 확정 후 아래 TODO 를 구현한다.
     */
    private fun sendCanFrame(canId: Long, dlc: Int, data: ByteArray) {
        txAttempts++
        // TODO(HW): binder!!.transact(TX_SEND_CAN_FRAME, parcelIn, parcelOut, 0)
        //   parcelIn 마샬링: writeInterfaceToken(BnMcuCanService 인터페이스 디스크립터) +
        //   canId(int) + dlc(int) + byte[] data  (정확한 순서/타입은 Ghidra 분석으로 확정)
        //   ↳ 확정 전까지 송신하지 않음(추측 금지). binderReady 여부만 추적.
        if (txAttempts % 50 == 1) {
            Log.w(TAG, "sendCanFrame TODO(미구현) id=0x%08X dlc=%d (binderReady=%s) — 마샬링 확정 필요"
                .format(canId, dlc, binderReady))
        }
    }

    /** ★ TODO(HW): setCANBaudrate(bitrate) — 트랜잭션 코드 확정 후 구현. agmo_dual 과 동일 250k 기본. */
    fun setBaudrate(@Suppress("UNUSED_PARAMETER") bitrate: Int) {
        // TODO(HW): binder transact (TX_SET_BAUDRATE). 미확정 → no-op.
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
