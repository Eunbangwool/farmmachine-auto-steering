package com.farmmachine.autosteer.can

import android.util.Log
import java.io.DataInputStream
import java.net.ServerSocket
import java.net.Socket
import kotlin.concurrent.thread

/**
 * Apollo 10 Pro CAN ↔ Python 브릿지 (localhost TCP).
 * Python 의 apollo_can.ApolloCanBus(backend="bridge") 가 클라이언트로 접속한다.
 *
 * 와이어: 양방향 13바이트 레코드 = id(u32 BE) | dlc(u8) | data(8B).
 *         id == 0x7FFFFFFF 는 heartbeat(keepalive).
 * (계약 상세: auto-steering/APOLLO_CAN.md)
 *
 * ★ 남은 작업: 벤더 CAN SDK 호출부(openCan/canSend/canReceive)를 채운다.
 *   Apollo 10 Pro(CPDEVICE)의 CAN SDK(JAR/JNI)로 교체.
 */
class ApolloCanBridge(private val port: Int = 47100) {

    @Volatile private var running = false
    private var serverThread: Thread? = null

    private val REC = 13
    private val HEARTBEAT = 0x7FFFFFFFL
    private val TAG = "ApolloCanBridge"

    fun start() {
        if (running) return
        running = true
        // TODO(vendor): openCan(channel = 0, bitrate = 500_000)
        serverThread = thread(name = "apollo-can-bridge") { serve() }
        Log.i(TAG, "CAN 브릿지 시작 :$port")
    }

    fun stop() {
        running = false
        // TODO(vendor): closeCan()
        serverThread = null
    }

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
        val inp = DataInputStream(sock.getInputStream())
        val out = sock.getOutputStream()

        // CAN → Python: 벤더 SDK 수신 프레임을 13B 레코드로 전송
        val rxThread = thread(name = "apollo-can-rx") {
            while (running && !sock.isClosed) {
                // val f = canReceive(timeoutMs = 10) ?: continue   // TODO(vendor)
                // synchronized(out) { out.write(encode(f.id, f.dlc, f.data)); out.flush() }
                try { Thread.sleep(10) } catch (e: InterruptedException) { break }
            }
        }
        // heartbeat (~1s) — Python 측 rx_timeout 단선 감지용
        val hbThread = thread(name = "apollo-can-hb") {
            while (running && !sock.isClosed) {
                try {
                    synchronized(out) { out.write(encode(HEARTBEAT, 0, ByteArray(8))); out.flush() }
                    Thread.sleep(1000)
                } catch (e: Exception) { break }
            }
        }
        // Python → CAN: 13B 레코드 파싱해 벤더 SDK 로 송신
        val rec = ByteArray(REC)
        try {
            while (running) {
                inp.readFully(rec)
                val id = (((rec[0].toLong() and 0xFF) shl 24) or
                          ((rec[1].toLong() and 0xFF) shl 16) or
                          ((rec[2].toLong() and 0xFF) shl 8) or
                           (rec[3].toLong() and 0xFF))
                val dlc = (rec[4].toInt() and 0xFF).coerceIn(0, 8)
                @Suppress("UNUSED_VARIABLE")
                val data = rec.copyOfRange(5, 5 + dlc)
                if (id != HEARTBEAT) {
                    // canSend(id, data)   // TODO(vendor): 조향모터/레벨러 밸브 송신
                }
            }
        } finally {
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
