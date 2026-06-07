package com.farmmachine.autosteer.can

import android.util.Log
import com.van.jni.VanMcu
import java.io.DataInputStream
import java.net.ServerSocket
import java.net.Socket
import java.util.concurrent.LinkedBlockingQueue
import java.util.concurrent.TimeUnit
import kotlin.concurrent.thread

/**
 * Apollo 10 Pro CAN ↔ Python 브릿지 (localhost TCP).
 * Python 의 apollo_can.ApolloCanBus(backend="bridge") 가 클라이언트로 접속한다.
 *
 * 와이어: 양방향 13바이트 레코드 = id(u32 BE) | dlc(u8) | data(8B).
 *         id == 0x7FFFFFFF 는 heartbeat(keepalive).
 * (계약 상세: auto-steering/APOLLO_CAN.md)
 *
 * 하드웨어 CAN = com.van.jni.VanMcu (libsysmcu.so). 송신=CanWrite,
 * 수신=setOnCanListener 콜백. device-owner 권한 필요(AdminReceiver):
 *   adb shell dpm set-device-owner com.farmmachine.autosteer/.AdminReceiver
 */
class ApolloCanBridge(
    private val port: Int = 47100,
    private val channel: Int = 0,           // Keya 조향모터 CAN 채널 (현장 확인)
    private val bitrate: Int = 250_000,     // Keya KY170 = 250kbps (매뉴얼)
) {

    @Volatile private var running = false
    private var serverThread: Thread? = null

    private val REC = 13
    private val HEARTBEAT = 0x7FFFFFFFL
    private val TAG = "ApolloCanBridge"

    // VanMcu 수신 콜백 → 이 큐 → 접속 클라이언트(rxThread)로 relay
    private val rxQueue = LinkedBlockingQueue<Pair<Int, ByteArray>>(512)
    @Volatile private var canReady = false

    fun start() {
        if (running) return
        running = true
        openCan()
        serverThread = thread(name = "apollo-can-bridge") { serve() }
        Log.i(TAG, "CAN 브릿지 시작 :$port (ch=$channel @${bitrate / 1000}kbps, canReady=$canReady)")
    }

    /** libsysmcu.so CAN 채널 오픈 + 수신 콜백 등록. */
    private fun openCan() {
        if (!VanMcu.available) {
            Log.w(TAG, "VanMcu 미탑재 → CAN 비활성(UI/Python 은 동작, 모터 송신 무시)")
            canReady = false; return
        }
        try {
            VanMcu.setCanSpeed(channel, bitrate)
            VanMcu.setCallback(true)
            VanMcu.setOnCanListener(object : VanMcu.OnCanListener {
                override fun OnCan(m: VanMcu.CanMsg) {
                    if (m.channel == channel) rxQueue.offer(m.id to m.data)
                }
            })
            canReady = true
            Log.i(TAG, "libsysmcu.so CAN open OK (ch=$channel @${bitrate / 1000}kbps)")
        } catch (e: Throwable) {
            canReady = false
            Log.w(TAG, "CAN open 실패(device-owner 미설정 가능): ${e.message}")
        }
    }

    fun stop() {
        running = false
        try { if (canReady) { VanMcu.setOnCanListener(null); VanMcu.setCallback(false) } } catch (_: Throwable) {}
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

        // CAN → Python: VanMcu 수신 큐를 13B 레코드로 relay
        val rxThread = thread(name = "apollo-can-rx") {
            while (running && !sock.isClosed) {
                val f = try { rxQueue.poll(50, TimeUnit.MILLISECONDS) } catch (e: InterruptedException) { break } ?: continue
                try { synchronized(out) { out.write(encode(f.first.toLong() and 0xFFFFFFFFL, f.second.size, f.second)); out.flush() } }
                catch (e: Exception) { break }
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
        // Python → CAN: 13B 레코드 파싱해 VanMcu.CanWrite 로 송신
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
                if (id != HEARTBEAT && canReady) {
                    try { VanMcu.CanWrite(channel, id.toInt(), data) }
                    catch (e: Throwable) { Log.w(TAG, "CanWrite 실패: ${e.message}") }
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
