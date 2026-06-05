package com.farmmachine.autosteer.can

import android.util.Log
import kotlinx.coroutines.*
import java.util.concurrent.LinkedBlockingQueue

/**
 * ApolloCanBus — KY170C 모터 CAN 제어
 * 매뉴얼 V2.4 프로토콜 확정.
 *
 * 사용:
 *   val bus = ApolloCanBus(channel=0)
 *   bus.start()
 *   bus.enableMotor()
 *   bus.sendSpeed(300)   // +24RPM
 *   bus.disableMotor()
 *   bus.stop()
 */
class ApolloCanBus(
    private val channel: Int = 0,
    private val bitrate: Int = 250_000,
) {
    private val scope = CoroutineScope(Dispatchers.IO + SupervisorJob())
    private var watchdogJob: Job? = null
    private val recvQueue = LinkedBlockingQueue<Pair<Int, ByteArray>>(256)
    var onHeartbeat: ((HeartbeatData) -> Unit)? = null

    fun start() {
        VanMcu.setCanSpeed(channel, bitrate)
        VanMcu.setCallback(true)
        VanMcu.setOnCanListener(object : VanMcu.OnCanListener {
            override fun onReceive(ch: Int, canId: Int, data: ByteArray) {
                if (ch != channel) return
                recvQueue.offer(canId to data)
                if (canId == KY170C.HEARTBEAT_ID) {
                    KY170C.parseHeartbeat(data)?.let { onHeartbeat?.invoke(it) }
                }
            }
        })
        Log.i("ApolloCanBus", "CAN ch=$channel started at ${bitrate/1000}kbps")
    }

    fun stop() {
        watchdogJob?.cancel()
        disableMotor()
        scope.cancel()
    }

    fun enableMotor() = send(KY170C.TX_ID, KY170C.CMD_ENABLE)
    fun disableMotor() = send(KY170C.TX_ID, KY170C.CMD_DISABLE)

    /**
     * 속도 명령 (-1000~+1000 ‰ of 80RPM).
     * +1000=+80RPM, +500=+40RPM, -1000=-80RPM.
     * Watchdog 1000ms — 자동으로 keepalive 전송.
     */
    fun sendSpeed(permille: Int) {
        send(KY170C.TX_ID, KY170C.cmdSpeed(permille))
        resetWatchdog(permille)
    }

    fun send(id: Int, data: ByteArray) {
        try { VanMcu.CanWrite(channel, id, data) }
        catch (e: Exception) { Log.w("ApolloCanBus", "send failed: ${e.message}") }
    }

    private fun resetWatchdog(permille: Int) {
        watchdogJob?.cancel()
        watchdogJob = scope.launch {
            delay(800)
            // 워치독 방지용 keepalive (같은 속도 재전송)
            send(KY170C.TX_ID, KY170C.cmdSpeed(permille))
        }
    }
}

// ─── KY170C 프로토콜 ───────────────────────────────────────────
object KY170C {
    const val TX_ID        = 0x06000001
    const val RX_ID        = 0x05800001
    const val HEARTBEAT_ID = 0x07000001

    val CMD_ENABLE  = byteArrayOf(0x23, 0x0D, 0x20, 0x01, 0x00, 0x00, 0x00, 0x00)
    val CMD_DISABLE = byteArrayOf(0x23, 0x0C, 0x20, 0x01, 0x00, 0x00, 0x00, 0x00)

    fun cmdSpeed(permille: Int): ByteArray {
        val v = permille.coerceIn(-1000, 1000)
        return byteArrayOf(0x23, 0x00, 0x20, 0x01) + encodeValue(v)
    }

    fun cmdPosition(counts: Int): ByteArray {
        return byteArrayOf(0x23, 0x02, 0x20, 0x01) + encodeValue(counts)
    }

    fun encodeValue(value: Int): ByteArray {
        val v = value.toLong() and 0xFFFFFFFFL
        val lw = (v and 0xFFFFL).toInt()
        val hw = ((v shr 16) and 0xFFFFL).toInt()
        return byteArrayOf(
            ((lw shr 8) and 0xFF).toByte(), (lw and 0xFF).toByte(),
            ((hw shr 8) and 0xFF).toByte(), (hw and 0xFF).toByte(),
        )
    }

    data class HeartbeatData(
        val angleRaw: Int, val speedRpm: Int,
        val current: Int, val errorD0: Int, val errorD1: Int,
        val faults: List<String>,
    )

    fun parseHeartbeat(data: ByteArray): HeartbeatData? {
        if (data.size < 8) return null
        val angle = ((data[0].toInt() and 0xFF) shl 8) or (data[1].toInt() and 0xFF)
        val speed = java.nio.ByteBuffer.wrap(data, 2, 2).order(java.nio.ByteOrder.BIG_ENDIAN).short.toInt()
        val curr  = java.nio.ByteBuffer.wrap(data, 4, 2).order(java.nio.ByteOrder.BIG_ENDIAN).short.toInt()
        val d0 = data[6].toInt() and 0xFF
        val d1 = data[7].toInt() and 0xFF
        return HeartbeatData(angle, speed, curr, d0, d1, parseFaults(d0, d1))
    }

    fun parseFaults(d0: Int, d1: Int) = buildList<String> {
        if (d0 and 0x01 != 0) add("Less phase")
        if (d0 and 0x02 != 0) add("Motor stall")
        if (d0 and 0x04 != 0) add("Hall failure")
        if (d0 and 0x20 != 0) add("CAN disconnected")
        if (d0 and 0x80 != 0) add("Motor stalled 2s")
        if (d1 and 0x01 != 0) add("Disabled")
        if (d1 and 0x02 != 0) add("Overvoltage")
        if (d1 and 0x20 != 0) add("Undervoltage")
        if (d1 and 0x80 != 0) add("Mode failure")
    }
}

private operator fun ByteArray.plus(other: ByteArray) = ByteArray(size + other.size).also {
    copyInto(it); other.copyInto(it, size)
}
