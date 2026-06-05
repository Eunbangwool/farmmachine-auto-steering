package com.farmmachine.can

import android.util.Log
import com.van.jni.VanMcu
import java.util.concurrent.ConcurrentLinkedQueue
import java.util.concurrent.atomic.AtomicBoolean
import java.util.concurrent.atomic.AtomicInteger

/**
 * ApolloCanBus.kt
 * Apollo 10 Pro CAN 버스 구현.
 *
 * VanMcu (libsysmcu.so) JNI를 사용해 CAN 프레임을 송수신.
 * autosteer_core.py / leveler_core.py 의 CanBus 인터페이스와 동일한 역할.
 *
 * 사용:
 *   val can = ApolloCanBus(channel = 0, bitrate = 250_000)
 *   can.start()
 *   can.send(0x201, byteArrayOf(0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00))
 *   can.setReceiveListener { id, data -> ... }
 *   can.stop()
 *
 * CAN 채널:
 *   channel=0 → AGMO 조향 모터 (PORT2, 250kbps)
 *   channel=1 → CHCNAV 앵글센서 등 (PORT3)
 *
 * GPIO (레이저 커넥터 UP/DOWN/HOLD):
 *   LaserConnectorGpio.up() / .down() / .hold()
 */
class ApolloCanBus(
    private val channel: Int = 0,
    private val bitrate: Int = 250_000
) {
    companion object {
        private const val TAG = "ApolloCanBus"
        private const val RX_QUEUE_MAX = 256
    }

    // ── 상태 ──────────────────────────────────────────────
    private val _running   = AtomicBoolean(false)
    private val _errorCount = AtomicInteger(0)
    val isRunning: Boolean get() = _running.get()

    // ── 수신 큐 (non-blocking) ───────────────────────────
    private val rxQueue = ConcurrentLinkedQueue<CanFrame>()

    // ── 수신 콜백 ────────────────────────────────────────
    private var receiveListener: ((id: Int, data: ByteArray) -> Unit)? = null

    // ── CAN 프레임 데이터 클래스 ─────────────────────────
    data class CanFrame(val id: Int, val data: ByteArray)

    // ── 시작 / 정지 ───────────────────────────────────────

    /**
     * CAN 버스 초기화 및 수신 콜백 등록.
     * Activity.onCreate 또는 Service.onStartCommand에서 호출.
     */
    fun start() {
        if (_running.get()) {
            Log.w(TAG, "이미 실행 중")
            return
        }
        try {
            // 비트레이트 설정
            val ok = VanMcu.setCanSpeed(channel, bitrate)
            Log.i(TAG, "CAN 속도 설정: ch=$channel ${bitrate/1000}kbps → $ok")

            // 수신 콜백 등록
            VanMcu.setOnCanListener(object : VanMcu.OnCanListener {
                override fun OnCan(canMsg: VanMcu.CanMsg) {
                    if (canMsg.channel != channel) return
                    val frame = CanFrame(canMsg.id, canMsg.data.copyOf())
                    // 큐에 추가 (최대 크기 제한)
                    if (rxQueue.size < RX_QUEUE_MAX) {
                        rxQueue.offer(frame)
                    }
                    // 즉시 콜백
                    receiveListener?.invoke(frame.id, frame.data)
                }
            })

            // 콜백 시스템 활성화
            VanMcu.setCallback(true)
            _running.set(true)
            _errorCount.set(0)
            Log.i(TAG, "ApolloCanBus 시작 (ch=$channel)")

        } catch (e: UnsatisfiedLinkError) {
            Log.e(TAG, "libsysmcu.so 로드 실패 — Apollo 기기에서만 동작합니다", e)
            throw RuntimeException("ApolloCanBus: libsysmcu.so 없음 — Apollo 전용", e)
        } catch (e: Exception) {
            Log.e(TAG, "CAN 초기화 실패", e)
            throw e
        }
    }

    /**
     * CAN 버스 정지.
     */
    fun stop() {
        if (!_running.get()) return
        VanMcu.setCallback(false)
        VanMcu.setOnCanListener(null)
        rxQueue.clear()
        _running.set(false)
        Log.i(TAG, "ApolloCanBus 정지")
    }

    // ── 송신 ─────────────────────────────────────────────

    /**
     * CAN 프레임 송신.
     * @param canId  CAN 메시지 ID
     * @param data   송신 데이터 (최대 8바이트)
     * @return 성공 여부
     */
    fun send(canId: Int, data: ByteArray): Boolean {
        if (!_running.get()) {
            Log.w(TAG, "send() 호출됐으나 CAN 미실행")
            return false
        }
        return try {
            VanMcu.CanWrite(channel, canId, data)
        } catch (e: Exception) {
            _errorCount.incrementAndGet()
            Log.e(TAG, "CAN 송신 오류 id=0x${canId.toString(16)}", e)
            false
        }
    }

    // ── 수신 ─────────────────────────────────────────────

    /**
     * 수신 리스너 등록 (이벤트 기반, 권장).
     * CAN 프레임 도착 즉시 Main Thread에서 호출됨.
     */
    fun setReceiveListener(listener: ((id: Int, data: ByteArray) -> Unit)?) {
        receiveListener = listener
    }

    /**
     * 수신 큐에서 꺼내기 (폴링 방식, 50ms 루프용).
     * @return 큐에 프레임 없으면 null
     */
    fun recv(): CanFrame? = rxQueue.poll()

    /**
     * 특정 CAN ID 필터로 수신 큐에서 꺼내기.
     * autosteer_core의 앵글센서 ID 필터링에 사용.
     */
    fun recvId(canId: Int): CanFrame? {
        val iter = rxQueue.iterator()
        while (iter.hasNext()) {
            val f = iter.next()
            if (f.id == canId) { iter.remove(); return f }
        }
        return null
    }

    // ── CAN 필터 ─────────────────────────────────────────

    /**
     * HW 수신 필터 설정 (불필요한 CAN ID 차단 → 성능 향상).
     * @param id   허용할 CAN ID
     * @param mask 비교 마스크 (0x7FF = 정확히 일치)
     */
    fun addFilter(id: Int, mask: Int = 0x7FF): Boolean {
        return VanMcu.CanHwFilterAdd(channel, id, mask)
    }

    fun clearFilters(): Boolean {
        return VanMcu.CanHwFilterClear(channel)
    }

    // ── 상태 조회 ─────────────────────────────────────────
    val errorCount: Int get() = _errorCount.get()
    val rxQueueSize: Int get() = rxQueue.size

    fun status(): Map<String, Any> = mapOf(
        "running"    to isRunning,
        "channel"    to channel,
        "bitrate"    to bitrate,
        "errors"     to errorCount,
        "rx_queued"  to rxQueueSize,
        "voltage"    to VanMcu.voltage,
        "temp_c"     to VanMcu.temperatureCelsius,
        "acc_on"     to VanMcu.isAccOn,
    )
}


// ═══════════════════════════════════════════════════════════════
//  레이저 커넥터 GPIO 제어
//  Kubota (7) 커넥터 UP/DOWN/HOLD 핀을 OutputSet으로 직접 제어
//  ★ 핀 번호는 MEASUREMENT_CHECKLIST.md A항목 실측 후 채울 것
// ═══════════════════════════════════════════════════════════════

object LaserConnectorGpio {
    private const val TAG = "LaserGpio"

    // ★ 실측 필요 — MEASUREMENT_CHECKLIST.md A4, A5 항목
    private var pinUp   = -1   // UP 핀 번호 ★
    private var pinDown = -1   // DOWN 핀 번호 ★
    private var activeHigh = true  // Active High/Low ★ A6 항목

    /**
     * 핀 설정.
     * @param up        UP 출력 핀 번호
     * @param down      DOWN 출력 핀 번호
     * @param highActive 12V 인가가 동작(true) / GND가 동작(false)
     */
    fun configure(up: Int, down: Int, highActive: Boolean = true) {
        pinUp = up; pinDown = down; activeHigh = highActive
        Log.i(TAG, "레이저 커넥터 핀 설정: UP=$up DOWN=$down ActiveHigh=$highActive")
    }

    private fun on()  = if (activeHigh) 1 else 0
    private fun off() = if (activeHigh) 0 else 1

    /** 블레이드 상승 명령 */
    fun up() {
        require(pinUp >= 0) { "핀 미설정 — configure() 먼저 호출" }
        VanMcu.OutputSet(pinUp, on())
        VanMcu.OutputSet(pinDown, off())
        Log.v(TAG, "UP")
    }

    /** 블레이드 하강 명령 */
    fun down() {
        require(pinDown >= 0) { "핀 미설정 — configure() 먼저 호출" }
        VanMcu.OutputSet(pinUp, off())
        VanMcu.OutputSet(pinDown, on())
        Log.v(TAG, "DOWN")
    }

    /** 현 위치 유지 */
    fun hold() {
        if (pinUp >= 0)   VanMcu.OutputSet(pinUp, off())
        if (pinDown >= 0) VanMcu.OutputSet(pinDown, off())
        Log.v(TAG, "HOLD")
    }

    val isConfigured: Boolean get() = pinUp >= 0 && pinDown >= 0
}



// ═══════════════════════════════════════════════════════════════
//  KY170C 모터 전용 헬퍼 — 매뉴얼 V2.4 확정 프로토콜
// ═══════════════════════════════════════════════════════════════

object KY170C {
    // CAN ID (motor_id=1, parameter 0018=1)
    const val TX_ID        = 0x06000001  // 명령 전송 ID
    const val RX_ID        = 0x05800001  // 응답 수신 ID
    const val HEARTBEAT_ID = 0x07000001  // 하트비트 ID (20ms)

    // 고정 명령
    val CMD_ENABLE  = byteArrayOf(0x23, 0x0D, 0x20, 0x01, 0x00, 0x00, 0x00, 0x00)
    val CMD_DISABLE = byteArrayOf(0x23, 0x0C, 0x20, 0x01, 0x00, 0x00, 0x00, 0x00)

    /**
     * 속도 명령 생성.
     * @param permille -1000~+1000 (rated 80RPM의 ‰)
     *   +1000 = +80RPM,  +500 = +40RPM,  -1000 = -80RPM
     * Watchdog: 1000ms 이내 재전송 필수.
     */
    fun cmdSpeed(permille: Int): ByteArray {
        val v = permille.coerceIn(-1000, 1000)
        return byteArrayOf(0x23, 0x00, 0x20, 0x01) + encodeValue(v)
    }

    /**
     * 위치 명령 생성.
     * @param counts 10000 counts/circle (양수=CCW, 음수=CW)
     */
    fun cmdPosition(counts: Int): ByteArray {
        return byteArrayOf(0x23, 0x02, 0x20, 0x01) + encodeValue(counts)
    }

    /**
     * 32비트 값 인코딩 (low word big-endian first).
     * 예) 1000 → [0x03][0xE8][0x00][0x00]
     *    -1000 → [0xFC][0x18][0xFF][0xFF]
     */
    fun encodeValue(value: Int): ByteArray {
        val v = value.toLong() and 0xFFFFFFFFL
        val lowWord  = (v and 0xFFFFL).toInt()
        val highWord = ((v shr 16) and 0xFFFFL).toInt()
        return byteArrayOf(
            ((lowWord  shr 8) and 0xFF).toByte(),
            (lowWord         and 0xFF).toByte(),
            ((highWord shr 8) and 0xFF).toByte(),
            (highWord        and 0xFF).toByte(),
        )
    }

    /**
     * 하트비트 파싱 (big-endian, 20ms 주기).
     * [0][1]=누적각도(360단위/circle) [2][3]=속도RPM [4][5]=전류 [6][7]=오류코드
     */
    data class HeartbeatData(
        val angleRaw:  Int,    // 누적 각도 (0~65535, 리셋됨)
        val speedRpm:  Int,    // 속도 (RPM, 부호있음)
        val current:   Int,    // 전류 (raw, 부호있음)
        val errorD0:   Int,    // 오류 바이트0
        val errorD1:   Int,    // 오류 바이트1
        val faults:    List<String>,
    )

    fun parseHeartbeat(data: ByteArray): HeartbeatData? {
        if (data.size < 8) return null
        val angleRaw = ((data[0].toInt() and 0xFF) shl 8) or (data[1].toInt() and 0xFF)
        val speedRpm = java.nio.ByteBuffer.wrap(data, 2, 2)
                        .order(java.nio.ByteOrder.BIG_ENDIAN).short.toInt()
        val current  = java.nio.ByteBuffer.wrap(data, 4, 2)
                        .order(java.nio.ByteOrder.BIG_ENDIAN).short.toInt()
        val d0 = data[6].toInt() and 0xFF
        val d1 = data[7].toInt() and 0xFF
        return HeartbeatData(angleRaw, speedRpm, current, d0, d1,
                             parseFaults(d0, d1))
    }

    fun parseFaults(d0: Int, d1: Int): List<String> {
        val faults = mutableListOf<String>()
        if (d0 and 0x01 != 0) faults += "Less phase"
        if (d0 and 0x02 != 0) faults += "Motor stall"
        if (d0 and 0x04 != 0) faults += "Hall failure"
        if (d0 and 0x10 != 0) faults += "232 disconnected"
        if (d0 and 0x20 != 0) faults += "CAN disconnected"
        if (d0 and 0x80 != 0) faults += "Motor stalled 2s"
        if (d1 and 0x01 != 0) faults += "Disabled"
        if (d1 and 0x02 != 0) faults += "Overvoltage"
        if (d1 and 0x08 != 0) faults += "Hardware protection"
        if (d1 and 0x10 != 0) faults += "EEPROM error"
        if (d1 and 0x20 != 0) faults += "Undervoltage"
        if (d1 and 0x80 != 0) faults += "Mode failure"
        return faults
    }
}

private operator fun ByteArray.plus(other: ByteArray): ByteArray {
    val result = ByteArray(this.size + other.size)
    this.copyInto(result)
    other.copyInto(result, this.size)
    return result
}

// ═══════════════════════════════════════════════════════════════
//  듀얼 채널 관리 (조향 + 레벨러 동시 사용)
// ═══════════════════════════════════════════════════════════════

object ApolloCanManager {
    /**
     * AGMO 조향 모터 채널 (ch=0, 250kbps 기본값).
     * ★ 실제 비트레이트는 멀티미터/CAN 트래픽 분석으로 확인 필요.
     */
    val steerBus = ApolloCanBus(channel = 0, bitrate = 250_000)

    /**
     * CHCNAV 앵글센서 채널 (ch=1).
     * NX510이 PORT3를 사용하므로 별도 채널 가능성.
     */
    val sensorBus = ApolloCanBus(channel = 1, bitrate = 250_000)

    /** KY170C 모터 활성화 (Enable 명령) */
    fun enableMotor() = steerBus.send(KY170C.TX_ID, KY170C.CMD_ENABLE)

    /** KY170C 모터 비활성화 */
    fun disableMotor() = steerBus.send(KY170C.TX_ID, KY170C.CMD_DISABLE)

    /**
     * 조향 속도 명령 (-1000~+1000, rated 80RPM의 ‰).
     * Watchdog 1000ms — 1초마다 재전송 필요.
     */
    fun sendSpeed(permille: Int) = steerBus.send(KY170C.TX_ID, KY170C.cmdSpeed(permille))

    fun startAll() {
        runCatching { steerBus.start() }
            .onFailure { Log.e("CanManager", "조향 CAN 시작 실패", it) }
        runCatching { sensorBus.start() }
            .onFailure { Log.e("CanManager", "센서 CAN 시작 실패", it) }
    }

    fun stopAll() {
        steerBus.stop()
        sensorBus.stop()
    }

    fun statusAll(): Map<String, Any> = mapOf(
        "steer"  to steerBus.status(),
        "sensor" to sensorBus.status(),
    )
}
