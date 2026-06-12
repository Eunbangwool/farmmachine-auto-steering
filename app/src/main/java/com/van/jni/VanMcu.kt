package com.van.jni

/**
 * VanMcu — Apollo 10 Pro 내장 CAN/GPIO 네이티브 라이브러리 JNI 래퍼.
 *
 * 네이티브 라이브러리: /system/lib64/libsysmcu.so (Apollo 펌웨어에 기본 탑재)
 * ★ JNI 심볼 패키지는 반드시 `com.van.jni` 여야 .so 심볼과 매칭된다(변경 금지).
 *
 * clean-room: 이 파일은 libsysmcu.so 의 **인터페이스 사실(함수 시그니처/라이브러리명/
 * 채널·속도 API)**만 선언하는 JNI 바인딩이다. 외부 앱 소스를 복붙하지 않는다.
 * 동작 사실 출처: Apollo 시스템 라이브러리 심볼.
 *
 * CAN 하드웨어 접근에는 device-owner 권한이 필요:
 *   adb shell dpm set-device-owner com.farmmachine.autosteer/.AdminReceiver
 */
object VanMcu {

    const val ACC = 1
    const val CAN = 2
    const val DEBUG = 8
    const val INPUT = 4

    @Volatile var available: Boolean = false
        private set

    init {
        try {
            System.loadLibrary("sysmcu")
            available = true
            android.util.Log.i("VanMcu", "libsysmcu.so loaded")
        } catch (e: Throwable) {
            available = false
            android.util.Log.w("VanMcu", "libsysmcu.so 미탑재(비-Apollo 기기): ${e.message}")
        }
    }

    /** CAN 메시지 (수신 콜백 전달용) */
    data class CanMsg(
        @JvmField var channel: Int = 0,
        @JvmField var id: Int = 0,
        @JvmField var data: ByteArray = byteArrayOf(),
    )

    interface OnCanListener { fun OnCan(canMsg: CanMsg) }
    interface OnAccListener { fun OnAcc(state: Int) }
    interface OnInputListener { fun OnInput(pin: Int, value: Int) }

    // ── CAN 통신 ──────────────────────────────────────────
    /** CAN 프레임 송신. channel(0/1), canId(표준/확장), data(≤8B). */
    @JvmStatic external fun CanWrite(channel: Int, canId: Int, data: ByteArray): Boolean
    /** CAN 비트레이트 설정 (250000/500000/1000000 …) */
    @JvmStatic external fun setCanSpeed(channel: Int, speed: Int): Boolean
    @JvmStatic external fun getCanSpeed(channel: Int): Int
    @JvmStatic external fun getCanCount(): Int

    // ── CAN 필터 ──────────────────────────────────────────
    @JvmStatic external fun CanFilterCtrl(channel: Int, enable: Int): Boolean
    @JvmStatic external fun CanHwFilterAdd(channel: Int, id: Int, mask: Int): Boolean
    @JvmStatic external fun CanHwFilterClear(channel: Int): Boolean

    // ── 콜백 등록 ─────────────────────────────────────────
    /**
     * 콜백 필터 비트마스크 설정 (native). 수신할 이벤트 종류를 OR 로 지정:
     *   ACC=1, CAN=2, INPUT=4, DEBUG=8.  0 = 전체 해제.
     * ★ libsysmcu.so 시그니처는 setCallback(int) — Boolean 으로 호출하면 true→1(ACC)
     *   로 들어가 CAN(2) 이 안 켜져 **CAN 수신이 영구 미발생**한다(과거 RX=0 의 진짜 원인).
     *   CAN 프레임을 받으려면 반드시 CAN 비트(2)를 포함해 호출할 것.
     */
    @JvmStatic external fun setCallback(filterBitmask: Int): Boolean

    @Volatile private var canListener: OnCanListener? = null
    /** CAN 수신 리스너 등록 (Kotlin 측 저장 — onCallback 이 디스패치). */
    @JvmStatic fun setOnCanListener(listener: OnCanListener?) { canListener = listener }

    /**
     * ★ libsysmcu.so 가 이벤트를 전달하는 정적 콜백 진입점(native → Java).
     * 시그니처 (I[B)V 가 .so 와 정확히 일치해야 함(이름/시그니처 고정).
     * type==CAN 일 때 data 를 CanMsg 로 파싱해 리스너에 전달.
     * RX 프레임 포맷(libsysmcu.so 인터페이스 사실):
     *   data[0]=channel, data[1..4]=id(**big-endian**), data[5]=DLC, data[6..6+DLC]=payload.
     */
    @Volatile private var cbCount = 0
    @JvmStatic
    fun onCallback(type: Int, data: ByteArray) {
        // 원시 포맷 캡처용 로그(처음 20개만 — 현장 1차 검증용).
        if (cbCount < 20) {
            android.util.Log.i("VanMcu", "onCallback #$cbCount type=$type len=${data.size} data=${data.joinToString(""){"%02X".format(it)}}")
            cbCount++
        }
        try {
            if (type == CAN) {
                val l = canListener ?: return
                if (data.size >= 6) {
                    val ch = data[0].toInt() and 0xFF
                    val id = ((data[1].toInt() and 0xFF) shl 24) or
                             ((data[2].toInt() and 0xFF) shl 16) or
                             ((data[3].toInt() and 0xFF) shl 8) or
                              (data[4].toInt() and 0xFF)
                    val dlc = (data[5].toInt() and 0xFF).coerceIn(0, 8)
                    val end = minOf(6 + dlc, data.size)
                    val payload = if (end > 6) data.copyOfRange(6, end) else ByteArray(0)
                    l.OnCan(CanMsg(ch, id, payload))
                }
            }
        } catch (e: Throwable) { /* RX 파싱 실패 무시 — 모터 송신(TX)에는 영향 없음 */ }
    }

    // ── GPIO (레벨러 밸브 UP/DOWN/HOLD 직접 제어용) ────────
    @JvmStatic external fun OutputSet(pin: Int, value: Int): Boolean
    @JvmStatic external fun OutputGet(pin: Int): Int
    @JvmStatic external fun InputGet(pin: Int): Int

    // ── 시스템 상태 ───────────────────────────────────────
    @JvmStatic external fun getAccState(): Int
    @JvmStatic external fun getPowerVoltage(): Int     // raw/1000.0 = V
    @JvmStatic external fun getTemperature(): Int      // raw/100.0  = ℃
    @JvmStatic external fun getVersion(): String

    val isAccOn: Boolean get() = try { getAccState() != 0 } catch (e: Throwable) { false }
}
