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
    /** 콜백 시스템 on/off (CAN 수신/ACC/입력 이벤트 수신하려면 true). */
    @JvmStatic external fun setCallback(enable: Boolean): Boolean

    @Volatile private var canListener: OnCanListener? = null
    /** CAN 수신 리스너 등록 (Kotlin 측 저장 — onCallback 이 디스패치). */
    @JvmStatic fun setOnCanListener(listener: OnCanListener?) { canListener = listener }

    /**
     * ★ libsysmcu.so 가 이벤트를 전달하는 정적 콜백 진입점(native → Java).
     * 시그니처 (I[B)V 가 .so 와 정확히 일치해야 함(없으면 setCallback 실패).
     * type==CAN 일 때 data 를 CanMsg 로 파싱해 리스너에 전달.
     * ⚠ data 바이트 포맷은 추정 — 실제 RX 포맷은 현장 캡처로 검증 필요(TX 와 무관).
     */
    @JvmStatic
    fun onCallback(type: Int, data: ByteArray) {
        try {
            if (type == CAN) {
                val l = canListener ?: return
                if (data.size >= 5) {
                    val ch = data[0].toInt() and 0xFF
                    val id = ((data[1].toInt() and 0xFF) shl 24) or
                             ((data[2].toInt() and 0xFF) shl 16) or
                             ((data[3].toInt() and 0xFF) shl 8) or
                              (data[4].toInt() and 0xFF)
                    val payload = if (data.size > 5) data.copyOfRange(5, data.size) else ByteArray(0)
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
