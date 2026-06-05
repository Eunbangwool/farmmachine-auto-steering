package com.farmmachine.autosteer.can

/**
 * VanMcu — libsysmcu.so JNI 래퍼
 * Apollo 10 Pro 시스템 라이브러리 직접 호출.
 *
 * ADB Device Admin 등록 후 CAN 하드웨어 접근 가능:
 *   adb shell dpm set-device-owner com.farmmachine.autosteer/.AdminReceiver
 */
object VanMcu {
    init {
        try {
            System.loadLibrary("sysmcu")
            android.util.Log.i("VanMcu", "libsysmcu.so loaded")
        } catch (e: UnsatisfiedLinkError) {
            android.util.Log.w("VanMcu", "libsysmcu.so not found (non-Apollo device)")
        }
    }

    @JvmStatic external fun CanWrite(channel: Int, canId: Int, data: ByteArray): Boolean
    @JvmStatic external fun setCanSpeed(channel: Int, speed: Int): Boolean
    @JvmStatic external fun setCallback(enable: Boolean): Boolean
    @JvmStatic external fun OutputSet(pin: Int, value: Int): Boolean
    @JvmStatic external fun getPowerVoltage(): Float
    @JvmStatic external fun getTemperature(): Float
    @JvmStatic external fun getVersion(): String

    interface OnCanListener {
        fun onReceive(channel: Int, canId: Int, data: ByteArray)
    }
    @JvmStatic external fun setOnCanListener(listener: OnCanListener)
}
