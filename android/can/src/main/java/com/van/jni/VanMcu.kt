package com.van.jni

/**
 * VanMcu.kt
 * Apollo 10 Pro 전용 네이티브 CAN/GPIO 라이브러리 JNI 래퍼.
 *
 * 네이티브 라이브러리: /system/lib64/libsysmcu.so
 * JNI 심볼 패키지: com.van.jni (이 파일의 패키지와 반드시 일치해야 함)
 *
 * 분석 출처: com.agmo.autokit APK 디컴파일 (jadx)
 * 클래스: com.van.jni.VanMcu (public final class)
 */
object VanMcu {

    // ── 상수 ──────────────────────────────────────────────
    const val ACC       = 1
    const val CAN       = 2
    const val DEBUG     = 8
    const val INPUT     = 4
    const val CAN_EFF_FLAG = 0   // Extended Frame Flag
    const val CAN_RTR_FLAG = 0   // Remote Transmission Request

    // ── 초기화 ────────────────────────────────────────────
    init {
        System.loadLibrary("sysmcu")
    }

    // ── 내부 데이터 클래스 ────────────────────────────────

    /** CAN 메시지 (수신 콜백에서 전달됨) */
    data class CanMsg(
        @JvmField var channel: Int = 0,
        @JvmField var id: Int = 0,
        @JvmField var data: ByteArray = byteArrayOf()
    )

    // ── 콜백 인터페이스 ───────────────────────────────────

    /** CAN 프레임 수신 콜백 */
    interface OnCanListener {
        fun OnCan(canMsg: CanMsg)
    }

    /** ACC(시동) 상태 변경 콜백 */
    interface OnAccListener {
        fun OnAcc(state: Int)
    }

    /** 디버그 메시지 콜백 */
    interface OnDebugListener {
        fun OnDebug(str: String)
    }

    /** 디지털 입력 변경 콜백 */
    interface OnInputListener {
        fun OnInput(pin: Int, value: Int)
    }

    // ── CAN 통신 ──────────────────────────────────────────

    /**
     * CAN 프레임 송신.
     * @param channel CAN 채널 (0 또는 1)
     * @param canId   CAN 메시지 ID
     * @param data    송신 데이터 (최대 8바이트)
     * @return 성공 여부
     */
    @JvmStatic external fun CanWrite(channel: Int, canId: Int, data: ByteArray): Boolean

    /** CAN 비트레이트 설정 (예: 250000, 500000, 1000000) */
    @JvmStatic external fun setCanSpeed(channel: Int, speed: Int): Boolean

    /** CAN 비트레이트 조회 */
    @JvmStatic external fun getCanSpeed(channel: Int): Int

    /** CAN 채널 수 */
    @JvmStatic external fun getCanCount(): Int

    // ── CAN 필터 ──────────────────────────────────────────

    /** CAN 필터 제어 (0=비활성, 1=활성) */
    @JvmStatic external fun CanFilterCtrl(channel: Int, enable: Int): Boolean

    /** HW 필터 추가 (channel, id, mask) */
    @JvmStatic external fun CanHwFilterAdd(channel: Int, id: Int, mask: Int): Boolean

    /** HW 필터 전체 삭제 */
    @JvmStatic external fun CanHwFilterClear(channel: Int): Boolean

    /** SW 필터 추가 */
    @JvmStatic external fun CanSwFilterAdd(channel: Int, id: Int, mask: Int): Boolean

    /** SW 필터 전체 삭제 */
    @JvmStatic external fun CanSwFilterClear(channel: Int): Boolean

    // ── 콜백 등록 ─────────────────────────────────────────

    /**
     * 콜백 시스템 활성화/비활성화.
     * CAN 수신, ACC 상태, 입력 변경 이벤트를 받으려면 true로 설정.
     */
    @JvmStatic external fun setCallback(enable: Boolean): Boolean

    /** 내부 콜백 처리기 (native → Java 호출, 직접 사용 금지) */
    @JvmStatic external fun onCallback(type: Int, data: ByteArray)

    /** CAN 수신 리스너 등록 */
    @JvmStatic external fun setOnCanListener(listener: OnCanListener?)

    /** ACC 상태 리스너 등록 */
    @JvmStatic external fun setOnAccListener(listener: OnAccListener?)

    /** 디버그 리스너 등록 */
    @JvmStatic external fun setOnDebugListener(listener: OnDebugListener?)

    /** 디지털 입력 리스너 등록 */
    @JvmStatic external fun setOnInputListener(listener: OnInputListener?)

    // ── 블록 I/O ─────────────────────────────────────────

    /** 블록 데이터 읽기 */
    @JvmStatic external fun BlockRead(channel: Int, buf: ByteArray, len: Int): Boolean

    /** 블록 데이터 쓰기 */
    @JvmStatic external fun BlockWrite(channel: Int, buf: ByteArray): Boolean

    /** 블록 채널 수 */
    @JvmStatic external fun getBlockCount(): Int

    // ── GPIO ─────────────────────────────────────────────

    /**
     * 디지털 출력 설정.
     * 레이저 커넥터 UP/DOWN/HOLD 핀 직접 제어용.
     * @param pin   핀 번호
     * @param value 0=LOW, 1=HIGH
     */
    @JvmStatic external fun OutputSet(pin: Int, value: Int): Boolean

    /** 디지털 출력 상태 읽기 */
    @JvmStatic external fun OutputGet(pin: Int): Int

    /** 디지털 입력 읽기 */
    @JvmStatic external fun InputGet(pin: Int): Int

    /** 디지털 입력 핀 수 */
    @JvmStatic external fun getInputCount(): Int

    /** 디지털 출력 핀 수 */
    @JvmStatic external fun getOutputCount(): Int

    // ── 시스템 상태 ───────────────────────────────────────

    /** ACC(시동) 상태 (0=OFF, 비영=ON) */
    @JvmStatic external fun getAccState(): Int

    /** 공급 전압 (raw, /1000.0 → 볼트) */
    @JvmStatic external fun getPowerVoltage(): Int

    /** 온도 (raw, /100.0 → 섭씨) */
    @JvmStatic external fun getTemperature(): Int

    /** MCU 펌웨어 버전 */
    @JvmStatic external fun getVersion(): String

    /** 전원 제어 */
    @JvmStatic external fun PowerCtrl(cmd: Int): Boolean

    /** 펌웨어 업데이트 */
    @JvmStatic external fun UpdateFirmware(path: String): Boolean

    // ── 유틸리티 ─────────────────────────────────────────

    /** 바이트 배열에서 Int 추출 (offset, length) */
    @JvmStatic external fun getInt(buf: ByteArray, offset: Int, len: Int): Int

    // ── 편의 메서드 (Kotlin 추가) ─────────────────────────

    /** 전압 (볼트) */
    val voltage: Double get() = getPowerVoltage() / 1000.0

    /** 온도 (섭씨) */
    val temperatureCelsius: Double get() = getTemperature() / 100.0

    /** 시동 켜짐 여부 */
    val isAccOn: Boolean get() = getAccState() != 0
}
