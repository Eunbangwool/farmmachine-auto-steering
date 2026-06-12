package com.farmmachine.autosteer.can

/**
 * CANopen 모터 통신 스캐폴드 (구조만) — CHCNAV `.so` 정적분석 기반 인터페이스 정의.
 *
 * ★ 출처(유일): repo 루트 `CHCNAV_PARAM_PROFILE.md` §2 + §8.
 *   "모터는 CANopen(CiA 301/402). SDO+PDO+NMT. CAN 2채널
 *    (send_can_message_C / send_can_message_CAN2)."
 *   함수명은 .so 심볼(SetControlWord/SetTargetTorque/Motor_Enable 등) 참고 — 동작 사실만 자체 구현(clean-room).
 *
 * ⚠ 실제 CAN ID / 객체사전 인덱스 / 바이트 구성은 PROFILE.md 에 없음(하드웨어 확인 필요).
 *   → 본 파일은 인터페이스와 TODO 만. 추측값 하드코딩 금지(작업 지시서 제약).
 *   기존 송수신은 [com.farmmachine.autosteer.can.ApolloCanBridge] (VanMcu/libsysmcu.so) 를 통한다.
 */

/** CiA 301 NMT 상태. */
enum class NmtState { INITIALISING, PRE_OPERATIONAL, OPERATIONAL, STOPPED }

/** CiA 402 모터 동작 모드(필요 부분만). */
enum class OperationMode { TORQUE, SPEED /* , POSITION … 확정 시 추가 */ }

/** CAN 채널 — PROFILE §2: 2채널(C / CAN2). 실제 어느 채널이 모터인지 TODO. */
enum class CanChannel { C, CAN2 }

/**
 * CANopen SDO/PDO/NMT 저수준 인터페이스.
 * 구현체는 ApolloCanBridge(VanMcu) 위에서 프레임을 구성한다(별도 작업).
 */
interface CanOpenBus {
    // ── SDO (객체사전 읽기/쓰기) ─────────────────────────────
    // TODO(HW): index/subIndex 는 CiA 402 표준이나 벤더 확장 여부 미확정 → 호출부에서 주입.
    fun sdoRead(channel: CanChannel, nodeId: Int, index: Int, subIndex: Int): ByteArray?
    fun sdoWrite(channel: CanChannel, nodeId: Int, index: Int, subIndex: Int, data: ByteArray): Boolean

    // ── PDO (프로세스 데이터 등록) ───────────────────────────
    // TODO(HW): PDO COB-ID 매핑 미확정.
    fun registerPdo(channel: CanChannel, cobId: Int, onData: (ByteArray) -> Unit): Boolean

    // ── NMT (네트워크 상태 제어) ─────────────────────────────
    fun setNmtState(channel: CanChannel, nodeId: Int, state: NmtState): Boolean
}

/**
 * CiA 402 모터 제어 인터페이스 — .so 심볼 미러.
 * SetControlWord / SetTargetTorque / Motor_Enable / SpeedModeMove / GetCurrentSpeed / GetCurrentPosition.
 */
interface CanOpenMotor {
    /** Motor_Enable — ControlWord 시퀀스로 드라이버 enable. */
    fun enable(): Boolean
    /** Motor_Disable. */
    fun disable(): Boolean

    /** SetControlWord — CiA 402 제어워드 직접 기입. */
    fun setControlWord(controlWord: Int): Boolean

    /** SetTargetTorque — 토크 모드 목표(raw, 드라이버 단위). */
    fun setTargetTorque(targetRaw: Int): Boolean

    /** SpeedModeMove — 속도 모드 목표(±, raw). AGMO 무WAS 속도제어와 대응. */
    fun speedModeMove(speedRaw: Int): Boolean

    /** GetCurrentSpeed — 현재 모터 속도(raw). 미수신 시 null. */
    fun getCurrentSpeed(): Int?
    /** GetCurrentPosition — 현재 누적 위치/각(raw). 미수신 시 null. */
    fun getCurrentPosition(): Int?
}

/**
 * 스캐폴드 구현 — 구조만. 실제 프레임 구성은 CAN ID/객체사전 확정 후 채운다.
 * 지금은 호출 시 미구현임을 분명히 알리고(TODO), 추측 프레임을 송신하지 않는다.
 */
class CanOpenMotorScaffold(
    private val bus: CanOpenBus,
    private val channel: CanChannel,   // TODO(HW): 모터가 C/CAN2 중 어느 채널인지 미확정
    private val nodeId: Int,           // TODO(HW): CANopen node-id 미확정
) : CanOpenMotor {

    private fun notWired(symbol: String): Nothing =
        throw NotImplementedError(
            "CANopen $symbol 미배선 — CAN ID/객체사전 인덱스가 CHCNAV_PARAM_PROFILE.md 에 없음. " +
            "하드웨어 캡처로 확정 후 구현(추측 프레임 송신 금지)."
        )

    // TODO(HW): CiA 402 enable 시퀀스(ControlWord 0x06→0x07→0x0F)와 객체 인덱스 확정 필요.
    override fun enable(): Boolean = notWired("Motor_Enable")
    override fun disable(): Boolean = notWired("Motor_Disable")
    override fun setControlWord(controlWord: Int): Boolean = notWired("SetControlWord(0x6040)")
    override fun setTargetTorque(targetRaw: Int): Boolean = notWired("SetTargetTorque(0x6071)")
    override fun speedModeMove(speedRaw: Int): Boolean = notWired("SpeedModeMove")
    override fun getCurrentSpeed(): Int? = notWired("GetCurrentSpeed(0x606C)")
    override fun getCurrentPosition(): Int? = notWired("GetCurrentPosition(0x6064)")
}
