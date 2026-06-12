package com.farmmachine.autosteer.safety

import com.farmmachine.autosteer.profile.SafetyLimits

/**
 * 자동조향 안전 가드 — engage 가능 조건 / 작동 중 자동 disengage 판정.
 *
 * ★ 출처(유일): repo 루트 `CHCNAV_PARAM_PROFILE.md` §5 + 부록(로그 검증).
 *   - engage 가능: 속도 <= max_open_speed(12km/h) AND 속도 >= vehicle_speed_min(0.7m/s)
 *   - 작동 중 속도 > max_speed(16km/h) → 자동 disengage
 *   - 모터 다중폴트(연속비트) → 즉시 disengage (06-04 08:03 사례)
 *
 * ⚠ 단위 주의: PROFILE §5 가 engage/limit=km/h, 최소작동=m/s 로 혼재 → 내부에서 km/h 통일 비교.
 */
class SafetyGuard(
    private val limits: SafetyLimits,
    val motorFault: MotorFault = MotorFault(),
) {
    /** disengage 사유. */
    enum class Reason { NONE, TOO_FAST, TOO_SLOW, OVER_LIMIT_SPEED, SEVERE_MOTOR_FAULT }

    private fun kmh(mps: Double) = mps * 3.6
    private val minSpeedKmh get() = kmh(limits.vehicleSpeedMinMps)

    /**
     * engage 허용 여부 — PROFILE §5.
     * 조건: vehicle_speed_min <= 속도 <= max_open_speed, 그리고 심각 모터폴트 없음.
     */
    fun canEngage(speedMps: Double): Boolean = engageBlockReason(speedMps) == Reason.NONE

    /** engage 차단 사유(없으면 NONE). */
    fun engageBlockReason(speedMps: Double): Reason {
        val v = kmh(speedMps)
        if (motorFault.isSevere()) return Reason.SEVERE_MOTOR_FAULT
        if (v > limits.maxOpenSpeedKmh) return Reason.TOO_FAST   // >12km/h: engage 불가
        if (v < minSpeedKmh) return Reason.TOO_SLOW              // <0.7m/s: 헤딩/제어 부정확
        return Reason.NONE
    }

    /**
     * 작동 중(engaged) 자동 disengage 사유 판정 — PROFILE §5 + 부록.
     * @return NONE 이면 유지, 그 외면 즉시 disengage.
     */
    fun disengageReason(speedMps: Double): Reason {
        if (motorFault.isSevere()) return Reason.SEVERE_MOTOR_FAULT // 모터 다중폴트 → 즉시 정지
        if (kmh(speedMps) > limits.maxSpeedKmh) return Reason.OVER_LIMIT_SPEED // >16km/h → 자동 disengage
        return Reason.NONE
    }

    /** 작동 중 disengage 가 필요한가. */
    fun shouldDisengage(speedMps: Double): Boolean = disengageReason(speedMps) != Reason.NONE
}
