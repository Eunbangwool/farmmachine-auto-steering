package com.farmmachine.autosteer.profile

/**
 * 안전 / 속도 제한 (safely_config + nav_setting) — 두 버전 공통.
 *
 * ★ 값 출처(유일): repo 루트 `CHCNAV_PARAM_PROFILE.md` §5.
 * ⚠ 단위 혼재(원문 그대로 보존): engage/limit 은 km/h, 최소작동은 m/s.
 */
data class SafetyLimits(
    val maxOpenSpeedKmh: Double,   // engage 가능 최대 [km/h] (매뉴얼 일치)
    val maxSpeedKmh: Double,       // 작동 한계 [km/h] (초과 시 disengage)
    val maxUTurnSpeedKmh: Double,  // 유턴 최대 [km/h]
    val vehicleSpeedMinMps: Double,// 최소 작동 [m/s] (≈2.5km/h)
    val motorMoment: Int,          // 토크
    val btMotorAutomaticEnable: Int, // 모터 자동 enable
) {
    companion object {
        fun chcnav(): SafetyLimits = SafetyLimits(
            maxOpenSpeedKmh = 12.0,     // CHCNAV safely_config.max_open_speed [km/h], 실차검증
            maxSpeedKmh = 16.0,         // CHCNAV safely_config.max_speed [km/h], 실차검증
            maxUTurnSpeedKmh = 8.0,     // CHCNAV nav_setting.max_u_turn_speed [km/h]
            vehicleSpeedMinMps = 0.7,   // CHCNAV nav_setting.vehicle_speed_min [m/s], 실차검증
            motorMoment = 5,            // CHCNAV safely_config.motor_moment
            btMotorAutomaticEnable = 1, // CHCNAV safely_config.bt_motor_automatic_enable
        )
    }
}
