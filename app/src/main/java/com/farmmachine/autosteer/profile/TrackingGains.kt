package com.farmmachine.autosteer.profile

/**
 * 추적 게인 (sceneGainConfig, 기본 모드 Ag_NX01_default / MODE2_64).
 *
 * ★ 값 출처(유일): repo 루트 `CHCNAV_PARAM_PROFILE.md` §4 (CHCNAV vehicle.db sceneGainConfig).
 * ★ CHCNAV 는 '일반'과 '초저속' 두 게인셋을 유지하며 control 게인(GainSteering)이 다름(40 vs 58).
 *   → [GainMode] 로 분기. heading 게인만 AntennaMode 로 추가 분기(PROFILE §4).
 */

/** 게인 모드 — 속도대역별 게인셋. PROFILE §4 일반/초저속 칼럼. */
enum class GainMode { NORMAL, ULTRA_LOW_SPEED }

/** 한 모드의 게인셋. 단위: raw 게인(차원 없음) / 거리 임계는 m, 진행도는 %. */
data class GainSet(
    val horizontal: Double,       // 횡오차 게인 GainxTrack
    val heading: Double,          // 헤딩 게인 GainxHeading (⚠ AntennaMode 분기)
    val control: Double,          // 제어 게인 GainSteering (일반 40 / 초저속 58)
    val turning: Double,          // 선회 게인
    val maxTurnAngleDeg: Double,  // [deg]
    val onLineProgress: Double,   // [%]
    val entryProgress: Double,    // [%]
    val onlineJudge: Double,      // 온라인 판정 임계 [m]
    val offlineJudge: Double,     // 오프라인 판정 임계 [m]
    val tThreshold: Double,
    val hThreshold: Double,
    val learning: Double,
    /** B적분 스위치 — NORMAL=true, ULTRA_LOW_SPEED=false (PROFILE §4). */
    val isBIntegralSwitch: Boolean,
)

/**
 * 모드별 게인셋 묶음 + 속도 기반 선택 로직.
 * @param mode 이 묶음이 만들어진 안테나 버전(heading 게인 분기 근거).
 */
data class TrackingGains(
    val mode: AntennaMode,
    val normal: GainSet,
    val ultraLowSpeed: GainSet,
) {
    /** 명시적 모드로 게인셋 조회. */
    fun forMode(gainMode: GainMode): GainSet = when (gainMode) {
        GainMode.NORMAL -> normal
        GainMode.ULTRA_LOW_SPEED -> ultraLowSpeed
    }

    /**
     * 속도(m/s)에 따라 게인 모드 선택.
     * ⚠ TODO(현장): 초저속↔일반 전환 임계(ULTRA_LOW_SPEED_THRESHOLD_MPS)는
     *   CHCNAV_PARAM_PROFILE.md 에 명시되지 않음. 아래 상수는 임의값이 아니라
     *   '미확정 표시용 placeholder' — 실차 로그/매뉴얼로 확정 전까지 NORMAL 만 쓰도록
     *   gainModeFor 가 항상 NORMAL 을 반환한다(잘못된 임계로 게인 튀는 것 방지).
     */
    fun gainModeFor(@Suppress("UNUSED_PARAMETER") speedMps: Double): GainMode {
        // TODO(현장): 초저속 전환 임계 확정되면 아래로 교체:
        //   return if (speedMps < ULTRA_LOW_SPEED_THRESHOLD_MPS) GainMode.ULTRA_LOW_SPEED else GainMode.NORMAL
        return GainMode.NORMAL
    }

    companion object {
        /**
         * ⚠ 초저속 전환 임계 — CHCNAV_PARAM_PROFILE.md 에 값 없음(TODO). 실차 확정 필요.
         * 확정 전까지 [gainModeFor] 가 이 값을 사용하지 않음(NORMAL 고정).
         */
        const val ULTRA_LOW_SPEED_THRESHOLD_MPS_TODO = -1.0  // 미확정 sentinel — 절대 비교에 쓰지 말 것

        /**
         * Kubota MR1157 게인 — CHCNAV 실차검증값.
         * 일반/초저속 공통값은 동일, control·onlineJudge·isBIntegralSwitch 만 다름.
         * heading 은 AntennaMode 로 분기(현재 단일/듀얼 모두 100.0, PROFILE §4).
         */
        fun kubotaMR1157(mode: AntennaMode): TrackingGains {
            // heading 게인 버전 분기 — PROFILE §4
            //  단일: 100.0 (CHCNAV 값 그대로)
            //  듀얼: 100.0 에서 시작(헤딩 더 정확 → 동일하게 두거나 필요 시 미세조정)
            val headingGain = when (mode) {
                AntennaMode.SINGLE -> 100.0  // CHCNAV vehicle.db sceneGainConfig.heading (단일안테나), 실차검증
                AntennaMode.DUAL -> 100.0    // PROFILE §4: 듀얼 시작값(=단일과 동일, 필요 시 미세조정)
            }
            val normal = GainSet(
                horizontal = 35.0,       // CHCNAV vehicle.db sceneGainConfig.horizontal (GainxTrack), 실차검증
                heading = headingGain,   // CHCNAV vehicle.db sceneGainConfig.heading (GainxHeading) — AntennaMode 분기
                control = 40.0,          // CHCNAV vehicle.db sceneGainConfig.control (GainSteering, 일반), 실차검증
                turning = 20.0,          // CHCNAV vehicle.db sceneGainConfig.turning
                maxTurnAngleDeg = 25.0,  // CHCNAV vehicle.db sceneGainConfig.maxTurnAngle [deg]
                onLineProgress = 100.0,  // CHCNAV vehicle.db sceneGainConfig.onLineProgress [%]
                entryProgress = 70.0,    // CHCNAV vehicle.db sceneGainConfig.entryProgress [%]
                onlineJudge = 1.0,       // CHCNAV vehicle.db sceneGainConfig.onlineJudge (일반)
                offlineJudge = 2.0,      // CHCNAV vehicle.db sceneGainConfig.offlineJudge
                tThreshold = 0.25,       // CHCNAV vehicle.db sceneGainConfig.tThreshold
                hThreshold = 3.0,        // CHCNAV vehicle.db sceneGainConfig.hThreshold
                learning = 10.0,         // CHCNAV vehicle.db sceneGainConfig.learning
                isBIntegralSwitch = true,// CHCNAV vehicle.db sceneGainConfig.isBIntegralSwitch (일반)
            )
            // 초저속: control·onlineJudge·isBIntegralSwitch 만 다름
            val ultraLowSpeed = normal.copy(
                control = 58.0,           // CHCNAV vehicle.db sceneGainConfig.control (GainSteering, 초저속), 실차검증
                onlineJudge = 1.2,        // CHCNAV vehicle.db sceneGainConfig.onlineJudge (초저속)
                isBIntegralSwitch = false,// CHCNAV vehicle.db sceneGainConfig.isBIntegralSwitch (초저속)
            )
            return TrackingGains(mode = mode, normal = normal, ultraLowSpeed = ultraLowSpeed)
        }
    }
}
