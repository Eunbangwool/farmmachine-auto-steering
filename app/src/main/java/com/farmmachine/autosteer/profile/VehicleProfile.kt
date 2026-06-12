package com.farmmachine.autosteer.profile

/**
 * 차량 파라미터 프로파일 — CHCNAV 역분석 실측값 이식.
 *
 * ★ 값 출처(유일): repo 루트 `CHCNAV_PARAM_PROFILE.md`
 *   = CHCNAV NX510(SW 5.3.0.20260429) 태블릿 ADB 추출 `vehicle.db`(code 63d18282)
 *     + motor_auto_info 로그 8일치(engage 1083회, 에러율 ~1%) + 네이티브 .so 정적분석.
 *   동일 차량(Kubota MR1157) + 동일 Keya 모터 계열 → AGMO 검증된 초기 프로파일로 직접 이식.
 *
 * ★ 단일/듀얼 분기 규칙(CLAUDE_CODE_TASK.md / PROFILE §0,§4,§6):
 *   - 헤딩 게인(gains.heading)과 설치 오프셋(installOffset.headingBias)만 AntennaMode 별로 분기.
 *   - 그 외 기구학/모터/WAS/안전/게인은 두 버전 공통.
 *
 * ⚠ PROFILE.md 에 없는 값(CAN ID, 초저속 전환 임계 등)은 임의 하드코딩 금지 — TODO 주석으로만 남김.
 */

/** AGMO 안테나 구성. PROFILE §0 — 헤딩 소스가 달라 헤딩 게인/바이어스만 분기. */
enum class AntennaMode {
    /** 단일안테나: 헤딩 = 속도벡터+IMU 융합(저속/정지 시 부정확). headingBias 교정 필요. */
    SINGLE,
    /** 듀얼안테나: 듀얼안테나 직접 헤딩(정지 시도 정확). headingBias 보통 0, 초기 정렬 빠름. */
    DUAL
}

/** §1 차량 기구학 (tractorConfig) — 두 버전 공통. 단위: m, deg. */
data class Kinematics(
    val wheelbaseM: Double,                 // 축거 frontBackWheelShaftSpacing [m]
    val receiverHeightM: Double,            // 안테나 높이 지면→안테나 [m]
    val limitSteerAngleDeg: Double,         // 최대 조향각 = max_was [deg]
    val gpsBackWheelShaftSpacingM: Double,  // GPS→후축 [m]
    val gpsCentralAxisSpacingM: Double,     // GPS→중심축 좌우 오프셋 [m]
    val frontSuspensionFrontWheelShaftSpacingM: Double, // [m]
    val driveMode: Int,
    val controllerType: Int,
)

/** §2 모터 제어 (wheelConfig) — Keya 모터(motorType 51). raw 단위(드라이버 내부). */
data class Motor(
    val currentGainP: Int,        // 전류루프 P — HUACE_SetCurrentLoopP
    val currentGainI: Int,        // 전류루프 I — HUACE_SetCurrentLoopI
    val differential: Int,        // D게인 (NX510 매뉴얼 DGain=80 일치)
    val differentialOther: Int,   // 보조 D게인
    val motorRatio: Double,       // 기어비 조향휠↔조향각
    val motorRatioShift: Double,  // 기어비 보정
    val overCurrent: Int,         // 과전류 보호 — HUACE_SetOverloadCurrent
    val overTime: Int,            // 과부하 시간 — HUACE_SetOverloadTime
    val angleDeadDeg: Int,        // 조향각 데드존 [deg]
    val motorDeadCnt: Double,     // 모터 데드존 카운트
    val motorMoment: Int,         // 토크
    val maximumSpeed: Int,        // 모터 최대속도
    val feedBackType51: Int,      // 모터타입51 피드백 — HUACE_SetFeedback_Map
    val motorControlType: Int,
    val softening: Int,           // 완충
)

/** §3 휠 각도 센서 (angleSensorConfig) — 차량 내장 WAS(MACHINE). raw 단위. */
data class WheelAngleSensor(
    val left: Int,        // 좌 끝 [raw]
    val right: Int,       // 우 끝 [raw]
    val middle: Int,      // 중립 [raw]
    val deadArea: Int,    // 센서 데드존 [raw]
) {
    /** 환산 ≈ 440 raw/deg ((right-left)/(2*25°)). PROFILE §3. */
    val rawPerDegree: Double get() = (right - left).toDouble() / 50.0
}

/** §6 설치 오프셋 (installOffsetConfig). headingBias 만 버전 분기. 단위: deg. */
data class InstallOffset(
    val headingBiasDeg: Double,   // ⚠ 단일=교정 필요 / 듀얼=보통 0
    val pitchOffsetDeg: Double,   // IMU 설치 보정(차량 고유) — AGMO IMU 위치 다르면 재측정
    val rollOffsetDeg: Double,    // IMU 설치 보정(차량 고유)
    val wBase: Double,
    /** 단일안테나는 정지 시 헤딩 부정확 → headingBias 교정 절차 필요(PROFILE §6). */
    val needsHeadingCalibration: Boolean,
)

/**
 * 차량 통합 프로파일. gains 는 GainMode 별 세트라 [TrackingGains.forMode] 로 조회.
 * @param antennaMode 이 프로파일이 어느 안테나 버전용인지(헤딩 게인/바이어스 분기 근거).
 */
data class VehicleProfile(
    val antennaMode: AntennaMode,
    val kinematics: Kinematics,
    val motor: Motor,
    val wheelAngleSensor: WheelAngleSensor,
    val gains: TrackingGains,
    val safety: SafetyLimits,
    val installOffset: InstallOffset,
) {
    companion object {
        /**
         * Kubota MR1157 기본 프로파일 — CHCNAV 실차검증값.
         * @param mode SINGLE/DUAL. 헤딩 게인·headingBias 만 달라지고 나머지는 동일.
         */
        fun kubotaMR1157(mode: AntennaMode): VehicleProfile = VehicleProfile(
            antennaMode = mode,
            // §1 기구학 — 공통
            kinematics = Kinematics(
                wheelbaseM = 2.4,                  // CHCNAV vehicle.db tractorConfig.frontBackWheelShaftSpacing, 실차검증
                receiverHeightM = 2.73,            // CHCNAV vehicle.db tractorConfig.receiverHeight, 실차검증
                limitSteerAngleDeg = 25.0,         // CHCNAV vehicle.db tractorConfig.limitSteerAngle, 실차검증
                gpsBackWheelShaftSpacingM = 0.5,   // CHCNAV vehicle.db tractorConfig.gpsBackWheelShaftSpacing, 실차검증
                gpsCentralAxisSpacingM = 0.01,     // CHCNAV vehicle.db tractorConfig.gpsCentralAxisSpacing, 실차검증
                frontSuspensionFrontWheelShaftSpacingM = 1.56, // CHCNAV vehicle.db tractorConfig.frontSuspensionFrontWheelShaftSpacing, 실차검증
                driveMode = 2,                     // CHCNAV vehicle.db tractorConfig.driveMode
                controllerType = 1,                // CHCNAV vehicle.db tractorConfig.controllerType
            ),
            // §2 모터 — 공통 (Keya motorType 51)
            motor = Motor(
                currentGainP = 600,        // CHCNAV vehicle.db wheelConfig.currentGainP (HUACE_SetCurrentLoopP), 실차검증
                currentGainI = 400,        // CHCNAV vehicle.db wheelConfig.currentGainI (HUACE_SetCurrentLoopI), 실차검증
                differential = 80,         // CHCNAV vehicle.db wheelConfig.differential (NX510 매뉴얼 DGain=80 일치), 실차검증
                differentialOther = 60,    // CHCNAV vehicle.db wheelConfig.differentialOther
                motorRatio = 17.5,         // CHCNAV vehicle.db wheelConfig.motorRatio (조향휠↔조향각 기어비), 실차검증
                motorRatioShift = -0.2,    // CHCNAV vehicle.db wheelConfig.motorRatioShift
                overCurrent = 300,         // CHCNAV vehicle.db wheelConfig.motorOverCurrent (HUACE_SetOverloadCurrent), 실차검증
                overTime = 10,             // CHCNAV vehicle.db wheelConfig.motorOverTime (HUACE_SetOverloadTime)
                angleDeadDeg = 2,          // CHCNAV vehicle.db wheelConfig.angleDead [deg]
                motorDeadCnt = 10.0,       // CHCNAV vehicle.db wheelConfig.motorDeadCnt
                motorMoment = 5,           // CHCNAV vehicle.db wheelConfig.motorMoment
                maximumSpeed = 20,         // CHCNAV vehicle.db wheelConfig.maximumSpeed
                feedBackType51 = 9,        // CHCNAV vehicle.db wheelConfig.feedBackType51 (HUACE_SetFeedback_Map)
                motorControlType = 1,      // CHCNAV vehicle.db wheelConfig.motorControlType
                softening = 100,           // CHCNAV vehicle.db wheelConfig.softening
            ),
            // §3 WAS — 공통
            wheelAngleSensor = WheelAngleSensor(
                left = -11000,   // CHCNAV vehicle.db angleSensorConfig.left [raw], 실차검증
                right = 11000,   // CHCNAV vehicle.db angleSensorConfig.right [raw], 실차검증
                middle = 0,      // CHCNAV vehicle.db angleSensorConfig.middle [raw]
                deadArea = 64,   // CHCNAV vehicle.db angleSensorConfig.deadArea [raw]
            ),
            // §4 게인 — heading 만 버전 분기(현재 두 버전 모두 100.0)
            gains = TrackingGains.kubotaMR1157(mode),
            // §5 안전 — 공통
            safety = SafetyLimits.chcnav(),
            // §6 설치 오프셋 — headingBias 만 버전 분기
            installOffset = when (mode) {
                // 단일: headingBias 0.0 에서 시작하되 교정 필요(정지 헤딩 부정확)
                AntennaMode.SINGLE -> InstallOffset(
                    headingBiasDeg = 0.0,        // CHCNAV vehicle.db installOffsetConfig.headingBias (단일=교정 필요)
                    pitchOffsetDeg = -0.33,      // CHCNAV vehicle.db installOffsetConfig.pitchOffset (IMU 설치 보정, 차량 고유)
                    rollOffsetDeg = -0.25,       // CHCNAV vehicle.db installOffsetConfig.rollOffset (IMU 설치 보정, 차량 고유)
                    wBase = 0.0,                 // CHCNAV vehicle.db installOffsetConfig.wBase
                    needsHeadingCalibration = true,
                )
                // 듀얼: 듀얼안테나 직접 헤딩 → headingBias 보통 0, 교정 불필요/최소
                AntennaMode.DUAL -> InstallOffset(
                    headingBiasDeg = 0.0,        // PROFILE §6: 듀얼=보통 0(교정 불필요)
                    pitchOffsetDeg = -0.33,      // CHCNAV vehicle.db installOffsetConfig.pitchOffset (IMU 설치 보정, 차량 고유)
                    rollOffsetDeg = -0.25,       // CHCNAV vehicle.db installOffsetConfig.rollOffset (IMU 설치 보정, 차량 고유)
                    wBase = 0.0,                 // CHCNAV vehicle.db installOffsetConfig.wBase
                    needsHeadingCalibration = false,
                )
            },
        )
    }
}
