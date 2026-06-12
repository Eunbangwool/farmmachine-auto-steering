# CONTEXT — 적용 내역 / 결정 이력

## CHCNAV 역분석 파라미터 프로파일 적용 (2026-06-12)

CHCNAV NX510 상용 시스템(SW 5.3.0.20260429)을 ADB 역분석해 확보한 **실차검증** 자동조향
파라미터를 AGMO 초기 프로파일로 이식했다. 동일 차량(Kubota MR1157) + 동일 Keya 모터 계열.

- **값 출처(유일)**: `CHCNAV_PARAM_PROFILE.md` (repo 루트). vehicle.db(code 63d18282) +
  motor_auto_info 로그 8일치(engage 1083회, 에러율 ~1%) + 네이티브 .so 정적분석.
- **작업 지시서**: `CLAUDE_CODE_TASK.md` (repo 루트).
- **단일/듀얼 분기**: 헤딩 게인(`gains.heading`)과 `installOffset.headingBias` 만 `AntennaMode`
  별로 분기. 기구학/모터/WAS/안전/그 외 게인은 두 버전 공통.

### 추가된 Kotlin (모든 수치에 출처 주석)
- `app/.../profile/VehicleProfile.kt` — 작업 1. `VehicleProfile`/`AntennaMode`(SINGLE/DUAL)/
  `Kinematics`/`Motor`/`WheelAngleSensor`/`InstallOffset` + 팩토리 `kubotaMR1157(mode)` (§1~§3,§6).
- `app/.../profile/TrackingGains.kt` — 작업 2. `GainMode`(NORMAL/ULTRA_LOW_SPEED) + `GainSet` +
  속도기반 `gainModeFor`. control 게인 40/58, isBIntegralSwitch true/false 분기 (§4).
- `app/.../profile/SafetyLimits.kt` — §5 한계값(공통).
- `app/.../safety/SafetyGuard.kt` + `MotorFault.kt` — 작업 3. engage(≤12km/h & ≥0.7m/s)/
  작동중 >16km/h 자동 disengage / 모터 다중폴트(연속비트) 즉시 disengage (§5 + 부록).
- `app/.../can/CanOpenMotor.kt` — 작업 4. CANopen(CiA 301/402) SDO/PDO/NMT + 모터제어
  인터페이스 **스캐폴드만**. 실제 CAN ID/객체사전은 미확정 → TODO(추측 프레임 송신 금지).

### 미확정(TODO로 남김 — 임의 하드코딩 안 함)
- **초저속↔일반 게인 전환 임계**: PROFILE.md 에 값 없음 → `gainModeFor` 가 확정 전까지 NORMAL 고정.
- **모터 CAN ID / 객체사전 인덱스 / 바이트맵**: 하드웨어 캡처 필요 → `CanOpenMotorScaffold` 호출 시
  `NotImplementedError`(추측 송신 차단).
- **모터 폴트 개별 비트 의미**: 매핑 없음 → '연속 다중비트' 구조적 판정만(`MotorFault.isSevere`).
- **레이저 레벨러(§8)**: 이번 범위 밖(하드웨어 미확보) — 코드 미작성.

### 빌드 확인
로컬은 네트워크 정책상 Android Gradle Plugin(8.6.1) 다운로드 차단으로 풀빌드 불가 →
CI(`.github/workflows/build-autosteer-apk.yml`)에서 검증. 새 파일은 android.* 의존 없는
순수 Kotlin(데이터 클래스/enum/interface).

---

## 결정 이력

| 날짜 | 결정 | 출처 |
|------|------|------|
| 2026-06-12 | CHCNAV 역분석 파라미터 프로파일 적용(작업 1~4), 단일/듀얼 헤딩만 분기 | CHCNAV_PARAM_PROFILE.md / CLAUDE_CODE_TASK.md |
