# Claude Code 작업 지시서 — CHCNAV 파라미터 프로파일 적용

> 이 파일과 `CHCNAV_PARAM_PROFILE.md`를 함께 repo에 올린 뒤, Claude Code에 아래 작업을 지시하세요.
> 대상 repo: `Eunbangwool/farmmachine-auto-steering`

---

## 배경 (Claude Code가 알아야 할 컨텍스트)

CHCNAV NX510 상용 시스템을 ADB로 역분석해서 실차 검증된 자동조향 파라미터를 확보했다.
같은 차량(Kubota MR1157) + 같은 Keya 모터 계열이므로 이 값을 AGMO 초기 프로파일로 이식한다.
전체 값과 근거는 `CHCNAV_PARAM_PROFILE.md` 참조.

**AGMO는 단일안테나 / 듀얼안테나 두 버전이 있다.** 코드는 두 버전을 분기 처리해야 한다.
CHCNAV 추출값은 단일안테나 세팅이라 단일 버전에 직접 적용, 듀얼은 헤딩 관련만 분리.

---

## 작업 1: 차량 파라미터 프로파일 데이터 클래스 생성

`CHCNAV_PARAM_PROFILE.md`의 §1~§6 값을 Kotlin 데이터 클래스로 구현.

요구사항:
- `VehicleProfile` 데이터 클래스 (kinematics, motor, wheelAngleSensor, gains, safety, installOffset 그룹)
- `AntennaMode` enum (SINGLE, DUAL)
- `gains.heading`, `installOffset.headingBias`는 AntennaMode에 따라 분기
- 단일/듀얼 공통값은 동일하게, 버전별 값만 분리
- MR1157 기본 프로파일을 팩토리 함수로 제공: `VehicleProfile.kubotaMR1157(mode: AntennaMode)`
- 모든 수치에 출처 주석 (예: `// CHCNAV vehicle.db wheelConfig.currentGainP, 실차검증`)
- 단위를 주석 또는 타입으로 명시 (m, deg, km/h, raw, m/s)

핵심 값 (CHCNAV_PARAM_PROFILE.md에서 가져옴):
- 기구학: wheelbase=2.4m, antennaHeight=2.73m, maxSteerAngle=25deg, gpsBackShaft=0.5m
- 모터: currentGainP=600, currentGainI=400, differential=80, motorRatio=17.5, overCurrent=300, angleDead=2
- WAS: left=-11000, right=11000, middle=0, deadArea=64 (≈440 raw/deg)
- 게인(일반): horizontal=35.0, heading=100.0, control=40.0, turning=20.0
- 게인(초저속): control=58.0 (나머지 동일)
- 안전: maxEngageSpeed=12.0km/h, maxSpeed=16.0km/h, minSpeed=0.7m/s

## 작업 2: 게인 모드 분리 (일반 / 초저속)

CHCNAV는 일반 모드와 초저속 모드의 control 게인이 다름 (40 vs 58).
- `GainMode` enum (NORMAL, ULTRA_LOW_SPEED)
- 속도에 따라 게인셋을 전환하는 로직 (초저속 임계는 일단 상수로 빼두고 TODO 주석)
- isBIntegralSwitch: NORMAL=true, ULTRA_LOW_SPEED=false

## 작업 3: 안전 가드 로직

`CHCNAV_PARAM_PROFILE.md` §5 + 부록(로그 검증) 기반:
- engage 가능 조건: 속도 <= 12km/h AND 속도 >= minSpeed(0.7m/s)
- 작동 중 속도 > 16km/h → 자동 disengage
- 모터 폴트 비트필드 처리: 연속 다중비트(예: col12 bit32~37 동시) = 심각 폴트 → 즉시 disengage
  (CHCNAV 06-04 08:03 에러 사례 참조)
- 폴트 상태를 비트필드로 관리 (`.so`의 HUACE_CheckIfFault / GetErrorCode / ClearError 패턴)

## 작업 4: CANopen 모터 통신 스캐폴드 (구조만)

`.so` 정적 분석 결과 기반 인터페이스 정의 (실제 CAN ID는 하드웨어 확인 필요 → TODO):
- CANopen CiA 301/402 기반, CAN 2채널
- SDO 읽기/쓰기, PDO 등록, NMT 상태 제어 인터페이스
- 모터 제어: Enable/Disable, SetTargetTorque, SpeedModeMove, GetCurrentSpeed/Position
- 함수명은 `.so` 심볼 참고 (SetControlWord, SetTargetTorque, Motor_Enable 등)
- **주의**: 실제 CAN ID/바이트 구성은 미확정. 인터페이스와 TODO 주석만. 추측값 하드코딩 금지.

---

## 제약 / 주의사항

- **추측 금지**: CHCNAV_PARAM_PROFILE.md에 있는 값만 사용. 없는 값(예: CAN ID, 초저속 전환 임계)은 TODO 주석으로 남기고 임의값 하드코딩 하지 말 것.
- **단일/듀얼 분기**: 헤딩 게인과 headingBias만 버전 분기. 나머지는 공통.
- **레이저 레벨러는 이번 작업 범위 밖**: 하드웨어 미확보 상태. 프로파일 문서 §8은 참고용으로만 둠.
- 각 파일/클래스에 출처를 주석으로 남겨 향후 추적 가능하게.
- 작업 완료 후 `CONTEXT.md`(또는 `CLAUDE.md`)에 "CHCNAV 역분석 파라미터 적용됨, 출처 CHCNAV_PARAM_PROFILE.md" 기록하고 push.

## 검증 체크리스트 (작업 후 확인)

- [ ] VehicleProfile.kubotaMR1157(SINGLE) / (DUAL) 둘 다 생성 가능한가
- [ ] 단일/듀얼에서 heading 게인, headingBias만 다르고 나머지 동일한가
- [ ] 모든 수치에 출처 주석이 있는가
- [ ] CAN ID 같은 미확정값을 임의로 하드코딩하지 않았는가 (TODO로 남겼는가)
- [ ] 빌드되는가
