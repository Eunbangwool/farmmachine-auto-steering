# CHCNAV 역분석 기반 파라미터 프로파일 — Kubota MR1157

> **출처**: CHCNAV NX510(SW 5.3.0.20260429) 태블릿에서 ADB로 추출한 실측 데이터
> - `vehicle.db` (가동 차량 프로파일, code `63d18282`)
> - `motor_auto_info` 로그 8일치 (2026-05-23 ~ 06-12, engage 1083회, 에러율 ~1%)
> - 네이티브 `.so` 정적 분석 (libmotor / libcontroller / libGNSSBladeControl 등)
>
> **목적**: AGMO 자동조향 시스템의 **검증된 초기 파라미터 프로파일** 확보.
> CHCNAV 세팅은 **단일안테나(doubleAntenna=0)** 구성 → AGMO 단일안테나 버전에 직접 이식 가능.
>
> **검증 상태**: 아래 값들로 실차에서 하루 100~170회 engage, 수십 시간 가동, 에러율 1%. 탁상값 아님.

---

## 0. AGMO 버전 구분 (중요)

AGMO는 **단일안테나 / 듀얼안테나 두 버전**이 존재. CHCNAV 추출 세팅은 단일안테나이므로:

| 구분 | 단일안테나 버전 | 듀얼안테나 버전 |
|------|----------------|----------------|
| 헤딩 소스 | 속도벡터 + IMU 융합 (저속/정지 시 부정확) | 듀얼안테나 직접 헤딩 (정지 시도 정확) |
| CHCNAV 세팅 이식성 | **그대로 적용 가능** | 차량/모터 파라미터는 동일, 헤딩 관련만 재조정 |
| headingBias 보정 | 필요 | 불필요 또는 최소 |
| 초기 정렬(line acquisition) | 느림 (헤딩 수렴 대기) | 빠름 |

**아래 모든 값은 두 버전 공통 적용**. 단, §4(헤딩 게인)와 §6(헤딩 보정)만 버전별 분기.

---

## 1. 차량 기구학 (tractorConfig) — 두 버전 공통

| 파라미터 | 값 | 단위 | 비고 |
|----------|-----|------|------|
| wheelbase (축거) | **2.4** | m | frontBackWheelShaftSpacing |
| receiverHeight (안테나 높이) | **2.73** | m | 지면→안테나 |
| limitSteerAngle (최대 조향각) | **25** | ° | = max_was |
| gpsBackWheelShaftSpacing | **0.5** | m | GPS→후축 |
| gpsCentralAxisSpacing | **0.01** | m | GPS→중심축 (좌우 오프셋) |
| frontSuspensionFrontWheelShaftSpacing | **1.56** | m | |
| driveMode | 2 | - | |
| controllerType | 1 | - | |

---

## 2. 모터 제어 (wheelConfig) — Keya 모터(motorType 51) 기준

> AGMO Keya KY170C와 동일 계열. `.so`의 `RQ_Motor_*` / `HUACE_*` 함수군이 이 값을 사용.

| 파라미터 | 값 | 매핑 (so 함수) | 비고 |
|----------|-----|----------------|------|
| **currentGainP** | **600** | HUACE_SetCurrentLoopP | 전류루프 P |
| **currentGainI** | **400** | HUACE_SetCurrentLoopI | 전류루프 I |
| **differential** (D게인) | **80** | - | NX510 매뉴얼 DGain=80과 일치 |
| differentialOther | 60 | - | 보조 |
| **motorRatio** (기어비) | **17.5** | - | 조향휠↔조향각 |
| motorRatioShift | -0.2 | - | 기어비 보정 |
| **motorOverCurrent** | **300** | HUACE_SetOverloadCurrent | 과전류 보호 |
| motorOverTime | 10 | HUACE_SetOverloadTime | 과부하 시간 |
| angleDead (조향각 데드존) | 2 | - | |
| motorDeadCnt | 10.0 | - | 모터 데드존 카운트 |
| motorMoment (토크) | 5 | - | |
| maximumSpeed | 20 | - | 모터 최대속도 |
| feedBackType51 | 9 | HUACE_SetFeedback_Map | 모터타입51 피드백 |
| motorControlType | 1 | - | |
| softening | 100 | - | 완충 |

**통신**: 모터는 CANopen (CiA 301/402). SDO+PDO+NMT. CAN 2채널(`send_can_message_C` / `send_can_message_CAN2`).

---

## 3. 휠 각도 센서 (angleSensorConfig) — WAS 캘리브레이션

> `libWheelAngle.so`가 사용. AGMO가 동일 차량 내장 WAS 사용 시 그대로 참고.

| 파라미터 | 값 | 비고 |
|----------|-----|------|
| angleSensorType | MACHINE | 차량 내장 WAS |
| **left** (좌 끝) | **-11000** | raw |
| **right** (우 끝) | **+11000** | raw |
| **middle** (중립) | **0** | raw |
| deadArea | 64 | 센서 데드존 |
| **환산** | **≈ 440 raw / degree** | (11000 / 25°) |

---

## 4. 추적 게인 (sceneGainConfig, 기본 모드 Ag_NX01_default / MODE2_64)

| 파라미터 | 일반 모드 | 초저속 모드 | 버전 적용 |
|----------|-----------|-------------|-----------|
| **horizontal** (횡오차 게인, GainxTrack) | **35.0** | 35.0 | 공통 |
| **heading** (헤딩 게인, GainxHeading) | **100.0** | 100.0 | ⚠️ 버전별 (아래) |
| **control** (제어 게인, GainSteering) | **40.0** | **58.0** | 공통 |
| turning (선회 게인) | 20.0 | 20.0 | 공통 |
| maxTurnAngle | 25.0 | 25.0 | 공통 |
| onLineProgress | 100.0 | 100.0 | 공통 |
| entryProgress | 70.0 | 70.0 | 공통 |
| onlineJudge | 1.0 | 1.2 | 공통 |
| offlineJudge | 2.0 | 2.0 | 공통 |
| tThreshold | 0.25 | 0.25 | 공통 |
| hThreshold | 3.0 | 3.0 | 공통 |
| learning | 10.0 | 10.0 | 공통 |
| isBIntegralSwitch | true | false | 공통 |

**버전별 heading 게인:**
- **단일안테나**: heading = **100.0** (CHCNAV 값 그대로)
- **듀얼안테나**: heading = **100.0에서 시작**, 헤딩이 더 정확하므로 동일하게 두거나 필요 시 미세 조정. 듀얼은 정지 시도 헤딩 유효 → 초기 정렬이 빠름.

---

## 5. 안전 / 속도 제한 (safely_config + nav_setting) — 두 버전 공통

| 파라미터 | 값 | 단위 | 비고 |
|----------|-----|------|------|
| **max_open_speed** (engage 가능 최대) | **12.0** | km/h | 매뉴얼 일치 |
| **max_speed** (작동 한계) | **16.0** | km/h | 초과 시 disengage |
| max_u_turn_speed | 8.0 | km/h | |
| **vehicle_speed_min** (최소 작동) | **0.7** | m/s | ≈ 2.5 km/h |
| motor_moment | 5 | - | |
| bt_motor_automatic_enable | 1 | - | 모터 자동 enable |

---

## 6. 설치 오프셋 (installOffsetConfig)

| 파라미터 | 값 | 단위 | 버전 적용 |
|----------|-----|------|-----------|
| headingBias | 0.0 | ° | ⚠️ 단일=교정 필요 / 듀얼=보통 0 |
| pitchOffset | -0.33 | ° | IMU 설치 보정 (차량 고유) |
| rollOffset | -0.25 | ° | IMU 설치 보정 (차량 고유) |
| wBase | 0.0 | - | |

> pitch/roll offset은 IMU 설치 각도 보정값. AGMO IMU 장착 위치가 다르면 재측정 필요. 단일안테나는 정지 시 헤딩이 부정확하므로 headingBias 교정 절차가 중요.

---

## 7. 시리얼 포트 매핑 (serial_port_info) — 참고

| 포트 | Baud | 용도 |
|------|------|------|
| /dev/ttyS6 | 115200 | GNSS 메인 (위성설정, 차분보정, GGA, 내부라디오) |
| /dev/ttyS4 | 115200 | 외부라디오 + NMEA 출력 |

> CHCNAV 하드웨어 기준. AGMO는 자체 안테나/모듈 사용하므로 포트 구성은 다를 수 있음. **모터는 시리얼이 아니라 CAN.**

---

## 8. 레이저 레벨러 (libGNSSBladeControl) — 정적 분석 결과

> 실측 불가(하드웨어 없음, `/sdcard/ControllerX/` 미생성). 아래는 `.so` 디컴파일로 복원한 로직.

**출력 경로**: `Send_pwm` → 구조체에 PWM 기록 → libAlgorithmProc가 분기:
`send_can_message_C/CAN2` (CAN) 또는 `sendDirectOutCtl` (GPIO 직접출력)

**제어 로직 (UP/DOWN/HOLD):**
- 차속 < 최소속도 → 무조건 HOLD
- up_flag/down_flag 둘 다 0 (오차 데드존 내) → HOLD
- UP = up_flag + PWM_Up 듀티 / DOWN = down_flag + PWM_Down 듀티

**복원된 상수:**
- PWM 듀티 범위: **10% (최소 작동) ~ 96% (상한)**
- 고도 오차 데드존: 다단계 (0.0001 ~ 0.07 m, mm급 정밀)
- 비례밸브 캘리브레이션: 4개 파라미터 (중립전압/최대전압/비례계수/오프셋)
- 캘리브레이션 파일: `/sdcard/ControllerX/IC100Paras.txt` (`%.3lf` 4개 값)
- 신호 필터: 칼만 + 이동평균
- 제어기: PID (조향의 MPC/SMC와 별개)

**MR1157 적용 시**: factory 레이저 레벨러 연동은 하드웨어(커넥터/컨트롤러) 확보 후 진행. (7) 커넥터가 GPIO 입력이면 `sendDirectOutCtl` 경로와 대응.

---

## 부록: 로그 검증 요약 (motor_auto_info)

- 기간 2026-05-23 ~ 06-12, 고유 이벤트 2170건, engage 1083회
- 에러(-6) 11회 = **에러율 ~1%**, 셋업일(05-24)·종료일(06-04)에 집중
- 한창 가동된 05-30/06-01/06-02/06-03은 **에러 0건**
- 06-04 08:03 심각 에러: col11=520(bit3+9), col12=0x3f00000000(bit32~37 연속) = 모터 드라이버 다중 폴트
- **결론**: §2~§5 파라미터가 실전에서 안정 동작 검증됨
