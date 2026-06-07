# farmmachine-auto-steering — CLAUDE.md
> Claude Code 세션 컨텍스트. 이 파일을 읽으면 이전 설계 결정을 전부 파악할 수 있다.

---

## 프로젝트 오너

- GitHub: Eunbangwool
- 배경: AGMO 자율주행 전직, 현재 농가 운영
- 보유 장비: Kubota MR1157H (115HP), Apollo 10 Pro (Android, CAN 내장), F9P RTK, L1/L2 안테나, LoRa 모듈

---

## 핵심 아키텍처 결정 (변경 금지)

### 자율조향 시스템 구성
```
[Apollo 10 Pro 태블릿]  ← 본인 앱 실행 (AgNav 대체)
    ├─ CAN → 조향 모터  (직접 제어)
    ├─ CAN ← 앵글센서   (조향각 피드백)
    ├─ USB → F9P       (RTK GNSS)
    └─ USB → LoRa      (NTRIP 보정신호)
```

**배경**: NX510(CHCNAV 자율조향)이 이미 설치되어 있으나, AgNav 앱을 본인 앱으로 교체하는 구조. 모터 회사와 모터 프로그램 보유. CAN 프로토콜은 AGMO 경유 입수 예정.

### Monorepo 구조
```
farmmachine-auto-steering/
├── auto-steering/          ← ★ 핵심 알고리즘 (Python)
│   └── src/*.py            # autosteer_core, apollo_can, app_main, sitl_sim …
├── app/                    ← ★ Android 앱 (Kotlin 셸 + Chaquopy 임베드 Python)
│   └── src/main/java/com/farmmachine/autosteer/
├── settings/build.gradle.kts, gradle/  ← Gradle 8.9 / AGP 8.6 / Chaquopy 16
├── ANDROID_APP.md          ← 앱 아키텍처/빌드, APOLLO_CAN.md ← CAN 브릿지 계약
├── rtk-leveling/ leveefollow/ apk-analysis/
```
**앱 = farm-work-manager(농작이)와 별개.** `com.farmmachine.autosteer`.
Kotlin(UI/서비스/CAN 브릿지) + Chaquopy 로 `auto-steering/src` Python 그대로 실행.
진입점 `app_main.py`(CPython 검증됨). CI: `.github/workflows/build-autosteer-apk.yml`.

---

## autosteer_core.py — 4계층 설계

### Layer 1: 경로 정의
- `Waypoint(x, y, speed, implement_down, section)`
- `ABLineStrategy`: 직선 평행 패스 + 헤드랜드 회전 (베지에)
- `ContourStrategy`: 등고선 기반 법선 오프셋 평행 경로
- `CustomStrategy`: 임의 웨이포인트

### Layer 2: 상태 추정
- `StateEstimator`: 확장 칼만 필터 (RTK 10Hz + IMU 100Hz 융합). `tune_for_receiver(spec)`로 INS heading 정확도 기반 R 튜닝
- `TractorParams` 자동 적용: update_rtk에서 파라미터 2(높이)+3(레버암) 보정, update_imu에서 파라미터 5(IMU 오프셋) 보정
- `GnssArbiter`: ✅ GNSS 소스 이중화 — **PA-3(주) + F9P(백업)** 우선순위/페일오버. `on_rtk(...,source=)`로 제출, active 소스 한 곳만 EKF 반영(이중카운팅 방지), PA-3 끊기면 F9P 자동 전환·복구 시 복귀

### Layer 3: 경로 추종 (3가지 알고리즘)
```
PurePursuit      — 로터리/방제 등 일반 작업
Stanley          — 직선 정밀 중요한 작업
ImplementReferenced  — 쟁기/균평 등 작업기 부하 큰 작업 ★
```

**ImplementReferenced가 핵심**: AGMO S자 진동 해결. CHCNAV "경로 인식 참조점: 차량 후면"과 동일 원리.
- 오차 계산 기준 = 안테나가 아닌 **작업기 위치**
- 수식: `δ = k_heading×h_err - atan2(k×e_impl, v+ks) - k_pred×e_pred + Ackermann보정`

### Layer 4: CAN 모터 제어
- `SteeringActuator`: 위치 P → 각속도 PI + 마찰 FF 이중 루프
- `ApolloCanInterface`: ✅ SocketCAN 구현 (python-can 우선 → raw socket 폴백, 하드웨어 없으면 available=False)
- `apollo_can.ApolloCanBus`: ✅ 백엔드 교체형(**bridge★/socketcan/slcan/mock**) + 연결감시·자동재연결. Apollo는 **Kotlin 브릿지**(localhost TCP 13B 레코드)가 주 경로. 조향+레벨러 같은 버스 공유. 계약: `APOLLO_CAN.md`
- `MockCanInterface`: 테스트용 (즉시 사용 가능)
- `ImuCalibrator`: 평지 30초 평균 → `ImuOffset` 생성 (파라미터 5 캘리브레이션)
- `f9p_client.py / F9pUsbClient`: F9P NMEA(GGA) 파싱 → `on_rtk(lat,lon,quality)` 콜백

---

## TractorParams — Kubota MR1157 (사진 확인값)

```python
KUBOTA_MR1157 = TractorParams(
    wheelbase         = 2.47,   # ★ 실측 필요 (AGMO 파라미터 1)
    antenna_height    = 2.73,   # 사진 E 확인값 (AGMO 파라미터 2)
    antenna_to_axle   = -0.50,  # 사진 D ≈0.5m, 뒤쪽=음수 (★ 실측 필요, AGMO 파라미터 3)
    antenna_to_impl   = 1.20,   # ★ 실측 필요 (AGMO 파라미터 4)
    hitch_to_impl     = 1.00,   # 사진 B1 확인값 (CHCNAV 추가)
    front_track_width = 1.56,   # 사진 G 확인값 (CHCNAV 추가, Ackermann 보정)
    max_was_deg       = 25.0,   # 사진 최대 WAS 확인값
    imu_offset = ImuOffset(roll=0.0, pitch=0.0, yaw=0.0),  # ★ 캘리브레이션 후 채움
)
```

**★ 지금 당장 실측해야 할 것:**
- `wheelbase`: 앞차축 중심 ↔ 뒷차축 중심 (줄자)
- `antenna_to_axle`: 뒷차축 ↔ 안테나 전후 거리 (뒤쪽이면 음수)

---

## CanSpec — ✅ 실모터 프로토콜 확정 (Keya KY170DD01005-08G, 매뉴얼 V2.4)

> 별도 세션에서 실제 모터를 특정하고 매뉴얼 기반 CAN 프로토콜을 코드에 반영함(이식 완료).

```python
class CanSpec:
    CAN_BITRATE   = 250_000        # 매뉴얼 확정 (parameter 0021=2, 공장기본). PA-3 500k 와 다름!
    # 29-bit Extended ID (motor_id=1): TX=0x06000001 RX=0x05800001 HB=0x07000001
    CMD_ENABLE  = 23 0D 20 01 00 00 00 00      # 속도제어 SDO
    CMD_SPEED   = 23 00 20 01 [value 4B]       # cmd_speed(±1000‰ = ±80RPM), 워치독 1000ms
    CMD_DISABLE = 23 0C 20 01 00 00 00 00
    # parse_heartbeat(20ms): 누적각/속도RPM/전류/폴트코드(_parse_fault, 매뉴얼 p.23)
    SENSOR_ANGLE_ID = 0x301        # WAS — ★AGMO 미사용(아래 참고). CHCNAV/FJD 장착 시만 캡처
```

**⚠ 비트레이트 충돌 주의**: 모터=250k(매뉴얼), PA-3=500k(데이터시트). 같은 버스 공유 불가 →
별도 버스이거나 한쪽 재설정 필요. 현장 확인 항목.
**✅ 무WAS 속도제어 조향 구현 완료(시뮬 검증)**: `SteeringActuator.speed_control=True` 시
조향각오차→목표각속도(P)→모터RPM(조향비17.5)→`cmd_speed`. 피드백=모터 하트비트
누적각(16비트 wrap 언랩) — `use_motor_encoder`. 안전가드: **하트비트 미수신 시 명령 금지(폭주 방지)**.
`app_main.set_vendor` 가 실차(bridge)+확정벤더일 때만 활성(데모는 byte-layout 유지).
검증: `test_speed_control.py`(좌/우 수렴·부호·안전가드), SITL 6/6 무손상.
**모터 부호 규약(현장 확정): `+permille = 좌회전`, `−permille = 우회전`.**
★ 실차 현황: RX 하트비트 **수신 0 확정**(필터 전체수용도 무효 → 모터가 CAN 피드백 미송신,
  Keya 하트비트 주기 param 0034 꺼짐 추정 or .so RX 미제공). → **하트비트 비의존 설계로 전환**:
  하트비트 없으면 **명령 permille 적분으로 조향각 추정(dead-reckoning, ±60° 클램프)**, 실제 경로는
  GNSS 헤딩 외부루프가 보정. 하트비트 들어오면 자동으로 실측 사용. (test_speed_control: 有/無 둘 다 수렴)
  남은 것: 직진 캘리브레이션 + 게인 튜닝(안테나 단계) + (원하면) Keya 하트비트 활성화.

**AgNav 5.0 사진에서 확인된 모터 관련 값:**
- 모터 피드백 유형: **홀(Hall) 센서**
- 모터 비례 이득(P Gain): 600 → `SteeringActuator.pos_kp` 기준
- 모터 필수 Gain: 400 → `vel_kp` 기준
- 최대 과부하 전류: 300 → `SafetyMonitor` 연동 필요
- 조향비: 17.5
- 핸들 데드존: 20
- 스티어링 데드존 오프셋: 0
- 제어 모드: Mode2 (P=25, D=80, 최대RPM=20, 연성=100)

---

## 앵글센서(WAS) 정책 — ★ 벤더별 상이 (오너 확인)

- **AGMO**: 앵글센서 **미사용** 알고리즘. 조향각 피드백 = **Keya 모터 하트비트 누적각**
  (Hall 인코더, parse_heartbeat 의 angle_raw). 별도 WAS 불필요 → **WAS CAN ID 캡처도 불필요.**
- **CHCNAV / FJD**: WAS 장착 **선택 가능하나 없어도 자동조향 가능**. 기본은 미장착(`uses_was=False`).
- 코드: `vendor_profiles.VendorProfile.uses_was`. `SteeringActuator` 의 각도 피드백 소스는
  WAS(SENSOR_ANGLE_ID) 대신 **모터 하트비트 각**을 쓰도록 전환 필요(★ 실차 autosteer 단계 작업).
- SafetyMonitor 의 "운전자 개입(앵글센서 급변)" 감지도 WAS 없으면 모터 인코더/토크 기반으로 대체.

---

## 헤딩(자세) 소스 — ★ 두 아키텍처 모두 지원 (오너 확인)

> 자율조향엔 위치뿐 아니라 **헤딩(차량 방향)+roll/pitch**가 필수(모든 추종 알고리즘이 `state.heading` 사용).
> 모터 하트비트(조향각)가 없어도 **헤딩만 있으면** GNSS로 폐루프가 닫힌다 → 헤딩 소스가 핵심 피드백.

- **ver1 (듀얼안테나 + IMU)**: heading=두 안테나 baseline, 각속도/자세=별도 IMU. `heading_source="dual"`
- **ver2 / CHCNAV NX510 / FJD (GNSS+INS 스마트안테나)**: 수신기가 heading/자세 융합 출력. `heading_source="ins"`
- 코드: `GnssReceiverSpec.heading_source`("ins"/"dual"/"none") + `VendorProfile.gnss_alt`(AGMO ver1/ver2 둘 다 등록).
  AGMO 프로파일: primary=ver2(INS), alt=ver1(dual). CHCNAV/FJD=ins. F9P 단독=none.
- **공통 경로(✅ 배선·검증)**: HDT(나침반 진헤딩) → `f9p_client.parse_hdt`/`on_heading` →
  `AutoSteerSystem.on_heading`(나침반→수학각 90-θ 변환) → `StateEstimator.update_heading`.
  GGA 위치 → `on_rtk`. IMU 없으면 `control_step` 이 EKF predict 수행(_imu_fed). 
  app_main: `start_gnss(port,baud)`(F9P/PA-3 시리얼) + 모듈 `on_heading()`(Kotlin 푸시용).
  검증: `test_closed_loop.py`(나침반↔수학각 0° 오차·북진 추종). 조향 수렴은 sitl_sim 6/6.
  ★ 실차: Apollo USB-serial 접근경로 확인(필요시 Kotlin USB-serial 브릿지). roll/pitch proprietary 추후.
- F9P 단독(none)은 헤딩 소스 없음 → 듀얼/IMU 필요.
- **ver1 융합·캘리브(✅ 구현·검증)**:
  - **듀얼+IMU 융합**: `on_heading`(듀얼 절대) + `on_gyro`(IMU yaw rate, 절대heading 미사용)
    → EKF predict 가 고레이트 평활 → 스네이크/지연 억제(테스트 ~1.4× 노이즈↓, 추가 평활은 Q튜닝).
  - **헤딩 바이어스 캘리브**(중심 치우침 해결): `calibration.HeadingCalibrator` — 직선 ~20m 주행 중
    (보고heading vs GPS 진로각) 원형평균 = 베이스라인 yaw 바이어스 → `set_heading_bias`/on_heading 보정.
    app_main: `start_heading_calib()` 후 직선 주행하면 자동 산출·적용. (테스트: +5° 복원 R=1.0)
  - **PA-3급 헤딩 업그레이드(✅ 방법 1·3·4·5, 자이로바이어스 6상태=방법2는 제외)**: 듀얼안테나 ver1 을
    단일안테나 INS(PA-3) 수준으로. (1) **무빙베이스 RTK 헤딩**: HDT 스칼라 대신 **UBX-NAV-RELPOSNED**
    (`f9p_client.parse_relposned`, `_StreamFramer` 바이트경로) — 에폭별 헤딩+`accHeading`+베이스라인+fix플래그.
    (3) **적응형 R + fix 게이팅**: `StateEstimator.update_heading_adaptive(σ)` + `AutoSteerSystem.on_heading_meas`
    가 `carrSoln==fixed && valid && acc≤max_hdg_acc_deg(0.6°)` 만 수용(아니면 predict coast, `heading_degraded` 플래그).
    (4) **베이스라인 틸트→roll** 경사보정 공급 + `calibration.RollPitchEstimator`(가속도 roll, 원심오염 배제) `on_accel`.
    (5) **진로각(COG) 보조**: `parse_vtg`→`on_velocity`→`update_cog`(횡슬립 beta 모델, 저속 게이트).
    배선: `app_main.start_gnss` 가 `on_heading_meas`/`on_velocity` 연결. 검증: `test_closed_loop.py` [5]~[9].
    ★ 전제: F9P 무빙베이스(안테나 2개) 구성 시 RELPOSNED 출력 — 없으면 HDT 폴백.
- **CHCNAV 성능 튜닝(✅ A)**: `vendor_profiles.CHCNAV_TUNING`(AgNav 문서값) → `select_vendor` 가
  `TrackingParams` 에 적용. `curve_coefficient` 도 예측항에 실제 반영(이전엔 미사용). 최종 갭은
  실하드웨어 `tuning.py`+`MotorResponseProbe` 튜닝 + 동일 PA-3 신호품질.

---

## GNSS 수신기 & 소스 이중화 (결정: PA-3 주 + F9P 백업)

### CHCNAV PA-3 스마트 안테나 (NX510 설치 안테나, 데이터시트)
- **GNSS+IMU 통합 수신기** (스마트 안테나). 위치+INS heading/자세를 직접 출력
- 인터페이스: **CAN 2포트 @500kb/s**, **RS232 2포트 ≤115200bps**, NMEA-0183
- 내장 IMU 100Hz: heading <0.3°, roll/pitch <0.1°, 속도 0.03 m/s
- 차분 **RTCM3.2/3.3** (rtk-lora-bridge LoRa NTRIP 포맷과 일치), 출력 ≤10Hz / 내부 50Hz
- 핀맵: COM1(M23 수) UART0_TX/RX(1,2)·CAN1(8,9)·CAN0(10,11)·NAVIGATE_IN(6, 외부 항법스위치)·12V검출(4) / COM2(M23 암) UART1_TX/RX(2,3)
- "OEM CAN/serial 프로토콜 커스터마이즈 제공" → **CanSpec CAN ID/바이트맵 입수 경로**
- → `CanSpec.CAN_BITRATE=500_000` 이 데이터시트로 확인됨 (모터가 같은 버스면 #2 비트레이트 확정)

### 소스 이중화 (코드: `GnssReceiverSpec`, `CHCNAV_PA3`/`UBLOX_F9P`, `GnssArbiter`)
- **결정**: PA-3를 주(primary), 본인 F9P를 백업으로 **둘 다** 사용 (이중화/비교)
- PA-3 NMEA = `ChcnavPa3SerialClient`(115200, source="pa3"), F9P = `F9pUsbClient`(38400, source="f9p")
- `AutoSteerSystem.rtk_callback("pa3"/"f9p")`로 각 클라이언트 연결 → `GnssArbiter`가 중재
- PA-3는 INS 융합 heading 출력 → IMU 캘리브(#5) 부담↓. ★ PA-3 CAN 출력 사용 시 CHCNAV OEM CAN 프로토콜 문서 필요

---

## 운영 UI = AGMO Solution 앱 화면 재현 (★ 필수 / 세션 간 유지)

> ⚠️ 이전에 매 세션마다 UI가 제각각으로 나온 원인: 이 요구가 CLAUDE.md에 없었고,
> AGMO 분석물은 `apk-analysis/findings/`(gitignore, 외부공유금지)에만 있어 레포로 안 따라옴.
> → **앞으로 운영 UI(`app/src/main/assets/autosteer_ui.html`)는 AGMO Solution 앱을 그대로 재현한다.**

- **대상 앱 = AGMO Solution v1.6.7** (AGMO 태블릿 기본앱). **AgNav = CHCNAV 앱**(별개, 혼동 금지)
- **clean-room 원칙**: AGMO 디컴파일 리소스/문자열/드로어블/스크린샷을 **레포에 커밋 금지**
  (public repo + findings 정책). 화면을 보고 레이아웃/색/배치/흐름만 자체 HTML로 재현.
- **디자인 시스템(디컴파일 colors.xml 확인값)**: 밝은 회색 패널(#eaeaea=white_gray)/흰 카드,
  **AGMO 틸 강조 `agmo_green #226b5d`**(헤더·토글ON·버튼·활성아이콘) + 밝은 액센트 `agmo_green_light #00b973`,
  텍스트 #333(dark_gray)·선 #dddddd(light_gray), 빨강 경고, 원근 그리드 필드, 상단 라이트 상태바, Noto Sans KR
  > AGMO 디컴파일 res/xml 자체는 커밋 금지(clean-room). 색 hex값만 CSS 토큰으로 반영함.
- **화면 구성**:
  1. 스플래시(AGMO 로고 + 트랙터가 진행바 위, %)
  2. 사용자 동의 안내(약관 + 체크 + 확인)
  3. 메인 주행화면: 상단 상태바[자율주행모드/비활성화 · 예상 종료 시간 · 오차 ◀◀ N cm ▶▶ · 현재 속도 km/h · 센서 상태] + 원근 필드(중앙 트랙터, 좌상단 카메라) + 하단 [⚙설정][🛞주행]
  4. 설정 드로어(좌 틸헤더 리스트 → 우 디테일, 마스터-디테일):
     사용자 정보(ID/언어/경고토글들/버전) · 차량 정보(1/2/3탭, 트랙터그림, 휠베이스, 변경하기) ·
     최적화(IMU영점/GPS중심/배속선회) · RTK 보정 신호(서버IP/포트/ID/PW/마운트포인트) ·
     시스템 상태(네트워크/GPS/RTK/IMU/조향모터 ON·OFF)
  5. 주행 모드 선택 모달: AB 직선 / AB 곡선 / 완전자율
  6. 경로 유형 모달: + 새 경로 추가 / 📁 기존 경로 사용
  7. 작업화면: 좌 경로 미리보기(전진, AB선+트랙터, 라인#) + 우 넛지(±)·유턴L/R·섹션수·**주행 시작** + 경고 다이얼로그(비정상 작동: GPS/IMU 등 + 빨강 주행 종료)
- **백엔드 배선**(JsBridge 유지): engaged→활성/비활성·주행시작/종료, xte_cm→오차, speed_mps→속도,
  motor_verified/vendor→배너, active_gnss→센서상태, engage()/disengage()/estop(), set_ab_line/setDemoAbLine
- 멀티벤더: 부팅 시 제조사 선택은 유지하되 AGMO 테마로. (CHCNAV/FJD는 추후 각 앱 룩 분기 가능)
- **우선순위(오너 지시)**: 먼저 **UI 구성(레이아웃/위젯배치/화면흐름) 일치**가 최우선. 색상은 후순위
  — 실제 색값은 확보했으나(위) 정밀 색맞춤은 나중에. 구성부터 1:1로 맞춘다.
- **AGMO Solution 2 (차기 반영 예정, 오너 보유)**: 1.6.7보다 **훨씬 세련된 UI**, **다른 태블릿+안테나**,
  **AB 직선 경로 각도 변경 기능** 보유. 추후 오너가 디컴파일해서 전달 예정 →
  받으면 이쪽 UI 로 다시 맞춘다(현재 1.6.7 구성은 그 전까지의 베이스).

---

## ⚖️ 준법 / clean-room 원칙 (★ 필수 / 모든 세션 적용)

> 레포는 **public**. 오너는 AGMO 자율주행 **전직**. 디컴파일/모방에는 저작권·영업비밀·
> NDA·경업금지 등 제약이 있을 수 있어 **상업화/배포 전 법률 검토 권고**(이 문서는 법률자문 아님).

**커밋 금지 (clean-room) — `.gitignore`로 차단됨:**
- 디컴파일 산출물(`*.apk/*.aab/*.dex/*.smali`, `smali*/`, `decompile*/`, `jadx/apktool-output/`),
  `apk-analysis/findings/`·`**/findings/`, 분석용 `screenshots/`
- 타사 앱의 리소스/문자열/드로어블/레이아웃 파일 **원본을 레포에 넣지 않는다**

**허용 (자체 구현):**
- 화면을 **보고** 레이아웃·배치·흐름을 자체 HTML로 재현(원본 파일 복제 X)
- 색 hex 등 **저작물성 낮은 사실 데이터**만 토큰으로 반영
- 상표("AGMO" 등) UI 미사용 (이미 제거 완료)

**★ 디컴파일 코드 이식(예: 모터 CAN) 핵심 규칙:**
> **기능적 사실(인터페이스/프로토콜)만 추출해 자체 구현한다. 소스/리소스 코드를 복붙하지 않는다.**
- 추출 OK = 비트레이트, CAN ID/바이트맵, `/dev` 경로, 함수 시그니처, 핸드셰이크 순서 등 **동작 사실**
- 금지 = 디컴파일된 소스/스마일/리소스를 그대로 옮겨 붙이기
- 멀티벤더 확장 시: 한 벤더(AGMO) 분석에서 얻은 정보를 타사(CHCNAV/FJD) 맥락에 전용하지 않도록 분리

---

## 멀티벤더 (제조사 선택) — ✅ `vendor_profiles.py`

> 컨셉: 이 앱을 **CHCNAV / AGMO / FJDynamics** 태블릿에 설치만 하면 그들의 하드웨어
> (조향모터 + GNSS + 앵글센서)를 그대로 사용. 앱 시작 시 제조사를 고르면 그 스택으로 구성.

- `VendorProfile`(frozen dataclass): 모터 CanSpec(dict) + GNSS 주/백업 + GnssArbiter 우선순위 + 기본 알고리즘 + `can_verified`
- `VENDOR_PROFILES` 레지스트리:
  - **agmo** ✅ `can_verified=True` — Keya KY170(250k) 확정 + AGMO GNSS(추정)
  - **chcnav** ★ `can_verified=False` — PA-3 GNSS+INS 확정 / 모터 CAN 미확정
  - **fjd** ★ `can_verified=False` — AT2 dome(추정) / 모터 CAN 미확정
- `apply_vendor(key)` → `field_config.apply_canspec()` 로 CanSpec 런타임 활성화
- `AutoSteerSystem.select_vendor(key)` → CanSpec + GnssArbiter 우선순위 + EKF 튜닝 + 알고리즘 적용.
  **`can_verified=False` 면 `engage()` 거부**(조향 출력 비활성, GNSS·표시는 동작) = 안전장치
- 진입: `app_main.list_vendors()` / `set_vendor(key)` ← JsBridge `listVendors()`/`setVendor()` ← UI 시작화면 오버레이(`#vendorOverlay`). 미확정 벤더는 하단 경고 배너(`#motorWarn`)
- status JSON 에 `vendor`/`vendor_name`/`motor_verified` 추가
- ★ 남은 일: CHCNAV/FJD 모터 CAN 프로토콜 입수 → `vendor_profiles` 의 canspec 채우고 `can_verified=True`. GNSS 추정 스펙(AGMO/FJD) 실측 교정

---

## AgNav 5.0 3-모드 시스템 (미구현, 다음 작업)

AgNav는 작업 상황별 3개 프로파일을 따로 유지:

| 모드 | WAS Gain | 크로스트랙 Gain | 방향감도 | 용도 |
|------|---------|----------------|---------|------|
| 1 일반 | 20 | 35 | 100 | 로터리/방제 |
| 2 과부하 ✅ | **15** | **100** | 100 | 쟁기/균평 |
| 3 모래토양 | **9** | 35 | 100 | 모래/사질토 |

**핵심 원리**: 과부하 모드에서 WAS Gain ↓ (조향 부드럽게) + 크로스트랙 Gain ↑ (경로 복귀 강하게) → S자 억제

**구현 방향**: `ControlProfile` 클래스로 3개 프로파일, `AutoSteerSystem.set_profile(mode)` API

---

## 추적 매개변수 (AgNav 사진 확인)

```
커브/해로우 공통:
  온라인 민감도: 1.5      ← 경로 위 추종 강도 (Stanley k_cross에 해당)
  접근 라인 민감도: 2.5   ← 경로 진입 시 (ImplementReferenced k_pred에 해당)
  온라인 임계값: 2.5      ← "경로 위" 판정 거리 (m)
  커브 계수: 1
```

---

## 안전 계층 (SafetyMonitor)

- 데드맨 스위치 (필수)
- RTK 품질 4(Fixed) 또는 5(Float)만 허용
- 운전자 개입 감지: 앵글센서 120deg/s 이상 급변
- 속도 제한: 2.5 m/s (≈9 km/h)
- 비상정지
- 최대 과부하 전류 초과 시 정지 (★ CanSpec과 연동 필요)

---

## ApolloCanInterface 구현 (★ 최우선 작업)

```python
class ApolloCanInterface(CanInterface):
    def start(self):
        # Apollo 10 Pro 내장 CAN 포트 초기화
        # Apollo SDK / android-can 라이브러리 사용
        # channel = "can0" 또는 Apollo 전용 경로
        pass

    def send(self, can_id: int, data: bytes):
        # CAN 프레임 전송
        pass

    def recv(self) -> Optional[tuple]:
        # CAN 프레임 수신 (non-blocking)
        # return (can_id, data) or None
        pass
```

Apollo 10 Pro는 CAN 내장 (IP65, ADB 환경). SDK 문서 확인 필요.

---

## 다음 작업 우선순위

1. **★ 실측**: wheelbase, antenna_to_axle — 물리 측정 (미완). ✅ 사전준비: `calibration.py`로 저속 주행 자동 추정 + `field_config.py`로 JSON 주입
2. ✅ **CanSpec 채우기**: Keya KY170 매뉴얼 V2.4 프로토콜 이식 완료(250k, 0x06000001 TX **확장프레임**, cmd_speed/parse_heartbeat). 실차 모터 회전 확인됨(확장ID 자동). ★ 남은 건 (a) autosteer `_send_motor` 를 cmd_speed 속도제어로 배선 + SITL 재검증, (b) 조향각 피드백을 **모터 하트비트 각**으로(AGMO=WAS 미사용). WAS CAN ID 캡처는 **CHCNAV/FJD 가 WAS 장착할 때만** 필요
3. ✅ **ApolloCanInterface/CAN 배선**: `apollo_can.ApolloCanBus`(bridge…) → Kotlin `ApolloCanBridge` → **`com.van.jni.VanMcu`(libsysmcu.so JNI)** 로 실제 송수신 배선 완료(`CanWrite`/`setCanSpeed`/`setOnCanListener`). VanMcu 는 com.agmo.autokit 디컴파일의 **인터페이스 사실**만 자체 구현(clean-room). **CAN 접근엔 device-owner 필요**: `adb shell dpm set-device-owner com.farmmachine.autosteer/.AdminReceiver`(AdminReceiver+device_admin.xml 추가). MainActivity 가 mock→**AutoSteerService(bridge)** 기동. ★ 남은 건 실차에서 모터 CAN **채널(0/1) 확정** + 확장프레임 플래그 검증
4. ✅ **RTK 연결**: `f9p_client.F9pUsbClient`/`ChcnavPa3SerialClient` → `on_rtk()` (GGA 파싱, 품질 4/5, sniff/UBX/보레이트탐색)
5. ✅ **IMU 캘리브레이션**: `ImuCalibrator` (평지 30초 평균 → ImuOffset)
6. ✅ **3-모드 프로파일**: `TuningProfile` + `PROFILE_NORMAL/HEAVY/SAND` + `set_profile()`
7. **저속 안전 검증**: 빈 농지 1km/h, 데드맨 + 비상정지 — 현장 (미완). ✅ 사전준비: `sitl_sim.py` 폐루프 + 안전 6종 시나리오 검증

### 사전 준비 도구 (실측/문서/현장 전에 전부 구현됨)
- `field_config.py`: TractorParams/CanSpec ↔ JSON. `write_template`/`load_config`로 코드 수정 없이 실측·CAN값 주입
- `calibration.py`: `WheelbaseEstimator`/`LeverArmEstimator` — 자전거모델 회귀로 wheelbase·안테나오프셋 자동 추정(저속 사인주행)
- `can_tools.py`: `CanBusAnalyzer`(ID별 주기/변동성), `correlate_with_signal`(외부 조향신호 상관→앵글센서 ID/바이트/부호), `CanLogger`(candump 호환)
- `sitl_sim.py`: 폐루프 시뮬. `BicycleModel`(yaw_tau=작업기부하 지연, dist_amp=횡저항 외란), `ServoCanInterface`(rate-limit+1차지연 현실 서보), `run_safety_scenarios`(6종 전부 PASS)
- `tuning.py`: SITL 위 게인 자동탐색. `evaluate/cost/tune_profile` — heavy 진동을 잡는 게인 도출(예: 모델기준 baseline 45cm✗진동 → 추천 9cm안착)
- ⚠ SITL 발견 + 튜닝 결론: 모델(서보 35°/s·80ms)에선 AgNav 사진값 k_cross=100 이 서보지연과 맞물려 진동. 예측리드 부족으로 '저게인'이 최적. **실모터 응답을 can_tools 로 계측→ServoCanInterface(rate/tau)에 반영→tuning 재탐색**해야 현장 heavy 게인 확정
- `can_tools.MotorResponseProbe`: 조향 스텝응답으로 실모터 servo(max_rate,tau) 계측 → tuning 입력 (시뮬서보 35/0.08 정확 복원 검증)
- `field_collect.py`: 현장 수집 오케스트레이터. stage_gnss/imu/kinematics/can_angle/motor/manual → `tractor.json`+리포트 자동생성. 합성데이터로 전 파이프라인 self-test
- `MEASUREMENT.md`: 실측값 전체 목록 + 수동 측정법(사진치수 A/D/E/B1/G) + 자동수집 도구 매핑 + 정상범위
- `auto-steering/README.md`: 현장 1일차 절차, `requirements.txt`

**남은 핵심 작업 (하드웨어/현장 의존)**: #1 실측값 입력, #2 실제 CAN ID/바이트맵 수집, #7 실차 안전검증.
도구는 다 준비됨 — 현장에선 수집→JSON 주입→SITL 재검증→1km/h 실주행 순.

---

## 관련 레포

- `Eunbangwool/farm-work-manager`: Android 앱 메인 (레벨러 UI 등)
- `Eunbangwool/rtk-lora-bridge`: LoRa NTRIP 기지국 (F9P + LoRa, RTCM 필터링)

---

## 핵심 파일

```
auto-steering/src/autosteer_core.py  — 메인 알고리즘 (Python, ~1200줄)
app/src/main/assets/autosteer_ui.html — ★ 태블릿 운영 UI (HTML/JS). WebView 로 로드,
    window.AndroidSteer(JsBridge)로 Python 연결. 운영 UI HTML 은 여기에 둔다(교체 지점).
app/src/main/java/com/farmmachine/autosteer/{MainActivity,JsBridge,SteerController}.kt
```
> ⚠ 과거 CLAUDE.md 가 `auto-steering/src/autosteer_ui.html`·`sim/path_following_sim.html`
> 을 핵심 파일로 적었으나 실제 커밋된 적 없음. 운영 UI 는 WebView+assets 로 재정의됨.

---

## 코드 실행

```bash
cd auto-steering/src
pip install numpy
python autosteer_core.py   # MockCAN으로 즉시 테스트 가능
```

