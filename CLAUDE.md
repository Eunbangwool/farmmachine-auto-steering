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
├── auto-steering/          ← ★ 핵심 (이 문서의 대상)
│   ├── src/
│   │   ├── autosteer_core.py   # 메인 알고리즘 (4계층)
│   │   └── autosteer_ui.html   # 태블릿 운영 UI
│   └── sim/
│       └── path_following_sim.html  # 경로 설계 시뮬레이터
├── rtk-leveling/           ← 레벨러 + LoRa NTRIP (별도 레포 rtk-lora-bridge와 연동)
├── leveefollow/            ← 라이다 헤드랜드 감지
└── apk-analysis/
```

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

## CanSpec — ★ 모터 문서로 채워야 함

```python
class CanSpec:
    CAN_BITRATE      = 500_000   # ★ 250000 / 500000 / 1000000
    MOTOR_CMD_ID     = 0x201     # ★ 실제 CAN ID
    SENSOR_ANGLE_ID  = 0x301     # ★ 앵글센서 CAN ID
    # + 바이트 레이아웃, 스케일, 활성화 시퀀스
```

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
2. **★ CanSpec 채우기**: 모터 CAN ID + 바이트 구조 — 모터 문서 (미완). ✅ 사전준비: `can_tools.py`로 버스 역추적(앵글/모터 ID 탐색), `field_config.py`로 JSON 주입
3. ✅ **ApolloCanInterface 구현**: SocketCAN(python-can/raw socket). ★ Apollo 전용 SDK면 start/send/recv 내부만 교체
4. ✅ **RTK 연결**: `f9p_client.F9pUsbClient`/`ChcnavPa3SerialClient` → `on_rtk()` (GGA 파싱, 품질 4/5, sniff/UBX/보레이트탐색)
5. ✅ **IMU 캘리브레이션**: `ImuCalibrator` (평지 30초 평균 → ImuOffset)
6. ✅ **3-모드 프로파일**: `TuningProfile` + `PROFILE_NORMAL/HEAVY/SAND` + `set_profile()`
7. **저속 안전 검증**: 빈 농지 1km/h, 데드맨 + 비상정지 — 현장 (미완). ✅ 사전준비: `sitl_sim.py` 폐루프 + 안전 6종 시나리오 검증

### 사전 준비 도구 (실측/문서/현장 전에 전부 구현됨)
- `field_config.py`: TractorParams/CanSpec ↔ JSON. `write_template`/`load_config`로 코드 수정 없이 실측·CAN값 주입
- `calibration.py`: `WheelbaseEstimator`/`LeverArmEstimator` — 자전거모델 회귀로 wheelbase·안테나오프셋 자동 추정(저속 사인주행)
- `can_tools.py`: `CanBusAnalyzer`(ID별 주기/변동성), `correlate_with_signal`(외부 조향신호 상관→앵글센서 ID/바이트/부호), `CanLogger`(candump 호환)
- `sitl_sim.py`: `BicycleModel`+`Simulator` 폐루프, `run_safety_scenarios`(데드맨/E-stop/RTK저하/RTK끊김/개입/과속 전부 PASS)
- ⚠ SITL 발견: `implement/heavy`가 단순 모터모델 폐루프에서 진동 → **현장 전 heavy 게인 재튜닝 필요**
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
auto-steering/src/autosteer_ui.html  — 태블릿 운영 UI (HTML/JS)
auto-steering/sim/path_following_sim.html  — 경로 설계 시뮬레이터
```

---

## 코드 실행

```bash
cd auto-steering/src
pip install numpy
python autosteer_core.py   # MockCAN으로 즉시 테스트 가능
```

