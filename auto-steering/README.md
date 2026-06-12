# auto-steering — 자율조향 핵심 모듈

Kubota MR1157 + Apollo 10 Pro + CHCNAV PA-3/F9P RTK 기반 자율조향.
설계 배경·결정은 저장소 루트 `CLAUDE.md` 참고.

## 모듈 구성

| 파일 | 역할 | 하드웨어 |
|------|------|----------|
| `src/autosteer_core.py` | 4계층 핵심(경로·EKF·추종·CAN모터) + 3모드 프로파일 + 안전계층 + GNSS 이중화 | 불필요(Mock) |
| `src/f9p_client.py` | F9P/PA-3 NMEA 수신 + **스트림 정찰(sniff)** | 시리얼 |
| `src/field_config.py` | TractorParams/CanSpec **JSON 외부화** (실측·CAN문서 주입) | 불필요 |
| `src/calibration.py` | 주행데이터로 **wheelbase·안테나 오프셋 자동 추정** | 불필요 |
| `src/can_tools.py` | **CAN 버스 역공학**(앵글센서/모터 ID 탐색) | CAN |
| `src/sitl_sim.py` | **폐루프 시뮬**(현실 서보+작업기부하) **+ 안전 시나리오 검증** | 불필요 |
| `src/tuning.py` | SITL 위 **프로파일 게인 자동탐색**(heavy 진동 잡기) | 불필요 |
| `src/field_collect.py` | **현장 데이터 자동수집 오케스트레이터**(수집→config→리포트) | 혼합 |
| `MEASUREMENT.md` | **실측값 목록 + 측정법 + 자동수집 매핑** | — |

모든 모듈은 하드웨어 없이 자체 테스트가 돈다:
```bash
cd auto-steering/src && pip install -r ../requirements.txt
for m in autosteer_core f9p_client field_config calibration can_tools sitl_sim; do python $m.py; done
```

## 실측/현장 전에 미리 끝낼 수 있는 것 (이미 준비됨)

CLAUDE.md 우선순위 #1·#2·#7 은 값/문서/현장이 있어야 "채워"지지만, 그 준비는 완료:

### #1 실측 (wheelbase, antenna_to_axle)
줄자 대신 **빈 농지 저속 주행 → 자동 추정**:
```python
from calibration import estimate_from_log
# 주행 중 매 스텝 dict(x,y,heading,yaw_rate,speed,steer_rad) 기록 → samples
res = estimate_from_log(samples)
print(res["wheelbase"].value, res["antenna_to_axle"].value)
```
좌우로 번갈아(사인) 저속 주행하면 R²>0.9 로 수렴. 줄자 실측이 있으면 교차검산용.

### #2 CanSpec (모터 CAN 문서)
문서가 오기 전 **버스를 떠서 ID/바이트 역추적**:
```python
from can_tools import CanLogger, CanBusAnalyzer, correlate_with_signal
log = CanLogger(apollo_can); log.run(20)          # 20초 캡처
an = CanBusAnalyzer(); an.feed_all(log.frames)
print(an.report())                                 # ID별 주기/변동성
# 운전대를 손으로 좌우로 흔든 신호[(t,deg)]와 상관 → 앵글센서 인코딩
hits = correlate_with_signal(log.frames, signal)
print(hits[0].describe())   # 0xID byte[hi,lo] BE/signed
```
문서/역추적 결과는 코드 대신 **JSON** 으로 주입:
```python
from field_config import write_template, load_config
write_template("tractor.json")     # ★ 자리 포함 템플릿
# tractor.json 편집 (실측값 + CAN ID/바이트맵) 후
params, n = load_config("tractor.json")   # CanSpec 런타임 반영
```

### #7 저속 안전 검증
현장 1km/h 실차 전에 **SITL 로 폐루프 + 안전 6종 사전검증**:
```bash
python sitl_sim.py     # 폐루프 추종 + 안전 6종(데드맨/E-stop/RTK저하/끊김/개입/과속)
python tuning.py       # heavy 프로파일 게인 자동탐색 (진동 → 안착)
```
SITL 은 현실적 플랜트(rate-limit 서보 + 작업기부하 yaw 지연 + 횡저항 외란)를
쓴다. 이 모델에서 AgNav 사진값 `heavy(k_cross=100)` 는 서보지연과 맞물려 진동 →
`tuning.py` 가 안착하는 게인(모델기준 ~9cm)을 자동 도출.
**단, 모델 추천치다.** 실모터 응답을 `can_tools` 로 계측해 `ServoCanInterface`
의 `max_rate_deg_s/tau` 에 반영한 뒤 `tuning.py` 를 다시 돌려야 현장 heavy 게인이 된다.

## GNSS 스트림 점검 (PA-3 115200 / F9P 38400)
```python
from f9p_client import GnssSniffer, detect_baudrate
print(GnssSniffer("/dev/ttyS1", 115200, echo=True).run(5).format_report())
baud, rep = detect_baudrate("/dev/ttyS1")   # 보레이트 모를 때
```

## ★ 현장 1단계 브링업 — AGMO ver1 (실차 폐루프 첫 검증)

> 목표: **위치 + heading + 조향 피드백** 폐루프를 실기기에서 처음 닫는다.
> 전제(오너 확인): GNSS = **AGMO ver1 듀얼안테나 → Apollo 내부 UART(u-blox)**, 모터 = Keya KY170 CAN.
> **device-owner 불필요**(모터 TX 권한 없이 동작 확인됨). USB 는 레벨러 전용이라 여기 안 씀.
> 진입점은 전부 `app_main.*`(JsBridge 로 UI 에서도 호출 가능).

### 0) 준비
- APK 빌드·설치(`build-autosteer-apk.yml`). device-owner 설정 **하지 않음**.
- 시작화면에서 제조사 = AGMO 선택(`set_vendor("agmo")`). 모터 verified=True 라 engage 허용.
- 안전: 바퀴 주변 사람/장애물 없음, 비상정지·데드맨 손 위.

### 1) 모터 CAN (TX + heartbeat RX)
```python
# 조그(hold-to-run): 좌(+)/우(-) 회전·정지. 부호 규약: +permille=좌, -permille=우.
motor_jog(150); ... ; motor_jog(0)
can_status()          # {"txCount":↑, "rxCount":?, ...}  ← logcat VanMcu onCallback 도 확인
```
- `txCount` 증가 + 모터 회전 → TX OK. 방향이 반대면 배선/부호 현장 확정.
- **`rxCount` 증가 / `onCallback` 로그** → heartbeat 살아남(= setCallback CAN 필터 수정 효과).
  - 살면: 모터 인코더 누적각을 실측 피드백으로 사용.
  - 0 이면: dead-reckoning 폴백으로 자동 동작(폐루프는 GNSS heading 이 닫음).
- 모터 CAN **채널(0/1)** 이 다르면 `setCanParams(ch, 250000, false)` 로 전환.

### 2) GNSS (내부 UART → 위치 + 듀얼안테나 heading)
```python
gnss_power_on()                       # 내부 u-blox 전원/standby ON (sysfs, best-effort)
scan_gnss()                           # /dev/ttyS* 자동 스니핑 → best 포트·baud
configure_moving_base("/dev/ttySX")   # UBX-NAV-RELPOSNED(듀얼헤딩)+PVT+VELNED+GGA/VTG 활성·저장
start_gnss("/dev/ttySX")              # 파서 가동 → on_rtk/on_heading_meas/on_velocity 배선
status_json()                         # active_gnss, pos, heading_deg, xte_cm, heading_degraded
```
- `scan_gnss().best` 가 None 이면 전원/배선/baud 확인(전원 sysfs 실패는 logcat 에 표시).
- `status` 에 위치 갱신 + RTK 품질 4/5 + `heading_deg` 변화 → GNSS 유입 OK.
- **RELPOSNED 가 안 나오거나 heading 이 안 잡히면**: 수신기가 무빙베이스/로버 모드가 아닐 수 있음
  → 현장 조사(2번 `configure_moving_base` 는 출력만 켤 뿐, 모드 자체는 공장설정 가정).

### 3) 듀얼안테나 heading 부호 확정 (base=좌/rover=우)
```python
start_mount_diag()    # 직선 ~15m 주행, 끝에 차를 우측으로 살짝 기울임
mount_diag_status()   # base_antenna / rec_baseline_offset_deg(+90?) / rec_dual_roll_sign(+1?)
```
- 추천값이 코드 기본(`dual_baseline_offset_deg=90`, `dual_roll_sign=+1`)과 다르면 그 두 상수만 맞춤.

### 4) 저속 폐루프 (1 km/h, 빈 농지)
- engage 게이트: 데드맨 + RTK 4/5 + heading 유효 (+ heartbeat 있으면 사용). 끊기면 즉시 disengage.
- `set_ab_line(...)` 또는 `setDemoAbLine()` → `engage()` → **일반 모드** 1km/h → 안정되면 과부하 모드.
- E-stop/데드맨 동작을 **주행 전** 반드시 먼저 확인.

### 문제 해결 빠른표
| 증상 | 우선 확인 |
|------|-----------|
| 모터 안 돎 | `can_status().canReady`, 채널(0/1), 250k, 확장프레임(자동) |
| 방향 반대 | data-dir/부호(현장), `+permille=좌` 규약 |
| heartbeat rxCount=0 | logcat `onCallback` 유무 → 없으면 dead-reckoning 으로 계속 진행 가능 |
| GNSS best=None | `gnss_power_on()` 로그, 다른 ttyS, baud(115200/38400/460800) |
| heading 안 잡힘 | RELPOSNED 출력 여부(무빙베이스 모드) · `heading_degraded` 플래그 |
| 중심 치우침 | `start_mount_diag`/`start_heading_calib` 로 바이어스 보정 |

## 현장 1일차 절차 (요약)
1. `field_config.write_template` → 줄자 실측값 입력 (또는 calibration 자동추정)
2. `f9p_client` sniff 로 PA-3/F9P NMEA·RTK fix(4/5) 확인
3. `can_tools` 로 앵글센서/모터 CAN ID 역추적 → `tractor.json` 의 canspec 채움
4. `sitl_sim` 재실행으로 실파라미터 폐루프 재검증
5. 데드맨 + 비상정지 동작 먼저 확인 → 1km/h 빈 농지 일반 모드 → 과부하 모드

## 남은 하드웨어 의존 항목
- 모터 CAN: Apollo `libsysmcu.so`(VanMcu **채널 API**, socketcan 아님) 경유 — 실차 채널(0/1) 확정.
  RX heartbeat 는 setCallback(CAN 비트) 수정 완료 → 실기기 `onCallback`/`rxCount` 로 수신 검증.
- GNSS: AGMO ver1 내부 UART 포트(`scan_gnss`) + 무빙베이스 RELPOSNED 출력 여부(모드=공장설정 가정) 현장 확인.
- PA-3/NX510(CAN/RS232): 실험 후 추가. CAN 출력은 CHCNAV OEM CAN 프로토콜 문서 필요.
- 실측값(wheelbase/레버암): `calibration` 자동추정 또는 줄자 → `tractor.json` 주입.
