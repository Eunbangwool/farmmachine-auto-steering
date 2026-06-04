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

## 현장 1일차 절차 (요약)
1. `field_config.write_template` → 줄자 실측값 입력 (또는 calibration 자동추정)
2. `f9p_client` sniff 로 PA-3/F9P NMEA·RTK fix(4/5) 확인
3. `can_tools` 로 앵글센서/모터 CAN ID 역추적 → `tractor.json` 의 canspec 채움
4. `sitl_sim` 재실행으로 실파라미터 폐루프 재검증
5. 데드맨 + 비상정지 동작 먼저 확인 → 1km/h 빈 농지 일반 모드 → 과부하 모드

## 남은 하드웨어 의존 항목
- ApolloCanInterface: 실차 `can0` 비트레이트/포트만 맞추면 동작(코드 완료)
- PA-3 CAN 출력(위치+자세): CHCNAV OEM CAN 프로토콜 문서 필요
- 실측값/모터 CAN ID: 위 도구로 수집 → JSON 주입
