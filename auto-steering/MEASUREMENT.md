# 실측 가이드 + 자동 수집 (MEASUREMENT)

Kubota MR1157 자율조향에 필요한 값 전체와 **측정법**, 그리고 측정 후 **자동 수집**
경로(`field_collect.py`)를 정리한다. 사진 치수 라벨(A/D/E/B1/G)은 `CLAUDE.md` 기준.

> 원칙: 가능한 값은 **자동 수집**(주행/센서/CAN)으로 받고, 줄자 값은 **교차검산**용으로도 쓴다.
> 모든 결과는 `tractor.json`(코드 수정 없이 주입) + 세션 리포트로 저장된다.

---

## A. 줄자/사진으로 재는 값 (수동)

| 값 | 의미 | 측정법 | 정상범위 | 비고 |
|---|---|---|---|---|
| `wheelbase` (A) | 앞차축 중심 ↔ 뒷차축 중심 | 차량 옆에서 두 차축 중심에 줄자. 좌우 평균 | 1.5~4.0 m | **자동추정과 교차검산**(아래 D2) |
| `antenna_height` (E) | 지면 ↔ GPS 안테나 | 안테나 바닥에서 지면까지 수직 줄자(타이어 공기압 정상 상태) | 1.5~3.5 m | 사진값 2.73 |
| `antenna_to_impl` | 안테나 ↔ 작업기 작용점 | 안테나 수직투영점에서 작업기까지 전후 거리 | 0~4.0 m | 작업기 장착 후 |
| `hitch_to_impl` (B1) | 히치 ↔ 작업기 | 3점히치 핀에서 작업날까지 | 0~3.0 m | 사진 B1=1.0 |
| `front_track_width` (G) | 전륜 좌우 폭 | 좌우 앞바퀴 중심 간 거리 | 1.0~2.5 m | 사진 1.56, Ackermann용 |
| `max_was_deg` | 최대 조향각 | 풀락까지 돌려 각도기, 또는 **CAN으로 자동**(아래 D4) | 10~50° | 사진 25 |

입력: `collector.stage_manual(antenna_height=2.73, antenna_to_impl=1.20, ...)`

---

## B. 평지 정지로 받는 값 — IMU 오프셋 (파라미터 5)

| 값 | 측정법 | 자동 도구 |
|---|---|---|
| `imu_offset.roll/pitch/yaw` | **수평 평지에 정지** 후 30초 IMU 원시값 평균 | `ImuCalibrator` |

절차: 트랙터를 평탄지에 정지 → `collector.collect_imu_live(read_rpy, 30)`.
yaw는 절대방위가 필요 → `heading_ref_rad`(RTK/PA-3 heading)를 주면 보정, 없으면 0.

---

## C. 저속 주행으로 받는 값 — 운동학 자동추정

줄자가 어려운 `wheelbase`/`antenna_to_axle`를 **주행 데이터로 역산**(`calibration.py`).

| 값 | 측정법(주행) | 원리 |
|---|---|---|
| `wheelbase` | 빈 농지에서 **저속(≈1km/h) 좌우 사인주행 30초** | ω=v·tanδ/L 회귀 |
| `antenna_to_axle` (D) | 위 주행 동시 기록(GPS·heading·yaw·speed·δ) | (COG−heading)=atan2(ω·d,v) 회귀 |

매 스텝 `dict(x,y,heading,yaw_rate,speed,steer_rad)` 기록 → `stage_kinematics(drive_log)`.
R²>0.9면 신뢰. 줄자값과 ±0.1m 이내면 교차검산 통과.

---

## D. CAN 버스로 받는 값 — 문서 없이 역추적 (`can_tools.py`)

| 값 | 측정법 | 도구 |
|---|---|---|
| `SENSOR_ANGLE_ID` + byte오프셋/부호 | 모터 OFF, **운전대를 손으로 좌우로 흔들며** CAN 캡처 + 흔든 각도 기록 → 상관분석 | `correlate_with_signal` |
| `MOTOR_CMD_ID` | **내가 명령 보낼 때만** 변하는 ID로 식별(같은 상관기법) | `CanBusAnalyzer` |
| `CAN_BITRATE` | PA-3 데이터시트 500k 확인. 모터 같은 버스면 동일 | (확정) |
| `max_was_deg` | 풀락 좌/우에서 앵글센서 최대/최소 읽어 환산 | 앵글센서 디코드 |
| **servo `max_rate`, `tau`** | **조향 스텝 명령 → 앵글센서 응답** 기록(큰스텝=각속도, 작은스텝=지연) | `MotorResponseProbe` |

입력: `stage_can_angle(frames, signal)` / `collect_motor_live(can)`.
⚠ 모터 계측은 **바퀴 잭업 또는 빈 농지 정지**, 사람 접근 금지, 데드맨 확보.

---

## E. GNSS 스트림 점검

| 값 | 측정법 | 도구 |
|---|---|---|
| baud / 포맷(NMEA/UBX) / RTK fix(4·5) | 포트 열어 5초 스니프 | `GnssSniffer` |

입력: `collect_gnss_live("/dev/ttyS1",115200,'pa3')` (F9P는 38400,'f9p').

---

## 전체 자동 수집 흐름 (`field_collect.py`)

```python
from field_collect import FieldDataCollector
fc = FieldDataCollector()

fc.collect_gnss_live("/dev/ttyS1", 115200, "pa3")     # E
fc.collect_imu_live(read_rpy, 30)                      # B
fc.stage_kinematics(drive_log)                         # C (주행 로그)
fc.stage_can_angle(frames, signal)                     # D (앵글센서)
fc.collect_motor_live(can)                             # D (servo)
fc.stage_manual(antenna_height=2.73, antenna_to_impl=1.20,
                hitch_to_impl=1.00, front_track_width=1.56,
                max_was_deg=25.0, wheelbase=2.56)       # A
fc.finalize("tractor.json", "collect_report.md")
```

산출물:
- `tractor.json` — TractorParams + CanSpec (코드 수정 없이 주입; `field_config.load_config`)
- `collect_report.md` — 항목별 값/출처/상태(✅/⚠️) + 재확인 목록
- `fc.session.servo` — `{servo_rate_deg_s, servo_tau}` → **`tuning.py`로 heavy 게인 재탐색**

이후: `tuning.py`(servo 반영) → `sitl_sim.py` 재검증 → 1km/h 실주행.
`python field_collect.py` 로 합성 데이터 전 파이프라인 데모 실행 가능.
