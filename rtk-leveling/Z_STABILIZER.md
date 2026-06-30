# VRS Z축 동적오차 방어 — GNSS 레벨러 고도 안정화 설계

> 구현: `rtk-leveling/src/z_stabilizer.py` (leveler_core 미수정 add-on, SITL 5/5).
> 목표: VRS(네트워크 RTK)의 약점(기지국 원거리 시 Z 오차·Fix 풀림)을 **하드웨어 추가 없이**
> SW 필터/퓨전만으로 방어 → 블레이드 Z 실시간 동적오차 1~2cm 유지.
> 전제 HW: 3주파 Multi-GNSS(BeiDou/Galileo 활성), 안테나 내장 고정밀 IMU.

---

## 1. Architectural Overview (데이터 흐름)

```
 [GNSS 수신기]                      [안테나 내장 IMU]
  NMEA 1Hz                           가속도/자이로/자세 (≈100Hz)
  GGA·GSV·GST                              │
     │                                     │
     ▼                                     ▼
 ┌─ M1: GnssQualityMonitor ──┐     ┌─ 틸트 삼각보정 ───────────┐
 │ 고도각 15° 마스크          │     │ imu_vertical_accel()       │
 │ 구성위성수(BeiDou/Gal)    │     │ a_up = f(ax,ay,az,roll,pit)│
 │ GST σ_alt → 적응형 R_z    │     │ blade_tip_z (레버암 보정)  │
 └───────────┬───────────────┘     └─────────────┬─────────────┘
   z_meas(틸트보정), σ_z                   a_up(수직가속), roll/pitch
             │                                     │
             ▼                                     ▼  (M3 입력 LPF 2.5Hz: 디젤진동 제거)
        ┌──────────────────── M2: VerticalEKF [z, vz, accel_bias] ────────────────────┐
        │  predict(a_up,dt)  →  update_z(z_meas, R=σ_z²)  →  ZUPT update_vz(0,σ_zupt)  │
        │  (GNSS 끊김 시 predict+ZUPT 만 = RTK Bridge 데드레코닝)                       │
        └───────────────────────────────┬─────────────────────────────────────────────┘
                                    z_hat, vz_hat, σ_z
                                         │
                          ┌──────────────┴───────────────┐
                          ▼                               ▼
              M3: 출력 LPF 0.35Hz                 페일세이프 FSM
              (1Hz GNSS 잡음 톱니 평활)      TRACK→BRIDGE→HOLD→STOP
                          │                               │
                          ▼                               ▼
              M4: 품질 적응형 데드밴드/게인  +  control_enabled 게이트
                          │
                          ▼
              ZEstimate → LevelingController / proportional_valve (유압 밸브)
```

핵심: **2개 시간상수 분리**. ① 빠른 IMU(수직가속) = 진동·끊김 대응(데드레코닝) ② 느린 GNSS(절대고도)
= 드리프트 보정. EKF가 둘을 융합하고, 입력 LPF(디젤 2.5Hz)·출력 LPF(GNSS잡음 0.35Hz)가 대역을 가른다.

---

## 2. Core Algorithm Logic (상호작용 메커니즘)

### M1. 고도각 마스크 + Multi-GNSS 적응형 R
- 지평선 부근(저고도각) 위성은 대기굴절·멀티패스 노이즈가 크다 → **고도각 ≥15° 만 가용 집계**.
- 마스크 상향으로 줄어든 위성수는 **RTCM3.2/MSM5 로 BeiDou/Galileo 대거 확보(≥30)** 해 상쇄.
- 측정 신뢰도를 **적응형 측정분산**으로 EKF에 전달:
  ```
  σ_z = base · dop_factor · sat_factor
    base       = GST σ_alt (있으면 1순위) else {Fix:1.2cm, Float:25cm}
    dop_factor = max(1, HDOP/1.0)
    sat_factor = 1.0 (마스크통과 ≥30) else 1 + (30−N)·0.05   # 위성 부족 시 R 팽창
  ```
  → 기지국이 멀어져 품질이 나빠지면 σ_z↑ → EKF가 GNSS를 덜 믿고 IMU 우위로 자연 전환.

### M2. GNSS+IMU EKF + RTK Bridge (데드레코닝)
- 상태 `x=[z, vz, b_a]` (고도, 수직속도, 가속도 바이어스). 1차원 수직 모델.
- **예측**(IMU 수직가속도 적분, dt):
  ```
  a   = a_up − b_a
  z  += vz·dt + ½·a·dt² ;  vz += a·dt ;  b_a += 0(random walk)
  P   = F·P·Fᵀ + Q,   F = [[1,dt,−½dt²],[0,1,−dt],[0,0,1]]
  ```
- **갱신**(틸트보정 GNSS 고도, 스칼라, H=[1,0,0]):
  ```
  S = P₀₀ + σ_z² ;  K = P[:,0]/S ;  x += K·(z_meas − z) ;  P −= K·P[0,:]
  ```
- **틸트 삼각보정**(마스트 흔들림/지면기울기 ≤30°): 안테나가 기울면 위상중심↔블레이드 레버암의
  수직투영이 변해 Z가 튄다 →
  ```
  blade_z = antenna_z − h·cosθ·cosφ − d·sinθ − ℓ·sinφ   (leveler_core.blade_tip_z 재사용)
  a_up    = az·cosφ·cosθ + ax·sinθ − ay·sinφ·cosθ − g    (월드 수직가속, 중력제거)
  ```
- **RTK Bridge**: VRS Float/끊김 시 `update_z` 를 건너뛰고 `predict` 만 → IMU 데드레코닝.
  단, 1Hz 위치측정만으론 vz 관측이 약해 그대로 두면 σ_z 가 폭증(1초에 ~50cm).
  → **ZUPT(수직 정지속도 보정)**: 필터링 a_up 이 임계값 미만(블레이드 높이 유지 중)이면
  `update_vz(0, σ_zupt)` 로 vz≈0 구속 → 드리프트를 IMU 바이어스 수준으로 묶음.
  **결과(SITL): 끊김 15초간 드리프트 0cm, σ_z 0.8→2.2cm 유지(<5cm 한계).**

### M3. 평활 / 대역분리 (디젤 고주파 vs 지면 저주파)
- **입력단** 2차 Butterworth LPF `accel_lpf_hz=2.5` : 디젤 엔진 고주파(>5Hz) 진동을 가속도에서
  제거(지면 응답대역은 통과) → EKF 예측이 진동에 오염되지 않음.
- **출력단** 2차 LPF `output_lpf_hz=0.35` : 1Hz GNSS 측정잡음이 만드는 톱니(EKF z 보정 스텝)를
  평활 → 밸브 제어신호 요동 방지. 레벨러는 느린 시스템이라 0.35Hz로도 추종 지장 없음.

### M4. 유압 밸브 데드밴드 + 게인 댐핑 (채터링 방지)
- 품질 적응형으로 `ZEstimate` 가 권고:
  ```
  TRACK : deadband = 1.0cm, gain = 1.0
  BRIDGE: deadband = 3.0cm, gain = max(0.3, 1 − σ_z/σ_limit)   # σ 클수록 게인 감쇠
  HOLD/STOP: gain = 0 (밸브 중립)
  ```
- 데드밴드 안(±1cm 미세떨림)에서는 밸브 HOLD → 땅 파먹는 채터링 차단. 기존
  `proportional_valve.ProportionalValveController`(PID+다단계 데드존)·`LevelingController`
  (히스테리시스)가 이 데드밴드/게인을 입력으로 받아 적용.

---

## 3. Configuration Guide (권장 세팅)

### GNSS 수신기
| 항목 | 권장값 | 비고 |
|---|---|---|
| Elevation Mask | **15°** | 저고도 멀티패스 차단 |
| Constellations | GPS + GLONASS + **BeiDou + Galileo** | MSM5 로 ≥30위성 확보 |
| Correction | RTCM **3.2 / MSM5** 마운트포인트 | 다중위성군 보정 |
| NMEA 출력 | **GGA + GSV + GST**, **1Hz** | GST(σ_alt)는 적응형 R 핵심 |
| Tilt/IMU | 안테나 내장 IMU 보정 ON | roll/pitch + 수직가속 |

### z_stabilizer (`ZStabilizerConfig`)
| 파라미터 | 기본 | 의미 |
|---|---|---|
| `elevation_mask_deg` | 15.0 | 가용위성 집계 마스크 |
| `accel_lpf_hz` | 2.5 | 입력: 디젤 진동 차단 |
| `output_lpf_hz` | 0.35 | 출력: GNSS 잡음 평활 |
| `ctrl_rate_hz` | 20.0 | 제어/예측 주기 |
| `bridge_max_s` | 15.0 | RTK Bridge 최대 유지(10~20s) |
| `bridge_sigma_limit_cm` | 5.0 | 브리지 σ 한계 → HOLD |
| `zupt_accel_thresh` | 0.10 m/s² | 이하면 수직정지 → ZUPT |
| `zupt_sigma_vz` | 0.02 m/s | ZUPT 구속 강도 |
| `base/float_deadband_cm` | 1.0 / 3.0 | 밸브 데드밴드(품질별) |
| EKF `q_z,q_v,q_b` | 5e-7,1e-6,1e-9 | ★IMU 등급별 현장 보정 |

★ 실차 보정 필요: IMU 부호/축 규약, `zupt_*`(저속작업 특성), `q_v`(IMU 가속도 잡음), GST 미출력 수신기는
`sigma_fix/float` 휴리스틱 의존.

---

## 4. Edge Case Handling (Fail-safe)

페일세이프 FSM (`ZStabilizer._fsm`), GGA 신선도 `age ≤ max(2s, 3/rate)` 기준:

| 상태 | 진입 조건 | 유압 동작 |
|---|---|---|
| **TRACK** | RTK Fix + 신선 | 정상 추종 (deadband 1cm) |
| **BRIDGE** | Fix 풀림/끊김 | **IMU 데드레코닝 + ZUPT 로 계속 제어**(deadband 3cm, σ↑ 시 게인 감쇠). 10~20s |
| **HOLD** | 브리지 > `bridge_max_s` 또는 σ_z > 한계 | **밸브 중립(블레이드 현재고 유지)**, 추종 중단 |
| **STOP** | 장기 끊김(>2×max) | **`control_enabled=False`, gain=0** → 상위 제어가 밸브 중립/estop |
| (복귀) | Fix 재획득 | TRACK 재개 (STOP 해제는 Fix 필요) |

- **VRS 완전 두절**: BRIDGE(10~20s cm급 유지) → 한계 후 HOLD(블레이드 정지, 땅 파먹기 방지)
  → 더 길어지면 STOP(유압 차단). **밸브를 갑자기 끊지 않고 단계적으로 안전하게 멈춤.**
- **Float 진입**: σ_z 자동 팽창 → EKF가 GNSS 비중 낮추고 IMU 우위(부드러운 저하).
- **위성 급감(<30)**: `sat_factor` 로 R 팽창 → 동일하게 보수적 동작.
- **상위 연계**: `control_enabled=False` 면 `LevelerSafetyMonitor`/`proportional_valve` 가
  Direction.NEUTRAL 출력(기존 안전로직과 합류).

---

## 통합 (leveler_core, opt-in)

`LevelerSystem.attach_z_stabilizer()` 한 줄로 부착 — 미부착 시 기존 동작 그대로(하위호환).
```python
sys = LevelerSystem(params, output)          # 기존 그대로
sys.attach_z_stabilizer()                    # ← Z 안정화 계층 부착(opt-in)
sys.set_auto(True)
# 루프(20Hz):
sys.on_gnss(nmea_line, now)                  # GGA/GSV/GST 자동 전달
sys.on_imu(roll, pitch, ax, ay, az, now)     # accel 주면 수직퓨전 사용
st = sys.control_step(now)                   # st["z_state"]/["z_sigma_cm"]/["deadband_cm"]…
```
부착 시 `control_step` 가: ① 안정화 blade_z 로 제어 ② **RTK 품질 게이팅 권한을 z_stab FSM 로**
(끊김 시 안전모니터의 RTK_LOST 를 오버라이드해 브리지 제어 유지, ESTOP/MANUAL/LIMIT 은 유지)
③ 품질 적응형 데드밴드 적용 ④ HOLD/STOP 시 밸브 NEUTRAL.

## 검증 (SITL, `python z_stabilizer.py`)
1. 고도각 마스크가 저고도 위성 제외 + 구성별 집계 ✓
2. Fix 추종: 6Hz 디젤진동 + ±1.5cm 잡음 속 **Z RMS 0.82cm** ✓
3. 틸트 30° 삼각보정 반영 ✓
4. VRS 끊김 → BRIDGE(드리프트 0cm/15s) → HOLD → Fix복귀 TRACK ✓
5. STOP 시 control_enabled=False·gain=0 ✓
6. leveler_core 통합 — 안정화 Z 가 컨트롤러 구동, 끊김 시 safety=SAFE 오버라이드(브리지 유지), 한계 후 NEUTRAL ✓
