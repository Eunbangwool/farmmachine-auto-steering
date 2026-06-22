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
| 2026-06-16 | AGMO Ver2 추가 + 제조사 4지선다(agmo_dual/agmo_single/chcnav/fjd). agmo_single=싱글+INS·ttyS4·SocketCAN(can1)·can_verified=False(모터 CAN ID 미확정). SocketCAN listen_only+CAN 스니핑(can_sniff) 추가. 기존 agmo→agmo_dual(별칭 유지, 동작 동일). | CLAUDE_CODE_TASK_VER2.md (autokit2 역분석+/proc/fd) |
| 2026-06-16 | 균평기 레벨 히트맵: 작업기 안테나(별도 USB GNSS) 독립 레이어 `implement_gnss.py` + 2D 탑다운 canvas 히트맵(편차 ±cm 색상). 차체 주행 GNSS 와 완전 분리, 4벤더 공통. | CLAUDE_CODE_TASK_LEVELER_UI.md |

## 균평기 레벨 히트맵 (작업기 안테나) — 2026-06-16

★ 균평기 GNSS 안테나 2개 구분: **차체**(위치+주행, 벤더별 포트) / **작업기**(레벨 측정 전용, 별도 USB GNSS). 표고는 **반드시 작업기 안테나**에서만(차체 표고 금지). 벤더 독립 레이어.
- **작업 0 파악**: 기존 필드는 Canvas 아닌 **장식용 SVG 원근 그리드**(`drawField`)뿐 — GNSS 좌표 커버리지/궤적/표고 기록 전무. `parse_gga`도 altitude 미파싱, on_rtk에 alt 없음, status에 원 lat/lon·alt 없음. → 재사용할 좌표변환 없어 **2D 탑다운 전용 canvas 신규**(사용자 결정).
- **`implement_gnss.py`(신규)**: 작업기 USB GNSS 독립 스레드 수신(`/dev/ttyUSB*` 자동탐색=GGA 나오는 포트만 채택→4G 모뎀 배제, baud 후보 순차·TODO). GGA→(lat,lon,**alt**,fix,sats). `LevelerGrid`=ENU 그리드(0.5m) 셀평균→기준면(시작구간 평균 or 영점버튼) 대비 편차 cm. fix 4/5만 누적. self-test 통과.
- **app_main**: start/stop/status·getLevelerGrid·setLevelerReference·clearLevelerGrid·setImplAntennaHeight. 탐지는 백그라운드(UI 프리즈 방지). status에 impl_gnss_ok/fix/port. 차체 루프 무간섭.
- **UI**: 작업화면 "📊 레벨 히트맵" → 2D 탑다운 overlay. 750ms 폴링, 편차 5색(빨강+5↑/주황/초록±2/하늘/파랑-5↓)+범례, 저신뢰(n<3) 반투명·Float 흐림, 미연결 시 "작업기 안테나 미연결"(가짜값 금지). 영점/초기화/안테나높이 버튼. Chrome44(canvas fillRect, ES5).
- ⚠ TODO(HW): 작업기 GNSS baud·안테나높이 실측, 셀 과다 시 컬링.

## AGMO Ver2 (agmo_single) 추가 — 2026-06-16

autokit2.apk 역분석 + /proc/fd 실측(Apollo2_10, Android 11) 기반 신규 벤더.
- **제조사 4지선다**: `agmo_dual`(=기존 agmo, key만 변경·동작 동일) / `agmo_single`(Ver2 신규) / `chcnav` / `fjd`. UI 동적 렌더 + 백엔드 list_vendors 4개.
- **agmo_single**: 싱글안테나+INS(헤딩=속도벡터+자이로 융합), GNSS `/dev/ttyS4` 115200(실패 시 460800 재시도), 모터=표준 SocketCAN(can1/can2). **모터 CAN ID·baudrate 미확정 → can_verified=False(조향 비활성), 스니핑+GNSS만. 추측 송신 금지(TODO).**
- **SocketCAN**: `SocketCanBackend(listen_only=)` 송신 차단 추가. `app_main.can_sniff(sec,ch)` = Listen-Only N초 캡처 → CAN ID 빈도 JSON + `/sdcard/farmmachine/can_sniff_*.txt` 저장. UI "📡 CAN 스니핑" 버튼(agmo_single/chcnav만 노출).
- **제약 준수**: 기존 agmo(dual) 제어 미수정(별칭 `agmo→agmo_dual`), CAN ID/baud 추측 하드코딩 없음(TODO), CAN 실패 graceful(can_state), Chrome44 호환(ES5), autokit2 충돌 경고 로그.
- 검증: vendor_profiles 4벤더 self-test, app_main mock 벤더전환·sniff graceful, test_closed_loop/sitl(6/6)/speed_control, HTML 구문 OK. ⚠ Kotlin 빌드는 CI 검증.

## AGMO Ver2 CAN — CpdeviceCanBridge 골격 (2026-06-22)
실측: Ver2(Apollo2_10)엔 SocketCAN 없음. CAN=spidev2.0→cpdevice MCU, binder `com.cpdevice.BnMcuCanService`(#34) 중계.
- C안 구현(골격): Python BridgeBackend(TCP 13B 계약) **불변**. Kotlin 활성 브리지만 벤더로 분기:
  `CanBridgeHost`(신규)가 같은 포트(47100)에 apollo(ApolloCanBridge/VanMcu, agmo_dual 불변) /
  cpdevice(`CpdeviceCanBridge` 신규)를 선택. UI 벤더선택 시 `JsBridge.selectCanBridge` 호출.
- `CpdeviceCanBridge`: ApolloCanBridge와 동일 TCP 계약 + `ServiceManager.getService`(리플렉션) binder 연결 시도.
  ★ TX(sendCanFrame)/RX(setCanFrameRxCallback)/baudrate **트랜잭션 코드·Parcel 마샬링 미확정 → TODO**
  (추측 송신 금지). 모터 프레임 ID(0x06000001/0x07000001)는 듀얼 CanSpec 그대로.
- 상태: status `can_bridge`(apollo/cpdevice)·`motor_ready`(can_verified=False → 골격단계 비활성)·note.
  canStatus()도 브리지별 JSON. engage 는 motor_verified=false 로 거부 유지.
- 다음(별도): Ghidra 로 BnMcuCanService onTransact 분석 → 마샬링 확정, 또는 libcpcomm.so JNI 번들(라이선스 확인).
