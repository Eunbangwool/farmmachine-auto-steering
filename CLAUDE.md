# farmmachine-auto-steering — CLAUDE.md
> 상세 근거·히스토리는 `docs/CLAUDE_DETAIL.md` (필요할 때만 열 것).
> 이 파일은 매 세션 자동 로드되는 **요약본**: 무엇을 하는 프로젝트인지 + 건드리면 안 되는 것 + 현재 상태.

---

## 프로젝트 한 줄
오너(AGMO 자율주행 전직, Kubota MR1157 농가)가 상용 자율조향(AGMO/CHCNAV) 앱을 **본인 앱으로 교체**.
Apollo 태블릿(Android) 위에서 **Kotlin 셸 + Chaquopy 임베드 Python**으로 조향 알고리즘 구동.
보유 장비: Apollo 10 Pro(CAN 내장), Keya 조향모터, F9P RTK, GNSS 듀얼안테나, LoRa.

## 핵심 아키텍처 결정 (★ 변경 금지)
- **앱 = `com.farmmachine.autosteer`** (farm-work-manager·레벨러와 별개). 진입점 `app_main.py`.
  Kotlin(UI/서비스/CAN 브릿지) + Chaquopy 로 `auto-steering/src` Python 실행. CI: `build-autosteer-apk.yml`.
- **신호 경로**: CAN→조향모터(직접제어) / CAN←모터 하트비트(조향각 피드백, AGMO=WAS 미사용) /
  GNSS(헤딩+위치) / 레벨러 안테나=USB(자율조향 GNSS 아님).
- **실기기 = ApolloPro(Qualcomm)**: GNSS=u-blox 듀얼안테나 NMEA, 포트 `/dev/ttyHSL0`,
  전원 `/dev/gpio_dev` 매직코드(ublox=100008, rs485=100021). **sysfs gpio 아님**(그건 Apollo2 변종).
- **모터 = Keya KY170 (CanSpec 확정)**: 250kbps, 29-bit 확장 ID(TX 0x06000001/RX 0x05800001/HB 0x07000001),
  cmd_speed 속도제어. **부호 규약: +permille=좌회전, −permille=우회전.**
- **추종 알고리즘 기본 = `pure_pursuit`**. `implement_ff`(작업기 곡선보정) 선택형.
  ⚠ `ImplementReferenced` 직접 P제어는 비최소위상으로 발산 → 쓰지 말 것(상세는 DETAIL).
- **헤딩 소스 2종 지원**: ver1 듀얼안테나+IMU(`dual`) / ver2·CHCNAV·FJD INS 스마트안테나(`ins`). F9P단독=none.
- **GNSS 이중화**: PA-3(주) + F9P(백업), `GnssArbiter` 중재.
- **멀티벤더**: `vendor_profiles.py` — agmo(✅ can_verified) / chcnav·fjd(★ 모터 CAN 미확정).
  `can_verified=False` 면 engage 거부(안전장치).
- **운영 UI = AGMO Solution v1.6.7 화면 재현** (`app/src/main/assets/autosteer_ui.html`, WebView+JsBridge).
  레이아웃/흐름 1:1 우선, 색은 후순위. 원본 리소스 복제 금지(아래 clean-room).

## ⚖️ 금지사항 (★ 필수 / 모든 세션)
- **clean-room**: 디컴파일 산출물(apk/dex/smali, findings/, screenshots/)·타사 리소스 원본 **커밋 금지**(.gitignore 차단).
  화면을 **보고** 자체 구현만 OK. 상표(AGMO 등) UI 미사용.
- **코드 이식 규칙**: 기능적 사실(비트레이트/CAN ID/`/dev` 경로/함수 시그니처)만 추출해 **자체 구현**. 소스 복붙 금지.
- **벤더 격리**: 한 벤더(AGMO) 분석 정보를 타사(CHCNAV/FJD) 맥락에 전용하지 말 것.
- **법적 준수**(legal/ 문서 연동): ①소스 비복제 ②UI에 "자율주행 아님, 운전자 상시 감독" 명시(약관 §2)
  ③조향명령·개입 로컬 기록 유지(약관 §8) ④상표 비혼용. 신규 HW 분석 시 `legal/INDEPENDENT_DEVELOPMENT_RECORD.md` §6 갱신.
- public repo. 상업화·배포 전 법률 검토 권고(이 문서는 법률자문 아님).

## 작업 규칙
- **`TOKEN_BUDGET.md` 규칙 준수**: 바뀐 것만·짧게·맞는 도구로. 코드/grep/빌드는 Claude Code에서, 전략·1회성 분석은 웹챗.
- 가격/기능/UX 등 주요 결정은 같은 turn에 `CONTEXT.md` 갱신 + push.
- 개발 브랜치: 지정된 feature 브랜치. PR은 명시 요청 시에만.
- 매직넘버 금지·출처 주석(`VehicleProfile` 모범). 실패는 조용히 먹지 말고 사유 기록(`lastError` 패턴). TODO엔 출처/조건 명시.

## 현재 상태 (✅ 구현 / ★ 현장·HW 의존)
- ✅ CanSpec(Keya) 이식, 무WAS 속도제어, EKF(RTK+IMU+무빙베이스 헤딩 적응형R·게이팅·틸트·COG),
  GNSS 이중화, IMU/헤딩/조향비 캘리브, SITL 폐루프+안전 6/6, 멀티벤더, implement_ff 곡선보정.
- ✅ CAN 배선: `apollo_can.ApolloCanBus`(bridge) → Kotlin `ApolloCanBridge` → `VanMcu`(libsysmcu.so).
- ✅ CHCNAV 역분석 파라미터 프로파일 적용(Kotlin `profile/`·`safety/`·`can/`). 출처 `CHCNAV_PARAM_PROFILE.md`.
- ★ 남은 핵심(HW/현장): wheelbase·antenna_to_axle 실측, 실차 모터 CAN 채널 확정, GNSS 포트 현장확인,
  1km/h 저속 안전검증, CHCNAV/FJD 모터 CAN 프로토콜 입수. (사전준비 도구는 전부 구현: calibration/field_config/can_tools/sitl_sim/tuning)

## 핵심 파일
```
auto-steering/src/autosteer_core.py        — 메인 알고리즘 (Python)
auto-steering/src/app_main.py              — Chaquopy 진입점 / Kotlin 호출표면
app/src/main/assets/autosteer_ui.html      — 운영 UI (WebView, AndroidSteer JsBridge)
app/src/main/java/com/farmmachine/autosteer/{MainActivity,JsBridge,SteerController}.kt
app/src/main/java/com/farmmachine/autosteer/{profile,safety,can}/  — CHCNAV 프로파일·안전·CANopen
```

## 관련 레포
- `Eunbangwool/farm-work-manager`: Android 앱 메인(레벨러 UI 등)
- `Eunbangwool/rtk-lora-bridge`: LoRa NTRIP 기지국(F9P+LoRa, RTCM 필터링)

## 코드 실행
```bash
cd auto-steering/src && pip install numpy && python autosteer_core.py   # MockCAN 즉시 테스트
```
