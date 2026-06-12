# 팜머신 자율조향 Android 앱 (FarmmachineAutoSteer)

farm-work-manager(농작이)와 **완전히 별개의 앱**. Apollo 10 Pro 태블릿에서 자율조향을
구동하는 전용 앱이다.

## 아키텍처 — Kotlin 셸 + Python(Chaquopy) 알고리즘

```
┌─────────────────── APK (com.farmmachine.autosteer) ───────────────────┐
│  MainActivity (Compose UI)  ── 상태폴링/버튼 ──► SteerController        │
│        │ startForegroundService                         │ callAttr     │
│        ▼                                                 ▼              │
│  AutoSteerService (foreground)        PythonEngine (Chaquopy)          │
│   ├─ ApolloCanBridge  ◄─── TCP :47100 ───►  app_main.py                │
│   │   (벤더 CAN SDK)                          └ ApolloCanBus(bridge)    │
│   └─ PythonEngine.boot                          └ AutoSteerSystem(50Hz)│
└────────────────────────────────────────────────────────────────────────┘
```

- **알고리즘은 재구현하지 않는다.** `auto-steering/src` 의 검증된 Python
  (autosteer_core / apollo_can / app_main …)을 Chaquopy 로 그대로 임베드해 실행.
- CAN 은 `ApolloCanBridge`(Kotlin, localhost TCP) ↔ `ApolloCanBus(backend="bridge")`(Python).
  계약: `auto-steering/APOLLO_CAN.md`.
- 센서(GNSS/IMU)가 안 들어오면 SafetyMonitor 가 자동 비활성(안전) 상태 유지.

## 구성 파일

```
settings.gradle.kts / build.gradle.kts / gradle.properties   ← Gradle 루트
gradle/wrapper, gradlew                                       ← Gradle 8.9
app/build.gradle.kts          ← AGP 8.6.1 / Kotlin 2.0.21 / Chaquopy 16 / compose
app/src/main/AndroidManifest.xml
app/src/main/java/com/farmmachine/autosteer/
  MainActivity.kt             ← Compose 운영 UI (상태/프로파일/제어/데드맨)
  AutoSteerService.kt         ← 포그라운드: CAN 브릿지 + Python boot
  SteerController.kt          ← UI ↔ app_main 브리지
  can/ApolloCanBridge.kt      ← CAN TCP 서버 (★ 벤더 SDK 채울 곳)
  py/PythonEngine.kt          ← Chaquopy 부팅/모듈 접근
auto-steering/src/app_main.py ← Python 진입점(Chaquopy가 호출, CPython 검증됨)
```

Chaquopy 가 `auto-steering/src` 를 Python 소스로 번들(`python.srcDir`), numpy pip 설치.

## 빌드

```bash
# 로컬 (Android SDK 필요)
./gradlew :app:assembleDebug
# 산출물: app/build/outputs/apk/debug/app-debug.apk
```
CI: `.github/workflows/build-autosteer-apk.yml` (JDK17 + Python3.11 + Gradle) 가
`claude/**`·`main` push 시 자동 빌드 → APK 아티팩트 업로드.

> ⚠️ 이 환경엔 Android SDK 가 없어 APK 컴파일은 CI/로컬에서 수행한다.
> Python 진입점은 CPython 으로 검증됨: `python auto-steering/src/app_main.py`.

## 운영 UI (WebView + JS 브리지)

운영 화면은 **HTML/JS** 로 만들어 `app/src/main/assets/autosteer_ui.html` 에 두고,
`MainActivity` 가 WebView 로 로드한다. HTML 은 `window.AndroidSteer`(`JsBridge.kt`)로
Python 코어를 호출한다.

```
WebView(autosteer_ui.html)
   │  window.AndroidSteer.*   (JsBridge @JavascriptInterface)
   ▼
SteerController → PythonEngine(Chaquopy) → app_main.py → AutoSteerSystem(50Hz)
```

**JS API 계약** (HTML 에서 호출):
| 호출 | 의미 |
|---|---|
| `AndroidSteer.statusJson()` | 상태 JSON(engaged, safety, profile, xte_cm, target_angle_deg, measured_angle_deg, speed_mps, can_state, can_available, can_tx/rx). ~100ms 폴링 |
| `AndroidSteer.engage()` / `disengage()` / `estop()` | 활성/해제/비상정지 |
| `AndroidSteer.setProfile("normal"\|"heavy"\|"sand")` | 3-모드 |
| `AndroidSteer.setDeadman(true\|false)` | 데드맨 누름/뗌 |
| `AndroidSteer.setAbLine(ax,ay,bx,by,width,passes,speed)` / `setDemoAbLine()` | 경로 설정 |

> **운영 UI HTML 교체**: 채팅 등에서 만든 HTML 로 `assets/autosteer_ui.html` 를 덮어쓰면 끝.
> 브라우저 단독 테스트를 위해 `window.AndroidSteer` 가 없을 때 가드(빈 동작)를 두면 좋다.
> 현재 커밋된 파일은 브리지 동작을 보여주는 **placeholder** 다.

## 남은 하드웨어 작업

1. **`ApolloCanBridge` 벤더 SDK 채우기** — Apollo 10 Pro(CPDEVICE) CAN SDK 의
   open/send/receive 를 `can/ApolloCanBridge.kt` 의 TODO 위치에 연결.
2. **GNSS/IMU 브릿지** — PA-3/F9P(USB-serial)를 읽어 `app_main.on_rtk/on_imu` 호출
   (USB Host API 또는 별도 TCP 브릿지). 현재는 미공급 시 안전상 비활성.
3. **실측/CAN ID** — `field_collect`→`tractor.json` 주입(앱 assets 또는 외부 저장소 로드).
