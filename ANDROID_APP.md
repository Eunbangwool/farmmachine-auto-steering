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

## 남은 하드웨어 작업

1. **`ApolloCanBridge` 벤더 SDK 채우기** — Apollo 10 Pro(CPDEVICE) CAN SDK 의
   open/send/receive 를 `can/ApolloCanBridge.kt` 의 TODO 위치에 연결.
2. **GNSS/IMU 브릿지** — PA-3/F9P(USB-serial)를 읽어 `app_main.on_rtk/on_imu` 호출
   (USB Host API 또는 별도 TCP 브릿지). 현재는 미공급 시 안전상 비활성.
3. **실측/CAN ID** — `field_collect`→`tractor.json` 주입(앱 assets 또는 외부 저장소 로드).
