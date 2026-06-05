# ApolloCanBus 사용 가이드

## 확정된 하드웨어 정보

```
네이티브 라이브러리: /system/lib64/libsysmcu.so
JNI 클래스: com.van.jni.VanMcu (libsysmcu.so)
CAN 수신: 콜백 방식 (setOnCanListener + setCallback(true))
GPIO: OutputSet(pin, value) — 레이저 커넥터 직접 제어 가능
패키지: com.van.jni (반드시 이 패키지명 유지)
```

## 파일 구조

```
app/src/main/java/
├── com/van/jni/
│   └── VanMcu.kt          ← libsysmcu.so JNI 래퍼 (패키지명 변경 금지)
└── com/farmmachine/can/
    └── ApolloCanBus.kt    ← CanBus 구현 (자율조향+레벨러 공용)
```

## 기본 사용

```kotlin
// 시작
val can = ApolloCanBus(channel = 0, bitrate = 250_000)
can.start()

// CAN 송신
can.send(0x201, byteArrayOf(0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00))

// CAN 수신 (콜백)
can.setReceiveListener { id, data ->
    when (id) {
        0x301 -> handleAngleSensor(data)  // 앵글센서
        0x401 -> handleMotorStatus(data)  // 모터 상태
    }
}

// 정지
can.stop()
```

## 레이저 커넥터 GPIO

```kotlin
// ★ 핀 번호는 MEASUREMENT_CHECKLIST.md A4,A5 실측 후 채울 것
LaserConnectorGpio.configure(up = 2, down = 3, highActive = true)
LaserConnectorGpio.up()   // 블레이드 상승
LaserConnectorGpio.down() // 블레이드 하강
LaserConnectorGpio.hold() // 현 위치 유지
```

## 자율조향 + 레벨러 동시 사용

```kotlin
// Application.onCreate()
ApolloCanManager.startAll()

// 자율조향: steerBus
ApolloCanManager.steerBus.send(motorCmdId, motorData)

// 레벨러: LaserConnectorGpio (CAN 아닌 GPIO)
LaserConnectorGpio.up()
```

## ★ 아직 확인 필요

1. CAN 비트레이트 — 250kbps vs 500kbps
   ```bash
   # AGMO 앱 실행 중 트래픽 캡처로 확인
   adb shell "cat /dev/ttyWK1 | xxd | head -20"
   ```

2. 모터 CAN ID — RTK Fix 후 조향 동작 캡처
   ```kotlin
   can.setReceiveListener { id, data ->
       Log.d("CAN", "ID=0x${id.toString(16)} data=${data.toHex()}")
   }
   ```

3. 레이저 커넥터 핀 번호 — MEASUREMENT_CHECKLIST.md A항목 실측
   ```kotlin
   // 핀 탐색 테스트
   for (pin in 0..15) {
       VanMcu.OutputSet(pin, 1)
       delay(500)
       VanMcu.OutputSet(pin, 0)
   }
   ```

## VanMcu 추가 유용 정보

```kotlin
// 시동 상태 확인
if (VanMcu.isAccOn) { ... }

// 전압/온도
val v = VanMcu.voltage          // 예: 12.6 볼트
val t = VanMcu.temperatureCelsius // 예: 45.3 섭씨

// 앵글센서 수신 필터 (불필요한 ID 차단)
can.addFilter(id = 0x301, mask = 0x7FF)
```
