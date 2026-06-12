// FarmmachineAutoSteer — 자율조향 Android 앱 (farm-work-manager 와 별개 앱)
// 알고리즘은 auto-steering/src 의 Python 을 Chaquopy 로 임베드해 그대로 실행.
// ★ 툴체인 = Chaquopy 15.0.1 호환 스택 (AGP 8.2 / Gradle 8.2 / Kotlin 2.0 / compileSdk 34).
//   이유: 실기기 = Apollo 6.0.1(API 23). Chaquopy 16 은 minSdk 24 강제(빌드 거부) →
//   minSdk 21 지원하는 마지막 버전 15.0.1 로 내림. 15.0.1 지원 AGP=8.1~8.2 → AGP/Gradle/
//   compileSdk 동반 하향. Compose 는 미사용이라 제거(코드는 WebView 단일 화면).
plugins {
    id("com.android.application") version "8.2.2" apply false
    id("org.jetbrains.kotlin.android") version "2.0.21" apply false
    id("com.chaquo.python") version "15.0.1" apply false
}
