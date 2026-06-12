// FarmmachineAutoSteer — 자율조향 Android 앱 (farm-work-manager 와 별개 앱)
// 알고리즘은 auto-steering/src 의 Python 을 Chaquopy 로 임베드해 그대로 실행.
// 툴체인은 Chaquopy 16 호환 스택으로 고정 (AGP 8.6 / Gradle 8.9 / Kotlin 2.0).
plugins {
    id("com.android.application") version "8.6.1" apply false
    id("org.jetbrains.kotlin.android") version "2.0.21" apply false
    id("org.jetbrains.kotlin.plugin.compose") version "2.0.21" apply false
    id("com.chaquo.python") version "16.0.0" apply false
}
