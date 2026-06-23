import java.util.Properties

plugins {
    id("com.android.application")
    id("org.jetbrains.kotlin.android")
    id("com.chaquo.python")
}

// VWorld 실지도 API 키 — 농작이(farm-work-manager)와 동일 주입: local.properties(vworld.api.key) 또는 env(VWORLD_API_KEY).
//   시크릿이라 저장소엔 없음. 미설정이면 빈 문자열 → UI 가 '지도 키 미설정' 안내로 폴백.
val vworldApiKey: String = run {
    val props = Properties()
    val lp = rootProject.file("local.properties")
    if (lp.exists()) lp.inputStream().use { props.load(it) }
    props.getProperty("vworld.api.key") ?: System.getenv("VWORLD_API_KEY") ?: ""
}

android {
    namespace = "com.farmmachine.autosteer"
    compileSdk = 34             // AGP 8.2 상한(35 는 AGP 8.6+ 필요). Chaquopy 15.0.1 스택과 정합

    defaultConfig {
        applicationId = "com.farmmachine.autosteer"
        minSdk = 23                 // Apollo 10 Pro 변종 = Android 6.0.1(API 23, armeabi-v7a). 실기기 getprop 확인
        targetSdk = 34
        versionCode = (project.findProperty("versionCodeOverride") as? String)?.toIntOrNull() ?: 1
        versionName = "0.1"

        // Apollo 10 Pro ABI. Chaquopy 가 이 ABI 용 Python + numpy 를 번들.
        ndk { abiFilters += listOf("arm64-v8a", "armeabi-v7a") }

        buildConfigField("String", "VWORLD_API_KEY", "\"$vworldApiKey\"")
    }

    buildFeatures { buildConfig = true }   // BuildConfig.VWORLD_API_KEY 노출

    // 고정 debug 키스토어로 서명 → CI 빌드마다 서명이 동일 → 덮어쓰기 설치 가능.
    // (기본값은 러너마다 새로 생성되는 ~/.android/debug.keystore 라 서명 충돌 발생)
    signingConfigs {
        getByName("debug") {
            storeFile = file("debug.keystore")
            storePassword = "android"
            keyAlias = "androiddebugkey"
            keyPassword = "android"
        }
    }

    buildTypes {
        release {
            isMinifyEnabled = false
            proguardFiles(
                getDefaultProguardFile("proguard-android-optimize.txt"),
                "proguard-rules.pro"
            )
        }
    }
    compileOptions {
        sourceCompatibility = JavaVersion.VERSION_17
        targetCompatibility = JavaVersion.VERSION_17
    }
    kotlinOptions { jvmTarget = "17" }
    // Compose 미사용(운영 UI 는 WebView+assets HTML). Chaquopy 15.0.1 다운그레이드 시 제거.
}

chaquopy {
    defaultConfig {
        version = "3.11"
        // EKF(numpy) 필요. pyserial = GNSS 시리얼(내부 UART/USB) 수신 필수
        //   — 빠지면 scan/configure/start 가 전부 "no-pyserial" 로 실패해 GNSS 불능.
        pip {
            install("numpy")
            install("pyserial")
        }
        // Chaquopy 진입점은 앱에서 Python.getModule("app_main") 으로 직접 호출.
    }
    // 자율조향 Python 소스 = 상위 auto-steering/src (복사 없이 직접 참조).
    // Kotlin DSL 에선 android.sourceSets 가 아니라 chaquopy.sourceSets 로 지정.
    sourceSets {
        getByName("main") {
            srcDir("../auto-steering/src")
        }
    }
}

dependencies {
    // Compose 제거 — UI 는 WebView(MainActivity.setContentView). ComponentActivity 는 activity-ktx 제공.
    implementation("androidx.core:core-ktx:1.13.1")
    implementation("androidx.lifecycle:lifecycle-runtime-ktx:2.8.6")
    implementation("androidx.activity:activity-ktx:1.9.2")
}
