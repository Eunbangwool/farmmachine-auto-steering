plugins {
    id("com.android.application")
    id("org.jetbrains.kotlin.android")
    id("org.jetbrains.kotlin.plugin.compose")
    id("com.chaquo.python")
}

android {
    namespace = "com.farmmachine.autosteer"
    compileSdk = 35

    defaultConfig {
        applicationId = "com.farmmachine.autosteer"
        minSdk = 24                 // Apollo 10 Pro = Android 9 (API 28)
        targetSdk = 35
        versionCode = (project.findProperty("versionCodeOverride") as? String)?.toIntOrNull() ?: 1
        versionName = "0.1"

        // Apollo 10 Pro ABI. Chaquopy 가 이 ABI 용 Python + numpy 를 번들.
        ndk { abiFilters += listOf("arm64-v8a", "armeabi-v7a") }
    }

    // 자율조향 Python 소스 = 상위 auto-steering/src (복사 없이 직접 참조)
    sourceSets {
        getByName("main") {
            python { srcDir("../auto-steering/src") }
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
    buildFeatures { compose = true }
}

chaquopy {
    defaultConfig {
        version = "3.11"
        // EKF(numpy) 필요. pyserial 은 USB-serial 브릿지 붙일 때 사용.
        pip { install("numpy") }
        // Chaquopy 진입점은 앱에서 Python.getModule("app_main") 으로 직접 호출.
    }
}

dependencies {
    val composeBom = platform("androidx.compose:compose-bom:2024.09.03")
    implementation(composeBom)
    implementation("androidx.core:core-ktx:1.13.1")
    implementation("androidx.lifecycle:lifecycle-runtime-ktx:2.8.6")
    implementation("androidx.activity:activity-compose:1.9.2")
    implementation("androidx.compose.ui:ui")
    implementation("androidx.compose.ui:ui-graphics")
    implementation("androidx.compose.ui:ui-tooling-preview")
    implementation("androidx.compose.material3:material3")
    debugImplementation("androidx.compose.ui:ui-tooling")
}
