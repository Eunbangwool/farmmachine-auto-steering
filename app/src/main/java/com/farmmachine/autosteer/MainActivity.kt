package com.farmmachine.autosteer

import android.annotation.SuppressLint
import android.os.Bundle
import android.view.View
import android.webkit.WebView
import androidx.activity.ComponentActivity
import com.farmmachine.autosteer.py.PythonEngine
import kotlin.concurrent.thread

/**
 * 운영 UI 호스트. HTML(assets/autosteer_ui.html)을 WebView 로 띄우고,
 * window.AndroidSteer(JsBridge) 로 Python 자율조향 코어와 연결한다.
 * (HTML 운영 UI 는 채팅에서 생성 → app/src/main/assets/autosteer_ui.html 로 교체)
 */
class MainActivity : ComponentActivity() {

    @SuppressLint("SetJavaScriptEnabled")
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)

        // 풀스크린(태블릿 운영). 화면 항상 켜짐은 manifest+activity 속성으로 처리.
        window.decorView.systemUiVisibility =
            View.SYSTEM_UI_FLAG_FULLSCREEN or View.SYSTEM_UI_FLAG_IMMERSIVE_STICKY or
            View.SYSTEM_UI_FLAG_HIDE_NAVIGATION

        // Chaquopy Python 백그라운드 부팅 — 실패해도 WebView 는 뜬다.
        // 하드웨어(CAN/GNSS) 미연결 동안은 mock 백엔드로 UI 동작 검증.
        thread(name = "py-boot") {
            runCatching { PythonEngine.boot(applicationContext, "mock") }
        }

        val web = WebView(this).apply {
            settings.javaScriptEnabled = true
            settings.domStorageEnabled = true
            settings.mediaPlaybackRequiresUserGesture = false
            addJavascriptInterface(JsBridge(), "AndroidSteer")
            loadUrl("file:///android_asset/autosteer_ui.html")
        }
        setContentView(web)
    }
}
