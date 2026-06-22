package com.farmmachine.autosteer

import android.annotation.SuppressLint
import android.content.Intent
import android.os.Build
import android.os.Bundle
import android.util.Log
import android.view.View
import android.webkit.ConsoleMessage
import android.webkit.WebChromeClient
import android.webkit.WebView
import androidx.activity.ComponentActivity

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

        // 자율조향 포그라운드 서비스 기동 → ApolloCanBridge(libsysmcu.so CAN) +
        // Python(bridge 백엔드) 50Hz 루프. 실패해도 WebView 는 뜬다.
        // (비-Apollo/권한 미설정이면 CAN 은 비활성, UI·탐색은 정상 동작)
        val svc = Intent(this, AutoSteerService::class.java)
        runCatching {
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) startForegroundService(svc)
            else startService(svc)
        }

        // JS 오류/콘솔을 logcat 으로 노출(튕김·화면전환 진단). chrome://inspect 원격 디버깅도 허용.
        WebView.setWebContentsDebuggingEnabled(true)

        val web = WebView(this).apply {
            settings.javaScriptEnabled = true
            settings.domStorageEnabled = true
            settings.mediaPlaybackRequiresUserGesture = false
            // JS console.* / 미처리 오류 → logcat "AutoSteerWeb" (adb logcat -s AutoSteerWeb)
            webChromeClient = object : WebChromeClient() {
                override fun onConsoleMessage(m: ConsoleMessage): Boolean {
                    Log.i("AutoSteerWeb", "${m.messageLevel()} ${m.message()} @${m.sourceId()}:${m.lineNumber()}")
                    return true
                }
            }
            addJavascriptInterface(JsBridge(), "AndroidSteer")
            loadUrl("file:///android_asset/autosteer_ui.html")
        }
        setContentView(web)
    }
}
