package com.farmmachine.autosteer

import android.webkit.JavascriptInterface

/**
 * WebView(HTML 운영 UI) ↔ Python(app_main) 브리지.
 * HTML/JS 에서 `window.AndroidSteer.<메서드>()` 로 호출한다.
 *
 * 계약(JS API):
 *   AndroidSteer.statusJson(): String   // 상태 JSON (engaged, safety, profile,
 *                                        //   xte_cm, target_angle_deg, measured_angle_deg,
 *                                        //   speed_mps, can_state, can_available, can_tx/rx …)
 *   AndroidSteer.engage(): Boolean
 *   AndroidSteer.disengage()
 *   AndroidSteer.estop()
 *   AndroidSteer.setProfile(name)        // "normal" | "heavy" | "sand"
 *   AndroidSteer.setDeadman(pressed)     // true: 누름 / false: 뗌
 *   AndroidSteer.setAbLine(ax,ay,bx,by,width,passes,speed)
 *   AndroidSteer.setDemoAbLine()
 *   AndroidSteer.listVendors(): String   // 제조사 목록 JSON (시작화면용)
 *   AndroidSteer.setVendor(key): String   // "agmo" | "chcnav" | "fjd"
 *
 * JS 쪽은 setInterval 로 statusJson() 을 ~100ms 폴링해 화면을 갱신하면 된다.
 */
class JsBridge {
    @JavascriptInterface fun statusJson(): String = SteerController.statusJson()
    @JavascriptInterface fun engage(): Boolean = SteerController.engage()
    @JavascriptInterface fun disengage() = SteerController.disengage()
    @JavascriptInterface fun estop() = SteerController.estop()
    @JavascriptInterface fun setProfile(name: String) = SteerController.setProfile(name)
    @JavascriptInterface fun setDeadman(pressed: Boolean) = SteerController.setDeadman(pressed)
    @JavascriptInterface fun listVendors(): String = SteerController.listVendors()
    @JavascriptInterface fun setVendor(key: String): String = SteerController.setVendor(key)
    @JavascriptInterface fun motorJog(permille: Int): String = SteerController.motorJog(permille)

    /** CAN 하드웨어 상태 (모터 점검 화면 표시용). logcat 없이 확인. */
    @JavascriptInterface fun canStatus(): String {
        val vm = com.van.jni.VanMcu.available
        val ready = com.farmmachine.autosteer.can.ApolloCanBridge.canReady
        val conn = com.farmmachine.autosteer.can.ApolloCanBridge.clientConnected
        return """{"vanmcu":$vm,"canReady":$ready,"connected":$conn}"""
    }

    @JavascriptInterface
    fun setAbLine(ax: Double, ay: Double, bx: Double, by: Double,
                  width: Double, passes: Int, speed: Double) =
        SteerController.setAbLine(ax, ay, bx, by, width, passes, speed)

    @JavascriptInterface fun setDemoAbLine() = SteerController.setDemoAbLine()
}
