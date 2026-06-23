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
    @JavascriptInterface fun setAlgorithm(name: String): String = SteerController.setAlgorithm(name)
    @JavascriptInterface fun setDeadman(pressed: Boolean) = SteerController.setDeadman(pressed)
    @JavascriptInterface fun listVendors(): String = SteerController.listVendors()
    @JavascriptInterface fun setVendor(key: String): String = SteerController.setVendor(key)
    @JavascriptInterface fun motorJog(permille: Int): String = SteerController.motorJog(permille)
    @JavascriptInterface fun motorCenter(): String = SteerController.motorCenter()
    @JavascriptInterface fun nudge(cm: Int): String = SteerController.nudge(cm)
    @JavascriptInterface fun setSectionCount(n: Int): String = SteerController.setSectionCount(n)
    @JavascriptInterface fun setWheelbase(m: Double): String = SteerController.setWheelbase(m)
    /** 차량 변수(실측값) 조회/입력. */
    @JavascriptInterface fun getVehicleParams(): String = SteerController.getVehicleParams()
    @JavascriptInterface fun setVehicleParams(wheelbase: Double, antennaHeight: Double,
                                              antennaToAxle: Double, antennaToImpl: Double,
                                              workWidth: Double): String =
        SteerController.setVehicleParams(wheelbase, antennaHeight, antennaToAxle, antennaToImpl, workWidth)
    @JavascriptInterface fun startHeadingCalib(): String = SteerController.startHeadingCalib()
    @JavascriptInterface fun startHeadingCalibDrive(): String = SteerController.startHeadingCalibDrive()
    @JavascriptInterface fun headingCalibStatus(): String = SteerController.headingCalibStatus()
    @JavascriptInterface fun startImuCalib(): String = SteerController.startImuCalib()
    @JavascriptInterface fun imuCalibStatus(): String = SteerController.imuCalibStatus()
    @JavascriptInterface fun startSteerRatioCalib(): String = SteerController.startSteerRatioCalib()
    @JavascriptInterface fun steerRatioCalibStatus(): String = SteerController.steerRatioCalibStatus()
    @JavascriptInterface fun startMountDiag(): String = SteerController.startMountDiag()
    @JavascriptInterface fun mountDiagStatus(): String = SteerController.mountDiagStatus()
    @JavascriptInterface fun gnssPowerOn(): String = SteerController.gnssPowerOn()
    @JavascriptInterface fun scanGnss(window: Double): String = SteerController.scanGnss(window)
    @JavascriptInterface fun configureMovingBase(port: String, baud: Int): String = SteerController.configureMovingBase(port, baud)
    @JavascriptInterface fun startGnss(port: String, baud: Int): String = SteerController.startGnss(port, baud)
    // 비동기(블로킹 방지) — UI 는 gnssJobStatus() 폴링. 포트탐지가 수십 초라 동기 호출은 화면을 멈춤.
    @JavascriptInterface fun scanGnssAsync(window: Double): String = SteerController.scanGnssAsync(window)
    @JavascriptInterface fun configureMovingBaseAsync(port: String, baud: Int): String = SteerController.configureMovingBaseAsync(port, baud)
    @JavascriptInterface fun startGnssAsync(port: String, baud: Int): String = SteerController.startGnssAsync(port, baud)
    @JavascriptInterface fun gnssJobStatus(): String = SteerController.gnssJobStatus()

    /** CAN 하드웨어 상태 (모터 점검 화면 표시용). 활성 브리지(apollo/cpdevice)별로 반환. */
    @JavascriptInterface fun canStatus(): String {
        if (com.farmmachine.autosteer.can.CanBridgeHost.kind == "cpdevice") {
            // Ver2: BnMcuCanService binder 골격(전송 마샬링 TODO). bridge="cpdevice" 명시.
            val c = com.farmmachine.autosteer.can.CpdeviceCanBridge
            return """{"vanmcu":false,"bridge":"cpdevice","binderReady":${c.binderReady},"canOpened":${c.canOpened},"channel":${c.channel},"baud":${c.baud},"connected":${c.clientConnected},"txCount":${c.txCount},"lastTxOk":${c.lastTxOk},"rxCount":${c.rxCount},"lastError":"${c.lastError.replace("\"","'")}"}"""
        }
        val vm = com.van.jni.VanMcu.available
        val b = com.farmmachine.autosteer.can.ApolloCanBridge
        return """{"vanmcu":$vm,"bridge":"apollo","canReady":${b.canReady},"connected":${b.clientConnected},"txCount":${b.txCount},"lastTxOk":${b.lastTxOk},"rxCount":${b.rxCount},"rxEnabled":${b.rxEnabled}}"""
    }

    /** 벤더별 CAN 브리지 선택: "cpdevice"(agmo_single) / "apollo"(그 외, 기본). 같은 TCP 포트 재바인딩. */
    /** VWorld 실지도 API 키(BuildConfig). 빈 문자열이면 UI 가 '키 미설정' 폴백. */
    @JavascriptInterface fun mapApiKey(): String = BuildConfig.VWORLD_API_KEY

    @JavascriptInterface fun selectCanBridge(kind: String): String {
        com.farmmachine.autosteer.can.CanBridgeHost.select(kind)
        return """{"bridge":"${com.farmmachine.autosteer.can.CanBridgeHost.kind}"}"""
    }

    /** Ver2 cpdevice TX 수동 활성/비활성(기본 OFF=RX 검증 우선, 모터 자동송신 금지). */
    @JavascriptInterface fun cpdevTxEnable(on: Boolean): String {
        com.farmmachine.autosteer.can.CpdeviceCanBridge.txEnabled = on
        return """{"txEnabled":$on}"""
    }

    /** Ver2 cpdevice 관찰전용 모드(기본 ON): ON=binder+RX 콜백만, 제어성 호출(TX/baudrate) 금지. */
    @JavascriptInterface fun cpdevObserveOnly(on: Boolean): String {
        com.farmmachine.autosteer.can.CpdeviceCanBridge.observeOnly = on
        return """{"observeOnly":$on}"""
    }

    /** RX 콜백 등록(code 16) 수동 시도 — 기본 OFF(code1 이 서비스를 죽였으므로 TX 와 분리). */
    @JavascriptInterface fun cpdevRegisterRx(on: Boolean): String {
        com.farmmachine.autosteer.can.CpdeviceCanBridge.registerRx = on
        val r = if (on) (com.farmmachine.autosteer.can.CpdeviceCanBridge.instance?.registerRxNow() ?: "no-instance") else "off"
        return """{"registerRx":$on,"result":"$r"}"""
    }

    /** Ver2 CAN 개통: setCANBaudrate(code3, 현재 ch/baud) 재실행 + registerCallback(code1). ch/baud 바꾼 뒤 호출. */
    @JavascriptInterface fun cpdevOpenCan(): String {
        val c = com.farmmachine.autosteer.can.CpdeviceCanBridge
        val ok = c.instance?.openCan() ?: false
        if (ok && c.registerRx) c.instance?.registerRxNow()
        return """{"opened":$ok,"channel":${c.channel},"baud":${c.baud}}"""
    }
    /** Ver2 baud 설정(무반응 시 250000→500000 등). baud2 동일값. 적용은 cpdevOpenCan() 호출 시. */
    @JavascriptInterface fun cpdevSetBaud(b: Int): String {
        com.farmmachine.autosteer.can.CpdeviceCanBridge.baud = b
        com.farmmachine.autosteer.can.CpdeviceCanBridge.baud2 = b
        return """{"baud":$b}"""
    }
    /** (구) 개통 code 탐색 스윕 — 프로토콜 확정으로 불필요. 유지(진단 백업). */
    @JavascriptInterface fun cpdevOpenSweep(): String {
        val r = com.farmmachine.autosteer.can.CpdeviceCanBridge.instance?.openCanSweep() ?: "no-instance"
        return """{"sweep":"$r"}"""
    }

    /** Ver2 수동 단발 TX 테스트: kind=hb/neutral/plus/minus/enable/disable. 1버튼=1프레임. */
    @JavascriptInterface fun cpdevTxTest(kind: String): String {
        val fr = SteerController.cpdevTestFrame(kind)   // {"id":int,"data":"hex"}
        return try {
            val o = org.json.JSONObject(fr)
            if (o.has("id")) {
                val id = o.getInt("id")
                val hex = o.getString("data")
                val data = ByteArray(hex.length / 2) { ((Character.digit(hex[it*2],16) shl 4) or Character.digit(hex[it*2+1],16)).toByte() }
                val ok = com.farmmachine.autosteer.can.CpdeviceCanBridge.instance?.txTestFrame(id, data) ?: false
                """{"kind":"$kind","sent":$ok}"""
            } else fr
        } catch (e: Throwable) { """{"error":"${e.message}"}""" }
    }

    /** Ver2 채널(byte0) 설정 — 실차 0/1/2 스윕. */
    @JavascriptInterface fun cpdevSetChannel(ch: Int): String {
        com.farmmachine.autosteer.can.CpdeviceCanBridge.channel = ch
        return """{"channel":$ch}"""
    }
    /** Ver2 ext 플래그 — 이제 ID워드 인코딩(((id<<3)|0x04))에 고정 반영되어 별도 설정 불필요(no-op, 호환 유지). */
    @JavascriptInterface fun cpdevSetExtFlag(flag: Int): String {
        return """{"extFlag":"fixed-in-id-word","ignored":$flag}"""
    }
    /** Ver2 버스트 TX: kind(plus/minus) 를 ms 동안 50ms 재전송(워치독 회피) 후 자동 정지. */
    @JavascriptInterface fun cpdevTxBurst(kind: String, ms: Int): String {
        return try {
            val sp = org.json.JSONObject(SteerController.cpdevTestFrame(kind))
            val en = org.json.JSONObject(SteerController.cpdevTestFrame("enable"))
            val di = org.json.JSONObject(SteerController.cpdevTestFrame("disable"))
            if (!sp.has("id")) return SteerController.cpdevTestFrame(kind)
            fun hx(s: String) = ByteArray(s.length / 2) { ((Character.digit(s[it*2],16) shl 4) or Character.digit(s[it*2+1],16)).toByte() }
            com.farmmachine.autosteer.can.CpdeviceCanBridge.instance?.txBurst(
                sp.getInt("id"), hx(sp.getString("data")), hx(en.getString("data")), hx(di.getString("data")), ms)
            """{"kind":"$kind","burst_ms":$ms}"""
        } catch (e: Throwable) { """{"error":"${e.message}"}""" }
    }

    /** Ver2 비상정지: 버스트 중단 + TX 차단 + 모터 disable 프레임 1회. */
    @JavascriptInterface fun cpdevEstop(): String {
        com.farmmachine.autosteer.can.CpdeviceCanBridge.instance?.stopBurst()
        com.farmmachine.autosteer.can.CpdeviceCanBridge.txEnabled = false
        return cpdevTxTest("disable")
    }

    /** 현장 진단: CAN 수신(RX) on/off — 모터 회전이 RX 와 충돌하는지 1회 검증. 기본 OFF(TX전용). */
    @JavascriptInterface fun setCanRx(on: Boolean): String {
        val ok = com.farmmachine.autosteer.can.ApolloCanBridge.instance?.setRx(on) ?: false
        return """{"rxEnabled":$ok}"""
    }

    /** NTRIP(RTK 보정신호) 접속/해제/상태. */
    @JavascriptInterface fun ntripConnect(host: String, port: Int, mount: String, user: String, pw: String): String =
        SteerController.ntripConnect(host, port, mount, user, pw)
    @JavascriptInterface fun ntripDisconnect(): String = SteerController.ntripDisconnect()
    @JavascriptInterface fun ntripStatus(): String = SteerController.ntripStatus()

    /** CAN 스니핑(Listen-Only) — N초 캡처 → 수신 CAN ID 빈도 JSON. 미확정 벤더 모터 ID 추적용. */
    @JavascriptInterface fun canSniff(seconds: Double): String = SteerController.canSniff(seconds)

    /** 작업기(균평기) 안테나 GNSS + 레벨 히트맵 (차체 주행 GNSS 와 독립, 벤더 무관). */
    @JavascriptInterface fun startImplementGnss(port: String): String = SteerController.startImplementGnss(port)
    @JavascriptInterface fun getImplementGnssStatus(): String = SteerController.getImplementGnssStatus()
    @JavascriptInterface fun getLevelerGrid(): String = SteerController.getLevelerGrid()
    @JavascriptInterface fun setLevelerReference(): String = SteerController.setLevelerReference()
    @JavascriptInterface fun clearLevelerGrid(): String = SteerController.clearLevelerGrid()

    /** 현장 진단: CAN 채널/비트레이트/확장ID 강제 전환. ch=0/1, br=250000/500000, eff=확장강제 */
    @JavascriptInterface fun setCanParams(ch: Int, br: Int, eff: Boolean): String {
        val ok = com.farmmachine.autosteer.can.ApolloCanBridge.instance?.reconfigure(ch, br, eff) ?: false
        return """{"ch":$ch,"br":$br,"eff":$eff,"canReady":$ok}"""
    }

    @JavascriptInterface
    fun setAbLine(ax: Double, ay: Double, bx: Double, by: Double,
                  width: Double, passes: Int, speed: Double) =
        SteerController.setAbLine(ax, ay, bx, by, width, passes, speed)

    @JavascriptInterface fun setDemoAbLine() = SteerController.setDemoAbLine()

    /** ⑥ 현장 AB 라인: 현재 위치 마킹(A/B) → 평행 패스 생성. */
    @JavascriptInterface fun markAb(which: String): String = SteerController.markAb(which)
    @JavascriptInterface fun buildAb(width: Double, passes: Int, speed: Double): String =
        SteerController.buildAb(width, passes, speed)
    @JavascriptInterface fun abStatus(): String = SteerController.abStatus()
}
