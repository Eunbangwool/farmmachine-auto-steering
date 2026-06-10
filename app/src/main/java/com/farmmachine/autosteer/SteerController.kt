package com.farmmachine.autosteer

import com.farmmachine.autosteer.py.PythonEngine

/** Compose UI ↔ Python(app_main) 얇은 브리지. 모든 호출은 예외 안전. */
object SteerController {
    private fun api() = PythonEngine.appMain()

    fun statusJson(): String =
        try { api().callAttr("status_json").toString() } catch (e: Throwable) { "{}" }

    fun engage(): Boolean =
        try { api().callAttr("engage").toBoolean() } catch (e: Throwable) { false }

    fun disengage() = safe { api().callAttr("disengage") }
    fun estop() = safe { api().callAttr("estop") }

    /** 모터 점검 조그: permille(±, 0=정지). bridge 모드에서만 실동작. */
    fun motorJog(permille: Int): String =
        try { api().callAttr("motor_jog", permille).toString() } catch (e: Throwable) { "error" }

    /** 모터 중앙(직진) 캘리브레이션 — 현재 누적각을 0 기준으로. */
    fun motorCenter(): String =
        try { api().callAttr("motor_center").toString() } catch (e: Throwable) { "error" }

    /** 경로 넛지(좌+/우- cm), 섹션 수, 휠베이스(m). */
    fun nudge(cm: Int): String =
        try { api().callAttr("nudge", cm).toString() } catch (e: Throwable) { "error" }
    fun setSectionCount(n: Int): String =
        try { api().callAttr("set_section_count", n).toString() } catch (e: Throwable) { "" }
    fun setWheelbase(m: Double): String =
        try { api().callAttr("set_wheelbase", m).toString() } catch (e: Throwable) { "error" }

    /** ver1 헤딩 바이어스 캘리브(직선 ~20m). */
    fun startHeadingCalib(): String =
        try { api().callAttr("start_heading_calib").toString() } catch (e: Throwable) { "error" }
    fun headingCalibStatus(): String =
        try { api().callAttr("heading_calib_status").toString() } catch (e: Throwable) { "{}" }

    /** 듀얼안테나 base/rover·부호 진단(직선 ~15m 주행). */
    fun startMountDiag(): String =
        try { api().callAttr("start_mount_diag").toString() } catch (e: Throwable) { "error" }
    fun mountDiagStatus(): String =
        try { api().callAttr("mount_diag_status").toString() } catch (e: Throwable) { "{}" }

    /** GNSS 1단계(AGMO ver1 내부 UART): 전원ON → 포트탐지 → 무빙베이스설정 → 시작. */
    fun gnssPowerOn(): String =
        try { api().callAttr("gnss_power_on").toString() } catch (e: Throwable) { "error" }
    fun scanGnss(window: Double): String =
        try { api().callAttr("scan_gnss", window).toString() } catch (e: Throwable) { "{}" }
    fun configureMovingBase(port: String, baud: Int): String =
        try { api().callAttr("configure_moving_base", port, baud).toString() } catch (e: Throwable) { "error" }
    fun startGnss(port: String, baud: Int): String =
        try { api().callAttr("start_gnss", port, baud).toString() } catch (e: Throwable) { "error" }
    fun setDeadman(pressed: Boolean) = safe { api().callAttr("set_deadman", pressed) }
    fun setProfile(name: String) = safe { api().callAttr("set_profile", name) }

    /** 제조사 선택화면용 목록 JSON. */
    fun listVendors(): String =
        try { api().callAttr("list_vendors").toString() } catch (e: Throwable) { "[]" }

    /** 제조사 선택 → 모터 CAN/GNSS/알고리즘 활성화. */
    fun setVendor(key: String): String =
        try { api().callAttr("set_vendor", key).toString() } catch (e: Throwable) { "" }

    /** NTRIP(RTK 보정신호) 접속/해제/상태. */
    fun ntripConnect(host: String, port: Int, mount: String, user: String, pw: String): String =
        try { api().callAttr("ntrip_connect", host, port, mount, user, pw).toString() } catch (e: Throwable) { "error" }
    fun ntripDisconnect(): String =
        try { api().callAttr("ntrip_disconnect").toString() } catch (e: Throwable) { "error" }
    fun ntripStatus(): String =
        try { api().callAttr("ntrip_status").toString() } catch (e: Throwable) { "{}" }

    fun setAbLine(ax: Double, ay: Double, bx: Double, by: Double,
                  width: Double, passes: Int, speed: Double) = safe {
        api().callAttr("set_ab_line", ax, ay, bx, by, width, passes, speed)
    }

    fun setDemoAbLine() = safe {
        // 데모용 AB 라인 (현장에선 field_config/tractor.json + 실제 경로로 대체)
        api().callAttr("set_ab_line", 0.0, 0.0, 0.0, 40.0, 3.0, 4, 1.2)
    }

    /** ⑥ 현장 AB 라인: 현재 위치를 A('a')/B('b') 로 마킹 → 평행 패스 생성. */
    fun markAb(which: String): String =
        try { api().callAttr("mark_ab", which).toString() } catch (e: Throwable) { "error" }
    fun buildAb(width: Double, passes: Int, speed: Double): String =
        try { api().callAttr("build_ab", width, passes, speed).toString() } catch (e: Throwable) { "error" }
    fun abStatus(): String =
        try { api().callAttr("ab_status").toString() } catch (e: Throwable) { "{}" }

    private inline fun safe(block: () -> Unit) {
        try { block() } catch (_: Throwable) {}
    }
}
