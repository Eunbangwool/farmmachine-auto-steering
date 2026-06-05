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
    fun setDeadman(pressed: Boolean) = safe { api().callAttr("set_deadman", pressed) }
    fun setProfile(name: String) = safe { api().callAttr("set_profile", name) }

    fun setAbLine(ax: Double, ay: Double, bx: Double, by: Double,
                  width: Double, passes: Int, speed: Double) = safe {
        api().callAttr("set_ab_line", ax, ay, bx, by, width, passes, speed)
    }

    fun setDemoAbLine() = safe {
        // 데모용 AB 라인 (현장에선 field_config/tractor.json + 실제 경로로 대체)
        api().callAttr("set_ab_line", 0.0, 0.0, 0.0, 40.0, 3.0, 4, 1.2)
    }

    private inline fun safe(block: () -> Unit) {
        try { block() } catch (_: Throwable) {}
    }
}
