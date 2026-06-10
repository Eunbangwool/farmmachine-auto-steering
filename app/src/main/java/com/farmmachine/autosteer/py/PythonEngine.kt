package com.farmmachine.autosteer.py

import android.content.Context
import com.chaquo.python.PyObject
import com.chaquo.python.Python
import com.chaquo.python.android.AndroidPlatform

/** Chaquopy 부팅 + app_main 모듈 접근. */
object PythonEngine {
    @Volatile private var booted = false

    fun ensureStarted(context: Context) {
        if (!Python.isStarted()) {
            Python.start(AndroidPlatform(context.applicationContext))
        }
    }

    fun appMain(): PyObject = Python.getInstance().getModule("app_main")

    /** 앱 시작 시 1회: Python 자율조향 컨트롤러 기동(50Hz 루프 시작). */
    @Synchronized
    fun boot(context: Context, backend: String = "bridge") {
        ensureStarted(context)
        if (!booted) {
            // 2번째 인자 = config_dir(filesDir): 차량 변수(tractor_params.json) 영속화 경로.
            appMain().callAttr("boot", backend,
                context.applicationContext.filesDir.absolutePath)
            booted = true
        }
    }

    fun shutdown() {
        if (booted) {
            try { appMain().callAttr("shutdown") } catch (_: Throwable) {}
            booted = false
        }
    }
}
