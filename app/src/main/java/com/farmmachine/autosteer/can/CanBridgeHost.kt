package com.farmmachine.autosteer.can

import android.content.Context
import android.util.Log

/**
 * CAN 브리지 호스트 — localhost TCP 포트(47100) 위 단일 활성 브리지를 벤더에 따라 선택.
 *   - "apollo"   : ApolloCanBridge (VanMcu/libsysmcu) — agmo_dual 등 (검증된 경로, 불변)
 *   - "cpdevice" : CpdeviceCanBridge (BnMcuCanService binder) — agmo_single(Ver2) 골격
 *
 * Python BridgeBackend 는 양쪽 동일(같은 포트/13바이트 레코드 계약) → Python 전송코드 0 변경.
 * vendor 선택 시 UI(JsBridge.selectCanBridge)가 호출 → 활성 브리지만 교체(같은 포트 재바인딩).
 */
object CanBridgeHost {
    private const val PORT = 47100
    private const val TAG = "CanBridgeHost"

    @Volatile var kind = "apollo"
        private set
    private var apollo: ApolloCanBridge? = null
    private var cpdev: CpdeviceCanBridge? = null
    // cpdevice 브리지는 com.android.guard 서비스 bindService 에 Context 필요(앱 컨텍스트 보관).
    @Volatile private var appCtx: Context? = null

    @Synchronized
    fun start(context: Context, initial: String = "apollo") {
        appCtx = context.applicationContext
        select(initial)
    }

    /** 활성 브리지 교체. agmo_dual=apollo(불변), agmo_single=cpdevice. */
    @Synchronized
    fun select(k: String) {
        val want = if (k == "cpdevice") "cpdevice" else "apollo"
        val already = if (want == "apollo") apollo != null else cpdev != null
        if (want == kind && already) return
        // 기존 브리지 중지(포트 해제) 후 원하는 것 기동
        try { apollo?.stop() } catch (_: Throwable) {}
        try { cpdev?.stop() } catch (_: Throwable) {}
        apollo = null; cpdev = null
        if (want == "cpdevice") {
            val ctx = appCtx
            if (ctx == null) { Log.e(TAG, "cpdevice 선택 실패: appCtx 없음(start(context) 먼저 호출 필요)"); return }
            cpdev = CpdeviceCanBridge(ctx, PORT).also { it.start() }
        } else {
            apollo = ApolloCanBridge(port = PORT).also { it.start() }
        }
        kind = want
        Log.i(TAG, "CAN 브리지 = $kind (port $PORT)")
    }

    @Synchronized
    fun stop() {
        try { apollo?.stop() } catch (_: Throwable) {}
        try { cpdev?.stop() } catch (_: Throwable) {}
        apollo = null; cpdev = null
    }
}
