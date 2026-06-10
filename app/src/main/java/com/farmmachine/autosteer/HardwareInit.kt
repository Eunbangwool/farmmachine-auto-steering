package com.farmmachine.autosteer

import android.content.Context
import android.content.Intent
import android.util.Log
import com.van.jni.VanMcu

/**
 * Apollo2(RK3568) 하드웨어 전원 인에이블 — GNSS(UM482) + CAN 트랜시버.
 *
 * ★ 디컴파일 사실(AGMO_VER1_FUNCTIONAL_ANALYSIS §4): AGMO 는 GPIO 를 직접 sysfs 로
 *   켜지 않고 **별도 시스템 서비스 `com.van.service` 에 `CAMERGPIOON` 브로드캐스트**를
 *   보내 켠다(시스템 권한). 일반 앱은 `/sys/class/gpio` 직접 쓰기가 막혀(no-gpio) 있으므로
 *   같은 경로를 시도한다:
 *     1) com.van.service 로 전원 브로드캐스트(시스템 서비스가 GPIO 를 켜도록)
 *     2) libsysmcu.so 네이티브 GPIO(OutputSet) — CAN 이 이 .so 로 동작하므로 GPIO 도 시도
 *   둘 다 best-effort. 실패해도 앱은 동작(전원이 이미 켜져 있거나 권한 필요).
 */
object HardwareInit {
    private const val TAG = "HardwareInit"

    // Apollo2 GPIO 번호(디컴파일 사실) — 전원 핀
    private val GNSS_PINS = intArrayOf(137 /*UM482_PWREN*/, 101 /*GNSS_LNA_EN*/, 136 /*GNSS_RST_N*/)
    private val CAN_PINS  = intArrayOf(61 /*CAN_PWR_EN*/, 99 /*CAN0*/, 154 /*CAN1*/, 128 /*CAN2*/)
    private const val RS485_EN = 134

    @Volatile var lastResult = "미실행"; private set

    fun enable(ctx: Context) {
        val log = StringBuilder()
        // 1) AGMO 메커니즘 — com.van.service 로 전원 브로드캐스트
        for (action in arrayOf("CAMERGPIOON", "com.van.service.CAMERGPIOON")) {
            try {
                ctx.sendBroadcast(Intent(action).apply { setPackage("com.van.service") })
                log.append("bcast:$action ")
            } catch (e: Throwable) { Log.w(TAG, "broadcast $action 실패: ${e.message}") }
        }
        // 2) libsysmcu.so 네이티브 GPIO(OutputSet) — CAN 과 같은 .so 경로
        if (VanMcu.available) {
            var ok = 0
            for (p in GNSS_PINS + CAN_PINS + intArrayOf(RS485_EN)) {
                try { if (VanMcu.OutputSet(p, 1)) ok++ } catch (e: Throwable) { /* 심볼 없음 등 */ }
            }
            log.append("OutputSet ok=$ok ")
        } else {
            log.append("VanMcu 미탑재 ")
        }
        lastResult = log.toString().trim()
        Log.i(TAG, "하드웨어 전원 인에이블: $lastResult")
    }
}
