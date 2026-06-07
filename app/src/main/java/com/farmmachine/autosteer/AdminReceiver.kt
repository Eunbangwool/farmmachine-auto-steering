package com.farmmachine.autosteer

import android.app.admin.DeviceAdminReceiver
import android.content.Context
import android.content.Intent
import android.util.Log

/**
 * 디바이스 관리자 리시버.
 *
 * Apollo 10 Pro 의 내장 CAN(libsysmcu.so)·GPIO 하드웨어 접근에는 device-owner
 * 권한이 필요하다. 설치 후 1회 등록(ADB):
 *   adb shell dpm set-device-owner com.farmmachine.autosteer/.AdminReceiver
 *
 * (이미 다른 device-owner 가 있으면 해제 후 등록. force-lock 등 정책은
 *  res/xml/device_admin.xml 참조 — 키오스크/잠금 운용용.)
 */
class AdminReceiver : DeviceAdminReceiver() {
    override fun onEnabled(context: Context, intent: Intent) {
        Log.i("AutoSteer", "Device Admin 활성화 — 하드웨어(CAN/GPIO) 접근 권한 확보")
    }
    override fun onDisabled(context: Context, intent: Intent) {
        Log.w("AutoSteer", "Device Admin 비활성화")
    }
}
