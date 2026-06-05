package com.farmmachine.autosteer

import android.app.admin.DeviceAdminReceiver
import android.content.Context
import android.content.Intent
import android.util.Log

class AdminReceiver : DeviceAdminReceiver() {
    override fun onEnabled(context: Context, intent: Intent) {
        Log.i("FarmMachine", "Device Admin enabled - full hardware access granted")
    }
    override fun onDisabled(context: Context, intent: Intent) {
        Log.w("FarmMachine", "Device Admin disabled")
    }
}
