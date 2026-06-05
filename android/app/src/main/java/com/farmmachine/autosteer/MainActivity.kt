package com.farmmachine.autosteer

import android.app.admin.DevicePolicyManager
import android.content.ComponentName
import android.content.Context
import android.os.Bundle
import android.util.Log
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.core.view.WindowCompat
import com.farmmachine.autosteer.ui.AgmoScreen
import com.farmmachine.autosteer.ui.AgmoTheme

class MainActivity : ComponentActivity() {
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        WindowCompat.setDecorFitsSystemWindows(window, false)
        checkDeviceAdmin()
        setContent {
            AgmoTheme { AgmoScreen() }
        }
    }
    private fun checkDeviceAdmin() {
        val dpm   = getSystemService(Context.DEVICE_POLICY_SERVICE) as DevicePolicyManager
        val admin = ComponentName(this, AdminReceiver::class.java)
        if (!dpm.isAdminActive(admin))
            Log.w("FarmMachine", "Device Admin inactive — CAN access may be limited")
    }
}
