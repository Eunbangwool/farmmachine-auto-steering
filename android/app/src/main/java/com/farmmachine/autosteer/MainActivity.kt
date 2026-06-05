package com.farmmachine.autosteer

import android.app.admin.DevicePolicyManager
import android.content.ComponentName
import android.content.Context
import android.os.Bundle
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.core.view.WindowCompat
import com.farmmachine.autosteer.ui.MainScreen
import com.farmmachine.autosteer.ui.theme.FarmMachineTheme

class MainActivity : ComponentActivity() {
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        WindowCompat.setDecorFitsSystemWindows(window, false)
        checkDeviceAdmin()
        setContent {
            FarmMachineTheme {
                MainScreen()
            }
        }
    }

    private fun checkDeviceAdmin() {
        val dpm = getSystemService(Context.DEVICE_POLICY_SERVICE) as DevicePolicyManager
        val admin = ComponentName(this, AdminReceiver::class.java)
        if (!dpm.isAdminActive(admin)) {
            // ADB로 등록: adb shell dpm set-device-owner com.farmmachine.autosteer/.AdminReceiver
            android.util.Log.w("FarmMachine", "Device Admin not active - CAN hardware may be restricted")
        }
    }
}
