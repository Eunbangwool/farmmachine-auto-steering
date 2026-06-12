package com.farmmachine.autosteer

import android.app.Notification
import android.app.NotificationChannel
import android.app.NotificationManager
import android.app.Service
import android.content.Intent
import android.content.pm.ServiceInfo
import android.os.Build
import android.os.IBinder
import com.farmmachine.autosteer.can.ApolloCanBridge
import com.farmmachine.autosteer.py.PythonEngine

/**
 * 자율조향 포그라운드 서비스.
 *  - ApolloCanBridge(TCP) 기동 → Python ApolloCanBus 접속 대상
 *  - Chaquopy Python(app_main.boot) 기동 → 50Hz 제어 루프
 * 안전: 화면/프로세스가 살아있는 동안 CAN·제어가 유지된다.
 */
class AutoSteerService : Service() {

    private val bridge = ApolloCanBridge(port = 47100)

    override fun onCreate() {
        super.onCreate()
        startForegroundCompat()
        HardwareInit.enable(this)   // GNSS(UM482)/CAN 전원 인에이블(com.van.service 브로드캐스트 + 네이티브 GPIO)
        bridge.start()
        PythonEngine.boot(this, backend = "bridge")
    }

    override fun onStartCommand(intent: Intent?, flags: Int, startId: Int): Int = START_STICKY

    override fun onDestroy() {
        PythonEngine.shutdown()
        bridge.stop()
        super.onDestroy()
    }

    override fun onBind(intent: Intent?): IBinder? = null

    private fun startForegroundCompat() {
        val channelId = "autosteer"
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
            val nm = getSystemService(NotificationManager::class.java)
            nm.createNotificationChannel(
                NotificationChannel(channelId, "자율조향",
                    NotificationManager.IMPORTANCE_LOW))
        }
        // Notification.Builder(Context, channelId) 생성자는 API 26+. API 23~25 는
        // 채널 없는 deprecated 생성자를 써야 함(채널은 O 미만에서 무의미) → SDK_INT 분기.
        val builder = if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
            Notification.Builder(this, channelId)
        } else {
            @Suppress("DEPRECATION") Notification.Builder(this)
        }
        val notif: Notification = builder
            .setContentTitle("팜머신 자율조향")
            .setContentText("CAN 브릿지 + 제어 루프 실행 중")
            .setSmallIcon(android.R.drawable.ic_menu_compass)
            .setOngoing(true)
            .build()
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.Q) {
            startForeground(1, notif, ServiceInfo.FOREGROUND_SERVICE_TYPE_CONNECTED_DEVICE)
        } else {
            startForeground(1, notif)
        }
    }
}
