package com.farmmachine.autosteer

import android.os.Bundle
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.compose.foundation.gestures.detectTapGestures
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.verticalScroll
import androidx.compose.material3.*
import androidx.compose.runtime.*
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.input.pointer.pointerInput
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.farmmachine.autosteer.py.PythonEngine
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.delay
import kotlinx.coroutines.withContext
import org.json.JSONObject

class MainActivity : ComponentActivity() {
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        // UI 를 먼저 띄운다. 포그라운드 서비스(connectedDevice FGS)는 Android 14+ 에서
        // 권한 없이 startForeground 하면 크래시하므로 실행 시 자동 시작하지 않는다.
        // CAN 벤더 SDK 가 준비되면 그때 적절히 기동한다.
        setContent { MaterialTheme(colorScheme = darkColorScheme()) { AutoSteerScreen() } }
    }
}

@Composable
fun AutoSteerScreen() {
    val ctx = LocalContext.current
    var booted by remember { mutableStateOf(false) }
    var bootError by remember { mutableStateOf<String?>(null) }
    var st by remember { mutableStateOf(JSONObject()) }

    // Chaquopy Python 을 백그라운드에서 부팅 — 실패해도 UI 는 유지하고 오류를 표시.
    // 현재는 하드웨어(CAN/GNSS) 미연결이라 mock 백엔드로 UI 동작을 검증.
    LaunchedEffect(Unit) {
        withContext(Dispatchers.Default) {
            runCatching { PythonEngine.boot(ctx, "mock") }
                .onSuccess { booted = true }
                .onFailure { bootError = it.message ?: it.toString() }
        }
    }
    LaunchedEffect(booted) {
        while (booted) {
            st = try { JSONObject(SteerController.statusJson()) } catch (e: Exception) { JSONObject() }
            delay(250)
        }
    }

    val engaged = st.optBoolean("engaged", false)
    val safety = st.optString("safety", "—")
    val canState = st.optString("can_state", "—")
    val canOk = st.optBoolean("can_available", false)

    Column(
        Modifier.fillMaxSize().padding(16.dp).verticalScroll(rememberScrollState()),
        verticalArrangement = Arrangement.spacedBy(10.dp)
    ) {
        Text("팜머신 자율조향  ·  데모(mock)", fontSize = 22.sp)

        if (bootError != null) {
            Card(colors = CardDefaults.cardColors(containerColor = Color(0xFF5D1A1A))) {
                Column(Modifier.padding(12.dp)) {
                    Text("Python 부팅 실패", color = Color(0xFFFFCDD2))
                    Text(bootError!!, fontSize = 12.sp, color = Color(0xFFFFCDD2))
                }
            }
        } else if (!booted) {
            Row(verticalAlignment = Alignment.CenterVertically,
                horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                CircularProgressIndicator(Modifier.size(18.dp), strokeWidth = 2.dp)
                Text("엔진 시작 중…")
            }
        }

        Card(Modifier.fillMaxWidth()) {
            Column(Modifier.padding(12.dp), verticalArrangement = Arrangement.spacedBy(4.dp)) {
                Row(horizontalArrangement = Arrangement.spacedBy(16.dp)) {
                    Badge("조향", if (engaged) "ENGAGED" else "대기", if (engaged) Color(0xFF2E7D32) else Color.Gray)
                    Badge("안전", safety, if (safety == "SAFE") Color(0xFF2E7D32) else Color(0xFFC62828))
                    Badge("CAN", canState, if (canOk) Color(0xFF2E7D32) else Color(0xFFC62828))
                }
                Spacer(Modifier.height(4.dp))
                Info("프로파일", st.optString("profile", "—"))
                Info("XTE (cm)", "%.1f".format(st.optDouble("xte_cm", 0.0)))
                Info("목표조향(°)", "%.1f".format(st.optDouble("target_angle_deg", 0.0)))
                Info("측정조향(°)", "%.1f".format(st.optDouble("measured_angle_deg", 0.0)))
                Info("속도(m/s)", "%.2f".format(st.optDouble("speed_mps", 0.0)))
                Info("CAN tx/rx", "${st.optInt("can_tx", 0)}/${st.optInt("can_rx", 0)}")
            }
        }

        Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
            listOf("normal" to "일반", "heavy" to "과부하", "sand" to "모래").forEach { (k, label) ->
                OutlinedButton(onClick = { SteerController.setProfile(k) }, enabled = booted) { Text(label) }
            }
            OutlinedButton(onClick = { SteerController.setDemoAbLine() }, enabled = booted) { Text("데모경로") }
        }

        Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
            Button(onClick = { SteerController.engage() }, enabled = booted && !engaged) { Text("자동 시작") }
            Button(onClick = { SteerController.disengage() }, enabled = booted && engaged) { Text("해제") }
            Button(
                onClick = { SteerController.estop() }, enabled = booted,
                colors = ButtonDefaults.buttonColors(containerColor = Color(0xFFC62828))
            ) { Text("비상정지") }
        }

        Text(
            "※ 현재 데모(mock) 모드: 실제 CAN/GNSS 미연결. 실차 연동은 벤더 CAN SDK + GNSS 브릿지 후.",
            fontSize = 11.sp, color = Color.Gray
        )

        Spacer(Modifier.height(8.dp))

        Button(
            onClick = {}, enabled = booted,
            modifier = Modifier.fillMaxWidth().height(72.dp).pointerInput(booted) {
                if (booted) detectTapGestures(onPress = {
                    SteerController.setDeadman(true)
                    tryAwaitRelease()
                    SteerController.setDeadman(false)
                })
            }
        ) { Text("데드맨 (누르고 있는 동안 작동)", fontSize = 18.sp) }
    }
}

@Composable
private fun Badge(label: String, value: String, color: Color) {
    Column(horizontalAlignment = Alignment.CenterHorizontally) {
        Text(label, fontSize = 12.sp, color = Color.Gray)
        Text(value, fontSize = 16.sp, color = color)
    }
}

@Composable
private fun Info(label: String, value: String) {
    Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.SpaceBetween) {
        Text(label, color = Color.Gray)
        Text(value)
    }
}
