package com.farmmachine.autosteer

import android.content.Intent
import android.os.Bundle
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.compose.foundation.gestures.detectTapGestures
import androidx.compose.foundation.layout.*
import androidx.compose.material3.*
import androidx.compose.runtime.*
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.input.pointer.pointerInput
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import androidx.core.content.ContextCompat
import kotlinx.coroutines.delay
import org.json.JSONObject

class MainActivity : ComponentActivity() {
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        // 포그라운드 서비스 = CAN 브릿지 + Python 제어 루프
        ContextCompat.startForegroundService(this, Intent(this, AutoSteerService::class.java))
        setContent { MaterialTheme(colorScheme = darkColorScheme()) { AutoSteerScreen() } }
    }
}

@Composable
fun AutoSteerScreen() {
    var st by remember { mutableStateOf(JSONObject()) }
    LaunchedEffect(Unit) {
        while (true) {
            st = try { JSONObject(SteerController.statusJson()) } catch (e: Exception) { JSONObject() }
            delay(250)
        }
    }

    val engaged = st.optBoolean("engaged", false)
    val safety = st.optString("safety", "—")
    val canState = st.optString("can_state", "—")
    val canOk = st.optBoolean("can_available", false)

    Column(Modifier.fillMaxSize().padding(16.dp), verticalArrangement = Arrangement.spacedBy(10.dp)) {
        Text("팜머신 자율조향", fontSize = 22.sp)

        // 상태 카드
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
                Info("GNSS 소스", st.optString("active_gnss", "—"))
                Info("CAN tx/rx", "${st.optInt("can_tx", 0)}/${st.optInt("can_rx", 0)} (재연결 ${st.optInt("can_reconnects", 0)})")
            }
        }

        // 프로파일 선택
        Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
            listOf("normal" to "일반", "heavy" to "과부하", "sand" to "모래").forEach { (k, label) ->
                OutlinedButton(onClick = { SteerController.setProfile(k) }) { Text(label) }
            }
            OutlinedButton(onClick = { SteerController.setDemoAbLine() }) { Text("데모경로") }
        }

        // 제어 버튼
        Row(horizontalArrangement = Arrangement.spacedBy(8.dp)) {
            Button(onClick = { SteerController.engage() }, enabled = !engaged) { Text("자동 시작") }
            Button(onClick = { SteerController.disengage() }, enabled = engaged) { Text("해제") }
            Button(
                onClick = { SteerController.estop() },
                colors = ButtonDefaults.buttonColors(containerColor = Color(0xFFC62828))
            ) { Text("비상정지") }
        }

        Spacer(Modifier.weight(1f))

        // 데드맨: 누르고 있는 동안만 활성 (떼면 자동 해제)
        Button(
            onClick = {},
            modifier = Modifier.fillMaxWidth().height(72.dp).pointerInput(Unit) {
                detectTapGestures(onPress = {
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
