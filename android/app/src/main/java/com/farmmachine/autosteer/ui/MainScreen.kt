package com.farmmachine.autosteer.ui

import androidx.compose.foundation.*
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.*
import androidx.compose.runtime.*
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.farmmachine.autosteer.ui.theme.*

@Composable
fun MainScreen() {
    var engaged by remember { mutableStateOf(false) }
    var showField by remember { mutableStateOf(false) }
    var levelMode by remember { mutableStateOf(false) }
    var targetLevel by remember { mutableFloatStateOf(0f) }
    var algo by remember { mutableStateOf("impl") }
    var profile by remember { mutableStateOf("일반") }
    var speed by remember { mutableFloatStateOf(1.4f) }
    var tab by remember { mutableStateOf("steer") }

    // 시뮬레이션 상태
    var xte by remember { mutableDoubleStateOf(0.0) }
    var curElev by remember { mutableDoubleStateOf(0.0) }
    var steerDeg by remember { mutableDoubleStateOf(0.0) }
    var motorRpm by remember { mutableIntStateOf(0) }
    var passNo by remember { mutableIntStateOf(0) }

    // 시뮬레이션 업데이트 (50Hz)
    LaunchedEffect(engaged, speed, algo, profile) {
        if (engaged) {
            var t = 0.0
            while (true) {
                t += 0.05
                xte = Math.sin(t * 0.3) * 2.5 * Math.random() * 0.5 + xte * 0.9
                curElev = Math.sin(t * 0.15) * 4.0 - targetLevel
                steerDeg = -xte * 3.0
                motorRpm = (steerDeg * 5).toInt().coerceIn(-80, 80)
                if (t > (passNo + 1) * 12.0) passNo = (passNo + 1).coerceAtMost(7)
                kotlinx.coroutines.delay(50)
            }
        }
    }

    val xteColor = when {
        Math.abs(xte) < 2.5 -> Green
        Math.abs(xte) < 5.0 -> WarnOrange
        else -> ErrorRed
    }
    val elevColor = levelColorKt(curElev.toFloat())

    Column(
        Modifier.fillMaxSize().background(DarkBg)
            .systemBarsPadding()
    ) {
        // ── 상태 바 ──
        Row(
            Modifier.fillMaxWidth().background(CardBg)
                .border(BorderStroke(0.5.dp, BorderColor), RoundedCornerShape(0.dp))
                .padding(horizontal = 10.dp, vertical = 6.dp),
            verticalAlignment = Alignment.CenterVertically,
            horizontalArrangement = Arrangement.spacedBy(8.dp)
        ) {
            Text("RTK FIX", color = if (engaged) Color.Black else Color.White,
                fontSize = 10.sp, fontWeight = FontWeight.Bold,
                modifier = Modifier.background(if (engaged) Green else Color(0xFF1A5A2A),
                    RoundedCornerShape(3.dp)).padding(horizontal = 6.dp, vertical = 2.dp))
            Text("⬡14", color = TextDim, fontSize = 11.sp)
            Spacer(Modifier.weight(1f))

            // 영역 토글
            OutlinedButton(
                onClick = { showField = !showField },
                border = BorderStroke(1.dp, if (showField) Green else Color(0xFF1A4A1E)),
                colors = ButtonDefaults.outlinedButtonColors(
                    containerColor = if (showField) Color(0x22000000) else Color.Transparent,
                    contentColor = if (showField) Green else TextDim),
                contentPadding = PaddingValues(horizontal = 8.dp, vertical = 2.dp),
                modifier = Modifier.height(28.dp)
            ) { Text("영역", fontSize = 11.sp, fontWeight = FontWeight.Bold) }

            // 레벨 토글
            OutlinedButton(
                onClick = { levelMode = !levelMode },
                border = BorderStroke(1.dp, if (levelMode) LevelYellow else Color(0xFF3A3A00)),
                colors = ButtonDefaults.outlinedButtonColors(
                    containerColor = Color.Transparent,
                    contentColor = if (levelMode) LevelYellow else Color(0xFF3A3A10)),
                contentPadding = PaddingValues(horizontal = 8.dp, vertical = 2.dp),
                modifier = Modifier.height(28.dp)
            ) { Text("레벨", fontSize = 11.sp, fontWeight = FontWeight.Bold) }

            Text("${speed.toInt()}.${((speed % 1) * 10).toInt()}m/s",
                color = Green, fontSize = 13.sp, fontFamily = FontFamily.Monospace)
        }

        // ── 필드 캔버스 ──
        Box(Modifier.fillMaxWidth().height(240.dp)) {
            FieldCanvas(
                engaged = engaged, showField = showField,
                levelMode = levelMode, targetLevel = targetLevel.toDouble(),
                xte = xte, steerDeg = steerDeg, passNo = passNo,
                modifier = Modifier.fillMaxSize()
            )
            if (!engaged) {
                Box(Modifier.fillMaxSize().background(Color(0x88060E08)),
                    contentAlignment = Alignment.Center) {
                    Text("STANDBY", color = Color(0x77000000),
                        fontSize = 18.sp, fontWeight = FontWeight.Bold,
                        letterSpacing = 4.sp)
                }
            }
            // 수치 표시
            val dispVal = if (levelMode) {
                "${if (curElev >= 0) "+" else ""}${String.format("%.1f", curElev)}cm"
            } else {
                "${if (xte >= 0) "+" else ""}${String.format("%.1f", xte)}cm"
            }
            Text(dispVal, color = if (levelMode) elevColor else xteColor,
                fontSize = 22.sp, fontFamily = FontFamily.Monospace, fontWeight = FontWeight.Bold,
                modifier = Modifier.align(Alignment.BottomStart).padding(10.dp))
        }

        // ── 게이지 행 ──
        Row(
            Modifier.fillMaxWidth().background(CardBg)
                .border(BorderStroke(0.5.dp, BorderColor))
        ) {
            val gauges = if (levelMode) listOf(
                Triple("레벨 오차", "${if (curElev>=0)"+" else ""}${String.format("%.1f",curElev)}cm", elevColor),
                Triple("목표 레벨", "${if(targetLevel>=0)"+" else ""}${String.format("%.0f",targetLevel)}cm", LevelYellow),
                Triple("RPM", "${if(motorRpm>=0)"+" else ""}$motorRpm", if(engaged) Green else TextDim),
                Triple("라인", "${passNo+1}/8", WarnOrange),
            ) else listOf(
                Triple("XTE", "${if(xte>=0)"+" else ""}${String.format("%.1f",xte)}cm", xteColor),
                Triple("WAS", "${String.format("%.1f",steerDeg*15)}°", Color(0xFF82C8FF)),
                Triple("RPM", "${if(motorRpm>=0)"+" else ""}$motorRpm", if(engaged) Green else TextDim),
                Triple("라인", "${passNo+1}/8", WarnOrange),
            )
            gauges.forEachIndexed { i, (label, value, color) ->
                Column(
                    Modifier.weight(1f).padding(vertical = 8.dp),
                    horizontalAlignment = Alignment.CenterHorizontally
                ) {
                    Text(label, color = TextDim, fontSize = 9.sp, letterSpacing = 0.5.sp)
                    Spacer(Modifier.height(2.dp))
                    Text(value, color = color, fontSize = 16.sp, fontFamily = FontFamily.Monospace,
                        fontWeight = FontWeight.Bold, maxLines = 1)
                }
                if (i < 3) Box(Modifier.width(0.5.dp).height(40.dp).background(BorderColor))
            }
        }

        // ── 컨트롤 ──
        Column(
            Modifier.fillMaxWidth().background(CardBg)
                .border(BorderStroke(0.5.dp, BorderColor))
                .padding(10.dp)
        ) {
            // 레벨 목표 슬라이더
            if (levelMode) {
                Row(verticalAlignment = Alignment.CenterVertically) {
                    Text("목표레벨", color = Color(0xFFAAAA00), fontSize = 10.sp)
                    Spacer(Modifier.weight(1f))
                    Text("${if(targetLevel>=0)"+" else ""}${String.format("%.0f",targetLevel)}cm",
                        color = LevelYellow, fontSize = 13.sp, fontFamily = FontFamily.Monospace)
                }
                Slider(value = targetLevel, onValueChange = { targetLevel = it },
                    valueRange = -10f..10f,
                    colors = SliderDefaults.colors(thumbColor = LevelYellow, activeTrackColor = LevelYellow),
                    modifier = Modifier.fillMaxWidth())
                Spacer(Modifier.height(4.dp))
            } else {
                // 알고리즘 선택
                Row(Modifier.fillMaxWidth().height(34.dp)
                    .clip(RoundedCornerShape(8.dp)).border(BorderStroke(1.dp, BorderColor), RoundedCornerShape(8.dp))) {
                    listOf("impl" to "작업기기준", "pure" to "퓨어퍼슈트").forEach { (k, n) ->
                        Box(Modifier.weight(1f).fillMaxHeight()
                            .background(if (algo==k) Color(0xFF0A3A1A) else Color.Transparent)
                            .clickable { algo = k }, contentAlignment = Alignment.Center) {
                            Text(n, color = if(algo==k) Green else TextDim,
                                fontSize = 11.sp, fontWeight = if(algo==k) FontWeight.Bold else FontWeight.Normal)
                        }
                    }
                }
                Spacer(Modifier.height(6.dp))
            }

            Row(verticalAlignment = Alignment.CenterVertically, horizontalArrangement = Arrangement.spacedBy(10.dp)) {
                Column(Modifier.weight(1f)) {
                    Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.SpaceBetween) {
                        Text("SPEED", color = TextDim, fontSize = 10.sp)
                        Text("${String.format("%.1f",speed)} m/s",
                            color = Green, fontSize = 13.sp, fontFamily = FontFamily.Monospace)
                    }
                    Slider(value = speed, onValueChange = { speed = it },
                        valueRange = 0.5f..3f,
                        colors = SliderDefaults.colors(thumbColor = Green, activeTrackColor = Green))
                }
                // ENGAGE 버튼
                Button(
                    onClick = { engaged = !engaged; if (!engaged) passNo = 0 },
                    colors = ButtonDefaults.buttonColors(
                        containerColor = if (engaged) Color(0xFF660000) else Color(0xFF0A4A1A)),
                    border = BorderStroke(2.dp, if (engaged) ErrorRed else Green),
                    shape = RoundedCornerShape(10.dp),
                    modifier = Modifier.size(90.dp, 64.dp)
                ) {
                    Column(horizontalAlignment = Alignment.CenterHorizontally) {
                        if (engaged) Text("ACTIVE", color = Color(0xAAFF8888),
                            fontSize = 9.sp, fontWeight = FontWeight.Bold)
                        Text(if (engaged) "DISENGAGE" else "ENGAGE",
                            color = if (engaged) ErrorRed else Green,
                            fontSize = if (engaged) 12.sp else 15.sp,
                            fontWeight = FontWeight.Bold, letterSpacing = 0.5.sp)
                    }
                }
            }
        }

        // ── 탭 바 ──
        Row(Modifier.fillMaxWidth().background(DarkBg).border(BorderStroke(0.5.dp, BorderColor))) {
            listOf("steer" to "조향", "level" to "레벨", "can" to "CAN", "motor" to "모터").forEach { (k, n) ->
                Column(
                    Modifier.weight(1f).clickable { tab = k }.padding(vertical = 9.dp),
                    horizontalAlignment = Alignment.CenterHorizontally
                ) {
                    Text(n, color = if (tab == k) Green else TextDim,
                        fontSize = 12.sp, fontWeight = if (tab == k) FontWeight.Bold else FontWeight.Normal,
                        letterSpacing = 0.5.sp)
                    if (tab == k) Box(Modifier.height(2.dp).width(32.dp).background(Green))
                }
            }
        }

        // ── 탭 내용 ──
        Column(Modifier.weight(1f).verticalScroll(rememberScrollState()).padding(12.dp)) {
            when (tab) {
                "steer" -> SteerTab(xte, steerDeg, passNo)
                "level" -> LevelTab(curElev, targetLevel.toDouble())
                "can"   -> CanTab(engaged, motorRpm)
                "motor" -> MotorTab(motorRpm, steerDeg)
            }
        }
    }
}

fun levelColorKt(diff: Float): Color = when {
    diff < -8f -> Color(0xFF0000CC)
    diff < -5f -> Color(0xFF0044FF)
    diff < -3f -> Color(0xFF0099FF)
    diff < -1f -> Color(0xFF00DDFF)
    diff < 1f  -> Color(0xFF00E676)
    diff < 3f  -> Color(0xFFAAEE00)
    diff < 5f  -> Color(0xFFFFCC00)
    diff < 8f  -> Color(0xFFFF6600)
    else       -> Color(0xFFCC0000)
}

@Composable
fun SteerTab(xte: Double, steerDeg: Double, passNo: Int) {
    InfoGrid(listOf(
        Triple("XTE", "${if(xte>=0)"+" else ""}${String.format("%.2f",xte)}cm",
            if(Math.abs(xte)<2.5) Green else if(Math.abs(xte)<5) WarnOrange else ErrorRed),
        Triple("WAS", "${String.format("%.1f",steerDeg*15)}°", Color(0xFF82C8FF)),
        Triple("라인", "${passNo+1}/8", WarnOrange),
        Triple("알고리즘", "IMPL.REF", Green),
    ))
}

@Composable
fun LevelTab(curElev: Double, targetLevel: Double) {
    InfoGrid(listOf(
        Triple("레벨 오차", "${if(curElev>=0)"+" else ""}${String.format("%.2f",curElev)}cm", levelColorKt(curElev.toFloat())),
        Triple("목표 레벨", "${if(targetLevel>=0)"+" else ""}${String.format("%.0f",targetLevel)}cm", LevelYellow),
        Triple("허용 오차", "±1.0cm", Color(0xFF82C8FF)),
        Triple("작업", if(Math.abs(curElev)>2) "절토·성토" else "기준 내", if(Math.abs(curElev)>2) WarnOrange else Green),
    ))
    Spacer(Modifier.height(8.dp))
    LevelLegend()
}

@Composable
fun LevelLegend() {
    Card(colors = CardDefaults.cardColors(containerColor = CardBg),
        border = BorderStroke(0.5.dp, BorderColor), modifier = Modifier.fillMaxWidth()) {
        Column(Modifier.padding(10.dp)) {
            Text("COLOR LEGEND", color = TextDim, fontSize = 9.sp, letterSpacing = 0.5.sp)
            Spacer(Modifier.height(6.dp))
            listOf(
                Triple(Color(0xFF0000CC), "< -8cm", "대성토"),
                Triple(Color(0xFF0099FF), "-5 ~ -3cm", "성토"),
                Triple(Color(0xFF00DDFF), "-3 ~ -1cm", "약성토"),
                Triple(Color(0xFF00E676), "-1 ~ +1cm", "✓ 기준"),
                Triple(Color(0xFFFFCC00), "+1 ~ +3cm", "약절토"),
                Triple(Color(0xFFFF6600), "+3 ~ +5cm", "절토"),
                Triple(Color(0xFFCC0000), "> +8cm", "대절토"),
            ).forEach { (c, range, desc) ->
                Row(verticalAlignment = Alignment.CenterVertically,
                    modifier = Modifier.padding(vertical = 3.dp)) {
                    Box(Modifier.size(16.dp, 12.dp).background(c, RoundedCornerShape(2.dp)))
                    Spacer(Modifier.width(8.dp))
                    Text(range, color = Color(0xFF82B882), fontSize = 10.sp,
                        fontFamily = FontFamily.Monospace, modifier = Modifier.width(80.dp))
                    Text(desc, color = TextDim, fontSize = 10.sp)
                }
            }
        }
    }
}

@Composable
fun CanTab(engaged: Boolean, motorRpm: Int) {
    Card(colors = CardDefaults.cardColors(containerColor = CardBg),
        border = BorderStroke(0.5.dp, BorderColor), modifier = Modifier.fillMaxWidth()) {
        Column(Modifier.padding(10.dp)) {
            Text("KY170C — 250kbps Extended 29-bit", color = TextDim, fontSize = 9.sp)
            Spacer(Modifier.height(6.dp))
            listOf("TX" to "0x06000001", "RX" to "0x05800001",
                "HB" to "0x07000001 (20ms)", "ENABLE" to "23 0D 20 01 00 00 00 00",
                "DISABLE" to "23 0C 20 01 00 00 00 00").forEach { (k, v) ->
                Row(Modifier.padding(vertical = 3.dp)) {
                    Text(k, color = TextDim, fontSize = 10.sp, modifier = Modifier.width(56.dp))
                    Text(v, color = Green, fontSize = 10.sp, fontFamily = FontFamily.Monospace)
                }
            }
        }
    }
}

@Composable
fun MotorTab(motorRpm: Int, steerDeg: Double) {
    InfoGrid(listOf(
        Triple("모터 RPM", "${if(motorRpm>=0)"+" else ""}$motorRpm", Green),
        Triple("WAS 각도", "${String.format("%.1f",steerDeg*15)}°", Color(0xFF82C8FF)),
        Triple("정격 RPM", "80", TextBright),
        Triple("CAN ID", "0x06000001", WarnOrange),
    ))
}

@Composable
fun InfoGrid(items: List<Triple<String, String, Color>>) {
    Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.spacedBy(8.dp)) {
        items.chunked(2).forEach { row ->
            Column(Modifier.weight(1f), verticalArrangement = Arrangement.spacedBy(8.dp)) {
                row.forEach { (label, value, color) ->
                    Card(colors = CardDefaults.cardColors(containerColor = CardBg),
                        border = BorderStroke(0.5.dp, BorderColor),
                        modifier = Modifier.fillMaxWidth()) {
                        Column(Modifier.padding(10.dp)) {
                            Text(label, color = TextDim, fontSize = 9.sp, letterSpacing = 0.5.sp)
                            Spacer(Modifier.height(4.dp))
                            Text(value, color = color, fontSize = 18.sp,
                                fontFamily = FontFamily.Monospace, fontWeight = FontWeight.Bold)
                        }
                    }
                }
            }
        }
    }
}
