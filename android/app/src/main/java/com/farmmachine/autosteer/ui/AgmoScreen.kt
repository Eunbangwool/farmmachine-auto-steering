package com.farmmachine.autosteer.ui

import androidx.compose.foundation.*
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.*
import androidx.compose.runtime.*
import androidx.compose.ui.*
import androidx.compose.ui.draw.clip
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.text.*
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.*
import kotlinx.coroutines.delay
import kotlin.math.*

@Composable
fun AgmoScreen() {
    val textMeasurer = rememberTextMeasurer()

    // ── 앱 상태 ──
    var engaged    by remember { mutableStateOf(false) }
    var workType   by remember { mutableStateOf(WorkType.STEER) }
    var profileIdx by remember { mutableIntStateOf(0) }
    var speed      by remember { mutableFloatStateOf(1.4f) }
    var targetLv   by remember { mutableFloatStateOf(0f) }
    var showPanel  by remember { mutableStateOf(false) }

    // ── 시뮬레이션 내부 상태 ──
    var sim     by remember { mutableStateOf(SimState()) }
    var metrics by remember { mutableStateOf(UiMetrics()) }

    val levelMode = workType == WorkType.LEVEL

    // ── 시뮬레이션 루프 (50Hz) ──
    LaunchedEffect(engaged, profileIdx, speed, levelMode, targetLv) {
        if (!engaged) { sim = SimState(); return@LaunchedEffect }
        val PASSES = 8; val GAP = 6f; val DT = 0.05f
        while (true) {
            val lx = sim.passNo * GAP - (PASSES * GAP) / 2f
            val xteM = (sim.tx - lx).toDouble()
            val hErr = (sim.th - PI/2).toDouble()
            val pr   = PROFILES[profileIdx]
            var steer = Algo.implRef(xteM, hErr, speed.toDouble(), pr).toFloat()
            steer += (Math.random().toFloat() - 0.5f) * 0.008f
            steer  = max(-Algo.MAX_S.toFloat(), min(Algo.MAX_S.toFloat(), steer))
            var next = Algo.stepTractor(sim, steer, speed, DT)
            if (next.ty > 30f) {
                next = next.copy(
                    ty    = -28f,
                    passNo = min(next.passNo + 1, PASSES - 1),
                    tx     = lx + xteM.toFloat() * 0.3f,
                )
            }
            sim = next
            val curElev = Algo.elevNoise(sim.tx, sim.ty) - targetLv
            val headDeg = ((sim.th * 180f / PI.toFloat()) % 360f + 360f) % 360f
            metrics = UiMetrics(
                xte       = (xteM * 100).toFloat(),
                curElev   = curElev,
                steerDeg  = (steer * 180f / PI.toFloat()),
                headDeg   = headDeg.toInt(),
                passNo    = sim.passNo,
                leftDist  = max(0, (30 - sim.ty).toInt()),
                stable    = abs(xteM) < 0.06 && abs(hErr) < 0.06,
                motorRpm  = (steer / Algo.MAX_S.toFloat() * 80f).toInt(),
            )
            delay(50)
        }
    }

    val xteColor = when {
        abs(metrics.xte) < 2.5f -> Ag.Green
        abs(metrics.xte) < 5f   -> Ag.Orange
        else                    -> Ag.Red
    }
    val elevColor = levelColor(metrics.curElev)

    Column(
        Modifier.fillMaxSize().background(Ag.Bg).systemBarsPadding()
    ) {

        // ══ HEADER ══════════════════════════════════════
        Row(
            Modifier.fillMaxWidth().background(Ag.Surface)
                .border(BorderStroke(0.5.dp, Ag.Border))
                .height(44.dp).padding(horizontal = 10.dp),
            verticalAlignment = Alignment.CenterVertically,
            horizontalArrangement = Arrangement.spacedBy(8.dp),
        ) {
            // AGMO 로고
            Text("AGMO", color = Ag.Green, fontSize = 20.sp, fontWeight = FontWeight.Bold,
                letterSpacing = 2.sp)
            Text("Solution", color = Ag.Dim, fontSize = 9.sp,
                modifier = Modifier.padding(start = 2.dp))
            Spacer(Modifier.weight(1f))
            // RTK 뱃지
            Row(
                Modifier.background(if (engaged) Ag.GreenBg else Color(0xFF1A1A1A),
                    RoundedCornerShape(4.dp))
                    .border(BorderStroke(1.dp, if (engaged) Ag.Green else Color(0xFF333333)), RoundedCornerShape(4.dp))
                    .padding(horizontal = 8.dp, vertical = 3.dp),
                verticalAlignment = Alignment.CenterVertically,
                horizontalArrangement = Arrangement.spacedBy(4.dp),
            ) {
                Box(Modifier.size(7.dp).background(if (engaged) Ag.Green else Color(0xFF444444), RoundedCornerShape(50)))
                Text("RTK FIX", color = if (engaged) Ag.Green else Color(0xFF555555),
                    fontSize = 11.sp, fontWeight = FontWeight.Bold)
            }
            Text("⬡14", color = Ag.Dim, fontSize = 11.sp)
            Text(if (engaged) "${"%.1f".format(speed)}m/s" else "0.0m/s",
                color = if (engaged) Ag.Text else Ag.Dim,
                fontSize = 13.sp, fontFamily = FontFamily.Monospace, fontWeight = FontWeight.Bold)
            Text("%03d°".format(metrics.headDeg), color = Ag.Dim,
                fontSize = 12.sp, fontFamily = FontFamily.Monospace)
            // 설정 버튼
            Text("⚙", color = Ag.Dim, fontSize = 18.sp,
                modifier = Modifier.clickable { showPanel = !showPanel }.padding(4.dp))
        }

        // ══ 알림 배너 ═══════════════════════════════════
        if (engaged && !metrics.stable) {
            Row(
                Modifier.fillMaxWidth().background(Color(0xFF2A1500))
                    .border(BorderStroke(0.dp, Ag.Orange)).padding(6.dp, 5.dp),
                verticalAlignment = Alignment.CenterVertically,
                horizontalArrangement = Arrangement.spacedBy(8.dp),
            ) {
                Text("⚠", fontSize = 14.sp, color = Ag.Orange)
                Text("안정화 대기 중 — 직선 주행하세요",
                    color = Ag.Orange, fontSize = 12.sp, fontWeight = FontWeight.Bold)
            }
        }

        // ══ 설정 패널 ════════════════════════════════════
        if (showPanel) {
            Column(
                Modifier.fillMaxWidth().background(Ag.Card)
                    .border(BorderStroke(0.5.dp, Ag.Border))
                    .padding(12.dp)
            ) {
                Text("민감도 프로파일", color = Ag.Dim, fontSize = 11.sp,
                    modifier = Modifier.padding(bottom = 6.dp))
                Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.spacedBy(6.dp)) {
                    PROFILES.forEachIndexed { i, pr ->
                        val sel = i == profileIdx
                        Box(
                            Modifier.weight(1f)
                                .background(if (sel) Ag.GreenBg else Color(0xFF161616), RoundedCornerShape(6.dp))
                                .border(BorderStroke(1.dp, if (sel) Ag.Green else Ag.Border), RoundedCornerShape(6.dp))
                                .clickable { profileIdx = i }.padding(vertical = 8.dp),
                            contentAlignment = Alignment.Center,
                        ) {
                            Text(pr.name, color = if (sel) Ag.Green else Ag.Dim,
                                fontSize = 12.sp, fontWeight = if (sel) FontWeight.Bold else FontWeight.Normal)
                        }
                    }
                }
                Spacer(Modifier.height(10.dp))
                Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.SpaceBetween) {
                    Text("작업 속도", color = Ag.Dim, fontSize = 11.sp)
                    Text("${"%.1f".format(speed)} m/s", color = Ag.Green,
                        fontSize = 12.sp, fontFamily = FontFamily.Monospace)
                }
                Slider(value = speed, onValueChange = { speed = it }, valueRange = 0.5f..3f,
                    colors = SliderDefaults.colors(thumbColor = Ag.Green, activeTrackColor = Ag.Green))
                if (levelMode) {
                    Spacer(Modifier.height(8.dp))
                    Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.SpaceBetween) {
                        Text("목표 표고", color = Ag.Dim, fontSize = 11.sp)
                        Text("${if(targetLv>=0)"+" else ""}${"%.0f".format(targetLv)} cm",
                            color = Ag.Yellow, fontSize = 12.sp, fontFamily = FontFamily.Monospace)
                    }
                    Slider(value = targetLv, onValueChange = { targetLv = it }, valueRange = -10f..10f,
                        colors = SliderDefaults.colors(thumbColor = Ag.Yellow, activeTrackColor = Ag.Yellow))
                }
            }
        }

        // ══ FIELD CANVAS ════════════════════════════════
        Box(Modifier.fillMaxWidth().weight(1f, fill = false).height(270.dp)) {
            AgmoFieldCanvas(
                sim       = if (engaged) sim else SimState(),
                levelMode = levelMode,
                targetLv  = targetLv,
                textMeasurer = textMeasurer,
                modifier  = Modifier.fillMaxSize(),
            )
            // 라인 오버레이
            Row(
                Modifier.align(Alignment.TopStart).padding(8.dp),
                horizontalArrangement = Arrangement.spacedBy(6.dp),
            ) {
                InfoOverlay("LINE", "${metrics.passNo+1}/8", Ag.Green)
                InfoOverlay(
                    if (levelMode) "표고오차" else "잔여거리",
                    if (levelMode) "${if(metrics.curElev>=0)"+" else ""}${"%.1f".format(metrics.curElev)}cm"
                    else "${metrics.leftDist}m",
                    if (levelMode) elevColor else Ag.Text
                )
            }
            // 안정 뱃지
            if (engaged) {
                Row(
                    Modifier.align(Alignment.TopEnd).padding(8.dp)
                        .background(Color(0xCC0A1409), RoundedCornerShape(5.dp))
                        .border(BorderStroke(1.dp, if (metrics.stable) Ag.Green else Ag.Orange), RoundedCornerShape(5.dp))
                        .padding(horizontal = 8.dp, vertical = 4.dp),
                    verticalAlignment = Alignment.CenterVertically,
                    horizontalArrangement = Arrangement.spacedBy(5.dp),
                ) {
                    Box(Modifier.size(6.dp).background(
                        if (metrics.stable) Ag.Green else Ag.Orange, RoundedCornerShape(50)))
                    Text(if (metrics.stable) "안정" else "불안정",
                        color = if (metrics.stable) Ag.Green else Ag.Orange,
                        fontSize = 10.sp, fontWeight = FontWeight.Bold)
                }
            }
            // STANDBY 오버레이
            if (!engaged) {
                Box(Modifier.fillMaxSize().background(Color(0x990A1509)),
                    contentAlignment = Alignment.Center) {
                    Text("STANDBY",
                        color = Color(0x55000000),
                        fontSize = 22.sp, fontWeight = FontWeight.Bold,
                        letterSpacing = 5.sp)
                }
            }
        }

        // ══ XTE 편차 바 ══════════════════════════════════
        Row(
            Modifier.fillMaxWidth().background(Ag.Surface)
                .border(BorderStroke(0.5.dp, Ag.Border))
                .padding(horizontal = 12.dp, vertical = 8.dp),
            verticalAlignment = Alignment.CenterVertically,
            horizontalArrangement = Arrangement.spacedBy(8.dp),
        ) {
            val dispVal = if (levelMode)
                "${if(metrics.curElev>=0)"+" else ""}${"%.1f".format(metrics.curElev)}"
            else
                "${if(metrics.xte>=0)"+" else ""}${"%.1f".format(metrics.xte)}"
            Text("$dispVal cm",
                color = if (levelMode) elevColor else xteColor,
                fontSize = 20.sp, fontFamily = FontFamily.Monospace,
                fontWeight = FontWeight.Bold, modifier = Modifier.width(88.dp))
            // 편차 바
            Box(Modifier.weight(1f).height(20.dp)) {
                Box(Modifier.fillMaxSize().background(Color(0xFF0D1A0A), RoundedCornerShape(10.dp))
                    .border(BorderStroke(0.5.dp, Ag.Border), RoundedCornerShape(10.dp)))
                Box(Modifier.align(Alignment.Center).width(2.dp).fillMaxHeight()
                    .background(Ag.GreenDim))
                val pct = if (levelMode)
                    (50f + (metrics.curElev / 20f * 50f)).coerceIn(2f, 98f) / 100f
                else
                    (50f + (metrics.xte / 10f * 50f)).coerceIn(2f, 98f) / 100f
                Box(Modifier.fillMaxWidth().fillMaxHeight(),
                    contentAlignment = Alignment.CenterStart) {
                    Box(Modifier.fillMaxWidth(pct).fillMaxHeight(),
                        contentAlignment = Alignment.CenterEnd) {
                        Box(Modifier.size(14.dp).background(
                            if (levelMode) elevColor else xteColor, RoundedCornerShape(50)))
                    }
                }
            }
            Text(if (levelMode) "표고" else "XTE",
                color = Ag.Dim, fontSize = 10.sp, modifier = Modifier.width(28.dp))
        }

        // ══ 작업 모드 탭 ════════════════════════════════
        Row(Modifier.fillMaxWidth().background(Ag.Surface)
            .border(BorderStroke(0.5.dp, Ag.Border))) {
            WorkType.values().forEach { wt ->
                val sel = workType == wt
                Column(
                    Modifier.weight(1f).clickable { workType = wt }
                        .padding(vertical = 8.dp)
                        .border(BorderStroke(0.dp, Color.Transparent))
                        .run { if (sel) borderBottom(2.dp, Ag.Green) else this },
                    horizontalAlignment = Alignment.CenterHorizontally,
                    verticalArrangement = Arrangement.spacedBy(2.dp),
                ) {
                    Text(wt.icon, fontSize = 16.sp)
                    Text(wt.ko, color = if (sel) Ag.Green else Ag.Dim,
                        fontSize = 10.sp, fontWeight = if (sel) FontWeight.Bold else FontWeight.Normal)
                }
            }
        }

        // ══ 컨트롤 패널 (프로파일 + RPM + ENGAGE) ════════
        Row(
            Modifier.fillMaxWidth().background(Ag.Surface)
                .border(BorderStroke(0.5.dp, Ag.Border))
                .padding(10.dp),
            verticalAlignment = Alignment.CenterVertically,
            horizontalArrangement = Arrangement.spacedBy(8.dp),
        ) {
            // 프로파일 표시
            Column(
                Modifier.background(Ag.Card, RoundedCornerShape(6.dp))
                    .border(BorderStroke(1.dp, Ag.Border), RoundedCornerShape(6.dp))
                    .padding(horizontal = 12.dp, vertical = 6.dp),
                horizontalAlignment = Alignment.CenterHorizontally,
            ) {
                Text("프로파일", color = Ag.Dim, fontSize = 9.sp)
                Text(PROFILES[profileIdx].name, color = Ag.Green, fontSize = 13.sp,
                    fontWeight = FontWeight.Bold)
            }
            // RPM
            Column(
                Modifier.weight(1f)
                    .background(Ag.Card, RoundedCornerShape(6.dp))
                    .border(BorderStroke(1.dp, Ag.Border), RoundedCornerShape(6.dp))
                    .padding(vertical = 6.dp),
                horizontalAlignment = Alignment.CenterHorizontally,
            ) {
                Text("RPM", color = Ag.Dim, fontSize = 9.sp)
                Text(
                    if (engaged) "${if(metrics.motorRpm>=0)"+" else ""}${metrics.motorRpm}"
                    else "---",
                    color = if (engaged) Ag.Green else Ag.Dim,
                    fontSize = 18.sp, fontFamily = FontFamily.Monospace, fontWeight = FontWeight.Bold
                )
            }
            // ENGAGE 버튼
            Button(
                onClick = { engaged = !engaged },
                colors = ButtonDefaults.buttonColors(
                    containerColor = if (engaged) Color(0xFF3A0000) else Color(0xFF0D3018)),
                border = BorderStroke(2.dp, if (engaged) Ag.Red else Ag.Green),
                shape = RoundedCornerShape(10.dp),
                modifier = Modifier.width(96.dp).height(60.dp),
            ) {
                Column(horizontalAlignment = Alignment.CenterHorizontally) {
                    if (engaged) Text("작동 중", color = Color(0xAAFF6666), fontSize = 9.sp)
                    Text(if (engaged) "해제" else "작동",
                        color = if (engaged) Ag.Red else Ag.Green,
                        fontSize = if (engaged) 14.sp else 16.sp,
                        fontWeight = FontWeight.Bold)
                    Text(if (engaged) "DISENGAGE" else "ENGAGE",
                        color = if (engaged) Color(0x66E74C3C) else Color(0x662DB33A),
                        fontSize = 8.sp)
                }
            }
        }

        // ══ 균평 범례 ════════════════════════════════════
        if (levelMode) {
            Column(
                Modifier.fillMaxWidth().background(Ag.Card)
                    .border(BorderStroke(0.5.dp, Ag.Border))
                    .padding(horizontal = 12.dp, vertical = 8.dp)
            ) {
                Text("표고 편차 (목표 기준)", color = Ag.Dim, fontSize = 10.sp,
                    fontWeight = FontWeight.Bold, letterSpacing = 0.5.sp,
                    modifier = Modifier.padding(bottom = 4.dp))
                Row(Modifier.fillMaxWidth().height(14.dp).clip(RoundedCornerShape(3.dp))) {
                    listOf(Color(0xFF0000CC),Color(0xFF0044FF),Color(0xFF0099FF),Color(0xFF00DDFF),
                        Color(0xFF00C864),Color(0xFFA0E600),Color(0xFFFFC800),Color(0xFFFF6400),Color(0xFFC80000)
                    ).forEach { c -> Box(Modifier.weight(1f).fillMaxHeight().background(c)) }
                }
                Row(Modifier.fillMaxWidth(), horizontalArrangement = Arrangement.SpaceBetween) {
                    listOf("-10","-5","0","+5","+10").forEach { t ->
                        Text("${t}cm", color = Ag.Dim, fontSize = 9.sp)
                    }
                }
                Spacer(Modifier.height(6.dp))
                Row(verticalAlignment = Alignment.CenterVertically,
                    horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                    Text("목표:", color = Ag.Dim, fontSize = 10.sp)
                    Slider(value = targetLv, onValueChange = { targetLv = it },
                        valueRange = -10f..10f, modifier = Modifier.weight(1f),
                        colors = SliderDefaults.colors(thumbColor = Ag.Yellow, activeTrackColor = Ag.Yellow))
                    Text("${if(targetLv>=0)"+" else ""}${"%.0f".format(targetLv)}cm",
                        color = Ag.Yellow, fontSize = 13.sp, fontFamily = FontFamily.Monospace,
                        modifier = Modifier.width(44.dp))
                }
            }
            // 균평 수치 그리드
            Row(Modifier.fillMaxWidth().border(BorderStroke(0.5.dp, Ag.Border))) {
                listOf(
                    Triple("현재 오차", "${if(metrics.curElev>=0)"+" else ""}${"%.2f".format(metrics.curElev)}cm", elevColor),
                    Triple("목표 표고", "${if(targetLv>=0)"+" else ""}${"%.0f".format(targetLv)}cm", Ag.Yellow),
                ).forEach { (l, v, c) ->
                    Column(
                        Modifier.weight(1f).background(Ag.Card)
                            .border(BorderStroke(0.5.dp, Ag.Border))
                            .padding(10.dp)
                    ) {
                        Text(l, color = Ag.Dim, fontSize = 9.sp)
                        Text(v, color = c, fontSize = 18.sp, fontFamily = FontFamily.Monospace,
                            fontWeight = FontWeight.Bold)
                    }
                }
            }
        }

        // ══ 하단 정보 바 ═════════════════════════════════
        Row(
            Modifier.fillMaxWidth().background(Color(0xFF0D0D0D))
                .padding(horizontal = 12.dp, vertical = 6.dp),
            horizontalArrangement = Arrangement.SpaceEvenly,
        ) {
            listOf(
                "헤딩" to "%03d°".format(metrics.headDeg),
                "라인" to "${metrics.passNo+1}/8",
                "잔여" to "${metrics.leftDist}m",
                if (levelMode) "표고" to "${"%.1f".format(metrics.curElev)}cm"
                else "XTE" to "${"%.1f".format(metrics.xte)}cm",
            ).forEach { (k, v) ->
                Column(horizontalAlignment = Alignment.CenterHorizontally) {
                    Text(k, color = Ag.Dim, fontSize = 9.sp, letterSpacing = 0.5.sp)
                    Text(v, color = Ag.Text, fontSize = 13.sp,
                        fontFamily = FontFamily.Monospace, fontWeight = FontWeight.Bold)
                }
            }
        }
    }
}

// ── 헬퍼 컴포저블 ──────────────────────────────────────────
@Composable
fun InfoOverlay(label: String, value: String, valueColor: Color) {
    Column(
        Modifier.background(Color(0xCC0A1409), RoundedCornerShape(5.dp))
            .border(BorderStroke(0.5.dp, Color(0xFF243020)), RoundedCornerShape(5.dp))
            .padding(horizontal = 10.dp, vertical = 5.dp)
    ) {
        Text(label, color = Color(0xFF78909C), fontSize = 9.sp, letterSpacing = 0.5.sp)
        Text(value, color = valueColor, fontSize = 18.sp,
            fontFamily = FontFamily.Monospace, fontWeight = FontWeight.Bold, lineHeight = 20.sp)
    }
}

// Modifier 확장: 하단 테두리
fun Modifier.borderBottom(width: Dp, color: Color): Modifier =
    this.drawWithContent {
        drawContent()
        drawLine(color, start = Offset(0f, size.height), end = Offset(size.width, size.height), strokeWidth = width.toPx())
    }
