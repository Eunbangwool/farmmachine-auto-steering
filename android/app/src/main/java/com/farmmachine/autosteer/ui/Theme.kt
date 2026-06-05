package com.farmmachine.autosteer.ui

import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.darkColorScheme
import androidx.compose.runtime.Composable
import androidx.compose.ui.graphics.Color

// AGMO Solution 컬러 팔레트
object Ag {
    val Bg       = Color(0xFF111111)
    val Surface  = Color(0xFF1A1A1A)
    val Card     = Color(0xFF1E2820)
    val Border   = Color(0xFF243020)
    val Green    = Color(0xFF2DB33A)
    val GreenDim = Color(0xFF1A6622)
    val GreenBg  = Color(0xFF0F2A12)
    val Orange   = Color(0xFFE67E22)
    val Red      = Color(0xFFE74C3C)
    val Text     = Color(0xFFECEFF1)
    val Dim      = Color(0xFF78909C)
    val Yellow   = Color(0xFFF0B429)
    val MapBg    = Color(0xFF0A1509)
}

fun levelColor(diff: Float): Color = when {
    diff < -8f -> Color(0xFF0000CC)
    diff < -5f -> Color(0xFF0044FF)
    diff < -3f -> Color(0xFF0099FF)
    diff < -1f -> Color(0xFF00DDFF)
    diff <  1f -> Color(0xFF00C864)
    diff <  3f -> Color(0xFFA0E600)
    diff <  5f -> Color(0xFFFFC800)
    diff <  8f -> Color(0xFFFF6400)
    else       -> Color(0xFFC80000)
}

@Composable
fun AgmoTheme(content: @Composable () -> Unit) {
    MaterialTheme(
        colorScheme = darkColorScheme(
            primary    = Ag.Green,
            background = Ag.Bg,
            surface    = Ag.Surface,
        ),
        content = content
    )
}
