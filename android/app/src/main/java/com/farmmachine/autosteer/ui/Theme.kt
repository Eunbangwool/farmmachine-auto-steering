package com.farmmachine.autosteer.ui.theme

import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.darkColorScheme
import androidx.compose.runtime.Composable
import androidx.compose.ui.graphics.Color

val Green = Color(0xFF00E676)
val DarkBg = Color(0xFF060E08)
val CardBg = Color(0xFF0D1F0F)
val BorderColor = Color(0xFF1A3D1E)
val TextDim = Color(0xFF3A6A3E)
val TextBright = Color(0xFFCFE8D0)
val LevelYellow = Color(0xFFFFCC00)
val WarnOrange = Color(0xFFFFAB00)
val ErrorRed = Color(0xFFF44336)

private val FarmColors = darkColorScheme(
    primary = Green,
    background = DarkBg,
    surface = CardBg,
    onBackground = TextBright,
    onSurface = TextBright,
)

@Composable
fun FarmMachineTheme(content: @Composable () -> Unit) {
    MaterialTheme(colorScheme = FarmColors, content = content)
}
