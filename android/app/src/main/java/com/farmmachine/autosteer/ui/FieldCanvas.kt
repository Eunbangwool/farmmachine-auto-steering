package com.farmmachine.autosteer.ui

import androidx.compose.foundation.Canvas
import androidx.compose.runtime.*
import androidx.compose.ui.Modifier
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.geometry.Size
import androidx.compose.ui.graphics.*
import androidx.compose.ui.graphics.drawscope.*
import kotlin.math.*

@Composable
fun FieldCanvas(
    engaged: Boolean, showField: Boolean,
    levelMode: Boolean, targetLevel: Double,
    xte: Double, steerDeg: Double, passNo: Int,
    modifier: Modifier = Modifier,
) {
    val tractorX by rememberUpdatedState(passNo * 7f - 28f)
    val tractorY by rememberUpdatedState(0f)
    val heading by rememberUpdatedState(PI.toFloat() / 2f)

    Canvas(modifier = modifier) {
        val W = size.width; val H = size.height
        val scale = min(W, H) / 90f
        val ox = W / 2 - tractorX * scale
        val oy = H / 2 - tractorY * scale

        fun sx(x: Float) = x * scale + ox
        fun sy(y: Float) = y * scale + oy

        // 배경
        drawRect(Color(0xFF0A1A0E), size = Size(W, H))

        // 격자
        for (i in -10..10) {
            drawLine(Color(0xFF0F2A12), Offset(sx(i * 8f), sy(-80f)), Offset(sx(i * 8f), sy(80f)), 0.5f)
            drawLine(Color(0xFF0F2A12), Offset(sx(-80f), sy(i * 8f)), Offset(sx(80f), sy(i * 8f)), 0.5f)
        }

        // 레벨 히트맵
        if (levelMode && showField) {
            val cols = 30; val rows = 30
            for (r in 0 until rows) for (c in 0 until cols) {
                val wx = c.toFloat() / cols * 80 - 40 + tractorX
                val wy = r.toFloat() / rows * 80 - 40 + tractorY
                val elev = (sin(wx * 0.37) * cos(wy * 0.29) + sin(wx * 0.11 + wy * 0.17) * 0.5f) * 4.5 - targetLevel
                val color = levelColorKt(elev.toFloat()).copy(alpha = 0.65f)
                val cellW = W / cols; val cellH = H / rows
                drawRect(color, Offset(c * cellW, r * cellH), Size(cellW + 1, cellH + 1))
            }
        }

        // 완료 구역 (조향 모드)
        if (!levelMode && showField) {
            for (p in 0 until passNo) {
                val lx = p * 7f - 28f
                drawRect(Color(0x1900B43C), Offset(sx(lx - 3.5f), sy(-50f)), Size(7 * scale, 100 * scale))
            }
        }

        // AB 라인
        val passes = 8
        for (p in 0 until passes) {
            val lx = p * 7f - 28f
            val color = if (p == passNo) Color(0xFF00E676) else Color(0xFF1A4A24)
            val width = if (p == passNo) 2f else 0.8f
            drawLine(color, Offset(sx(lx), sy(-50f)), Offset(sx(lx), sy(50f)), width)
        }

        // 트랙터 (화살표)
        val tx = sx(tractorX); val ty = sy(tractorY)
        withTransform({ translate(tx, ty); rotate(-90f) }) {
            drawRect(Color(0xFF1A3A22), Offset(-8f, -14f), Size(16f, 28f))
            drawRect(color = Color.Transparent, Offset(-8f, -14f), Size(16f, 28f),
                style = Stroke(1.5f), colorFilter = ColorFilter.tint(Color(0xFF00E676)))
            val path = Path().apply { moveTo(0f,-18f); lineTo(-5f,-10f); lineTo(5f,-10f); close() }
            drawPath(path, Color(0xFF00E676))
        }

        // 나침반
        drawCircle(Color(0xCC0A1A0E), 18f, Offset(W - 26f, 26f))
        drawCircle(Color(0xFF1E4A22), 18f, Offset(W - 26f, 26f), style = Stroke(1f))
        val path2 = Path().apply {
            moveTo(W-26f, 26f-14f); lineTo(W-29f, 26f-4f); lineTo(W-23f, 26f-4f); close()
        }
        drawPath(path2, Color(0xFF00E676))
    }
}
