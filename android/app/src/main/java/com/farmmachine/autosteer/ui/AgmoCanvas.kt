package com.farmmachine.autosteer.ui

import androidx.compose.foundation.Canvas
import androidx.compose.runtime.Composable
import androidx.compose.ui.Modifier
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.geometry.Size
import androidx.compose.ui.graphics.*
import androidx.compose.ui.graphics.drawscope.*
import androidx.compose.ui.text.*
import androidx.compose.ui.unit.sp
import kotlin.math.*

@Composable
fun AgmoFieldCanvas(
    sim: SimState,
    levelMode: Boolean,
    targetLv: Float,
    textMeasurer: TextMeasurer,
    modifier: Modifier = Modifier,
) {
    Canvas(modifier = modifier) {
        val W = size.width; val H = size.height
        val sc = min(W, H) / 90f
        val ox = W/2 - sim.tx*sc
        val oy = H/2 - sim.ty*sc
        fun sx(x: Float) = x*sc + ox
        fun sy(y: Float) = y*sc + oy

        // 배경
        drawRect(Ag.MapBg, size = size)

        // 레벨 히트맵
        if (levelMode) {
            val COLS = 32; val ROWS = 32
            val cw = W / COLS; val ch = H / ROWS
            for (r in 0 until ROWS) for (c in 0 until COLS) {
                val wx = (c.toFloat()/COLS*100-50) + sim.tx
                val wy = (r.toFloat()/ROWS*100-50) + sim.ty
                val elev = Algo.elevNoise(wx, wy) - targetLv
                drawRect(
                    levelColor(elev).copy(alpha = 0.62f),
                    topLeft = Offset(c*cw, r*ch),
                    size = Size(cw+1, ch+1)
                )
            }
        }

        // 격자
        val gridColor = if (levelMode) Color(0x10FFFFFF) else Color(0xFF142A10)
        for (i in -12..12) {
            drawLine(gridColor, Offset(sx(i*8f), sy(-90f)), Offset(sx(i*8f), sy(90f)), 0.5f)
            drawLine(gridColor, Offset(sx(-90f), sy(i*8f)), Offset(sx(90f), sy(i*8f)), 0.5f)
        }

        // 완료 구역
        if (!levelMode) {
            for (p in 0 until sim.passNo) {
                val lx = p*6f - 24f
                drawRect(
                    Color(0x0A1EA03C),
                    topLeft = Offset(sx(lx-3f), sy(-60f)),
                    size = Size(6*sc, 120*sc)
                )
            }
        }

        // AB 라인 (8패스, gap=6)
        val PASSES = 8; val GAP = 6f
        for (p in 0 until PASSES) {
            val lx = p*GAP - (PASSES*GAP)/2f
            val cur = p == sim.passNo
            val pathEffect = if (cur) null else PathEffect.dashPathEffect(floatArrayOf(6f,10f))
            drawLine(
                color = if (cur) Ag.Green else Color(0xFF1A3D18),
                start = Offset(sx(lx), sy(-60f)),
                end   = Offset(sx(lx), sy(60f)),
                strokeWidth = if (cur) 2.5f else 1f,
                pathEffect = pathEffect,
            )
            if (cur) {
                drawText(textMeasurer, "A${p+1}",
                    topLeft = Offset(sx(lx)+4f, sy(-60f)+4f),
                    style = TextStyle(color=Ag.Green, fontSize=10.sp, fontWeight=androidx.compose.ui.text.font.FontWeight.Bold)
                )
            }
        }

        // 경로 흔적
        if (sim.trail.size > 1) {
            val path = Path()
            val (fx, fy) = sim.trail.first()
            path.moveTo(sx(fx), sy(fy))
            sim.trail.drop(1).forEach { (tx2,ty2) -> path.lineTo(sx(tx2), sy(ty2)) }
            drawPath(path, Color(0x662DB33A), style = Stroke(width=2f))
        }

        // 트랙터
        val txPx = sx(sim.tx); val tyPx = sy(sim.ty)
        withTransform({
            translate(txPx, tyPx)
            rotate(degrees = (-sim.th * 180f / PI.toFloat()) + 90f)
        }) {
            // 본체
            drawRoundRect(Ag.Card, topLeft=Offset(-9f,-16f), size=Size(18f,32f), cornerRadius=androidx.compose.ui.geometry.CornerRadius(4f))
            drawRoundRect(Color.Transparent, topLeft=Offset(-9f,-16f), size=Size(18f,32f),
                cornerRadius=androidx.compose.ui.geometry.CornerRadius(4f),
                style=Stroke(1.5f), colorFilter=ColorFilter.tint(Ag.Green))
            // 캐빈
            drawRoundRect(Color(0xFF243D22), topLeft=Offset(-6f,-10f), size=Size(12f,14f), cornerRadius=androidx.compose.ui.geometry.CornerRadius(2f))
            // 방향 화살표
            val arrow = Path().apply { moveTo(0f,-20f); lineTo(-5f,-12f); lineTo(5f,-12f); close() }
            drawPath(arrow, Ag.Green)
            // 바퀴 (조향각)
            val sw = max(-0.4f, min(0.4f, sim.steer))
            listOf(-6f to -16f, 6f to -16f).forEach { (bx, by) ->
                withTransform({ translate(bx, by); rotate(sw * 180f / PI.toFloat()) }) {
                    drawRoundRect(Color(0xFF0F2010), topLeft=Offset(-2.5f,-5f), size=Size(5f,10f), cornerRadius=androidx.compose.ui.geometry.CornerRadius(1f))
                    drawRoundRect(Color.Transparent, topLeft=Offset(-2.5f,-5f), size=Size(5f,10f),
                        cornerRadius=androidx.compose.ui.geometry.CornerRadius(1f),
                        style=Stroke(1f), colorFilter=ColorFilter.tint(Ag.Green))
                }
            }
        }

        // 레벨 모드: 현재 지점 표고 텍스트
        if (levelMode) {
            val elev = Algo.elevNoise(sim.tx, sim.ty) - targetLv
            val ec = levelColor(elev)
            drawText(textMeasurer,
                text = "${if(elev>=0)"+" else ""}${"%.1f".format(elev)}cm",
                topLeft = Offset(txPx+14f, tyPx-20f),
                style = TextStyle(color=ec, fontSize=13.sp, fontWeight=androidx.compose.ui.text.font.FontWeight.Bold)
            )
        }

        // 나침반
        val cxc = W-34f; val cyc = 34f; val cr = 22f
        drawCircle(Color(0xE00A1409), cr, Offset(cxc,cyc))
        drawCircle(Color.Transparent, cr, Offset(cxc,cyc), style=Stroke(1.5f), colorFilter=ColorFilter.tint(Color(0xFF1E4A1A)))
        withTransform({ translate(cxc,cyc); rotate((-sim.th * 180f / PI.toFloat())) }) {
            val nArrow = Path().apply { moveTo(0f,-cr+5f); lineTo(-4f,-5f); lineTo(4f,-5f); close() }
            val sArrow = Path().apply { moveTo(0f,cr-5f); lineTo(-4f,5f); lineTo(4f,5f); close() }
            drawPath(nArrow, Ag.Green)
            drawPath(sArrow, Color(0xFF263A24))
        }
        drawText(textMeasurer, "N",
            topLeft = Offset(cxc-5f, cyc-cr+4f),
            style = TextStyle(color=Ag.Green, fontSize=9.sp, fontWeight=androidx.compose.ui.text.font.FontWeight.Bold)
        )
    }
}
