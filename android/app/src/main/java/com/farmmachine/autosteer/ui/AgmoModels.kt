package com.farmmachine.autosteer.ui

import kotlin.math.*

enum class WorkType(val ko: String, val icon: String) {
    STEER("조향", "⚙"), PLOW("경운", "⛏"),
    PLANT("이앙", "🌾"), LEVEL("균평", "📐");
}

data class Profile(val name: String, val kH: Double, val kC: Double, val g: Double)

val PROFILES = listOf(
    Profile("일반",   100.0, 35.0,  1.00),
    Profile("과부하", 100.0, 100.0, 0.75),
    Profile("모래",   100.0, 35.0,  0.45),
)

data class SimState(
    val tx: Float = -28f, val ty: Float = -28f, val th: Float = (PI/2).toFloat(),
    val passNo: Int = 0, val steer: Float = 0f,
    val trail: List<Pair<Float,Float>> = emptyList(),
    val stepCount: Int = 0,
)

data class UiMetrics(
    val xte: Float = 0f, val curElev: Float = 0f,
    val steerDeg: Float = 0f, val headDeg: Int = 0,
    val passNo: Int = 0, val leftDist: Int = 0,
    val stable: Boolean = true, val humanInt: Boolean = false,
    val motorRpm: Int = 0,
)

// 알고리즘 (tracking.so ImplementReferenced)
object Algo {
    private const val WB  = 2.47
    private const val A2I = 1.2
    const val MAX_S = 0.436

    fun implRef(xteM: Double, hErr: Double, spd: Double, pr: Profile): Double {
        val ie = xteM + A2I * sin(hErr)
        val d  = (pr.kH/100)*hErr - atan2((pr.kC/100)*ie, max(0.1,spd)+0.5)
        return max(-MAX_S, min(MAX_S, d)) * pr.g
    }

    fun stepTractor(s: SimState, steer: Float, spd: Float, dt: Float): SimState {
        val d = max(-0.4f, min(0.4f, steer))
        val tx = s.tx + spd * cos(s.th) * dt
        val ty = s.ty + spd * sin(s.th) * dt
        val th = s.th + spd * tan(d.toDouble()).toFloat() / WB.toFloat() * dt
        val newTrail = (s.trail + (tx to ty)).takeLast(280)
        return s.copy(tx=tx, ty=ty, th=th, trail=newTrail, stepCount=s.stepCount+1)
    }

    fun elevNoise(x: Float, y: Float): Float =
        ((sin(x*0.37+2)*cos(y*0.29+3) + sin(x*0.11+y*0.17)*0.5) * 4.2).toFloat()
}
