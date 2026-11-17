package br.usp.pcs3858.rover.telemetry

import br.usp.pcs3858.rover.map.SONAR_GEOMETRY
import java.util.Random
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sin
import kotlinx.coroutines.delay
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.flow

/* Port of client/simulation.py: a room with a box in the middle, driven as an
 * L with two turns. Loops forever; each loop restarts the timestamp, which the
 * map takes as a reboot and clears. */
object Simulator {

    private const val WALL = 0.05
    private val ROOM = doubleArrayOf(0.0, -1.0, 3.6, 1.8)

    private val OBSTACLES = listOf(
        doubleArrayOf(ROOM[0], ROOM[1], ROOM[2], ROOM[1] + WALL),
        doubleArrayOf(ROOM[0], ROOM[3] - WALL, ROOM[2], ROOM[3]),
        doubleArrayOf(ROOM[0], ROOM[1], ROOM[0] + WALL, ROOM[3]),
        doubleArrayOf(ROOM[2] - WALL, ROOM[1], ROOM[2], ROOM[3]),
        doubleArrayOf(1.30, 0.10, 1.95, 0.75),
    )

    /* speed m/s, rotation rad/s, duration s */
    private val ROUTE = listOf(
        Triple(0.20, 0.0, 12.0),
        Triple(0.00, 0.60, 2.6),
        Triple(0.20, 0.0, 9.0),
        Triple(0.00, 0.60, 2.6),
        Triple(0.20, 0.0, 11.0),
    )

    private val START = Pose(0.35, -0.55, 0.0)
    private const val SONAR_PERIOD_MS = 100L
    private const val TELEMETRY_PERIOD_MS = 200L
    private const val NOISE_M = 0.010

    fun feed(): Flow<Telemetry> = flow {
        while (true) {
            for (sample in samples()) {
                emit(sample)
                delay(TELEMETRY_PERIOD_MS)
            }
        }
    }

    private fun samples(): Sequence<Telemetry> = sequence {
        val rng = Random(7)
        val names = Telemetry.SONARS
        val last = HashMap<String, Pair<Double, Long>>()
        val step = SONAR_PERIOD_MS / 1000.0
        var x = START.x
        var y = START.y
        var yaw = START.yaw
        var i = 0L
        var seq = 0L

        for ((v, w, duration) in ROUTE) {
            repeat((duration / step).toInt()) {
                yaw += w * step
                x += v * cos(yaw) * step
                y += v * sin(yaw) * step
                val t = i * SONAR_PERIOD_MS
                val pose = Pose(x, y, yaw)

                val target = names[(i % names.size).toInt()]
                measure(pose, target, rng)?.let { last[target] = it to t }
                i++

                if (t % TELEMETRY_PERIOD_MS == 0L) {
                    yield(
                        Telemetry(
                            seq = seq++,
                            timestampMs = t,
                            pose = pose,
                            speed = v,
                            sonar = names.associateWith { last[it]?.first },
                            sonarAgeMs = names.associateWith { n -> last[n]?.let { t - it.second } },
                            timeoutActive = false,
                        )
                    )
                }
            }
        }
    }

    private fun measure(pose: Pose, name: String, rng: Random): Double? {
        val g = SONAR_GEOMETRY.getValue(name)
        val sx = pose.x + g.offsetX * cos(pose.yaw) - g.offsetY * sin(pose.yaw)
        val sy = pose.y + g.offsetX * sin(pose.yaw) + g.offsetY * cos(pose.yaw)
        val ang = pose.yaw + g.bearing
        val hit = OBSTACLES.mapNotNull { rayBox(sx, sy, ang, it) }.minOrNull() ?: return null
        val d = hit + rng.nextGaussian() * NOISE_M
        return if (d < 0.02 || d > 4.0) null else d
    }

    /* Distance along the ray to the rectangle, or null if it misses. */
    private fun rayBox(ox: Double, oy: Double, ang: Double, box: DoubleArray): Double? {
        val dirs = doubleArrayOf(cos(ang), sin(ang))
        val origins = doubleArrayOf(ox, oy)
        var enter = 0.0
        var exit = Double.POSITIVE_INFINITY
        for (axis in 0..1) {
            val o = origins[axis]
            val d = dirs[axis]
            val low = box[axis]
            val high = box[axis + 2]
            if (abs(d) < 1e-9) {
                if (o < low || o > high) return null
                continue
            }
            var ta = (low - o) / d
            var tb = (high - o) / d
            if (ta > tb) ta = tb.also { tb = ta }
            enter = max(enter, ta)
            exit = min(exit, tb)
        }
        return if (enter > exit || exit < 0) null else enter
    }
}
