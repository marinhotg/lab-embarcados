package br.usp.pcs3858.rover.map

import br.usp.pcs3858.rover.telemetry.Pose
import br.usp.pcs3858.rover.telemetry.Telemetry
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.exp
import kotlin.math.roundToInt
import kotlin.math.sin

data class SonarGeometry(val offsetX: Double, val offsetY: Double, val bearing: Double)

/* Mirrors SONAR_OFFSET_* in firmware/main/config.h; calibrate both together. */
val SONAR_GEOMETRY = mapOf(
    "front" to SonarGeometry(0.10, 0.00, 0.0),
    "rear" to SonarGeometry(-0.10, 0.00, PI),
    "left" to SonarGeometry(0.00, 0.07, PI / 2),
    "right" to SonarGeometry(0.00, -0.07, -PI / 2),
)

data class Ray(val x0: Double, val y0: Double, val x1: Double, val y1: Double)

/* Port of OccupancyMap in client/map_client.py: a log-odds grid where each
 * sonar reading frees the cells along its cone and marks the echo cell. Each
 * reading is projected from the pose the rover had when it was measured. */
class OccupancyMap {

    private val times = ArrayList<Long>()
    private val poses = ArrayList<Pose>()
    private val seen = HashSet<String>()
    private var lastSeq: Long? = null
    private var lastTs = -1L

    val grid = HashMap<Long, Float>()
    val trajectory = ArrayList<Pose>()
    val rays = HashMap<String, Ray>()
    var pose: Pose? = null
        private set

    fun ingest(t: Telemetry) {
        if (t.timestampMs < lastTs) reset()
        lastTs = t.timestampMs

        if (t.seq != null && t.seq == lastSeq) return
        lastSeq = t.seq

        times += t.timestampMs
        poses += t.pose
        trajectory += t.pose
        pose = t.pose
        project(t)
    }

    fun reset() {
        times.clear(); poses.clear(); trajectory.clear()
        grid.clear(); seen.clear(); rays.clear()
        lastSeq = null
        pose = null
    }

    /* Occupancy probability; cells without evidence are not stored (unknown). */
    fun probability(logOdds: Float): Float = (1.0 / (1.0 + exp(-logOdds.toDouble()))).toFloat()

    private fun project(t: Telemetry) {
        rays.clear()
        for ((name, g) in SONAR_GEOMETRY) {
            val dist = t.sonar[name] ?: continue
            val age = t.sonarAgeMs[name] ?: continue
            val p = poseAt(t.timestampMs - age) ?: continue

            val sx = p.x + g.offsetX * cos(p.yaw) - g.offsetY * sin(p.yaw)
            val sy = p.y + g.offsetX * sin(p.yaw) + g.offsetY * cos(p.yaw)
            val heading = p.yaw + g.bearing

            if (age <= MAX_RAY_AGE_MS) {
                rays[name] = Ray(sx, sy, sx + dist * cos(heading), sy + dist * sin(heading))
            }

            // Telemetry runs at 200 ms and each sonar at 400 ms: the same
            // reading arrives twice.
            if (!seen.add("$name:${t.timestampMs - age}")) continue
            trace(sx, sy, dist, heading)
        }
    }

    /* Free along the whole cone, occupied only at the central echo: the echo
     * may have come from anywhere in the cone. */
    private fun trace(sx: Double, sy: Double, dist: Double, heading: Double) {
        val cx0 = (sx / CELL_M).roundToInt()
        val cy0 = (sy / CELL_M).roundToInt()
        for (offset in doubleArrayOf(-HALF_CONE_RAD, 0.0, HALF_CONE_RAD)) {
            val ang = heading + offset
            val reach = if (offset == 0.0) dist else dist * 0.95
            val cx = ((sx + reach * cos(ang)) / CELL_M).roundToInt()
            val cy = ((sy + reach * sin(ang)) / CELL_M).roundToInt()
            bresenham(cx0, cy0, cx, cy) { x, y -> update(x, y, L_FREE) }
            if (offset == 0.0) update(cx, cy, L_OCCUPIED)
        }
    }

    private fun update(cx: Int, cy: Int, delta: Float) {
        val key = key(cx, cy)
        grid[key] = ((grid[key] ?: 0f) + delta).coerceIn(-L_CLAMP, L_CLAMP)
    }

    private fun poseAt(t: Long): Pose? {
        if (times.isEmpty()) return null
        if (t <= times.first()) return poses.first()
        if (t >= times.last()) return poses.last()

        var i = times.binarySearch(t)
        if (i >= 0) return poses[i]
        i = -i - 1
        val t0 = times[i - 1]
        val t1 = times[i]
        val p0 = poses[i - 1]
        val p1 = poses[i]
        val f = (t - t0).toDouble() / (t1 - t0)
        // Shortest arc, so crossing +-pi does not spin the other way round.
        val dYaw = atan2(sin(p1.yaw - p0.yaw), cos(p1.yaw - p0.yaw))
        return Pose(p0.x + f * (p1.x - p0.x), p0.y + f * (p1.y - p0.y), p0.yaw + f * dYaw)
    }

    companion object {
        const val CELL_M = 0.06
        const val MAX_RAY_AGE_MS = 400L
        const val L_FREE = -0.7f
        const val L_OCCUPIED = 1.1f
        const val L_CLAMP = 4f
        val HALF_CONE_RAD = Math.toRadians(7.5)

        fun key(cx: Int, cy: Int): Long = (cx.toLong() shl 32) or (cy.toLong() and 0xffffffffL)
        fun cellX(key: Long): Int = (key shr 32).toInt()
        fun cellY(key: Long): Int = key.toInt()
    }
}

/* Cells crossed by the ray, excluding the last one. */
private inline fun bresenham(x0: Int, y0: Int, x1: Int, y1: Int, visit: (Int, Int) -> Unit) {
    var x = x0
    var y = y0
    val dx = abs(x1 - x0)
    val dy = abs(y1 - y0)
    val sx = if (x0 < x1) 1 else -1
    val sy = if (y0 < y1) 1 else -1
    var err = dx - dy
    while (x != x1 || y != y1) {
        visit(x, y)
        val e2 = 2 * err
        if (e2 > -dy) { err -= dy; x += sx }
        if (e2 < dx) { err += dx; y += sy }
    }
}
