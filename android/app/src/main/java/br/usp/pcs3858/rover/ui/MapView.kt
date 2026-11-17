package br.usp.pcs3858.rover.ui

import androidx.compose.foundation.Canvas
import androidx.compose.runtime.Composable
import androidx.compose.ui.Modifier
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.geometry.Size
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.Path
import androidx.compose.ui.graphics.drawscope.Stroke
import androidx.compose.ui.graphics.lerp
import br.usp.pcs3858.rover.map.OccupancyMap
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sin

val SONAR_COLORS = mapOf(
    "front" to Color(0xFFFF5252),
    "rear" to Color(0xFFB388FF),
    "left" to Color(0xFF69F0AE),
    "right" to Color(0xFFFFAB40),
)

/* World frame: x forward from the start pose, y to the left. Drawn with x to
 * the right and y up, scaled to fit everything seen so far. `version` only
 * forces a redraw when the map changes. */
@Composable
fun MapView(
    map: OccupancyMap,
    version: Int,
    unknown: Color,
    free: Color,
    occupied: Color,
    track: Color,
    rover: Color,
    modifier: Modifier = Modifier,
) {
    Canvas(modifier = modifier) {
        @Suppress("UNUSED_EXPRESSION") version
        drawRect(unknown)

        val cell = OccupancyMap.CELL_M
        var minX = Double.MAX_VALUE
        var maxX = -Double.MAX_VALUE
        var minY = Double.MAX_VALUE
        var maxY = -Double.MAX_VALUE
        fun include(x: Double, y: Double) {
            minX = min(minX, x); maxX = max(maxX, x)
            minY = min(minY, y); maxY = max(maxY, y)
        }
        for (key in map.grid.keys) {
            include(OccupancyMap.cellX(key) * cell, OccupancyMap.cellY(key) * cell)
        }
        map.trajectory.forEach { include(it.x, it.y) }
        val pose = map.pose
        if (pose == null) {
            include(-1.0, -1.0); include(1.0, 1.0)
        } else {
            include(pose.x - 1.0, pose.y - 1.0); include(pose.x + 1.0, pose.y + 1.0)
        }

        val margin = 0.2
        minX -= margin; maxX += margin; minY -= margin; maxY += margin
        val scale = min(size.width / (maxX - minX), size.height / (maxY - minY)).toFloat()
        val ox = (size.width - (maxX - minX).toFloat() * scale) / 2f
        val oy = (size.height - (maxY - minY).toFloat() * scale) / 2f
        fun sx(x: Double) = ox + ((x - minX) * scale).toFloat()
        fun sy(y: Double) = size.height - oy - ((y - minY) * scale).toFloat()

        val side = (cell * scale).toFloat()
        for ((key, logOdds) in map.grid) {
            if (abs(logOdds) < 0.1f) continue
            val p = map.probability(logOdds)
            val color = if (p < 0.5f) lerp(unknown, free, (0.5f - p) * 2f) else lerp(unknown, occupied, (p - 0.5f) * 2f)
            val cx = OccupancyMap.cellX(key) * cell
            val cy = OccupancyMap.cellY(key) * cell
            drawRect(color, topLeft = Offset(sx(cx) - side / 2, sy(cy) - side / 2), size = Size(side, side))
        }

        if (map.trajectory.size > 1) {
            val path = Path()
            map.trajectory.forEachIndexed { i, p ->
                if (i == 0) path.moveTo(sx(p.x), sy(p.y)) else path.lineTo(sx(p.x), sy(p.y))
            }
            drawPath(path, track, style = Stroke(width = 2f))
        }

        for ((name, ray) in map.rays) {
            val color = SONAR_COLORS[name] ?: track
            drawLine(color, Offset(sx(ray.x0), sy(ray.y0)), Offset(sx(ray.x1), sy(ray.y1)), strokeWidth = 3f)
            drawCircle(color, radius = 5f, center = Offset(sx(ray.x1), sy(ray.y1)))
        }

        if (pose != null) {
            val hl = 0.10
            val hw = 0.075
            val c = cos(pose.yaw)
            val s = sin(pose.yaw)
            val corners = listOf(hl to hw, hl to -hw, -hl to -hw, -hl to hw).map { (dx, dy) ->
                Offset(sx(pose.x + dx * c - dy * s), sy(pose.y + dx * s + dy * c))
            }
            val body = Path().apply {
                moveTo(corners[0].x, corners[0].y)
                corners.drop(1).forEach { lineTo(it.x, it.y) }
                close()
            }
            drawPath(body, rover)
            val nose = Offset(sx(pose.x + 0.16 * c), sy(pose.y + 0.16 * s))
            drawLine(rover, Offset(sx(pose.x), sy(pose.y)), nose, strokeWidth = 4f)
        }
    }
}
