package br.usp.pcs3858.rover.ui

import androidx.compose.foundation.Canvas
import androidx.compose.foundation.gestures.awaitEachGesture
import androidx.compose.foundation.gestures.awaitFirstDown
import androidx.compose.foundation.layout.size
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Modifier
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.drawscope.Stroke
import androidx.compose.ui.input.pointer.pointerInput
import androidx.compose.ui.input.pointer.positionChange
import androidx.compose.ui.unit.Dp
import androidx.compose.ui.unit.dp
import kotlin.math.abs
import kotlin.math.hypot

/* x and y in [-1, 1], y up; (0, 0) on release. */
@Composable
fun Joystick(
    onChange: (x: Float, y: Float) -> Unit,
    size: Dp = 260.dp,
    ring: Color,
    knob: Color,
    modifier: Modifier = Modifier,
) {
    var knobOffset by remember { mutableStateOf(Offset.Zero) }

    Canvas(
        modifier = modifier
            .size(size)
            .pointerInput(Unit) {
                val radius = this.size.width / 2f
                val center = Offset(radius, radius)

                fun report(position: Offset) {
                    var d = position - center
                    val len = hypot(d.x, d.y)
                    if (len > radius) d *= radius / len
                    knobOffset = d
                    val x = deadZone(d.x / radius)
                    val y = deadZone(-d.y / radius)
                    onChange(x, y)
                }

                awaitEachGesture {
                    val down = awaitFirstDown()
                    report(down.position)
                    do {
                        val event = awaitPointerEvent()
                        val change = event.changes.firstOrNull { it.id == down.id } ?: break
                        if (change.positionChange() != Offset.Zero) change.consume()
                        report(change.position)
                    } while (change.pressed)
                    knobOffset = Offset.Zero
                    onChange(0f, 0f)
                }
            }
    ) {
        val radius = this.size.minDimension / 2f
        drawCircle(color = ring.copy(alpha = 0.12f), radius = radius)
        drawCircle(color = ring, radius = radius - 2.dp.toPx(), style = Stroke(width = 2.dp.toPx()))
        drawLine(ring.copy(alpha = 0.3f), Offset(center.x, 0f), Offset(center.x, this.size.height))
        drawLine(ring.copy(alpha = 0.3f), Offset(0f, center.y), Offset(this.size.width, center.y))
        drawCircle(color = knob, radius = radius * 0.28f, center = center + knobOffset)
    }
}

private fun deadZone(v: Float): Float = if (abs(v) < 0.08f) 0f else v
