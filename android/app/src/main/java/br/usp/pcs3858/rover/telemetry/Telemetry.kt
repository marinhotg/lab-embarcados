package br.usp.pcs3858.rover.telemetry

import org.json.JSONObject

data class Pose(val x: Double, val y: Double, val yaw: Double)

/* One message of rover/{id}/telemetry, as documented in the README. */
data class Telemetry(
    val seq: Long?,
    val timestampMs: Long,
    val pose: Pose,
    val speed: Double?,
    val sonar: Map<String, Double?>,
    val sonarAgeMs: Map<String, Long?>,
    val timeoutActive: Boolean,
) {
    companion object {
        val SONARS = listOf("front", "rear", "left", "right")

        /* Null when the message lacks the fields the map needs. */
        fun parse(text: String): Telemetry? = runCatching {
            val o = JSONObject(text)
            val p = o.getJSONObject("pose")
            val dist = o.optJSONObject("sonar_m")
            val age = o.optJSONObject("sonar_age_ms")
            Telemetry(
                seq = o.optLongOrNull("seq"),
                timestampMs = o.getLong("timestamp_ms"),
                pose = Pose(p.getDouble("x_m"), p.getDouble("y_m"), p.getDouble("yaw_rad")),
                speed = o.optDoubleOrNull("speed_m_s"),
                sonar = SONARS.associateWith { dist?.optDoubleOrNull(it) },
                sonarAgeMs = SONARS.associateWith { age?.optLongOrNull(it) },
                timeoutActive = o.optJSONObject("safety")?.optBoolean("timeout_active") ?: false,
            )
        }.getOrNull()
    }
}

private fun JSONObject.optDoubleOrNull(key: String): Double? =
    if (!has(key) || isNull(key)) null else getDouble(key)

private fun JSONObject.optLongOrNull(key: String): Long? =
    if (!has(key) || isNull(key)) null else getLong(key)
