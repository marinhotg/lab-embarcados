"""Synthetic telemetry in the same JSON schema the firmware publishes, so the
real map_client.py consumes it, not a mock.

Scene: a rectangular room with a box in the middle. The rover drives an L with
two turns, so all four sonars sweep the walls and the box.
"""

from __future__ import annotations

import math
import random

WALL = 0.05
ROOM = (0.0, -1.0, 3.6, 1.8)

# Rectangles as (x_min, y_min, x_max, y_max).
OBSTACLES = [
    (ROOM[0], ROOM[1], ROOM[2], ROOM[1] + WALL),
    (ROOM[0], ROOM[3] - WALL, ROOM[2], ROOM[3]),
    (ROOM[0], ROOM[1], ROOM[0] + WALL, ROOM[3]),
    (ROOM[2] - WALL, ROOM[1], ROOM[2], ROOM[3]),
    (1.30, 0.10, 1.95, 0.75),
]

# (speed m/s, rotation rad/s, duration s)
ROUTE = [
    (0.20, 0.0, 12.0),
    (0.00, 0.60, 2.6),
    (0.20, 0.0, 9.0),
    (0.00, 0.60, 2.6),
    (0.20, 0.0, 11.0),
]

START_POSE = (0.35, -0.55, 0.0)

SONAR_PERIOD_MS = 100
TELEMETRY_PERIOD_MS = 200
MAX_RANGE_M = 4.0
MIN_RANGE_M = 0.02
NOISE_M = 0.010
YAW_DRIFT_RAD_S = 0.004
HALF_WHEEL_BASE_M = 0.075


def _ray_box_distance(ox, oy, ang, box):
    """Distance to the rectangle along the ray, or None if it misses."""
    x0, y0, x1, y1 = box
    dx, dy = math.cos(ang), math.sin(ang)
    t_enter, t_exit = 0.0, float("inf")

    for origin, direction, low, high in ((ox, dx, x0, x1), (oy, dy, y0, y1)):
        if abs(direction) < 1e-9:
            if origin < low or origin > high:
                return None
            continue
        ta, tb = (low - origin) / direction, (high - origin) / direction
        if ta > tb:
            ta, tb = tb, ta
        t_enter, t_exit = max(t_enter, ta), min(t_exit, tb)

    if t_enter > t_exit or t_exit < 0:
        return None
    return t_enter


def _measure(pose, geometry, rng):
    px, py, yaw = pose
    ox, oy = geometry["offset"]
    cos_y, sin_y = math.cos(yaw), math.sin(yaw)
    sx = px + ox * cos_y - oy * sin_y
    sy = py + ox * sin_y + oy * cos_y
    ang = yaw + geometry["bearing"]

    hits = [d for d in (_ray_box_distance(sx, sy, ang, b) for b in OBSTACLES)
            if d is not None]
    if not hits:
        return None

    dist = min(hits) + rng.gauss(0.0, NOISE_M)
    if dist < MIN_RANGE_M or dist > MAX_RANGE_M:
        return None
    return dist


def _trajectory(step_s):
    x, y, yaw = START_POSE
    for v, w, duration in ROUTE:
        for _ in range(int(duration / step_s)):
            yaw += w * step_s
            x += v * math.cos(yaw) * step_s
            y += v * math.sin(yaw) * step_s
            yield x, y, yaw, v, w


def generate(sonars: dict, seed: int = 7):
    """One telemetry sample every TELEMETRY_PERIOD_MS."""
    rng = random.Random(seed)
    names = list(sonars)
    step_s = SONAR_PERIOD_MS / 1000.0

    last = {n: (None, None) for n in names}
    seq = 0
    yaw_odom = START_POSE[2]

    for i, (x, y, yaw, v, w) in enumerate(_trajectory(step_s)):
        t_ms = i * SONAR_PERIOD_MS
        pose = (x, y, yaw)

        target = names[i % len(names)]
        reading = _measure(pose, sonars[target], rng)
        if reading is not None:
            last[target] = (reading, t_ms)

        # The fused yaw is the ground truth; odometry drifts slowly, as it
        # would with wheel slip.
        yaw_odom += w * step_s + YAW_DRIFT_RAD_S * step_s

        if t_ms % TELEMETRY_PERIOD_MS:
            continue

        distances, ages = {}, {}
        for n in names:
            value, stamp = last[n]
            distances[n] = None if value is None else round(value, 2)
            ages[n] = None if stamp is None else t_ms - stamp

        yield {
            "schema_version": 1,
            "seq": seq,
            "timestamp_ms": t_ms,
            "pose": {"x_m": round(x, 3), "y_m": round(y, 3),
                     "yaw_rad": round(yaw, 4)},
            "speed_m_s": round(v, 3),
            "wheel_speed_m_s": {
                "left": round(v - HALF_WHEEL_BASE_M * w, 3),
                "right": round(v + HALF_WHEEL_BASE_M * w, 3)},
            "sonar_m": distances,
            "sonar_age_ms": ages,
            "imu": {"accel_m_s2": {"x": 0.0, "y": 0.0, "z": 9.81},
                    "gyro_rad_s": {"x": 0.0, "y": 0.0, "z": round(w, 4)},
                    "yaw_fused_rad": round(yaw, 4),
                    "yaw_odom_rad": round(yaw_odom, 4),
                    "age_ms": 0, "valid": True},
            "safety": {
                "timeout_active": False,
                "auto_brake": bool(
                    distances["front"] is not None
                    and distances["front"] < 0.30 and v > 0),
                "front_invalid": distances["front"] is None},
        }
        seq += 1
