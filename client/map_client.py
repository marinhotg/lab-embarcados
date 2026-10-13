#!/usr/bin/env python3
"""Real-time 2D map of the rover from its MQTT telemetry.

Builds a log-odds occupancy grid: each sonar reading is a ray whose traversed
cells gain free-space evidence and whose echo cell gains obstacle evidence. The
free corridor emerges from accumulation, with no prior knowledge of the room.

Each distance is projected from the pose the rover had when the measurement
was taken, interpolated from the pose history with sonar_age_ms, not from the
current pose.
"""

from __future__ import annotations

import argparse
import bisect
import json
import math
import os
import threading
import time

if os.environ.get("WAYLAND_DISPLAY") and not os.environ.get("QT_QPA_PLATFORM"):
    os.environ["QT_QPA_PLATFORM"] = "wayland"
    os.environ.setdefault("QT_LOGGING_RULES", "qt.qpa.wayland.warning=false")

import matplotlib
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Polygon
import paho.mqtt.client as mqtt

# Mirrors SONAR_OFFSET_* in firmware/main/config.h; calibrate both together.
SONARS = {
    "front": {"offset": (0.10, 0.00), "bearing": 0.0},
    "rear":  {"offset": (-0.10, 0.00), "bearing": math.pi},
    "left":  {"offset": (0.00, 0.07), "bearing": math.pi / 2},
    "right": {"offset": (0.00, -0.07), "bearing": -math.pi / 2},
}

COLORS = {"front": "#ff5252", "rear": "#b388ff",
          "left": "#69f0ae", "right": "#ffab40"}

CELL_M = 0.06
ROVER_SIZE_M = (0.20, 0.15)
MAX_RAY_AGE_MS = 400

# Inverse sensor model. An echo is stronger evidence than a pass-through, since
# a pass-through may be an angle error while the echo came from something solid.
L_FREE = -0.7
L_OCCUPIED = 1.1
L_CLAMP = 4.0

# The HC-SR04 sees a ~15 deg cone, not a ray: at 1 m the beam is already 26 cm
# wide. Tracing both edges along with the center covers the cone without
# claiming precision the sensor does not have.
HALF_CONE_RAD = math.radians(7.5)


def _bresenham(x0: int, y0: int, x1: int, y1: int):
    """Grid cells crossed by the ray, excluding the last one."""
    dx, dy = abs(x1 - x0), abs(y1 - y0)
    sx, sy = (1 if x0 < x1 else -1), (1 if y0 < y1 else -1)
    err = dx - dy
    while (x0, y0) != (x1, y1):
        yield x0, y0
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x0 += sx
        if e2 < dx:
            err += dx
            y0 += sy


class OccupancyMap:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._t_ms: list[int] = []
        self._poses: list[tuple[float, float, float]] = []
        self._trajectory: list[tuple[float, float]] = []
        self._grid: dict[tuple[int, int], float] = {}
        self._seen: set[tuple[str, int]] = set()
        self._rays: dict[str, tuple] = {}
        self._last_seq: int | None = None
        self._last_ts = -1
        self._sample: dict = {}

    def on_message(self, _client, _userdata, msg) -> None:
        try:
            sample = json.loads(msg.payload.decode())
        except (json.JSONDecodeError, UnicodeDecodeError) as exc:
            print(f"invalid payload: {exc}")
            return
        self.ingest(sample)

    def ingest(self, sample: dict) -> None:
        with self._lock:
            self._process(sample)

    def _process(self, sample: dict) -> None:
        ts, pose = sample.get("timestamp_ms"), sample.get("pose")
        if ts is None or pose is None:
            return

        if ts < self._last_ts:
            print("rover rebooted, clearing the map")
            self._reset()
        self._last_ts = ts

        seq = sample.get("seq")
        if seq is not None and seq == self._last_seq:
            return
        self._last_seq = seq

        self._t_ms.append(ts)
        self._poses.append((pose["x_m"], pose["y_m"], pose["yaw_rad"]))
        self._trajectory.append((pose["x_m"], pose["y_m"]))
        self._sample = sample
        self._project(sample, ts)

    def _update(self, cell, delta) -> None:
        value = self._grid.get(cell, 0.0) + delta
        self._grid[cell] = max(-L_CLAMP, min(L_CLAMP, value))

    def _trace(self, sx, sy, dist, heading) -> None:
        """Free along the whole cone, occupied only at the central echo.

        The edges do not mark occupancy because the echo may have come from
        anywhere in the cone; assuming the edge would invent obstacles."""
        c0 = (round(sx / CELL_M), round(sy / CELL_M))

        for offset in (-HALF_CONE_RAD, 0.0, HALF_CONE_RAD):
            ang = heading + offset
            reach = dist if offset == 0.0 else dist * 0.95
            cx = round((sx + reach * math.cos(ang)) / CELL_M)
            cy = round((sy + reach * math.sin(ang)) / CELL_M)
            for cell in _bresenham(*c0, cx, cy):
                self._update(cell, L_FREE)
            if offset == 0.0:
                self._update((cx, cy), L_OCCUPIED)

    def _project(self, sample: dict, ts: int) -> None:
        distances = sample.get("sonar_m") or {}
        ages = sample.get("sonar_age_ms") or {}
        self._rays.clear()

        for name, geometry in SONARS.items():
            dist, age = distances.get(name), ages.get(name)
            if dist is None or age is None:
                continue

            pose = self._pose_at(ts - age)
            if pose is None:
                continue

            px, py, yaw = pose
            ox, oy = geometry["offset"]
            cos_y, sin_y = math.cos(yaw), math.sin(yaw)
            sx = px + ox * cos_y - oy * sin_y
            sy = py + ox * sin_y + oy * cos_y

            heading = yaw + geometry["bearing"]
            hx, hy = sx + dist * math.cos(heading), sy + dist * math.sin(heading)

            if age <= MAX_RAY_AGE_MS:
                self._rays[name] = (sx, sy, hx, hy)

            # Sonars refresh every 400 ms but telemetry publishes every 200 ms,
            # so the same reading arrives twice.
            key = (name, ts - age)
            if key in self._seen:
                continue
            self._seen.add(key)
            self._trace(sx, sy, dist, heading)

    def _pose_at(self, t_ms: int):
        if not self._t_ms:
            return None
        if t_ms <= self._t_ms[0]:
            return self._poses[0]
        if t_ms >= self._t_ms[-1]:
            return self._poses[-1]

        i = bisect.bisect_left(self._t_ms, t_ms)
        t0, t1 = self._t_ms[i - 1], self._t_ms[i]
        p0, p1 = self._poses[i - 1], self._poses[i]
        if t1 == t0:
            return p1
        f = (t_ms - t0) / (t1 - t0)
        # Shortest arc, so crossing +-pi does not spin the other way round.
        delta = math.atan2(math.sin(p1[2] - p0[2]), math.cos(p1[2] - p0[2]))
        return (p0[0] + f * (p1[0] - p0[0]),
                p0[1] + f * (p1[1] - p0[1]),
                p0[2] + f * delta)

    def _reset(self) -> None:
        self._t_ms.clear(); self._poses.clear(); self._trajectory.clear()
        self._grid.clear(); self._seen.clear(); self._rays.clear()
        self._last_seq = None

    def image(self):
        """Occupancy probability as a matrix plus its extent in meters. Cells
        without evidence stay at 0.5: unknown."""
        if not self._grid:
            return None, None
        xs = [c[0] for c in self._grid]
        ys = [c[1] for c in self._grid]
        x0, x1, y0, y1 = min(xs), max(xs), min(ys), max(ys)

        grid = np.full((y1 - y0 + 1, x1 - x0 + 1), 0.5, dtype=float)
        for (cx, cy), l in self._grid.items():
            grid[cy - y0, cx - x0] = 1.0 / (1.0 + math.exp(-l))

        extent = ((x0 - 0.5) * CELL_M, (x1 + 0.5) * CELL_M,
                  (y0 - 0.5) * CELL_M, (y1 + 0.5) * CELL_M)
        return grid, extent

    def snapshot(self) -> dict:
        with self._lock:
            grid, extent = self.image()
            return {"trajectory": list(self._trajectory), "grid": grid,
                    "extent": extent, "rays": dict(self._rays),
                    "pose": self._poses[-1] if self._poses else None,
                    "sample": dict(self._sample)}


def _rover_outline(pose):
    x, y, yaw = pose
    hl, hw = ROVER_SIZE_M[0] / 2, ROVER_SIZE_M[1] / 2
    local = [(-hl, -hw), (hl * 0.6, -hw), (hl, 0.0), (hl * 0.6, hw), (-hl, hw)]
    cos_y, sin_y = math.cos(yaw), math.sin(yaw)
    return [(x + px * cos_y - py * sin_y, y + px * sin_y + py * cos_y)
            for px, py in local]


def _status_text(sample: dict) -> str:
    if not sample:
        return "waiting for telemetry"

    pose, imu = sample["pose"], sample.get("imu", {})
    safety = sample.get("safety", {})
    dist = sample.get("sonar_m", {})
    age = sample.get("sonar_age_ms", {})
    wheels = sample.get("wheel_speed_m_s") or {}

    lines = [
        f"seq {sample.get('seq', 0):>5}    t {sample.get('timestamp_ms', 0)/1000:6.1f} s",
        "",
        f"x     {pose['x_m']:+7.2f} m",
        f"y     {pose['y_m']:+7.2f} m",
        f"yaw   {math.degrees(pose['yaw_rad']):+7.1f} deg",
        "",
        f"v     {sample.get('speed_m_s', 0):+7.2f} m/s",
        f" left {wheels.get('left', 0):+7.2f}",
        f" right{wheels.get('right', 0):+7.2f}",
        "",
        "sonars",
    ]
    for name in SONARS:
        d, a = dist.get(name), age.get(name)
        lines.append(f" {name:<9}" + ("   --  " if d is None else f"{d:6.2f} ")
                     + ("     " if a is None else f"{a:4d}ms"))

    if imu.get("valid"):
        fused = math.degrees(imu.get("yaw_fused_rad", 0))
        odom = math.degrees(imu.get("yaw_odom_rad", 0))
        lines += ["", "yaw fused / odometry",
                  f" {fused:+7.1f}  {odom:+7.1f}",
                  f" divergence {fused - odom:+.1f} deg"]
    else:
        lines += ["", "IMU invalid"]

    alerts = [label for label, key in (("timeout", "timeout_active"),
                                       ("auto brake", "auto_brake"),
                                       ("front invalid", "front_invalid"))
              if safety.get(key)]
    if alerts:
        lines += ["", "ALERT: " + ", ".join(alerts)]
    return "\n".join(lines)


def run(broker, port, rover_id, duration=None, save=None) -> None:
    occupancy = OccupancyMap()
    topic = f"rover/{rover_id}/telemetry"

    if save and duration is None:
        duration = 30.0

    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    client.on_message = occupancy.on_message
    client.on_connect = lambda c, *_: (
        print(f"connected to broker, subscribing to {topic}"), c.subscribe(topic))
    client.connect(broker, port, keepalive=30)
    client.loop_start()

    if save:
        matplotlib.use("Agg")
    elif matplotlib.get_backend().lower() == "agg":
        raise SystemExit(
            "matplotlib has no GUI backend, no window can open.\n"
            "  install:  .venv/bin/pip install PyQt5\n"
            "  or render to a file:  --save map.png")
    else:
        plt.ion()

    fig, (ax, panel) = plt.subplots(
        1, 2, figsize=(13, 7.5), gridspec_kw={"width_ratios": [3, 1]})
    fig.patch.set_facecolor("#0f1216")
    ax.set_facecolor("#808080")
    panel.set_facecolor("#0f1216")
    panel.axis("off")

    for spine in ax.spines.values():
        spine.set_color("#39404a")
    ax.tick_params(colors="#8b93a0", labelsize=8)
    ax.set_xlabel("x (m)", color="#8b93a0")
    ax.set_ylabel("y (m)", color="#8b93a0")
    ax.set_aspect("equal")
    ax.set_title("white: free   black: obstacle   gray: unknown",
                 color="#8b93a0", fontsize=9)

    grid_image = ax.imshow(np.full((1, 1), 0.5), cmap="gray_r", vmin=0, vmax=1,
                           origin="lower", extent=(0, 1, 0, 1),
                           interpolation="nearest", zorder=1)

    trajectory_line, = ax.plot([], [], "-", color="#2979ff", linewidth=1.4, zorder=4)
    ray_lines = {n: ax.plot([], [], "-", color=COLORS[n], linewidth=1.1,
                            alpha=0.8, zorder=5)[0] for n in SONARS}
    body = Polygon([(0, 0)], closed=True, facecolor="#2979ff",
                   edgecolor="#cfe4ff", linewidth=1.0, zorder=6)
    ax.add_patch(body)

    status = panel.text(0.0, 1.0, "", va="top", ha="left", family="monospace",
                        fontsize=9.5, color="#c9d3e0",
                        transform=panel.transAxes)

    start = time.monotonic()
    try:
        while save or plt.fignum_exists(fig.number):
            if duration and time.monotonic() - start > duration:
                break

            data = occupancy.snapshot()

            if data["grid"] is not None:
                grid_image.set_data(data["grid"])
                grid_image.set_extent(data["extent"])
                x0, x1, y0, y1 = data["extent"]
                ax.set_xlim(x0 - 0.1, x1 + 0.1)
                ax.set_ylim(y0 - 0.1, y1 + 0.1)

            if data["trajectory"]:
                tx, ty = zip(*data["trajectory"])
                trajectory_line.set_data(tx, ty)

            for name, line in ray_lines.items():
                r = data["rays"].get(name)
                line.set_data(*((r[0], r[2]), (r[1], r[3])) if r else ((), ()))

            if data["pose"]:
                body.set_xy(_rover_outline(data["pose"]))
            status.set_text(_status_text(data["sample"]))

            if save:
                time.sleep(0.1)
            else:
                fig.canvas.draw_idle()
                plt.pause(0.03)
    except KeyboardInterrupt:
        pass
    finally:
        if save:
            fig.savefig(save, dpi=110, facecolor=fig.get_facecolor(),
                        bbox_inches="tight")
            print(f"map saved to {save}")
        client.loop_stop()
        client.disconnect()


def main() -> None:
    p = argparse.ArgumentParser(description="Rover 2D map")
    p.add_argument("--broker", default="localhost")
    p.add_argument("--port", type=int, default=1883)
    p.add_argument("--id", dest="rover_id", default="01")
    p.add_argument("--duration", type=float, help="seconds until exit")
    p.add_argument("--save", help="render the map to a PNG and exit")
    args = p.parse_args()

    run(args.broker, args.port, args.rover_id, args.duration, args.save)


if __name__ == "__main__":
    main()
