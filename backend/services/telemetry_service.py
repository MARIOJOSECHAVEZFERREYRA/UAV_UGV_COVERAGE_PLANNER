"""Mock telemetry: interpolates UAV and UGV positions along mission waypoints."""
import asyncio
import math
import time
from collections.abc import AsyncGenerator

from backend.schemas.telemetry import TelemetryFrame, VehicleState


def _lerp(a: float, b: float, t: float) -> float:
    return a + (b - a) * t


def _heading(x0: float, y0: float, x1: float, y1: float) -> float:
    return math.degrees(math.atan2(y1 - y0, x1 - x0)) % 360


async def stream_telemetry(
    waypoints: list[tuple[float, float]],
    uav_speed: float = 5.0,      # m/s
    ugv_speed: float = 1.5,      # m/s
    interval_ms: int = 500,
) -> AsyncGenerator[TelemetryFrame, None]:
    """
    Yields a TelemetryFrame every interval_ms milliseconds until both
    vehicles have traversed all waypoints.
    """
    if not waypoints:
        return

    def make_cursor(speed: float) -> dict:
        return {"seg": 0, "t": 0.0, "speed": speed, "done": False}

    uav = make_cursor(uav_speed)
    ugv = make_cursor(ugv_speed)

    def advance(cursor: dict, dt: float) -> tuple[float, float, int]:
        seg = cursor["seg"]
        if cursor["done"] or seg >= len(waypoints) - 1:
            cursor["done"] = True
            wx, wy = waypoints[-1]
            return wx, wy, len(waypoints) - 1

        p0 = waypoints[seg]
        p1 = waypoints[seg + 1]
        seg_len = math.hypot(p1[0] - p0[0], p1[1] - p0[1])

        if seg_len < 1e-6:
            cursor["seg"] += 1
            return advance(cursor, dt)

        advance_m = cursor["speed"] * dt
        remaining = seg_len * (1.0 - cursor["t"])

        if advance_m >= remaining:
            cursor["seg"] += 1
            cursor["t"] = 0.0
            leftover = advance_m - remaining
            if cursor["seg"] < len(waypoints) - 1:
                cursor["t"] = min(leftover / seg_len, 1.0)
        else:
            cursor["t"] += advance_m / seg_len

        p0 = waypoints[cursor["seg"]] if cursor["seg"] < len(waypoints) else waypoints[-1]
        p1_idx = min(cursor["seg"] + 1, len(waypoints) - 1)
        p1 = waypoints[p1_idx]
        x = _lerp(p0[0], p1[0], cursor["t"])
        y = _lerp(p0[1], p1[1], cursor["t"])
        return x, y, cursor["seg"]

    dt = interval_ms / 1000.0
    battery = 100.0

    while not (uav["done"] and ugv["done"]):
        t_ms = int(time.time() * 1000)
        ux, uy, ui = advance(uav, dt)
        gx, gy, gi = advance(ugv, dt)
        battery = max(0.0, battery - 0.02)

        next_uav = waypoints[min(ui + 1, len(waypoints) - 1)]
        next_ugv = waypoints[min(gi + 1, len(waypoints) - 1)]

        frame = TelemetryFrame(
            timestamp_ms=t_ms,
            vehicles=[
                VehicleState(
                    vehicle_id="uav",
                    x=ux, y=uy,
                    heading=_heading(ux, uy, next_uav[0], next_uav[1]),
                    speed=uav_speed,
                    waypoint_index=ui,
                    battery_pct=round(battery, 1),
                ),
                VehicleState(
                    vehicle_id="ugv",
                    x=gx, y=gy,
                    heading=_heading(gx, gy, next_ugv[0], next_ugv[1]),
                    speed=ugv_speed,
                    waypoint_index=gi,
                ),
            ],
        )
        yield frame
        await asyncio.sleep(dt)
