"""
simulation_service.py

Drives mission simulation tick-by-tick. This module:

  1. Operates on pre-built VehicleRoute objects (from route_builder), not a
     flat waypoint list.
  2. Uses physics-accurate battery and reagent tracking.
  3. Emits SimulationFrame with full VehicleSimState per vehicle per tick.
  4. Supports playback_speed multiplier (time compression).
  5. Supports both static and dynamic UGV modes via VehicleRoute polymorphism.
"""
import asyncio
import math
import time
from collections.abc import AsyncGenerator

from backend.schemas.simulation import SegmentType, SimulationFrame, VehicleSimState
from backend.services.route_builder import VehicleRoute, RouteSegment


class VehicleCursor:
    """
    Tracks a single vehicle's position within its VehicleRoute.

    Maintains:
      - current segment index
      - fractional progress along current segment [0.0, 1.0)
      - accumulated resource state (energy_wh, reagent_l)
      - done flag
    """

    def __init__(
        self,
        route: VehicleRoute,
        initial_energy_wh: float,
        initial_reagent_l: float,
    ):
        self.route = route
        self.seg_idx = 0
        self.t = 0.0  # fractional progress in current segment
        self.energy_wh = initial_energy_wh
        self.reagent_l = initial_reagent_l
        self.initial_energy_wh = initial_energy_wh
        self.initial_reagent_l = initial_reagent_l
        self.done = False
        self.sim_time_s = 0.0

    @property
    def current_segment(self) -> RouteSegment | None:
        if self.seg_idx >= len(self.route.segments):
            return None
        return self.route.segments[self.seg_idx]

    def advance(self, dt_real: float, playback_speed: float) -> None:
        """
        Advances cursor by dt_real * playback_speed simulated seconds.

        Handles segment transitions and resource depletion.
        Resources are consumed proportionally as the cursor moves through
        each segment (not lump-sum at segment end).
        """
        if self.done:
            return

        dt_sim = dt_real * playback_speed
        self.sim_time_s += dt_sim
        time_remaining = dt_sim

        while time_remaining > 1e-6 and self.seg_idx < len(self.route.segments):
            seg = self.route.segments[self.seg_idx]

            # Ensure minimum duration to avoid division by zero
            seg_duration = max(0.001, seg.duration_s)
            time_in_seg = self.t * seg_duration
            time_left_in_seg = seg_duration - time_in_seg

            if time_left_in_seg <= time_remaining + 1e-6:
                # Finish this segment completely (with small tolerance)
                self.t = 1.0

                # Drain all remaining resources for this segment
                fraction_remaining = max(0.0, 1.0 - (time_in_seg / seg_duration))
                self.energy_wh -= seg.energy_cost_wh * fraction_remaining
                self.reagent_l -= seg.reagent_consumed_l * fraction_remaining

                time_remaining -= time_left_in_seg
                prev_type = seg.segment_type

                # Move to next segment
                self.seg_idx += 1
                self.t = 0.0

                # Recharge: when leaving a service segment reset battery/reagent
                # so each new cycle starts from full resources.
                if prev_type == SegmentType.service and self.seg_idx < len(self.route.segments):
                    self.energy_wh = self.initial_energy_wh
                    self.reagent_l = self.initial_reagent_l
            else:
                # Partial advance within segment
                advance_fraction = time_remaining / seg_duration
                self.t += advance_fraction
                self.t = min(1.0, self.t)  # Clamp to [0, 1]

                # Drain proportional resources
                self.energy_wh -= seg.energy_cost_wh * advance_fraction
                self.reagent_l -= seg.reagent_consumed_l * advance_fraction

                time_remaining = 0.0

        if self.seg_idx >= len(self.route.segments):
            self.done = True

    def get_state(self, vehicle_id: str, drone) -> VehicleSimState:
        """
        Interpolates current position and returns VehicleSimState.
        Heading is computed from the current segment direction vector.
        battery_pct is derived from energy_wh vs. drone.battery_capacity_wh.
        """
        if self.done or self.seg_idx >= len(self.route.segments):
            # Return final position
            last_seg = self.route.segments[-1]
            pos = last_seg.p2
            return VehicleSimState(
                vehicle_id=vehicle_id,
                x=pos[0],
                y=pos[1],
                heading=0.0,
                speed=0.0,
                segment_type=SegmentType.service,
                cycle_index=len(self.route.segments) - 1,
                waypoint_index=len(self.route.segments) - 1,
                battery_pct=0.0,
                energy_remaining_wh=max(0.0, self.energy_wh),
                reagent_l=max(0.0, self.reagent_l),
                pump_active=False,
                is_done=True,
            )

        seg = self.current_segment
        if seg is None:
            self.done = True
            return VehicleSimState(
                vehicle_id=vehicle_id,
                x=0.0,
                y=0.0,
                heading=0.0,
                speed=0.0,
                segment_type=SegmentType.service,
                cycle_index=0,
                waypoint_index=0,
                battery_pct=0.0,
                energy_remaining_wh=0.0,
                reagent_l=0.0,
                pump_active=False,
                is_done=True,
            )

        # Interpolate position along current segment
        # Clamp t to [0, 1] to avoid overshooting
        t_clamped = max(0.0, min(1.0, self.t))

        if seg.distance_m > 1e-6:
            x = seg.p1[0] + (seg.p2[0] - seg.p1[0]) * t_clamped
            y = seg.p1[1] + (seg.p2[1] - seg.p1[1]) * t_clamped
        else:
            # Zero-distance segment (stay at p1 = p2)
            x, y = seg.p1

        # Compute heading
        if seg.distance_m > 1e-6:
            heading = math.degrees(math.atan2(seg.p2[1] - seg.p1[1], seg.p2[0] - seg.p1[0])) % 360
        else:
            heading = 0.0

        # Compute speed (instantaneous speed during this segment)
        if seg.duration_s > 0:
            speed = seg.distance_m / seg.duration_s
        else:
            speed = 0.0

        # Battery percentage
        if drone:
            battery_capacity = drone.battery_capacity_wh
            reserve_pct = drone.battery_reserve_pct
            usable_capacity = battery_capacity * (1.0 - reserve_pct / 100.0)
            battery_pct = (self.energy_wh / usable_capacity * 100.0) if usable_capacity > 0 else 0.0
            battery_pct = max(0.0, min(100.0, battery_pct))
        else:
            battery_pct = 100.0

        # Pump active only during spray segments
        pump_active = seg.segment_type == SegmentType.spray

        # Handle service segment resource reset (conceptually battery/reagent reset here)
        energy_to_report = self.energy_wh
        reagent_to_report = self.reagent_l

        if seg.segment_type == SegmentType.service:
            # At service segment, resources are "reset" for next cycle
            # Report as full for visualization
            energy_to_report = drone.battery_capacity_wh * (1.0 - drone.battery_reserve_pct / 100.0) if drone else self.energy_wh
            reagent_to_report = drone.mass_tank_full_kg if drone else self.reagent_l
            battery_pct = 100.0

        return VehicleSimState(
            vehicle_id=vehicle_id,
            x=x,
            y=y,
            heading=heading,
            speed=speed,
            segment_type=seg.segment_type,
            cycle_index=seg.cycle_index,
            waypoint_index=self.seg_idx,
            battery_pct=battery_pct,
            energy_remaining_wh=max(0.0, energy_to_report),
            reagent_l=max(0.0, reagent_to_report),
            pump_active=pump_active,
            is_done=False,
        )


class SimulationState:
    """Mutable state that can be updated by client."""
    def __init__(self):
        self.playback_speed = 1.0


async def stream_simulation(
    uav_route: VehicleRoute,
    ugv_route: VehicleRoute,
    drone,
    state: SimulationState,
    interval_ms: int = 200,
) -> AsyncGenerator[SimulationFrame, None]:
    """
    Yields SimulationFrame every interval_ms wall-clock milliseconds.

    Simulation ends when both vehicles are done.

    Uses a shared SimulationState object that can be updated to change playback_speed.
    """
    uav_cursor = VehicleCursor(
        uav_route,
        initial_energy_wh=drone.battery_capacity_wh * (1.0 - drone.battery_reserve_pct / 100.0),
        initial_reagent_l=drone.mass_tank_full_kg,
    )
    ugv_cursor = VehicleCursor(
        ugv_route,
        initial_energy_wh=float("inf"),
        initial_reagent_l=0.0,
    )

    dt = interval_ms / 1000.0

    while not (uav_cursor.done and ugv_cursor.done):
        t_ms = int(time.time() * 1000)
        sim_time = uav_cursor.sim_time_s
        current_speed = state.playback_speed

        uav_cursor.advance(dt, current_speed)
        ugv_cursor.advance(dt, current_speed)

        frame = SimulationFrame(
            timestamp_ms=t_ms,
            sim_time_s=sim_time,
            vehicles=[
                uav_cursor.get_state("uav", drone),
                ugv_cursor.get_state("ugv", drone),
            ],
            playback_speed=current_speed,
        )
        yield frame
        await asyncio.sleep(dt)
