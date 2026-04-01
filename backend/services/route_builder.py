"""
route_builder.py

Converts mission_cycles (from MissionSegmenter) into VehicleRoute objects
for the UAV and UGV respectively.

Key decision: ferry vs deadhead classification.
  - A segment with spraying=False that starts and ends INSIDE the work polygon
    boundary (both endpoints contained or on boundary) is a ferry.
  - A segment with spraying=False where at least one endpoint is the base_point
    OR is outside the polygon is a deadhead.

This matches the semantic intent: ferry connects two sweep passes within the
field, deadhead goes to/from the service location.
"""
import json
import math
from dataclasses import dataclass
from typing import Optional

from shapely.geometry import Point, Polygon

from backend.algorithms.drone.energy_model import DroneEnergyModel
from backend.schemas.simulation import SegmentType


@dataclass
class RouteSegment:
    """A single segment in a vehicle's route."""
    p1: tuple[float, float]
    p2: tuple[float, float]
    segment_type: SegmentType
    cycle_index: int
    distance_m: float
    duration_s: float
    energy_cost_wh: float
    reagent_consumed_l: float


@dataclass
class VehicleRoute:
    """Complete route for a vehicle."""
    vehicle_id: str
    segments: list[RouteSegment]
    total_duration_s: float
    total_energy_wh: float
    total_reagent_l: float


class UAVRouteBuilder:
    """
    Builds the UAV VehicleRoute from mission_cycles.

    The UAV visits every segment in every cycle: deadhead-out, sweep/ferry
    passes, deadhead-back, service dwell.
    """

    def __init__(self, drone, work_polygon: Polygon):
        self.drone = drone
        self.energy_model = DroneEnergyModel(drone)
        self.polygon = work_polygon

    def build(self, mission_cycles: list[dict]) -> VehicleRoute:
        """
        Parameters
        ----------
        mission_cycles : list of cycle dicts from MissionSegmenter.segment_path()
            Each cycle has: path, segments, visual_groups, swath_width, base_point

        Returns
        -------
        VehicleRoute for the UAV
        """
        segments = []
        total_energy = 0.0
        total_reagent = 0.0
        total_duration = 0.0

        for cycle_idx, cycle in enumerate(mission_cycles):
            base_point = tuple(cycle["base_point"])
            cycle_segments = cycle.get("segments", [])

            # Current energy/reagent state at start of cycle
            energy_remaining = self.energy_model.usable_energy_wh()
            reagent_remaining = self.drone.mass_tank_full_kg

            for seg in cycle_segments:
                p1 = tuple(seg["p1"][:2])
                p2 = tuple(seg["p2"][:2])
                is_spraying = seg["spraying"]

                seg_type = self._classify_segment(p1, p2, is_spraying, base_point)
                distance = math.hypot(p2[0] - p1[0], p2[1] - p1[1])

                # Compute costs
                if seg_type == SegmentType.spray:
                    duration = self.energy_model.time_straight(distance)
                    energy_cost = self.energy_model.energy_straight(distance, reagent_remaining)
                    reagent_cost = self.energy_model.reagent_consumed(distance)
                else:
                    # ferry or deadhead
                    duration = self.energy_model.time_transit(distance)
                    energy_cost = self.energy_model.energy_transit(distance, reagent_remaining)
                    reagent_cost = 0.0

                route_seg = RouteSegment(
                    p1=p1,
                    p2=p2,
                    segment_type=seg_type,
                    cycle_index=cycle_idx,
                    distance_m=distance,
                    duration_s=duration,
                    energy_cost_wh=energy_cost,
                    reagent_consumed_l=reagent_cost,
                )
                segments.append(route_seg)

                total_energy += energy_cost
                total_reagent += reagent_cost
                total_duration += duration

                energy_remaining -= energy_cost
                reagent_remaining -= reagent_cost

            # Add service dwell segment at end of cycle (except for last cycle simulation detail)
            service_seg = RouteSegment(
                p1=base_point,
                p2=base_point,
                segment_type=SegmentType.service,
                cycle_index=cycle_idx,
                distance_m=0.0,
                duration_s=self.drone.service_time_s,
                energy_cost_wh=0.0,  # Energy is reset to full during dwell
                reagent_consumed_l=0.0,  # Reagent is reset to full during dwell
            )
            segments.append(service_seg)
            total_duration += self.drone.service_time_s

        return VehicleRoute(
            vehicle_id="uav",
            segments=segments,
            total_duration_s=total_duration,
            total_energy_wh=total_energy,
            total_reagent_l=total_reagent,
        )

    def _classify_segment(
        self,
        p1: tuple[float, float],
        p2: tuple[float, float],
        is_spraying: bool,
        base_point: tuple[float, float],
    ) -> SegmentType:
        """
        Classifies a segment as spray, ferry, or deadhead.

        Rules:
        1. If is_spraying=True → SegmentType.spray
        2. If either endpoint is the base_point (within 0.5m) → deadhead
        3. If midpoint is inside the work polygon → ferry
        4. Otherwise → deadhead
        """
        if is_spraying:
            return SegmentType.spray

        BASE_TOL = 0.5  # meters
        p1_dist_to_base = math.hypot(p1[0] - base_point[0], p1[1] - base_point[1])
        p2_dist_to_base = math.hypot(p2[0] - base_point[0], p2[1] - base_point[1])

        if p1_dist_to_base < BASE_TOL or p2_dist_to_base < BASE_TOL:
            return SegmentType.deadhead

        # Check midpoint
        mid_x = (p1[0] + p2[0]) / 2
        mid_y = (p1[1] + p2[1]) / 2
        mid_pt = Point(mid_x, mid_y)

        if self.polygon.contains(mid_pt) or self.polygon.boundary.distance(mid_pt) < 0.1:
            return SegmentType.ferry

        return SegmentType.deadhead


class UGVRouteBuilder:
    """
    Builds the UGV VehicleRoute.

    Static mode: UGV drives from its garage to the base_point of the first
    cycle, waits there for the entire mission duration, then returns to garage.

    Dynamic mode (future): UGV intercepts UAV at each cycle's base_point by
    computing arrival time constraints.
    """

    UGV_SPEED_MS = 1.5
    UGV_GARAGE = (0.0, 0.0)

    def build_static(
        self,
        mission_cycles: list[dict],
        uav_total_duration_s: float,
        ugv_garage: Optional[tuple[float, float]] = None,
    ) -> VehicleRoute:
        """
        Builds the static-mode UGV route:
        The UGV simply waits at the base_point for the entire UAV mission duration.
        No movement to or from garage - it's already there waiting.
        """
        if not mission_cycles:
            return VehicleRoute(
                vehicle_id="ugv",
                segments=[],
                total_duration_s=0.0,
                total_energy_wh=0.0,
                total_reagent_l=0.0,
            )

        base_point = tuple(mission_cycles[0]["base_point"])

        # Single service segment: UGV waits at base_point for entire UAV mission
        segments = [
            RouteSegment(
                p1=base_point,
                p2=base_point,
                segment_type=SegmentType.service,
                cycle_index=0,
                distance_m=0.0,
                duration_s=uav_total_duration_s,
                energy_cost_wh=0.0,
                reagent_consumed_l=0.0,
            )
        ]

        return VehicleRoute(
            vehicle_id="ugv",
            segments=segments,
            total_duration_s=uav_total_duration_s,
            total_energy_wh=0.0,
            total_reagent_l=0.0,
        )

    def build_dynamic(
        self,
        mission_cycles: list[dict],
        uav_route: VehicleRoute,
        ugv_garage: Optional[tuple[float, float]] = None,
    ) -> VehicleRoute:
        """
        Future: UGV drives to each rendezvous point timed to UAV arrivals.
        Raises NotImplementedError until dynamic mode is implemented.
        """
        raise NotImplementedError("Dynamic UGV mode not yet implemented.")
