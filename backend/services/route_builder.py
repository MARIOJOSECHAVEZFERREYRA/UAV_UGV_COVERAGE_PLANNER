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
    # Three-phase kinematic phase durations (s). Both 0.0 means no phase info
    # (service/deadhead segments) → energy drains uniformly in simulation.
    t_acc_s: float = 0.0
    t_dec_s: float = 0.0


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

    def build(self, mission_cycles: list[dict],
              service_duration_s: Optional[float] = None) -> VehicleRoute:
        """
        Parameters
        ----------
        mission_cycles : list of cycle dicts from MissionSegmenter.segment_path()
            Each cycle has: path, segments, visual_groups, swath_width, base_point
        service_duration_s : float, optional
            Duration of each service dwell in seconds.
            Defaults to drone.service_time_s when None.
            Pass ugv_t_service here for mobile rendezvous missions so that the
            UAV timeline matches the actual inter-rendezvous planning intervals.

        Returns
        -------
        VehicleRoute for the UAV
        """
        svc_dur = float(service_duration_s) if service_duration_s is not None \
            else float(self.drone.service_time_s)

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
                explicit = seg.get("segment_type", "")

                # Trust the explicit segment_type set by the segmenter; only fall
                # back to geometric classification when the type is unknown.
                if is_spraying:
                    seg_type = SegmentType.spray
                elif explicit == "deadhead":
                    seg_type = SegmentType.deadhead
                elif explicit == "ferry":
                    seg_type = SegmentType.ferry
                else:
                    seg_type = self._classify_segment(p1, p2, is_spraying, base_point)
                distance = math.hypot(p2[0] - p1[0], p2[1] - p1[1])

                # Compute costs and three-phase timings for visualization accuracy
                if seg_type == SegmentType.spray:
                    duration = self.energy_model.time_straight(distance)
                    energy_cost = self.energy_model.energy_straight(distance, reagent_remaining)
                    reagent_cost = self.energy_model.reagent_consumed(distance)
                    t_a, _, t_d, _ = self.energy_model._straight_profile(
                        distance,
                        self.drone.speed_cruise_ms,
                        self.drone.accel_horizontal_ms2,
                        self.drone.decel_horizontal_ms2,
                    )
                else:
                    # ferry or deadhead
                    duration = self.energy_model.time_transit(distance)
                    energy_cost = self.energy_model.energy_transit(distance, reagent_remaining)
                    reagent_cost = 0.0
                    t_a, _, t_d, _ = self.energy_model._straight_profile(
                        distance,
                        self.drone.speed_max_ms,
                        self.drone.accel_horizontal_ms2,
                        self.drone.decel_horizontal_ms2,
                    )

                route_seg = RouteSegment(
                    p1=p1,
                    p2=p2,
                    segment_type=seg_type,
                    cycle_index=cycle_idx,
                    distance_m=distance,
                    duration_s=duration,
                    energy_cost_wh=energy_cost,
                    reagent_consumed_l=reagent_cost,
                    t_acc_s=t_a,
                    t_dec_s=t_d,
                )
                segments.append(route_seg)

                total_energy += energy_cost
                total_reagent += reagent_cost
                total_duration += duration

                energy_remaining -= energy_cost
                reagent_remaining -= reagent_cost

            # Service dwell at end of each cycle
            service_seg = RouteSegment(
                p1=base_point,
                p2=base_point,
                segment_type=SegmentType.service,
                cycle_index=cycle_idx,
                distance_m=0.0,
                duration_s=svc_dur,
                energy_cost_wh=0.0,
                reagent_consumed_l=0.0,
            )
            segments.append(service_seg)
            total_duration += svc_dur

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

    Static mode  — UGV waits at the fixed base_point for the whole mission.
    Mobile mode  — UGV travels along the user-defined polyline, stopping at
                   each rendezvous point in sync with the UAV timeline.
    """

    def build_static(
        self,
        mission_cycles: list[dict],
        uav_total_duration_s: float,
        ugv_garage: Optional[tuple[float, float]] = None,
    ) -> VehicleRoute:
        """
        Static mode: UGV waits at mission_cycles[0]["base_point"] for the
        entire UAV mission duration.
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

    def build_mobile(
        self,
        mission_cycles: list[dict],
        uav_route: "VehicleRoute",
        ugv_polyline: list,
        ugv_speed: float,
        ugv_t_service: float,
    ) -> VehicleRoute:
        """
        Mobile mode: UGV starts at ugv_polyline[0] and moves to each rendezvous
        point (the base_point of every cycle except the last, which is the UAV's
        final position rather than a rendezvous).  Between rendezvous events the
        UGV drives at ugv_speed; after the last service it continues toward
        ugv_polyline[-1].

        Service segments in the UGV route are aligned to the UAV route's service
        segment start times so both vehicles share the same simulation clock.

        Parameters
        ----------
        mission_cycles : list of cycle dicts from MissionSegmenter.segment_path_mobile()
        uav_route      : already-built VehicleRoute for the UAV
        ugv_polyline   : user-defined route as list of [x, y] points
        ugv_speed      : UGV cruise speed in m/s
        ugv_t_service  : service dwell duration in seconds (same value used in planning)
        """
        if not mission_cycles or not ugv_polyline or len(ugv_polyline) < 2:
            return self.build_static(mission_cycles, uav_route.total_duration_s)

        ugv_speed = max(float(ugv_speed), 0.1)
        n_cycles  = len(mission_cycles)
        ugv_poly  = [(float(p[0]), float(p[1])) for p in ugv_polyline]

        # segment_path_mobile closes all cycles except the last at a rendezvous.
        # The last cycle ends at the UAV's final position (not a rendezvous).
        n_rv = max(0, n_cycles - 1)
        rv_points: list[tuple[float, float]] = [
            (float(mission_cycles[i]["base_point"][0]),
             float(mission_cycles[i]["base_point"][1]))
            for i in range(n_rv)
        ]

        # Project each rv_point onto the polyline to get its distance_along.
        rv_d = [self._find_distance_along(ugv_poly, rv) for rv in rv_points]
        total_poly_len = self._polyline_length(ugv_poly)

        # Extract service segment (start_time, duration) pairs from the UAV
        # route in cycle order — there is exactly one service segment per cycle.
        t_acc = 0.0
        all_svc: list[tuple[float, float]] = []
        for seg in uav_route.segments:
            if seg.segment_type == SegmentType.service:
                all_svc.append((t_acc, seg.duration_s))
            t_acc += seg.duration_s

        # Only the first n_rv services correspond to real rendezvous events.
        rv_svc = all_svc[:n_rv]

        total_uav_dur = uav_route.total_duration_s
        segments: list[RouteSegment] = []

        if n_rv == 0 or not rv_svc:
            # No rendezvous — UGV traverses its full polyline over the mission
            self._append_polyline_transit(
                segments, ugv_poly, total_uav_dur, ugv_speed, cycle_index=0
            )
        else:
            # Phase 0: polyline start → first rendezvous (following the polyline)
            T0_start, _ = rv_svc[0]
            path0 = self._subpath_along(ugv_poly, 0.0, rv_d[0])
            self._append_polyline_transit(segments, path0, T0_start, ugv_speed, cycle_index=0)

            for i in range(n_rv):
                t_svc_start, svc_dur = rv_svc[i]

                # Service dwell at rendezvous i
                segments.append(RouteSegment(
                    p1=rv_points[i], p2=rv_points[i],
                    segment_type=SegmentType.service,
                    cycle_index=i,
                    distance_m=0.0,
                    duration_s=max(svc_dur, 0.001),
                    energy_cost_wh=0.0,
                    reagent_consumed_l=0.0,
                ))

                if i + 1 < n_rv:
                    # Transit to next rendezvous within the inter-service window,
                    # following the actual polyline between the two distances.
                    t_next_start, _ = rv_svc[i + 1]
                    available = max(t_next_start - (t_svc_start + svc_dur), 0.001)
                    path_i = self._subpath_along(ugv_poly, rv_d[i], rv_d[i + 1])
                    self._append_polyline_transit(
                        segments, path_i, available, ugv_speed, cycle_index=i
                    )

            # Final phase: last rendezvous → polyline end
            last_t_start, last_dur = rv_svc[-1]
            remaining = total_uav_dur - (last_t_start + last_dur)
            if remaining > 0.001:
                path_final = self._subpath_along(ugv_poly, rv_d[-1], total_poly_len)
                self._append_polyline_transit(
                    segments, path_final, remaining, ugv_speed, cycle_index=n_rv - 1
                )

        total_duration = sum(seg.duration_s for seg in segments)
        return VehicleRoute(
            vehicle_id="ugv",
            segments=segments,
            total_duration_s=total_duration,
            total_energy_wh=0.0,
            total_reagent_l=0.0,
        )

    # ------------------------------------------------------------------
    # Polyline geometry helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _polyline_length(polyline: list) -> float:
        """Total Euclidean length of an ordered list of (x, y) points."""
        total = 0.0
        for i in range(len(polyline) - 1):
            p1, p2 = polyline[i], polyline[i + 1]
            total += math.hypot(p2[0] - p1[0], p2[1] - p1[1])
        return total

    @staticmethod
    def _interp_at_distance(polyline: list, d: float) -> tuple:
        """Interpolate (x, y) at accumulated distance d along polyline."""
        d = max(0.0, d)
        accumulated = 0.0
        for i in range(len(polyline) - 1):
            p1 = polyline[i]
            p2 = polyline[i + 1]
            seg_len = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
            if accumulated + seg_len >= d - 1e-9:
                if seg_len < 1e-9:
                    return (float(p1[0]), float(p1[1]))
                t = max(0.0, min(1.0, (d - accumulated) / seg_len))
                return (p1[0] + t * (p2[0] - p1[0]), p1[1] + t * (p2[1] - p1[1]))
            accumulated += seg_len
        return (float(polyline[-1][0]), float(polyline[-1][1]))

    @staticmethod
    def _find_distance_along(polyline: list, point: tuple) -> float:
        """
        Project point onto polyline; return its accumulated distance along it.
        Finds the closest perpendicular projection across all segments.
        """
        min_dist = float('inf')
        best_d   = 0.0
        accumulated = 0.0
        for i in range(len(polyline) - 1):
            p1 = polyline[i]
            p2 = polyline[i + 1]
            dx = p2[0] - p1[0]
            dy = p2[1] - p1[1]
            seg_len_sq = dx * dx + dy * dy
            if seg_len_sq < 1e-18:
                continue
            seg_len = math.sqrt(seg_len_sq)
            t = max(0.0, min(1.0,
                ((point[0] - p1[0]) * dx + (point[1] - p1[1]) * dy) / seg_len_sq))
            px = p1[0] + t * dx
            py = p1[1] + t * dy
            dist = math.hypot(point[0] - px, point[1] - py)
            if dist < min_dist:
                min_dist = dist
                best_d = accumulated + t * seg_len
            accumulated += seg_len
        return best_d

    @staticmethod
    def _subpath_along(polyline: list, d_from: float, d_to: float) -> list:
        """
        Extract the sub-path of polyline between accumulated distances d_from
        and d_to.  Returns a list of (x, y) that follows the actual polyline
        vertices.  Always includes interpolated start and end points.
        """
        if d_to <= d_from + 1e-6:
            pt = UGVRouteBuilder._interp_at_distance(polyline, d_from)
            return [pt, pt]

        p_start = UGVRouteBuilder._interp_at_distance(polyline, d_from)
        p_end   = UGVRouteBuilder._interp_at_distance(polyline, d_to)
        path    = [p_start]
        accumulated = 0.0

        for i in range(len(polyline) - 1):
            p1 = polyline[i]
            p2 = polyline[i + 1]
            seg_len = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
            vertex_d = accumulated + seg_len
            # Include interior vertices that fall strictly between d_from and d_to
            if d_from + 1e-6 < vertex_d < d_to - 1e-6:
                path.append((float(p2[0]), float(p2[1])))
            accumulated = vertex_d

        if math.hypot(path[-1][0] - p_end[0], path[-1][1] - p_end[1]) > 1e-6:
            path.append(p_end)
        return path

    @staticmethod
    def _append_polyline_transit(
        segments: list,
        path: list,
        available_s: float,
        ugv_speed: float,
        cycle_index: int,
    ) -> None:
        """
        Append route segments that make the UGV follow path within available_s.

        - Each sub-segment of path becomes its own RouteSegment (deadhead).
        - If actual travel < available_s: a service (wait) segment pads the end.
        - If actual travel > available_s: time is compressed proportionally.
        - Degenerate path (< 2 pts or zero length): single wait segment only.
        """
        available_s = max(float(available_s), 0.001)

        if len(path) < 2:
            pt = tuple(path[0]) if path else (0.0, 0.0)
            segments.append(RouteSegment(
                p1=pt, p2=pt,
                segment_type=SegmentType.service,
                cycle_index=cycle_index,
                distance_m=0.0, duration_s=available_s,
                energy_cost_wh=0.0, reagent_consumed_l=0.0,
            ))
            return

        sub_dists = [
            math.hypot(float(path[i + 1][0]) - float(path[i][0]),
                       float(path[i + 1][1]) - float(path[i][1]))
            for i in range(len(path) - 1)
        ]
        total_d = sum(sub_dists)

        if total_d < 0.1:
            segments.append(RouteSegment(
                p1=tuple(path[0]), p2=tuple(path[-1]),
                segment_type=SegmentType.service,
                cycle_index=cycle_index,
                distance_m=0.0, duration_s=available_s,
                energy_cost_wh=0.0, reagent_consumed_l=0.0,
            ))
            return

        actual_travel_s = total_d / ugv_speed

        if actual_travel_s <= available_s:
            for i in range(len(path) - 1):
                d = sub_dists[i]
                if d < 1e-9:
                    continue
                segments.append(RouteSegment(
                    p1=tuple(path[i]), p2=tuple(path[i + 1]),
                    segment_type=SegmentType.deadhead,
                    cycle_index=cycle_index,
                    distance_m=d,
                    duration_s=d / ugv_speed,
                    energy_cost_wh=0.0, reagent_consumed_l=0.0,
                ))
            wait_dur = available_s - actual_travel_s
            if wait_dur > 0.001:
                segments.append(RouteSegment(
                    p1=tuple(path[-1]), p2=tuple(path[-1]),
                    segment_type=SegmentType.service,
                    cycle_index=cycle_index,
                    distance_m=0.0, duration_s=wait_dur,
                    energy_cost_wh=0.0, reagent_consumed_l=0.0,
                ))
        else:
            # Compress: distribute available_s proportionally across sub-segments
            for i in range(len(path) - 1):
                d = sub_dists[i]
                if d < 1e-9:
                    continue
                seg_dur = available_s * (d / total_d)
                segments.append(RouteSegment(
                    p1=tuple(path[i]), p2=tuple(path[i + 1]),
                    segment_type=SegmentType.deadhead,
                    cycle_index=cycle_index,
                    distance_m=d,
                    duration_s=max(seg_dur, 0.001),
                    energy_cost_wh=0.0, reagent_consumed_l=0.0,
                ))
