import math
import numpy as np
from shapely.geometry import LineString

from .energy_model import DroneEnergyModel

_SNAP_THRESHOLD = 0.6   # endpoint must be at most this fraction of d_current from base
_SNAP_LOOKAHEAD = 2     # max atomic segments to look ahead for a near-base landing


class MissionSegmenter:
    """
    Splits a typed route into work cycles based solely on drone energy
    and liquid constraints.

    Responsibilities:
    - Expand multi-point route segments into atomic 2-point segments.
    - Determine where battery/liquid limits force a service stop.
    - Return raw work cycles (sweep/ferry segments only, no deadheads).

    NOT responsible for:
    - Deadhead path planning — handled downstream by the rendezvous component.
    - Choosing service point location — handled by rendezvous/planner.py.
    - Obstacle-aware routing — handled by coverage/path_assembler.py.

    Feasibility checks use Euclidean distance to base_point as a conservative
    lower bound on the actual return cost. Obstacle-aware deadhead costs are
    computed later when actual paths are planned.
    """

    def __init__(self, drone, target_rate_l_ha=20.0, work_speed_kmh=None,
                 swath_width=None, energy_model=None):
        self.drone = drone
        self.rate_l_ha = target_rate_l_ha
        self.energy_model = energy_model if energy_model is not None else DroneEnergyModel(drone)

        self.speed_kmh = work_speed_kmh if work_speed_kmh is not None else drone.speed_cruise_ms * 3.6
        self.speed_ms = self.speed_kmh / 3.6

        self.swath_width = swath_width if swath_width is not None else drone.spray_swath_max_m
        self.liters_per_meter = (self.rate_l_ha * self.swath_width) / 10000.0
        self.tank_capacity = drone.mass_tank_full_kg

    @staticmethod
    def _xy(point):
        return (float(point[0]), float(point[1]))

    @staticmethod
    def _pts_equal(a, b, tol=1e-6):
        return abs(a[0] - b[0]) < tol and abs(a[1] - b[1]) < tol

    @staticmethod
    def _dist(a, b):
        return math.hypot(b[0] - a[0], b[1] - a[1])

    @staticmethod
    def _path_length(path_coords):
        if not path_coords or len(path_coords) < 2:
            return 0.0
        total = 0.0
        for i in range(len(path_coords) - 1):
            total += np.linalg.norm(np.array(path_coords[i]) - np.array(path_coords[i + 1]))
        return float(total)

    def _segment_energy(self, seg_type, distance_m, liquid_remaining):
        if seg_type == "sweep":
            return self.energy_model.energy_straight(distance_m, liquid_remaining)
        return self.energy_model.energy_transit(distance_m, liquid_remaining)

    def _segment_time(self, seg_type, distance_m):
        if seg_type == "sweep":
            return self.energy_model.time_straight(distance_m)
        return self.energy_model.time_transit(distance_m)

    def _segment_liquid_use(self, seg_type, distance_m):
        if seg_type == "sweep":
            return distance_m * self.liters_per_meter
        return 0.0

    @staticmethod
    def _path_to_segments(path_coords, segment_type, spraying, cell_id=None):
        if not path_coords or len(path_coords) < 2:
            return []
        segments = []
        for i in range(len(path_coords) - 1):
            p1 = path_coords[i]
            p2 = path_coords[i + 1]
            segments.append({
                "p1": p1,
                "p2": p2,
                "spraying": spraying,
                "segment_type": segment_type,
                "distance_m": float(LineString([p1, p2]).length),
                "cell_id": cell_id,
            })
        return segments

    def _expand_route_segments(self, route_segments):
        atomic = []
        for seg in route_segments or []:
            seg_type = seg.get("segment_type")
            if seg_type not in ("sweep", "ferry", "deadhead"):
                raise ValueError("Unknown or missing segment_type: {}".format(seg_type))
            spraying = bool(seg.get("spraying", seg_type == "sweep"))
            cell_id = seg.get("cell_id")
            path = seg.get("path", [])
            if not path or len(path) < 2:
                continue
            normalized = [self._xy(pt[:2]) if len(pt) >= 2 else self._xy(pt) for pt in path]
            atomic.extend(self._path_to_segments(normalized, seg_type, spraying, cell_id))
        return atomic

    def segments_to_path(self, segments):
        if not segments:
            return []
        path = [segments[0]["p1"]]
        for seg in segments:
            if not self._pts_equal(path[-1], seg["p2"]):
                path.append(seg["p2"])
        return path

    def segment_path(self, route_segments, base_point, dist_fn=None):
        """
        Splits the typed route into work cycles based on energy/liquid limits.

        Parameters
        ----------
        route_segments : list[dict]
            Typed sweep/ferry segments from coverage path planner.
        base_point : tuple[float, float]
            Service point used only for feasibility distance estimates.
        dist_fn : callable(a, b) -> float, optional
            Obstacle-aware distance function. If None, uses Euclidean.
            Passing assembler.find_connection distance avoids underestimating
            return costs for cross-obstacle paths.

        Returns
        -------
        list[dict]
            Each cycle:
            {
                "segments":    list[dict],  # sweep/ferry only, no deadheads
                "start_pt":    (x, y),
                "end_pt":      (x, y),
                "swath_width": float,
            }
        """
        _dist = dist_fn if dist_fn is not None else self._dist

        atomic = self._expand_route_segments(route_segments)
        if not atomic:
            return []

        base_point = self._xy(base_point[:2])
        usable_energy = self.energy_model.usable_energy_wh()

        energy_remaining = usable_energy
        liquid_remaining = self.tank_capacity

        init_dist = _dist(base_point, atomic[0]["p1"])
        energy_remaining -= self.energy_model.energy_transit(init_dist, liquid_remaining)

        cycles = []
        current_segs = []

        i = 0
        while i < len(atomic):
            seg = atomic[i]
            p1, p2 = seg["p1"], seg["p2"]
            seg_type = seg["segment_type"]
            spraying = seg["spraying"]
            dist_step = seg.get("distance_m", self._dist(p1, p2))

            # Cell-boundary break: if this segment belongs to a different
            # cell than the last segment in the current cycle, close the
            # cycle here so each cycle stays within one cell (visually
            # coherent mission chunks). Only kicks in when both cell_ids
            # are present and current_segs already has some sweep in it.
            if current_segs:
                last_cid = current_segs[-1].get("cell_id")
                this_cid = seg.get("cell_id")
                if (last_cid is not None and this_cid is not None
                        and last_cid != this_cid
                        and any(s["segment_type"] == "sweep" for s in current_segs)):
                    cycles.append(self._make_cycle(current_segs))
                    energy_remaining = usable_energy
                    liquid_remaining = self.tank_capacity
                    dist_commute = _dist(base_point, p1)
                    energy_remaining -= self.energy_model.energy_transit(
                        dist_commute, liquid_remaining
                    )
                    current_segs = []

            liq_step = self._segment_liquid_use(seg_type, dist_step)
            energy_step = self._segment_energy(seg_type, dist_step, liquid_remaining)
            dist_return = _dist(p2, base_point)

            can_do = self.energy_model.feasible_after_segment_static(
                energy_remaining, liquid_remaining, energy_step, liq_step, dist_return
            )

            if can_do or not current_segs:
                current_segs.append({
                    "p1": p1, "p2": p2,
                    "spraying": spraying,
                    "segment_type": seg_type,
                    "distance_m": float(dist_step),
                })
                energy_remaining -= energy_step
                liquid_remaining -= liq_step
                i += 1
            else:
                # --- Base-side snapping ---
                # Before closing the cycle at the current position, look up to
                # _SNAP_LOOKAHEAD atomic segments ahead. In a boustrophedon a
                # strip that ends far from the base is typically followed by a
                # short ferry + a return strip heading back toward the base.
                # If a nearby segment endpoint is significantly closer to the
                # base (< _SNAP_THRESHOLD * current distance) AND the drone can
                # afford all lookahead segments plus the return from there, snap
                # to that endpoint before closing. This eliminates the long
                # deadheads that occur when a cycle closes on the "far" side.
                current_end = current_segs[-1]["p2"] if current_segs else base_point
                d_current = _dist(current_end, base_point)

                snapped = False
                if d_current > 1e-3:
                    e_acc = energy_remaining
                    q_acc = liquid_remaining
                    snap_buffer = []

                    for j in range(i, min(i + _SNAP_LOOKAHEAD, len(atomic))):
                        look = atomic[j]
                        ltype = look["segment_type"]
                        ldist = look.get("distance_m", self._dist(look["p1"], look["p2"]))
                        le = self._segment_energy(ltype, ldist, q_acc)
                        lq = self._segment_liquid_use(ltype, ldist)
                        e_acc -= le
                        q_acc = max(0.0, q_acc - lq)
                        snap_buffer.append((look, le, lq))

                        d_end = _dist(look["p2"], base_point)
                        if d_end >= d_current * _SNAP_THRESHOLD:
                            continue

                        e_snap_ret = self.energy_model.energy_to_service_static(d_end, max(q_acc, 0.0))
                        e_reserve = self.energy_model.reserve_wh_static()

                        if e_acc - e_snap_ret >= e_reserve and q_acc >= 0.0:
                            for snap_seg, snap_e, snap_q in snap_buffer:
                                current_segs.append({
                                    "p1": snap_seg["p1"], "p2": snap_seg["p2"],
                                    "spraying": snap_seg["spraying"],
                                    "segment_type": snap_seg["segment_type"],
                                    "distance_m": float(snap_seg.get(
                                        "distance_m",
                                        self._dist(snap_seg["p1"], snap_seg["p2"]),
                                    )),
                                })
                                energy_remaining -= snap_e
                                liquid_remaining = max(0.0, liquid_remaining - snap_q)
                            i += len(snap_buffer)
                            snapped = True
                        break

                cycles.append(self._make_cycle(current_segs))

                energy_remaining = usable_energy
                liquid_remaining = self.tank_capacity
                next_start = atomic[i]["p1"]
                dist_commute = _dist(base_point, next_start)
                energy_remaining -= self.energy_model.energy_transit(dist_commute, liquid_remaining)
                current_segs = []

        if current_segs:
            cycles.append(self._make_cycle(current_segs))

        return cycles

    def _make_cycle(self, segments):
        return {
            "segments": segments,
            "start_pt": segments[0]["p1"] if segments else None,
            "end_pt": segments[-1]["p2"] if segments else None,
            "swath_width": self.swath_width,
        }

    def compress_segments(self, segments):
        """Merges adjacent same-type segments into visual groups for rendering."""
        if not segments:
            return []
        groups = []
        current_path = [segments[0]["p1"], segments[0]["p2"]]
        current_type = segments[0]["segment_type"]
        current_spraying = segments[0]["spraying"]
        for seg in segments[1:]:
            if seg["segment_type"] == current_type:
                current_path.append(seg["p2"])
            else:
                groups.append({
                    "path": current_path,
                    "segment_type": current_type,
                    "is_spraying": current_spraying,
                })
                current_path = [seg["p1"], seg["p2"]]
                current_type = seg["segment_type"]
                current_spraying = seg["spraying"]
        groups.append({
            "path": current_path,
            "segment_type": current_type,
            "is_spraying": current_spraying,
        })
        return groups
