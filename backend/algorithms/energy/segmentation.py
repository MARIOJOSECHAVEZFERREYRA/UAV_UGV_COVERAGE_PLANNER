import math
import numpy as np
from shapely.geometry import LineString

from .energy_model import DroneEnergyModel


class MissionSegmenter:
    """
    Pure ordered-route slicer: splits a typed route into work cycles
    based on drone energy and liquid budget, without reordering the
    input. Preserves the exact sequence produced upstream by the
    PathAssembler.

    Pipeline
    --------
    1. Expand multi-point route segments into atomic 2-point segments,
       copying every piece of metadata (cell_id, spraying, type, ...).
    2. Group atomic segments into SPATIAL RUNS — maximal contiguous
       slices of the route that stay inside one work area. A run ends
       whenever the route crosses a natural boundary:
         * cell_id transitions from one cell to another
         * a ferry/deadhead whose length exceeds a spatial threshold
           (long translations across the field)
    3. Pack runs into cycles respecting the energy/liquid budget and a
       reserved return-to-base margin. Cycles are preferred to close
       at run boundaries so a single cycle never contains work in two
       disconnected areas of the field. A cycle only splits INSIDE a
       run when that run on its own exceeds a full battery cycle.

    Not responsible for
    -------------------
    - Opening/closing deadheads — added downstream by StaticMissionPlanner.
    - Obstacle-aware routing — built upstream by PathAssembler.
    - Rendezvous logic — handled by rendezvous/planner.py for dynamic
      missions.
    """

    # A ferry/deadhead segment longer than this (meters) is treated as
    # a spatial boundary: the cycle must close before it. Threshold is
    # max(_LONG_FERRY_BASE_M, _LONG_FERRY_SWATH_MULT * swath_width) so
    # it scales with the density of the boustrophedon.
    _LONG_FERRY_BASE_M = 15.0
    _LONG_FERRY_SWATH_MULT = 3.0

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

    # ------------------------------------------------------------------
    # Geometry utilities
    # ------------------------------------------------------------------

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
            total += np.linalg.norm(
                np.array(path_coords[i]) - np.array(path_coords[i + 1])
            )
        return float(total)

    # ------------------------------------------------------------------
    # Per-segment cost helpers
    # ------------------------------------------------------------------

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

    # ------------------------------------------------------------------
    # Route expansion and reconstruction
    # ------------------------------------------------------------------

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
        for idx, seg in enumerate(route_segments or []):
            seg_type = seg.get("segment_type")
            if seg_type not in ("sweep", "ferry", "deadhead"):
                raise ValueError(
                    "Unknown or missing segment_type: {}".format(seg_type)
                )
            spraying = bool(seg.get("spraying", seg_type == "sweep"))
            cell_id = seg.get("cell_id")
            path = seg.get("path", [])
            if not path or len(path) < 2:
                continue
            normalized = [
                self._xy(pt[:2]) if len(pt) >= 2 else self._xy(pt)
                for pt in path
            ]
            for atom in self._path_to_segments(
                normalized, seg_type, spraying, cell_id
            ):
                # Record the source index in the original route so later
                # consumers can keep spatial continuity information.
                atom["source_index"] = idx
                atomic.append(atom)
        return atomic

    def segments_to_path(self, segments):
        if not segments:
            return []
        path = [segments[0]["p1"]]
        for seg in segments:
            if not self._pts_equal(path[-1], seg["p2"]):
                path.append(seg["p2"])
        return path

    # ------------------------------------------------------------------
    # Spatial run detection
    # ------------------------------------------------------------------

    def _long_ferry_threshold(self):
        return max(
            self._LONG_FERRY_BASE_M,
            self._LONG_FERRY_SWATH_MULT * float(self.swath_width or 0.0),
        )

    def _build_spatial_runs(self, atomic):
        """Group ordered atomic segments into locally-coherent runs.

        A run is a maximal contiguous slice of the route sharing
        spatial continuity. Boundaries between runs come from:
          * cell_id transition between two segments that both know
            their cell — a soft spatial edge.
          * ferry/deadhead longer than _long_ferry_threshold() —
            a "bridge" segment connecting two distant work areas.

        Long ferries are NOT automatically dropped here. Instead,
        the bridge is attached to the following run via the
        ``bridge_before`` field. The cycle packer then decides:
          * If the next run (bridge + segments) still fits in the
            current cycle's energy/liquid budget, the bridge is
            emitted inside the cycle and the drone keeps working.
          * If the next run does not fit, the packer closes the
            current cycle, drops the bridge, and opens a fresh
            cycle at the first non-bridge segment of the run.
            The downstream StaticMissionPlanner will then add a
            real deadhead from base to that entry point.

        Each returned run is a dict:
            {
                "segments":     list[dict],  # atomic 2-pt segments
                "bridge_before": dict | None, # long ferry separating
                                              # this run from the prev,
                                              # or None.
            }
        """
        if not atomic:
            return []

        long_ferry = self._long_ferry_threshold()
        runs = []
        current = []
        pending_bridge = None
        prev_cid = None

        def _flush():
            nonlocal current, pending_bridge
            if current:
                runs.append({
                    "segments": current,
                    "bridge_before": pending_bridge,
                })
                pending_bridge = None
            current = []

        for seg in atomic:
            cid = seg.get("cell_id")
            stype = seg["segment_type"]
            dist = seg.get("distance_m", 0.0)

            # Long ferry: close the current run and remember the bridge
            # so the next run inherits it. The segment itself is kept
            # only if the packer later chooses to merge across it.
            if stype in ("ferry", "deadhead") and dist > long_ferry:
                _flush()
                pending_bridge = seg
                prev_cid = None
                continue

            # Soft boundary: cell_id transition between tagged segments.
            if (current and prev_cid is not None and cid is not None
                    and cid != prev_cid):
                _flush()

            current.append(seg)
            if cid is not None:
                prev_cid = cid

        _flush()
        return runs

    # ------------------------------------------------------------------
    # Cycle packing — ordered-slicer main entry
    # ------------------------------------------------------------------

    def segment_path(self, route_segments, base_point, dist_fn=None):
        """Slice the ordered route into feasible work cycles.

        Parameters
        ----------
        route_segments : list[dict]
            Typed sweep/ferry/deadhead segments in their original order
            (usually the output of PathAssembler.assemble_connected).
        base_point : tuple[float, float]
            Service base used for return-to-base feasibility checks.
        dist_fn : callable(a, b) -> float, optional
            Obstacle-aware distance function. Defaults to Euclidean.

        Returns
        -------
        list[dict]
            One dict per work cycle:
                {
                    "segments":    list[dict],  # atomic segments, metadata preserved
                    "start_pt":    (x, y),
                    "end_pt":      (x, y),
                    "swath_width": float,
                }
            No open/close deadheads — StaticMissionPlanner wraps each
            cycle with its own base-to-start and end-to-base deadheads.
        """
        _dist = dist_fn if dist_fn is not None else self._dist

        atomic = self._expand_route_segments(route_segments)
        if not atomic:
            return []

        base_point = self._xy(base_point[:2])
        em = self.energy_model
        usable = em.usable_energy_wh()
        reserve = em.reserve_wh_static()

        runs = self._build_spatial_runs(atomic)
        if not runs:
            return []

        cycles = []
        cycle_segs = []
        e_rem = 0.0
        q_rem = 0.0
        cycle_active = False

        def _open_cycle(first_p1):
            nonlocal cycle_segs, e_rem, q_rem, cycle_active
            cycle_segs = []
            q_rem = self.tank_capacity
            e_rem = usable
            commute = _dist(base_point, first_p1)
            e_rem -= em.energy_transit(commute, q_rem)
            cycle_active = True

        def _trim_trailing_nonwork():
            """Drop trailing ferry/deadhead segments that lead nowhere.

            If the cycle ends after a pre-positioning ferry toward work
            that was never executed, the closing deadhead would start
            from that ferry endpoint instead of from the last sprayed
            point — corrupting the static mission geometry. Strip any
            non-sweep segments from the tail and refund their energy to
            keep the running state coherent with the new cycle end.
            """
            nonlocal cycle_segs, e_rem
            while cycle_segs and cycle_segs[-1]["segment_type"] != "sweep":
                dropped = cycle_segs.pop()
                d = dropped.get("distance_m", 0.0)
                # Ferries/deadheads don't consume liquid, so q_rem is
                # unchanged by the trim; only energy needs refund, and
                # it was originally billed with the same q_rem we hold
                # now (no sweep between the trimmed ferry and here).
                e_rem += em.energy_transit(d, q_rem)

        def _close_cycle():
            nonlocal cycle_segs, cycle_active
            _trim_trailing_nonwork()
            if cycle_segs:
                cycles.append(self._make_cycle(cycle_segs))
            cycle_segs = []
            cycle_active = False

        def _simulate_run(sim_e, sim_q, run_segs, fresh_commute_from_base):
            """Dry run: can the whole run fit with current budget?

            Returns (feasible, final_e, final_q).
            """
            if fresh_commute_from_base:
                d = _dist(base_point, run_segs[0]["p1"])
                sim_e -= em.energy_transit(d, sim_q)
            for seg in run_segs:
                stype = seg["segment_type"]
                d = seg.get("distance_m", 0.0)
                sim_e -= self._segment_energy(stype, d, sim_q)
                sim_q -= self._segment_liquid_use(stype, d)
                if sim_q < -1e-9:
                    return False, sim_e, sim_q
            d_ret = _dist(run_segs[-1]["p2"], base_point)
            ret_e = em.energy_transit(d_ret, max(sim_q, 0.0))
            return (sim_e - ret_e) >= reserve, sim_e, sim_q

        def _append_run(run_segs):
            nonlocal e_rem, q_rem
            for seg in run_segs:
                stype = seg["segment_type"]
                d = seg.get("distance_m", 0.0)
                # Preserve cell_id and every other field by deep-copying
                # the atomic dict — this was the bug that lost cell_id.
                cycle_segs.append(dict(seg))
                e_rem -= self._segment_energy(stype, d, q_rem)
                q_rem -= self._segment_liquid_use(stype, d)

        def _split_run_atomically(run_segs):
            """Last-resort: pack a run segment-by-segment across cycles.

            Only used when a whole run cannot fit in a single cycle. We
            still try to extend the current cycle as far as possible
            before breaking inside the run.
            """
            nonlocal cycle_segs, e_rem, q_rem, cycle_active
            for seg in run_segs:
                stype = seg["segment_type"]
                d = seg.get("distance_m", 0.0)
                liq_step = self._segment_liquid_use(stype, d)

                if not cycle_active:
                    _open_cycle(seg["p1"])

                e_step = self._segment_energy(stype, d, q_rem)
                d_ret = _dist(seg["p2"], base_point)
                can = em.feasible_after_segment_static(
                    e_rem, q_rem, e_step, liq_step, d_ret,
                )
                if not can and cycle_segs:
                    _close_cycle()
                    _open_cycle(seg["p1"])
                    e_step = self._segment_energy(stype, d, q_rem)
                    liq_step = self._segment_liquid_use(stype, d)

                cycle_segs.append(dict(seg))
                e_rem -= e_step
                q_rem -= liq_step

        for run_dict in runs:
            run_segs = run_dict["segments"]
            if not run_segs:
                continue
            bridge = run_dict.get("bridge_before")

            # Try to merge (bridge + run) into the currently open
            # cycle. Long ferries no longer force closure by themselves
            # — closure only happens when the budget cannot absorb
            # what comes next.
            if cycle_active:
                merge_segs = ([bridge] + run_segs) if bridge else run_segs
                ok, _ne, _nq = _simulate_run(
                    e_rem, q_rem, merge_segs, fresh_commute_from_base=False,
                )
                if ok:
                    _append_run(merge_segs)
                    continue
                # Next run does not fit — close the current cycle and
                # let the next one start fresh. The bridge is DROPPED
                # on closure: the downstream StaticMissionPlanner will
                # attach a real deadhead from base to the next run's
                # first work segment, which is cheaper than flying the
                # bridge out of a fresh cycle.
                _close_cycle()

            # Fresh cycle: try the run alone (no bridge).
            ok, _ne, _nq = _simulate_run(
                usable, self.tank_capacity, run_segs,
                fresh_commute_from_base=True,
            )
            if ok:
                _open_cycle(run_segs[0]["p1"])
                _append_run(run_segs)
            else:
                # Run is bigger than a full cycle — split inside it.
                _split_run_atomically(run_segs)

        _close_cycle()
        return cycles

    def _make_cycle(self, segments):
        return {
            "segments": segments,
            "start_pt": segments[0]["p1"] if segments else None,
            "end_pt": segments[-1]["p2"] if segments else None,
            "swath_width": self.swath_width,
        }

    # ------------------------------------------------------------------
    # Visual compression
    # ------------------------------------------------------------------

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
