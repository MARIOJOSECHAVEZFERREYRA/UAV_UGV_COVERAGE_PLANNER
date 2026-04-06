import math
from shapely.geometry import Polygon, LineString, Point
from shapely.ops import substring
from shapely.prepared import prep

from .visibility_graph import VisibilityGraph


class PathAssembler:
    """
    Assembles sweep segments into an ordered route.

    Ferry routing strategy:
    - Direct line if it lies entirely inside the polygon (no obstacles crossed).
    - VisibilityGraph shortest path when both endpoints are inside the polygon
      (handles interior obstacles/holes correctly).
    - Exterior ring walk as a last resort, or when an endpoint is outside the
      polygon (e.g. base_point on deadhead segments — holes cannot intercept
      the approach because they are interior to the polygon).
    """

    def __init__(self, polygon):
        self.polygon = polygon
        self._prepared = prep(polygon.buffer(0.01))
        self._ring_line = LineString(polygon.exterior.coords)

        try:
            self._vgraph = VisibilityGraph(polygon)
        except Exception:
            self._vgraph = None

    @staticmethod
    def _pts_equal(a, b, tol=1e-6):
        return abs(a[0] - b[0]) < tol and abs(a[1] - b[1]) < tol

    @staticmethod
    def _segment_record(path_coords, segment_type, spraying):
        line = LineString(path_coords)
        return {
            "segment_type": segment_type,
            "spraying": spraying,
            "path": list(path_coords),
            "distance_m": float(line.length),
        }

    @staticmethod
    def _segments_to_path(route_segments):
        if not route_segments:
            return []

        path = list(route_segments[0]["path"])
        for seg in route_segments[1:]:
            seg_path = seg["path"]
            if not seg_path:
                continue

            if path and seg_path and PathAssembler._pts_equal(path[-1], seg_path[0]):
                path.extend(seg_path[1:])
            else:
                path.extend(seg_path)

        return path

    def _safe_ferry(self, start, end):
        """
        Connect two points without crossing obstacles.

        Priority:
        1. Direct line if it lies entirely inside the polygon.
        2. Direct line if either endpoint is outside the polygon — the UAV
           flies freely in open airspace; interior holes cannot intercept an
           approach from outside, so there is nothing to route around.
        3. VisibilityGraph shortest path when both points are inside the
           polygon (handles interior obstacles/holes correctly).
        4. Exterior ring walk as a last resort when both points are inside
           but the VG fails.
        """
        direct = LineString([start, end])
        dist = math.hypot(end[0] - start[0], end[1] - start[1])

        if self._prepared.contains(direct):
            return [start, end], dist

        start_inside = self.polygon.covers(Point(start))
        end_inside = self.polygon.covers(Point(end))

        # If either endpoint is outside the field, fly direct — no obstacle
        # can intercept an approach from open airspace.
        if not start_inside or not end_inside:
            return [start, end], dist

        # Both inside: use VG to navigate around interior holes.
        if self._vgraph is not None:
            try:
                path, vg_dist = self._vgraph.find_shortest_path(start, end)
                return path, vg_dist
            except (ValueError, Exception):
                pass

        # Last resort: ring walk (both inside, VG unavailable or failed).
        d_start = self._ring_line.project(Point(start))
        d_end = self._ring_line.project(Point(end))

        if abs(d_start - d_end) < 1e-6:
            return [start, end], dist

        path_coords = self._build_ring_walk(d_start, d_end, start, end)
        walk_dist = LineString(path_coords).length
        return path_coords, walk_dist

    def safe_connection(self, start, end):
        return self._safe_ferry(start, end)

    def _build_ring_walk(self, d_start, d_end, start_pt, end_pt):
        """Build shortest walk along the exterior ring."""
        total_len = self._ring_line.length

        if d_start < d_end:
            fwd_len = d_end - d_start
            bwd_len = total_len - fwd_len
        else:
            bwd_len = d_start - d_end
            fwd_len = total_len - bwd_len

        if fwd_len <= bwd_len:
            if d_start < d_end:
                seg = substring(self._ring_line, d_start, d_end)
            else:
                seg1 = substring(self._ring_line, d_start, total_len)
                seg2 = substring(self._ring_line, 0, d_end)
                seg = LineString(list(seg1.coords) + list(seg2.coords)[1:])
        else:
            if d_end < d_start:
                seg_fwd = substring(self._ring_line, d_end, d_start)
            else:
                seg1 = substring(self._ring_line, d_end, total_len)
                seg2 = substring(self._ring_line, 0, d_start)
                seg_fwd = LineString(list(seg1.coords) + list(seg2.coords)[1:])
            coords = list(seg_fwd.coords)
            coords.reverse()
            seg = LineString(coords)

        path_coords = list(seg.coords)
        path_coords[0] = start_pt
        path_coords[-1] = end_pt
        return path_coords

    @staticmethod
    def _sweep_endpoint(path, reversed_):
        """Return the exit endpoint of a sweep given its traversal direction."""
        return path[0] if reversed_ else path[-1]

    @staticmethod
    def _sweep_startpoint(path, reversed_):
        """Return the entry endpoint of a sweep given its traversal direction."""
        return path[-1] if reversed_ else path[0]

    def _two_opt_improve(self, sweeps, order, max_iter=5):
        """
        Apply 2-opt local search to the sweep visit order using Euclidean cost.

        The greedy nearest-neighbor step handles the initial ordering and per-sweep
        direction selection. This pass iteratively reverses subsequences to reduce
        the total Euclidean ferry cost. Using Euclidean distance keeps each
        evaluation O(1) so the overall pass is O(n^2) per iteration. The final
        ferry paths are still built with the obstacle-aware _safe_ferry.

        Parameters
        ----------
        sweeps : list of sweep dicts (path, distance_m, ...)
        order  : list of (sweep_idx, is_reversed) from the greedy step
        max_iter : maximum number of improvement passes

        Returns
        -------
        Improved order as list of (sweep_idx, is_reversed).
        """
        n = len(order)
        if n < 3:
            return order

        def eucl(a, b):
            return math.hypot(b[0] - a[0], b[1] - a[1])

        def edge_eucl(order_, i):
            idx_i, rev_i = order_[i]
            idx_j, rev_j = order_[i + 1]
            a = self._sweep_endpoint(sweeps[idx_i]["path"], rev_i)
            b = self._sweep_startpoint(sweeps[idx_j]["path"], rev_j)
            return eucl(a, b)

        order = list(order)

        for _ in range(max_iter):
            improved = False
            for i in range(n - 2):
                for j in range(i + 2, n):
                    # Old edges: (i → i+1) and (j → j+1 if not last)
                    cost_old = edge_eucl(order, i)
                    if j < n - 1:
                        cost_old += edge_eucl(order, j)

                    # Reverse subsequence order[i+1 : j+1] and flip each direction flag
                    reversed_sub = [(idx, not rev) for idx, rev in reversed(order[i + 1: j + 1])]
                    new_middle = reversed_sub

                    # New edges connect i → new[0] and new[-1] → j+1
                    idx_i, rev_i = order[i]
                    idx_a, rev_a = new_middle[0]
                    idx_z, rev_z = new_middle[-1]

                    a_end = self._sweep_endpoint(sweeps[idx_i]["path"], rev_i)
                    a_start = self._sweep_startpoint(sweeps[idx_a]["path"], rev_a)
                    cost_new = eucl(a_end, a_start)

                    if j < n - 1:
                        idx_next, rev_next = order[j + 1]
                        z_end = self._sweep_endpoint(sweeps[idx_z]["path"], rev_z)
                        next_start = self._sweep_startpoint(sweeps[idx_next]["path"], rev_next)
                        cost_new += eucl(z_end, next_start)

                    if cost_new < cost_old - 1e-9:
                        order = order[:i + 1] + new_middle + order[j + 1:]
                        improved = True

            if not improved:
                break

        return order

    def assemble_connected(self, sweep_segments):
        """
        Assemble typed sweep segments into an ordered route with explicit ferry segments.

        Expected input:
            [
                {
                    "segment_type": "sweep",
                    "spraying": True,
                    "path": [(x, y), ...],
                    "distance_m": float,
                },
                ...
            ]

        Returns:
            {
                "route_segments": [...],
                "combined_path": [...],
                "distances": {
                    "sweep_m": float,
                    "ferry_m": float,
                    "total_m": float,
                },
            }
        """
        sweeps = [
            {
                "segment_type": "sweep",
                "spraying": True,
                "path": list(seg["path"]),
                "distance_m": float(seg.get("distance_m", LineString(seg["path"]).length)),
            }
            for seg in (sweep_segments or [])
            if seg and len(seg.get("path", [])) >= 2
        ]

        if not sweeps:
            return {
                "route_segments": [],
                "combined_path": [],
                "distances": {
                    "sweep_m": 0.0,
                    "ferry_m": 0.0,
                    "total_m": 0.0,
                },
            }

        if len(sweeps) == 1:
            return {
                "route_segments": [sweeps[0]],
                "combined_path": list(sweeps[0]["path"]),
                "distances": {
                    "sweep_m": float(sweeps[0]["distance_m"]),
                    "ferry_m": 0.0,
                    "total_m": float(sweeps[0]["distance_m"]),
                },
            }

        # --- Greedy nearest-neighbor: build visit order as [(sweep_idx, is_reversed)] ---
        remaining = list(range(len(sweeps)))
        current_idx = remaining.pop(0)
        order = [(current_idx, False)]

        while remaining:
            _, last_rev = order[-1]
            last_idx, _ = order[-1]
            current_end = self._sweep_endpoint(sweeps[last_idx]["path"], last_rev)

            best_dist = float("inf")
            best_idx = None
            best_reverse = False

            for idx in remaining:
                candidate_path = sweeps[idx]["path"]

                _, fwd_dist = self._safe_ferry(current_end, candidate_path[0])
                if fwd_dist < best_dist:
                    best_dist = fwd_dist
                    best_idx = idx
                    best_reverse = False

                _, rev_dist = self._safe_ferry(current_end, candidate_path[-1])
                if rev_dist < best_dist:
                    best_dist = rev_dist
                    best_idx = idx
                    best_reverse = True

            remaining.remove(best_idx)
            order.append((best_idx, best_reverse))

        # --- 2-opt improvement on the visit order (Euclidean cost, O(n^2) per pass) ---
        order = self._two_opt_improve(sweeps, order)

        # --- Build route_segments from optimized order using obstacle-aware ferries ---
        route_segments = []
        total_sweep_dist = 0.0
        total_ferry_dist = 0.0

        for pos, (idx, rev) in enumerate(order):
            sw = dict(sweeps[idx])
            path = list(sw["path"])
            if rev:
                path.reverse()
            sw["path"] = path
            sw["distance_m"] = float(LineString(path).length)

            if pos > 0:
                prev_idx, prev_rev = order[pos - 1]
                prev_end = self._sweep_endpoint(sweeps[prev_idx]["path"], prev_rev)
                # After possible reversal in previous iteration, use the already-set path
                prev_path = route_segments[-1]["path"]
                prev_end = prev_path[-1]

                connection_path, conn_dist = self._safe_ferry(prev_end, path[0])
                if connection_path and len(connection_path) > 1:
                    ferry_seg = self._segment_record(
                        connection_path,
                        segment_type="ferry",
                        spraying=False,
                    )
                    route_segments.append(ferry_seg)
                    total_ferry_dist += ferry_seg["distance_m"]

            route_segments.append(sw)
            total_sweep_dist += sw["distance_m"]

        combined_path = self._segments_to_path(route_segments)

        return {
            "route_segments": route_segments,
            "combined_path": combined_path,
            "distances": {
                "sweep_m": float(total_sweep_dist),
                "ferry_m": float(total_ferry_dist),
                "total_m": float(total_sweep_dist + total_ferry_dist),
            },
        }