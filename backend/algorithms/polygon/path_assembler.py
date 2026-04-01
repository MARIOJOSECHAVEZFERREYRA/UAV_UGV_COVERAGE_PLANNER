import math
from typing import List, Tuple
from shapely.geometry import Polygon, LineString, Point
from shapely.ops import substring
from shapely.prepared import prep


class PathAssembler:
    """
    Assembles disjoint Boustrophedon paths into a single continuous mission
    path by finding the shortest safe routes between them.

    Uses direct lines when obstacle-free, otherwise walks along the polygon
    boundary (exterior ring).
    """

    def __init__(self, polygon: Polygon):
        self.polygon = polygon
        self._prepared = prep(polygon.buffer(0.01))
        self._ring_line = LineString(polygon.exterior.coords)
        self.sweep_segments: list = []
        self.ferry_segments: list = []

    def _safe_ferry(self, start: Tuple[float, float],
                    end: Tuple[float, float]) -> Tuple[List[Tuple[float, float]], float]:
        """
        Connect two points without crossing obstacles.
        Direct line if clear, otherwise shortest boundary walk.
        """
        direct = LineString([start, end])
        dist = math.hypot(end[0] - start[0], end[1] - start[1])

        if self._prepared.contains(direct):
            return [start, end], dist

        d_start = self._ring_line.project(Point(start))
        d_end = self._ring_line.project(Point(end))

        if abs(d_start - d_end) < 1e-6:
            return [start, end], dist

        path_coords = self._build_ring_walk(d_start, d_end, start, end)
        walk_dist = LineString(path_coords).length
        return path_coords, walk_dist

    def _build_ring_walk(self, d_start: float, d_end: float,
                         start_pt: tuple, end_pt: tuple) -> List[tuple]:
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

    def assemble_connected(self, sub_paths: List[List[Tuple[float, float]]]) -> Tuple[List[Tuple[float, float]], float]:
        """
        Assembles pre-connected sub-polygon paths (each is a flat list of waypoints).
        Uses boundary walk for ferry connections between sub-polygons.
        """
        self.sweep_segments = []
        self.ferry_segments = []

        paths = [p for p in sub_paths if p and len(p) >= 2]

        if not paths:
            return [], 0.0
        if len(paths) == 1:
            self.sweep_segments = [list(paths[0])]
            return list(paths[0]), 0.0

        remaining = list(range(len(paths)))

        current_idx = remaining.pop(0)
        self.sweep_segments.append(list(paths[current_idx]))
        combined = list(paths[current_idx])
        total_ferry_dist = 0.0

        while remaining:
            current_end = combined[-1]
            best_dist = float("inf")
            best_idx = None
            best_reverse = False
            best_connection_path = []

            for idx in remaining:
                p = paths[idx]

                fwd_connection, fwd_dist = self._safe_ferry(current_end, p[0])
                if fwd_dist < best_dist:
                    best_dist = fwd_dist
                    best_idx = idx
                    best_reverse = False
                    best_connection_path = fwd_connection

                rev_connection, rev_dist = self._safe_ferry(current_end, p[-1])
                if rev_dist < best_dist:
                    best_dist = rev_dist
                    best_idx = idx
                    best_reverse = True
                    best_connection_path = rev_connection

            remaining.remove(best_idx)

            if len(best_connection_path) > 1:
                self.ferry_segments.append(best_connection_path)
                combined.extend(best_connection_path[1:])

            p_next = list(paths[best_idx])
            if best_reverse:
                p_next.reverse()

            self.sweep_segments.append(p_next)
            if self._pts_equal(combined[-1], p_next[0]):
                combined.extend(p_next[1:])
            else:
                combined.extend(p_next)

            total_ferry_dist += best_dist

        return combined, total_ferry_dist

    @staticmethod
    def _pts_equal(a: Tuple[float, float], b: Tuple[float, float], tol: float = 1e-6) -> bool:
        return abs(a[0] - b[0]) < tol and abs(a[1] - b[1]) < tol
