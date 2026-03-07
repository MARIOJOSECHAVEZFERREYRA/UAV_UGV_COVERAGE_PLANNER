import math
from shapely.geometry import Polygon, LineString, MultiLineString, Point
from shapely import affinity
from shapely.ops import substring
from shapely.prepared import prep
from shapely.validation import make_valid
from typing import List, Tuple


class BoustrophedonPlanner:
    """
    Implementation of Phase 3: Boustrophedon (Zig-Zag) Path Generation.
    Adapted to return fitness metrics according to Li et al. (2023).
    """

    def __init__(self, spray_width: float = 5.0):
        self.spray_width = spray_width

    def generate_path(self, polygon: Polygon, angle_deg: float, global_y_origin: float = None, rotation_origin=None) -> Tuple[List[tuple], float, float, int]:
        """
        Generates a coverage path for a given angle and calculates metrics.

        :param polygon: Work zone polygon (may have holes).
        :param angle_deg: Sweep angle (heading) in degrees.
        :param global_y_origin: Global min_y for sweep grid alignment across sub-polygons.
        :param rotation_origin: Common rotation origin (e.g. whole polygon centroid).
                                If None, uses this polygon's own centroid.
        :return: (waypoints, flight_distance_l, coverage_area_S_prime, turn_count)
        """
        if not polygon.is_valid:
            polygon = make_valid(polygon)
            if polygon.geom_type != 'Polygon':
                return [], 0.0, 0.0, 0

        origin = rotation_origin if rotation_origin is not None else polygon.centroid
        rotated_poly = affinity.rotate(polygon, -angle_deg, origin=origin)

        min_x, min_y, max_x, max_y = rotated_poly.bounds
        sweep_ext = (max_x - min_x) + 1

        # Generate sweep line segments
        lines = []
        if global_y_origin is not None:
            first_y = global_y_origin + (self.spray_width / 2)
            if first_y < min_y:
                n = math.ceil((min_y - first_y) / self.spray_width)
                first_y += n * self.spray_width
            y_current = first_y
        else:
            y_current = min_y + (self.spray_width / 2)
        direction = True  # True = Left -> Right
        total_spray_length = 0.0

        while y_current < max_y:
            sweepline = LineString([(min_x - sweep_ext, y_current),
                                    (max_x + sweep_ext, y_current)])
            intersection = sweepline.intersection(rotated_poly)

            if not intersection.is_empty:
                if isinstance(intersection, LineString):
                    segs = [intersection]
                elif isinstance(intersection, MultiLineString):
                    segs = list(intersection.geoms)
                else:
                    segs = [g for g in intersection.geoms
                            if isinstance(g, LineString)]

                segs.sort(key=lambda s: s.coords[0][0])

                if not direction:
                    segs.reverse()

                for seg in segs:
                    coords = list(seg.coords)
                    total_spray_length += seg.length
                    if not direction:
                        coords.reverse()
                    lines.append(coords)

            y_current += self.spray_width
            direction = not direction

        if not lines:
            return [], 0.0, 0.0, 0

        # Build continuous path with safe connections
        buffered_poly = rotated_poly.buffer(0.01)
        prepared_buffered = prep(buffered_poly)

        continuous_path = list(lines[0])
        turn_count = 0

        for i in range(1, len(lines)):
            prev_end = continuous_path[-1]
            curr_start = lines[i][0]

            connection = self._safe_connection(
                prev_end, curr_start, rotated_poly, prepared_buffered)

            # Count turns: transition between different scanlines
            y_diff = abs(prev_end[1] - curr_start[1])
            if y_diff >= 1e-4:
                turn_count += 1

            # Append connection (skip first point to avoid duplicate)
            if len(connection) > 1:
                continuous_path.extend(connection[1:])

            # Append segment (skip first point if it matches last connection point)
            seg = lines[i]
            if continuous_path and len(seg) > 0:
                if self._pts_equal(continuous_path[-1], seg[0]):
                    continuous_path.extend(seg[1:])
                else:
                    continuous_path.extend(seg)

        # Un-rotate path back to original coordinates
        final_waypoints = []
        flight_distance_l = 0.0

        if len(continuous_path) > 1:
            path_line = LineString(continuous_path)
            restored_path = affinity.rotate(path_line, angle_deg, origin=origin)
            final_waypoints = list(restored_path.coords)
            flight_distance_l = restored_path.length
        elif len(continuous_path) == 1:
            p = Point(continuous_path[0])
            restored_p = affinity.rotate(p, angle_deg, origin=origin)
            final_waypoints = [restored_p.coords[0]]

        coverage_area_s_prime = total_spray_length * self.spray_width

        return final_waypoints, flight_distance_l, coverage_area_s_prime, turn_count

    @staticmethod
    def _pts_equal(a, b, tol=1e-6):
        return abs(a[0] - b[0]) < tol and abs(a[1] - b[1]) < tol

    def _safe_connection(self, start_pt: tuple, end_pt: tuple,
                         polygon: Polygon, prepared_buffered=None) -> List[tuple]:
        """
        Returns a path from start_pt to end_pt that stays inside the polygon.
        If the direct line is contained, returns [start_pt, end_pt].
        Otherwise, walks along the nearest boundary (exterior or hole).
        """
        direct = LineString([start_pt, end_pt])

        # Quick check: if direct line stays inside (with small tolerance), use it
        if prepared_buffered is not None:
            if prepared_buffered.contains(direct):
                return [start_pt, end_pt]
        elif polygon.buffer(0.01).contains(direct):
            return [start_pt, end_pt]

        # Walk along exterior ring (sub-cells should have no holes after decomposition)
        start_p = Point(start_pt)
        end_p = Point(end_pt)

        ring_line = LineString(polygon.exterior.coords)
        d_start = ring_line.project(start_p)
        d_end = ring_line.project(end_p)

        if abs(d_start - d_end) < 1e-6:
            return [start_pt, end_pt]

        return self._build_ring_walk(ring_line, d_start, d_end, start_pt, end_pt)

    @staticmethod
    def _build_ring_walk(ring_line: LineString, d_start: float, d_end: float,
                         start_pt: tuple, end_pt: tuple) -> List[tuple]:
        """Build shortest walk along a ring between two projected distances."""
        total_len = ring_line.length

        if d_start < d_end:
            fwd_len = d_end - d_start
            bwd_len = total_len - fwd_len
        else:
            bwd_len = d_start - d_end
            fwd_len = total_len - bwd_len

        if fwd_len <= bwd_len:
            if d_start < d_end:
                seg = substring(ring_line, d_start, d_end)
            else:
                seg1 = substring(ring_line, d_start, total_len)
                seg2 = substring(ring_line, 0, d_end)
                seg = LineString(list(seg1.coords) + list(seg2.coords)[1:])
        else:
            if d_end < d_start:
                seg_fwd = substring(ring_line, d_end, d_start)
            else:
                seg1 = substring(ring_line, d_end, total_len)
                seg2 = substring(ring_line, 0, d_start)
                seg_fwd = LineString(list(seg1.coords) + list(seg2.coords)[1:])
            coords = list(seg_fwd.coords)
            coords.reverse()
            seg = LineString(coords)

        path_coords = list(seg.coords)
        path_coords[0] = start_pt
        path_coords[-1] = end_pt
        return path_coords
