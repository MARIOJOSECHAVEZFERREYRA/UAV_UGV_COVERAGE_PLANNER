from shapely.geometry import Polygon, LineString, MultiLineString, Point, LinearRing
from shapely import affinity
from shapely.ops import substring
from shapely.validation import make_valid
from typing import List, Tuple


class BoustrophedonPlanner:
    """
    Implementation of Phase 3: Boustrophedon (Zig-Zag) Path Generation.
    Adapted to return fitness metrics according to Li et al. (2023).
    """

    def __init__(self, spray_width: float = 5.0):
        self.spray_width = spray_width

    def generate_path(self, polygon: Polygon, angle_deg: float) -> Tuple[List[tuple], float, float, int]:
        """
        Generates a coverage path for a given angle and calculates metrics.

        :param polygon: Work zone polygon (may have holes).
        :param angle_deg: Sweep angle (heading) in degrees.
        :return: (waypoints, flight_distance_l, coverage_area_S_prime, turn_count)
        """
        if not polygon.is_valid:
            polygon = make_valid(polygon)
            if polygon.geom_type != 'Polygon':
                return [], 0.0, 0.0, 0

        centroid = polygon.centroid
        rotated_poly = affinity.rotate(polygon, -angle_deg, origin=centroid)

        min_x, min_y, max_x, max_y = rotated_poly.bounds

        # Generate sweep line segments
        lines = []
        y_current = min_y + (self.spray_width / 2)
        direction = True  # True = Left -> Right
        total_spray_length = 0.0

        while y_current < max_y:
            sweepline = LineString([(min_x - 1000, y_current), (max_x + 1000, y_current)])
            intersection = sweepline.intersection(rotated_poly)

            if not intersection.is_empty:
                if isinstance(intersection, MultiLineString):
                    segs = list(intersection.geoms)
                else:
                    segs = [intersection]

                segs.sort(key=lambda s: s.coords[0][0])

                if not direction:
                    segs.reverse()

                for seg in segs:
                    coords = list(seg.coords)
                    seg_len = Point(coords[0]).distance(Point(coords[-1]))
                    total_spray_length += seg_len
                    if not direction:
                        coords.reverse()
                    lines.append(coords)

            y_current += self.spray_width
            direction = not direction

        if not lines:
            return [], 0.0, 0.0, 0

        # Build continuous path with safe connections
        continuous_path = list(lines[0])
        turn_count = 0

        for i in range(1, len(lines)):
            prev_end = continuous_path[-1]
            curr_start = lines[i][0]

            connection = self._safe_connection(prev_end, curr_start, rotated_poly)

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
            restored_path = affinity.rotate(path_line, angle_deg, origin=centroid)
            final_waypoints = list(restored_path.coords)
            flight_distance_l = restored_path.length
        elif len(continuous_path) == 1:
            p = Point(continuous_path[0])
            restored_p = affinity.rotate(p, angle_deg, origin=centroid)
            final_waypoints = [restored_p.coords[0]]

        coverage_area_s_prime = total_spray_length * self.spray_width

        return final_waypoints, flight_distance_l, coverage_area_s_prime, turn_count

    @staticmethod
    def _pts_equal(a, b, tol=1e-6):
        return abs(a[0] - b[0]) < tol and abs(a[1] - b[1]) < tol

    def _safe_connection(self, start_pt: tuple, end_pt: tuple, polygon: Polygon) -> List[tuple]:
        """
        Returns a path from start_pt to end_pt that stays inside the polygon.
        If the direct line is contained, returns [start_pt, end_pt].
        Otherwise, walks along the nearest boundary (exterior or hole).
        """
        direct = LineString([start_pt, end_pt])

        # Quick check: if direct line stays inside (with small tolerance), use it
        if polygon.buffer(0.01).contains(direct):
            return [start_pt, end_pt]

        # Find which boundary ring to walk along.
        # Project both points onto all rings, pick the ring where both are closest.
        start_p = Point(start_pt)
        end_p = Point(end_pt)

        all_rings = [polygon.exterior] + [LinearRing(h.coords) for h in polygon.interiors]

        best_ring = None
        best_score = float('inf')

        for ring in all_rings:
            d1 = ring.distance(start_p)
            d2 = ring.distance(end_p)
            score = d1 + d2
            if score < best_score:
                best_score = score
                best_ring = ring

        if best_ring is None:
            return [start_pt, end_pt]

        # Get distances along the ring
        ring_line = LineString(best_ring.coords)
        d_start = ring_line.project(start_p)
        d_end = ring_line.project(end_p)
        total_len = ring_line.length

        if abs(d_start - d_end) < 1e-6:
            return [start_pt, end_pt]

        # Two possible paths along the ring; pick the shorter one
        if d_start < d_end:
            fwd_len = d_end - d_start
            bwd_len = total_len - fwd_len
        else:
            bwd_len = d_start - d_end
            fwd_len = total_len - bwd_len

        if fwd_len <= bwd_len:
            # Forward path
            if d_start < d_end:
                seg = substring(ring_line, d_start, d_end)
            else:
                seg1 = substring(ring_line, d_start, total_len)
                seg2 = substring(ring_line, 0, d_end)
                coords1 = list(seg1.coords)
                coords2 = list(seg2.coords)
                seg = LineString(coords1 + coords2[1:])
        else:
            # Backward path: get forward from end->start, then reverse
            if d_end < d_start:
                seg_fwd = substring(ring_line, d_end, d_start)
            else:
                seg1 = substring(ring_line, d_end, total_len)
                seg2 = substring(ring_line, 0, d_start)
                coords1 = list(seg1.coords)
                coords2 = list(seg2.coords)
                seg_fwd = LineString(coords1 + coords2[1:])
            coords = list(seg_fwd.coords)
            coords.reverse()
            seg = LineString(coords)

        # Build final path: start_pt -> boundary walk -> end_pt
        path_coords = list(seg.coords)

        # Replace first/last with exact start/end to avoid floating point gaps
        path_coords[0] = start_pt
        path_coords[-1] = end_pt

        return path_coords
