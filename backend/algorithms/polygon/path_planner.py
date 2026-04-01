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

    def generate_path(self, polygon: Polygon, angle_deg: float, global_y_origin: float = None, rotation_origin=None, obstacles=None) -> Tuple[List[tuple], float, float, int]:
        """
        Generates a coverage path for a given angle and calculates metrics.

        :param polygon: Work zone polygon (may have holes).
        :param angle_deg: Sweep angle (heading) in degrees.
        :param global_y_origin: Global min_y for sweep grid alignment across sub-polygons.
        :param rotation_origin: Common rotation origin (e.g. whole polygon centroid).
                                If None, uses this polygon's own centroid.
        :return: (waypoints [list of separate segments], flight_distance_l, coverage_area_S_prime, turn_count)
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
            # We don't need to forcefully alternate `direction` anymore,
            # as PathAssembler will determine the optimal TSP routing and direction 
            # for every individual sweep segment.
            direction = not direction

        if not lines:
            return [], 0.0, 0.0, 0

        # Un-rotate all independent sweep line segments back to original coordinates
        final_segments = []
        flight_distance_l = 0.0

        for segment_coords in lines:
            if len(segment_coords) > 1:
                path_line = LineString(segment_coords)
                restored_path = affinity.rotate(path_line, angle_deg, origin=origin)
                final_segments.append(list(restored_path.coords))
                flight_distance_l += restored_path.length
            elif len(segment_coords) == 1:
                p = Point(segment_coords[0])
                restored_p = affinity.rotate(p, angle_deg, origin=origin)
                final_segments.append([restored_p.coords[0]])

        # Post-process: clip sweep segments against precomputed obstacle union
        if obstacles is not None:
            prepared_obs = prep(obstacles)
            clipped_segments = []
            clipped_distance = 0.0
            for seg_coords in final_segments:
                if len(seg_coords) < 2:
                    clipped_segments.append(seg_coords)
                    continue
                seg_line = LineString(seg_coords)
                # Fast rejection: skip expensive difference if no intersection
                if not prepared_obs.intersects(seg_line):
                    clipped_segments.append(seg_coords)
                    clipped_distance += seg_line.length
                    continue
                diff = seg_line.difference(obstacles)
                if diff.is_empty:
                    continue
                if isinstance(diff, LineString):
                    parts = [diff]
                elif isinstance(diff, MultiLineString):
                    parts = list(diff.geoms)
                else:
                    parts = [g for g in diff.geoms if isinstance(g, LineString)]
                for part in parts:
                    if part.length > 1e-6:
                        clipped_segments.append(list(part.coords))
                        clipped_distance += part.length
            final_segments = clipped_segments
            flight_distance_l = clipped_distance

        coverage_area_s_prime = total_spray_length * self.spray_width
        turn_count = len(final_segments) - 1 if len(final_segments) > 0 else 0

        # Connect sweep segments internally using safe connections
        # Sub-polygons after decomposition have no holes, so boundary walk is safe
        if len(final_segments) > 1:
            prepared_buffered = prep(polygon.buffer(0.01))
            ring_line = LineString(polygon.exterior.coords)
            connected_path = list(final_segments[0])
            for i in range(1, len(final_segments)):
                seg = final_segments[i]
                if not seg or len(seg) < 1:
                    continue
                start_pt = connected_path[-1]
                end_pt = seg[0]
                # Try direct connection first (fast path)
                direct = LineString([start_pt, end_pt])
                if prepared_buffered.contains(direct):
                    connection = [start_pt, end_pt]
                else:
                    d_start = ring_line.project(Point(start_pt))
                    d_end = ring_line.project(Point(end_pt))
                    if abs(d_start - d_end) < 1e-6:
                        connection = [start_pt, end_pt]
                    else:
                        connection = self._build_ring_walk(
                            ring_line, d_start, d_end, start_pt, end_pt
                        )
                # Add connection (skip first point = current end) then sweep segment
                flight_distance_l += LineString(connection).length
                connected_path.extend(connection[1:])
                connected_path.extend(seg[1:] if self._pts_equal(connected_path[-1], seg[0]) else seg)
            final_segments = connected_path
        elif len(final_segments) == 1:
            final_segments = final_segments[0]
        else:
            final_segments = []

        return final_segments, flight_distance_l, coverage_area_s_prime, turn_count

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
