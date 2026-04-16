import math
from shapely.geometry import Polygon, LineString, MultiLineString, Point
from shapely import affinity
from shapely.prepared import prep
from shapely.validation import make_valid


class BoustrophedonPlanner:
    """Zig-zag sweep generator for a convex(ish) polygon cell.

    Rotates the polygon to align sweeps horizontally, scans parallel
    lines spaced by ``spray_width`` (alternating direction), rotates
    results back, and clips against optional obstacles. Returns only
    typed sweep segments — ferries between sweeps are built later by
    PathAssembler.
    """

    def __init__(self, spray_width=5.0):
        self.spray_width = spray_width

    def generate_path(self, polygon, angle_deg, global_y_origin=None, rotation_origin=None, obstacles=None):
        polygon = self._validate_polygon(polygon)
        if polygon is None:
            return self._empty_result(angle_deg)

        origin = rotation_origin if rotation_origin is not None else polygon.centroid
        rotated_poly = affinity.rotate(polygon, -angle_deg, origin=origin)

        raw_segments, n_passes = self._scan_sweeps(rotated_poly, global_y_origin)
        if not raw_segments:
            return self._empty_result(angle_deg)

        restored = self._restore_segments(raw_segments, angle_deg, origin)
        sweep_segments = self._build_sweep_records(restored)
        sweep_segments = self._clip_sweep_segments(sweep_segments, obstacles)

        return self._build_result(angle_deg, sweep_segments, n_passes)

    def _validate_polygon(self, polygon):
        if not polygon.is_valid:
            polygon = make_valid(polygon)
        if polygon.geom_type != "Polygon":
            return None
        return polygon

    def _scan_sweeps(self, rotated_poly, global_y_origin):
        """Scan horizontal rows at swath spacing and return the raw segments.

        Returns ``(raw_segments, n_passes)`` where n_passes counts rows
        that produced at least one intersection (used for turn_count).
        """
        min_x, min_y, max_x, max_y = rotated_poly.bounds
        sweep_ext = (max_x - min_x) + 1.0

        y_current = self._first_y(global_y_origin, min_y)
        direction = True
        raw_segments = []
        n_passes = 0

        while y_current < max_y:
            sweepline = LineString([
                (min_x - sweep_ext, y_current),
                (max_x + sweep_ext, y_current),
            ])
            intersection = sweepline.intersection(rotated_poly)

            if not intersection.is_empty:
                segs = self._extract_lines(intersection)
                segs.sort(key=lambda s: s.coords[0][0])

                if not direction:
                    segs.reverse()

                if segs:
                    n_passes += 1

                for seg in segs:
                    coords = list(seg.coords)
                    if not direction:
                        coords.reverse()
                    raw_segments.append(coords)

            y_current += self.spray_width
            direction = not direction

        return raw_segments, n_passes

    def _restore_segments(self, raw_segments, angle_deg, origin):
        """Rotate raw segments back to the original polygon orientation."""
        restored = []
        for coords in raw_segments:
            if len(coords) > 1:
                line = LineString(coords)
                rotated = affinity.rotate(line, angle_deg, origin=origin)
                restored.append(list(rotated.coords))
            elif len(coords) == 1:
                pt = Point(coords[0])
                rotated = affinity.rotate(pt, angle_deg, origin=origin)
                restored.append([rotated.coords[0]])
        return restored

    def _build_sweep_records(self, restored_segments):
        return [
            self._segment_record(coords, segment_type="sweep", spraying=True)
            for coords in restored_segments
            if len(coords) >= 2
        ]

    def _clip_sweep_segments(self, sweep_segments, obstacles):
        if obstacles is None or getattr(obstacles, "is_empty", False):
            return sweep_segments

        prepared_obs = prep(obstacles)
        clipped = []

        for seg in sweep_segments:
            seg_line = LineString(seg["path"])

            if not prepared_obs.intersects(seg_line):
                clipped.append(seg)
                continue

            for part in self._extract_lines(seg_line.difference(obstacles)):
                if part.length > 1e-6:
                    clipped.append(
                        self._segment_record(list(part.coords), segment_type="sweep", spraying=True)
                    )

        return clipped

    def _build_result(self, angle_deg, sweep_segments, n_passes):
        spray_distance_m = sum(seg["distance_m"] for seg in sweep_segments)
        return {
            "angle_deg": angle_deg,
            "sweep_segments": sweep_segments,
            "metrics": {
                "spray_distance_m": float(spray_distance_m),
                "coverage_area_m2": float(spray_distance_m * self.spray_width),
                # Turns are row transitions; clip fragments are ferries, not turns.
                "turn_count": max(0, n_passes - 1),
            },
        }

    def _first_y(self, global_y_origin, min_y):
        """Align the first sweep line to the shared global_y_origin."""
        if global_y_origin is not None:
            first_y = global_y_origin + (self.spray_width / 2.0)
            if first_y < min_y:
                n = math.ceil((min_y - first_y) / self.spray_width)
                first_y += n * self.spray_width
            return first_y
        return min_y + (self.spray_width / 2.0)

    def _empty_result(self, angle_deg):
        return {
            "angle_deg": angle_deg,
            "sweep_segments": [],
            "metrics": {
                "spray_distance_m": 0.0,
                "coverage_area_m2": 0.0,
                "turn_count": 0,
            },
        }

    @staticmethod
    def _extract_lines(geom):
        if geom.is_empty:
            return []
        if isinstance(geom, LineString):
            return [geom]
        if isinstance(geom, MultiLineString):
            return list(geom.geoms)
        return [g for g in getattr(geom, "geoms", []) if isinstance(g, LineString)]

    @staticmethod
    def _segment_record(path_coords, segment_type="sweep", spraying=True):
        line = LineString(path_coords)
        return {
            "segment_type": segment_type,
            "spraying": spraying,
            "path": list(path_coords),
            "distance_m": float(line.length),
        }
