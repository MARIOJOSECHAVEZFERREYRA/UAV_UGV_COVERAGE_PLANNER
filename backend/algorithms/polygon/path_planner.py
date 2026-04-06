import math
from shapely.geometry import Polygon, LineString, MultiLineString, Point
from shapely import affinity
from shapely.prepared import prep
from shapely.validation import make_valid


class BoustrophedonPlanner:
    """
    Boustrophedon (zig-zag) sweep generation.

    - This planner returns typed sweep segments only.
    - It does NOT connect them into one flattened path.
    - Ferry/safe connections must be built later by PathAssembler.
    """

    def __init__(self, spray_width=5.0):
        self.spray_width = spray_width

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

    def _clip_sweep_segments(self, sweep_segments, obstacles):
        if obstacles is None or getattr(obstacles, "is_empty", False):
            return sweep_segments

        prepared_obs = prep(obstacles)
        clipped_segments = []

        for seg in sweep_segments:
            path_coords = seg["path"]
            if len(path_coords) < 2:
                continue

            seg_line = LineString(path_coords)

            if not prepared_obs.intersects(seg_line):
                clipped_segments.append(seg)
                continue

            diff = seg_line.difference(obstacles)
            parts = self._extract_lines(diff)

            for part in parts:
                if part.length > 1e-6:
                    clipped_segments.append(
                        self._segment_record(list(part.coords), segment_type="sweep", spraying=True)
                    )

        return clipped_segments

    def generate_path(self,polygon,angle_deg,global_y_origin=None,rotation_origin=None,obstacles=None,
    ):
        
        if not polygon.is_valid:
            polygon = make_valid(polygon)
            if polygon.geom_type != "Polygon":
                return {
                    "angle_deg": angle_deg,
                    "sweep_segments": [],
                    "metrics": {
                        "spray_distance_m": 0.0,
                        "coverage_area_m2": 0.0,
                        "turn_count": 0,
                    },
                }

        origin = rotation_origin if rotation_origin is not None else polygon.centroid
        rotated_poly = affinity.rotate(polygon, -angle_deg, origin=origin)

        min_x, min_y, max_x, max_y = rotated_poly.bounds
        sweep_ext = (max_x - min_x) + 1.0

        raw_segments = []

        if global_y_origin is not None:
            first_y = global_y_origin + (self.spray_width / 2.0)
            if first_y < min_y:
                n = math.ceil((min_y - first_y) / self.spray_width)
                first_y += n * self.spray_width
            y_current = first_y
        else:
            y_current = min_y + (self.spray_width / 2.0)

        direction = True
        n_passes_with_segments = 0

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
                    n_passes_with_segments += 1

                for seg in segs:
                    coords = list(seg.coords)
                    if not direction:
                        coords.reverse()
                    raw_segments.append(coords)

            y_current += self.spray_width
            direction = not direction

        if not raw_segments:
            return {
                "angle_deg": angle_deg,
                "sweep_segments": [],
                "metrics": {
                    "spray_distance_m": 0.0,
                    "coverage_area_m2": 0.0,
                    "turn_count": 0,
                },
            }

        restored_segments = []

        for segment_coords in raw_segments:
            if len(segment_coords) > 1:
                path_line = LineString(segment_coords)
                restored_path = affinity.rotate(path_line, angle_deg, origin=origin)
                restored_segments.append(list(restored_path.coords))
            elif len(segment_coords) == 1:
                p = Point(segment_coords[0])
                restored_p = affinity.rotate(p, angle_deg, origin=origin)
                restored_segments.append([restored_p.coords[0]])

        sweep_segments = []
        for path_coords in restored_segments:
            if len(path_coords) >= 2:
                sweep_segments.append(
                    self._segment_record(path_coords, segment_type="sweep", spraying=True)
                )

        sweep_segments = self._clip_sweep_segments(sweep_segments, obstacles)

        spray_distance_m = sum(seg["distance_m"] for seg in sweep_segments)
        coverage_area_m2 = spray_distance_m * self.spray_width
        # Cada pasada de y_current es una fila de barrido; el numero de giros es
        # el numero de filas con al menos un segmento menos 1. El clipeo de obstaculos
        # puede dividir un segmento en varios, pero eso produce ferries, no giros.
        turn_count = max(0, n_passes_with_segments - 1)

        return {
            "angle_deg": angle_deg,
            "sweep_segments": sweep_segments,
            "metrics": {
                "spray_distance_m": float(spray_distance_m),
                "coverage_area_m2": float(coverage_area_m2),
                "turn_count": int(turn_count),
            },
        }