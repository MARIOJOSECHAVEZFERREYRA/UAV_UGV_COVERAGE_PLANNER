import numpy as np
import math
from shapely.geometry import Polygon, LineString, Point, MultiPolygon
from shapely.ops import split
from shapely.geometry.polygon import orient
from shapely.validation import make_valid
from shapely.prepared import prep


class ConcaveDecomposer:
    """Type-2 concavity decomposition (Li et al. 2023, sec. 2.3–2.4).

    Cuts a polygon along type-2 concave vertices so each sub-polygon
    can be covered by a single boustrophedon pass in the given heading.
    Holes are first absorbed into the exterior via a thin topological
    slit so they also appear as cuttable concavities.
    """

    @staticmethod
    def _find_interior_direction(polygon: Polygon, vx: float, vy: float,
                                  hx: float, hy: float) -> float | None:
        """Return +1/-1 for the heading side that points inward, or None."""
        eps = max(0.5, polygon.length * 0.002)
        for sign in (+1.0, -1.0):
            test_pt = Point(vx + sign * eps * hx, vy + sign * eps * hy)
            if polygon.contains(test_pt):
                return sign
        return None

    @staticmethod
    def _extract_hit_points(intersection) -> list[Point]:
        """Return only the Point hits from an intersection result."""
        if intersection.is_empty:
            return []
        if intersection.geom_type == 'Point':
            return [intersection]
        if intersection.geom_type == 'MultiPoint':
            return list(intersection.geoms)
        if intersection.geom_type in ('LineString', 'MultiLineString'):
            return []
        return [g for g in getattr(intersection, 'geoms', [])
                if g.geom_type == 'Point']

    @staticmethod
    def _cast_ray_to_boundary(polygon: Polygon, vx: float, vy: float,
                               dx: float, dy: float,
                               ray_len: float = 1e6) -> Point | None:
        """Nearest hit of a ray from (vx,vy) against exterior + interiors."""
        origin = Point(vx, vy)
        far_pt = (vx + ray_len * dx, vy + ray_len * dy)
        ray_line = LineString([(vx, vy), far_pt])

        boundary_lines = [polygon.exterior] + list(polygon.interiors)
        hit_pt = None
        min_dist = ray_len

        for ring in boundary_lines:
            candidates = ConcaveDecomposer._extract_hit_points(
                ray_line.intersection(ring)
            )
            for pt in candidates:
                d = pt.distance(origin)
                if d > 1e-6 and d < min_dist:
                    min_dist = d
                    hit_pt = pt

        return hit_pt

    @staticmethod
    def _cast_ray_to_ring(ring, vx: float, vy: float,
                           dx: float, dy: float,
                           ray_len: float = 1e6) -> Point | None:
        """Nearest hit of a ray from (vx,vy) against a single ring."""
        origin = Point(vx, vy)
        ray_line = LineString([(vx, vy), (vx + ray_len * dx, vy + ray_len * dy)])
        candidates = ConcaveDecomposer._extract_hit_points(ray_line.intersection(ring))
        hit_pt, min_dist = None, ray_len
        for pt in candidates:
            d = pt.distance(origin)
            if d > 1e-6 and d < min_dist:
                min_dist = d
                hit_pt = pt
        return hit_pt

    @staticmethod
    def _min_bounding_width(polygon: Polygon) -> float:
        """Minimum width of the polygon's minimum rotated bounding rectangle."""
        mrr = polygon.minimum_rotated_rectangle
        coords = list(mrr.exterior.coords)
        d1 = math.hypot(coords[1][0] - coords[0][0], coords[1][1] - coords[0][1])
        d2 = math.hypot(coords[2][0] - coords[1][0], coords[2][1] - coords[1][1])
        return min(d1, d2)

    @staticmethod
    def _validate_and_recurse(polygon: Polygon, sub_polygons: list[Polygon],
                               heading_angle_deg: float,
                               depth: int,
                               channel_width: float = 1.0,
                               min_swath: float = 0.0) -> tuple[list[Polygon], bool]:
        """Reject degenerate splits and recurse into the remaining pieces."""
        valid_subs = [
            s for s in sub_polygons
            if s.area > 0.1 and (
                min_swath <= 0.0 or
                ConcaveDecomposer._min_bounding_width(s) >= min_swath
            )
        ]
        if not valid_subs:
            return [], False

        # Reject splits that failed to reduce the polygon meaningfully.
        if len(valid_subs) == 1:
            v = valid_subs[0]
            if (len(list(v.interiors)) >= len(list(polygon.interiors))
                    or v.area < 0.95 * polygon.area):
                return [], False
        else:
            if sum(s.area for s in valid_subs) < 0.95 * polygon.area:
                return [], False

        result = []
        for sub in valid_subs:
            sub = orient(sub, sign=1.0)
            result.extend(
                ConcaveDecomposer.decompose(sub, heading_angle_deg, depth + 1, channel_width, min_swath)
            )
        return result, True

    @staticmethod
    def _find_concave_indices(coords: list) -> list[int]:
        """Return indices of concave vertices (right turns under CCW)."""
        n = len(coords)
        concave = []
        for i in range(n):
            curr = np.array(coords[i])
            prev = np.array(coords[(i - 1) % n])
            nxt = np.array(coords[(i + 1) % n])
            vec_prev = prev - curr
            vec_next = nxt - curr
            cross = vec_next[0] * vec_prev[1] - vec_next[1] * vec_prev[0]
            if cross < -1e-3:
                concave.append(i)
        return concave

    @staticmethod
    def _count_sweep_segments(polygon: Polygon, prepared_polygon,
                               pt: np.ndarray, heading: np.ndarray,
                               ray: float = 1e5) -> int:
        """Count how many pieces a virtual sweep line through `pt` splits into."""
        sweep = LineString([
            (pt[0] - ray * heading[0], pt[1] - ray * heading[1]),
            (pt[0] + ray * heading[0], pt[1] + ray * heading[1]),
        ])

        if not prepared_polygon.intersects(sweep):
            return 0

        inter = sweep.intersection(polygon)

        if inter.is_empty: 
            return 0
        if inter.geom_type == 'LineString':
            return 1
        if inter.geom_type == 'MultiLineString': 
            return len(list(inter.geoms))
        
        if hasattr(inter, 'geoms'):
            return sum(1 for g in inter.geoms if g.geom_type == 'LineString')
        
        return 0
        

    @staticmethod
    def _classify_type2_batch(polygon: Polygon, coords: list,
                               concave_indices: list[int],
                               heading_rad: float) -> list[int]:
        """Return the subset of concave vertices that are type-2.

        A vertex is type-2 when the number of sweep-line segments
        changes as the sweep advances past it.
        """
        if not concave_indices:
            return []

        eps = 0.5
        heading = np.array([np.cos(heading_rad), np.sin(heading_rad)])
        advancing = np.array([-np.sin(heading_rad), np.cos(heading_rad)])

        prepared = prep(polygon)

        type2 = []
        for i in concave_indices:
            curr = np.array(coords[i])
            n_before = ConcaveDecomposer._count_sweep_segments(
                polygon, prepared, curr - eps * advancing, heading
            )
            n_after = ConcaveDecomposer._count_sweep_segments(
                polygon, prepared, curr + eps * advancing, heading
            )
            if n_before != n_after:
                type2.append(i)

        return type2

    @staticmethod
    def _classify_hole_type2_batch(polygon: Polygon, hole_coords: list,
                                    heading_rad: float) -> list[int]:
        """Type-2 classification for a hole's vertices (same criterion as exterior)."""
        if not hole_coords:
            return []

        eps = 0.5
        heading = np.array([np.cos(heading_rad), np.sin(heading_rad)])
        advancing = np.array([-np.sin(heading_rad), np.cos(heading_rad)])

        prepared = prep(polygon)

        type2 = []
        for i, vertex in enumerate(hole_coords):
            curr = np.array(vertex)
            n_before = ConcaveDecomposer._count_sweep_segments(
                polygon, prepared, curr - eps * advancing, heading
            )
            n_after = ConcaveDecomposer._count_sweep_segments(
                polygon, prepared, curr + eps * advancing, heading
            )
            if n_before != n_after:
                type2.append(i)

        return type2

    @staticmethod
    def _split_polygon_at_vertex(polygon: Polygon, vertex_coords: tuple,
                                  heading_rad: float) -> list[Polygon]:
        """Cut the polygon with a ray from a type-2 vertex along the heading."""
        vx, vy = vertex_coords
        hx, hy = np.cos(heading_rad), np.sin(heading_rad)

        interior_sign = ConcaveDecomposer._find_interior_direction(
            polygon, vx, vy, hx, hy
        )
        if interior_sign is None:
            return []

        dx = interior_sign * hx
        dy = interior_sign * hy

        hit_pt = ConcaveDecomposer._cast_ray_to_boundary(
            polygon, vx, vy, dx, dy
        )
        if hit_pt is None:
            return []

        epsilon = 1e-3
        p1 = Point(vx - epsilon * dx, vy - epsilon * dy)
        p2 = Point(hit_pt.x + epsilon * dx, hit_pt.y + epsilon * dy)

        def _attempt_split(cut_p2):
            cut_line = LineString([p1, cut_p2])
            try:
                coll = split(polygon, cut_line)
                return [g for g in coll.geoms if isinstance(g, Polygon)]
            except Exception:
                return []

        polys = _attempt_split(p2)

        # Degenerate: cut collinear with an edge returned the same polygon.
        if len(polys) == 1:
            diff_holes = len(list(polys[0].interiors)) - len(list(polygon.interiors))
            if diff_holes == 0 and abs(polys[0].area - polygon.area) < 1e-6:
                polys = []

        # Retry with a tiny angle nudge to force a transverse crossing.
        if not polys:
            angle_perturbation = 0.01
            new_angle = heading_rad + angle_perturbation
            new_dx = interior_sign * np.cos(new_angle)
            new_dy = interior_sign * np.sin(new_angle)
            ray_dist = math.hypot(hit_pt.x - vx, hit_pt.y - vy)
            perturbed_p2 = Point(
                vx + (ray_dist + epsilon) * new_dx,
                vy + (ray_dist + epsilon) * new_dy,
            )
            polys = _attempt_split(perturbed_p2)

        return polys

    # Razor-thin slit used only to split topology; must NOT be
    # swath-wide or we would delete a sweep row per obstacle. The
    # BoustrophedonPlanner clips sweeps against the real holes later.
    _SLIT_RADIUS = 0.1

    @staticmethod
    def _connect_hole_to_exterior(polygon: Polygon, vertex_coords: tuple,
                                   heading_rad: float,
                                   channel_width: float = 1.0) -> list[Polygon]:
        """Absorb a hole into the exterior via a thin topological slit.

        Cuts from a type-2 concave vertex of the hole along the heading
        direction until it reaches the exterior, crossing any
        intermediate holes on the way. `channel_width` is kept for
        signature compatibility and no longer affects the slit width.
        """
        vx, vy = vertex_coords
        hx, hy = np.cos(heading_rad), np.sin(heading_rad)

        interior_sign = ConcaveDecomposer._find_interior_direction(
            polygon, vx, vy, hx, hy
        )
        if interior_sign is None:
            return [polygon]

        dx = interior_sign * hx
        dy = interior_sign * hy

        # Ray casts only against the exterior so intermediate holes
        # are absorbed along the slit instead of stopping it.
        hit_pt = ConcaveDecomposer._cast_ray_to_ring(
            polygon.exterior, vx, vy, dx, dy
        )
        if hit_pt is None:
            return [polygon]

        # Extend past the endpoints so the boolean difference cuts
        # cleanly through the boundary (avoids tangent-only results).
        ext = 0.5
        p1 = Point(vx - ext * dx, vy - ext * dy)
        p2 = Point(hit_pt.x + ext * dx, hit_pt.y + ext * dy)

        channel = LineString([p1, p2]).buffer(
            ConcaveDecomposer._SLIT_RADIUS, cap_style=2,
        )
        result = polygon.difference(channel)

        if isinstance(result, MultiPolygon):
            return list(result.geoms)
        elif isinstance(result, Polygon):
            return [result]
        return [polygon]

    @staticmethod
    def _connect_holes_into_exterior(polygon: Polygon, heading_rad: float,
                            max_iter: int = 100,
                            channel_width: float = 1.0) -> Polygon | list[Polygon]:
        """Absorb every hole into the exterior; may split the polygon."""
        for _ in range(max_iter):
            if not list(polygon.interiors):
                return polygon

            found = False
            for interior in polygon.interiors:
                hole_coords = list(interior.coords)
                if hole_coords[0] == hole_coords[-1]:
                    hole_coords = hole_coords[:-1]

                hole_t2 = ConcaveDecomposer._classify_hole_type2_batch(
                    polygon, hole_coords, heading_rad
                )
                if not hole_t2:
                    continue

                for idx in hole_t2:
                    subs = ConcaveDecomposer._connect_hole_to_exterior(
                        polygon, hole_coords[idx], heading_rad, channel_width
                    )
                    valid = [s for s in subs if s.area > 0.1]
                    if not valid:
                        continue

                    total_holes_before = len(list(polygon.interiors))
                    total_holes_after = sum(len(list(s.interiors)) for s in valid)
                    if total_holes_after >= total_holes_before:
                        continue

                    if len(valid) == 1:
                        polygon = orient(valid[0], sign=1.0)
                        found = True
                        break
                    else:
                        return valid

                if found:
                    break

            if not found:
                break

        return polygon

    @staticmethod
    def decompose(polygon: Polygon, heading_angle_deg: float,
                   depth: int = 0,
                   channel_width: float = 1.0,
                   min_swath: float = 0.0) -> list[Polygon]:
        """Recursively decompose a polygon on type-2 concavities.

        Holes are absorbed into the exterior first. Then, for each
        remaining type-2 vertex, the polygon is split with a ray along
        the heading and every resulting piece is re-entered. Pieces
        without further type-2 concavities are returned as leaf cells.
        """
        if depth > 25:
            return [polygon]

        polygon = orient(polygon, sign=1.0)
        heading_rad = np.radians(heading_angle_deg)

        resolved = ConcaveDecomposer._connect_holes_into_exterior(polygon, heading_rad, channel_width=channel_width)
        if isinstance(resolved, list):
            result = []
            for sub in resolved:
                sub = orient(sub, sign=1.0)
                result.extend(ConcaveDecomposer.decompose(sub, heading_angle_deg, depth + 1, channel_width, min_swath))
            return result
        polygon = resolved

        coords = list(polygon.exterior.coords)
        if coords[0] == coords[-1]:
            coords = coords[:-1]

        concave_indices = ConcaveDecomposer._find_concave_indices(coords)
        type2_indices = ConcaveDecomposer._classify_type2_batch(
            polygon, coords, concave_indices, heading_rad
        )

        for i in type2_indices:
            sub_polygons = ConcaveDecomposer._split_polygon_at_vertex(
                polygon, coords[i], heading_rad
            )
            result, success = ConcaveDecomposer._validate_and_recurse(
                polygon, sub_polygons, heading_angle_deg, depth, channel_width, min_swath
            )
            if success:
                return result

        # No further type-2 concavities → this piece is a leaf cell.
        if not polygon.is_valid:
            polygon = make_valid(polygon)
            if polygon.geom_type == 'MultiPolygon':
                return [p for p in polygon.geoms if p.area > 0.1]
            elif polygon.geom_type != 'Polygon':
                return []
        return [polygon]