import numpy as np
import math
from shapely.geometry import Polygon, LineString, Point, MultiPolygon
from shapely.ops import split
from shapely.geometry.polygon import orient
from shapely.validation import make_valid
from shapely.prepared import prep


class ConcaveDecomposer:
    """
    Implementation of Phase 2: Concavity Detection and Decomposition.
    Based on Sections 2.3 and 2.4 of the paper by Li et al. (2023).

    Optimized with:
    - PreparedGeometry for batch sweep-line intersections
    - Precomputed concave vertex detection
    - Batch Type 2 classification
    """

    # ------------------------------------------------------------------ #
    #                        Private Helpers                              #
    # ------------------------------------------------------------------ #

    @staticmethod
    def _find_interior_direction(polygon: Polygon, vx: float, vy: float,
                                  hx: float, hy: float) -> float | None:
        """
        Determine which side of the heading direction (+1 or -1)
        points into the polygon interior from the vertex (vx, vy).
        Returns +1.0, -1.0, or None if neither side is interior.
        """
        eps = max(0.5, polygon.length * 0.002)
        for sign in (+1.0, -1.0):
            test_pt = Point(vx + sign * eps * hx, vy + sign * eps * hy)
            if polygon.contains(test_pt):
                return sign
        return None

    @staticmethod
    def _extract_hit_points(intersection) -> list[Point]:
        """
        Extract Point candidates from any Shapely intersection result.
        Skips LineString/MultiLineString (collinear overlaps).
        """
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
        """
        Cast a ray from (vx, vy) in direction (dx, dy) and return
        the nearest boundary hit point, or None.
        """
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
        """
        Cast a ray from (vx, vy) in direction (dx, dy) against a single
        ring (exterior or interior) and return the nearest hit point.
        """
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
    def _validate_and_recurse(polygon: Polygon, sub_polygons: list[Polygon],
                               heading_angle_deg: float,
                               depth: int) -> tuple[list[Polygon], bool]:
        """
        Filter valid sub-polygons, check for degenerate cuts,
        and recurse. Returns (result_list, success_bool).
        """
        valid_subs = [s for s in sub_polygons if s.area > 0.1]
        if not valid_subs:
            return [], False

        # Check degeneracy
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
                ConcaveDecomposer.decompose(sub, heading_angle_deg, depth + 1)
            )
        return result, True

    # ------------------------------------------------------------------ #
    #                      Detection Methods                              #
    # ------------------------------------------------------------------ #

    @staticmethod
    def _find_concave_indices(coords: list) -> list[int]:
        """
        Return indices of all concave vertices in one pass.
        In CCW winding, a negative cross product indicates a right turn (concavity).
        """
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
        """
        Count how many segments the sweep-line produces at a given point.
        Uses PreparedGeometry for fast intersection pre-check.
        """
        sweep = LineString([
            (pt[0] - ray * heading[0], pt[1] - ray * heading[1]),
            (pt[0] + ray * heading[0], pt[1] + ray * heading[1]),
        ])
        # Fast rejection with prepared geometry's spatial index
        if not prepared_polygon.intersects(sweep):
            return 0

        inter = sweep.intersection(polygon)
        if inter.is_empty:
            return 0
        if inter.geom_type == 'LineString':
            return 1
        if inter.geom_type == 'MultiLineString':
            return len(list(inter.geoms))
        return sum(1 for g in inter.geoms if g.geom_type == 'LineString')

    @staticmethod
    def _classify_type2_batch(polygon: Polygon, coords: list,
                               concave_indices: list[int],
                               heading_rad: float) -> list[int]:
        """
        Classify all concave exterior vertices as Type 2 in batch,
        using a single PreparedGeometry for all sweep-line tests.

        A vertex is Type 2 if the number of sweep-line segments changes
        when crossing through the vertex in the advancing direction.
        """
        if not concave_indices:
            return []

        eps = 0.5
        heading = np.array([np.cos(heading_rad), np.sin(heading_rad)])
        advancing = np.array([-np.sin(heading_rad), np.cos(heading_rad)])

        # Build spatial index ONCE for all vertices
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
        """
        Batch classify hole vertices as Type 2 using shared PreparedGeometry.
        Returns indices into hole_coords that are Type 2.
        """
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

    # ------------------------------------------------------------------ #
    #                       Cutting Operations                            #
    # ------------------------------------------------------------------ #

    @staticmethod
    def _split_polygon_at_vertex(polygon: Polygon, vertex_coords: tuple,
                                  heading_rad: float) -> list[Polygon]:
        """
        Cuts the polygon with a one-sided ray from the T2 vertex inward,
        parallel to the heading direction.

        The ray originates at the concave vertex, points toward the polygon
        interior, and terminates at the first boundary hit.
        """
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

        # Check degenerate (collinear — split returned the same polygon)
        if len(polys) == 1:
            diff_holes = len(list(polys[0].interiors)) - len(list(polygon.interiors))
            if diff_holes == 0 and abs(polys[0].area - polygon.area) < 1e-6:
                polys = []

        # Collinear failure — perturb angle to force transverse crossing
        if not polys:
            angle_perturbation = 0.01  # ~0.6 degrees
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

    @staticmethod
    def _connect_hole_to_exterior(polygon: Polygon, vertex_coords: tuple,
                                   heading_rad: float) -> list[Polygon]:
        """
        Connects an interior ring (obstacle) to the exterior boundary
        by erasing a thin channel along the heading direction.

        The resulting polygon has one fewer hole (absorbed into the
        exterior as a concavity) which recursion will decompose.
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

        # Cast ray to EXTERIOR only — skip other holes so the channel
        # traverses any intermediate holes and absorbs them.
        hit_pt = ConcaveDecomposer._cast_ray_to_ring(
            polygon.exterior, vx, vy, dx, dy
        )
        if hit_pt is None:
            return [polygon]

        ext = 0.1
        p1 = Point(vx - ext * dx, vy - ext * dy)
        p2 = Point(hit_pt.x + ext * dx, hit_pt.y + ext * dy)

        channel = LineString([p1, p2]).buffer(0.01, cap_style=2)
        result = polygon.difference(channel)

        if isinstance(result, MultiPolygon):
            return list(result.geoms)
        elif isinstance(result, Polygon):
            return [result]
        return [polygon]

    # ------------------------------------------------------------------ #
    #                         Orchestrator                                #
    # ------------------------------------------------------------------ #

    @staticmethod
    def _resolve_all_holes(polygon: Polygon, heading_rad: float,
                            max_iter: int = 100) -> Polygon | list[Polygon]:
        """
        Iteratively connect all holes to the exterior boundary.
        Returns a Polygon (no holes) or list[Polygon] if the channel split it.
        """
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
                        polygon, hole_coords[idx], heading_rad
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
                   depth: int = 0) -> list[Polygon]:
        """
        Recursive main function.
        Verifies if the polygon has concavities 'Type 2' that obstruct the flight
        at the given angle. If there are any, cuts the polygon and processes the parts.

        :param polygon: Shapely Polygon (may contain holes/obstacles).
        :param heading_angle_deg: Flight heading in degrees.
        :param depth: Current recursion depth.
        :return: List of convex polygons (or safe to fly).
        """
        if depth > 25:
            return [polygon]

        heading_rad = np.radians(heading_angle_deg)

        # --- Pre-step: resolve ALL holes iteratively ---
        resolved = ConcaveDecomposer._resolve_all_holes(polygon, heading_rad)
        if isinstance(resolved, list):
            result = []
            for sub in resolved:
                sub = orient(sub, sign=1.0)
                result.extend(ConcaveDecomposer.decompose(sub, heading_angle_deg, depth + 1))
            return result
        polygon = resolved

        # --- Exterior vertices ---
        coords = list(polygon.exterior.coords)
        if coords[0] == coords[-1]:
            coords = coords[:-1]

        # Batch: find concave, then classify T2 with shared spatial index
        concave_indices = ConcaveDecomposer._find_concave_indices(coords)
        type2_indices = ConcaveDecomposer._classify_type2_batch(
            polygon, coords, concave_indices, heading_rad
        )

        # Try cuts only on T2 vertices
        for i in type2_indices:
            sub_polygons = ConcaveDecomposer._split_polygon_at_vertex(
                polygon, coords[i], heading_rad
            )
            result, success = ConcaveDecomposer._validate_and_recurse(
                polygon, sub_polygons, heading_angle_deg, depth
            )
            if success:
                return result

        # --- No Type 2 found — polygon is ready ---
        if not polygon.is_valid:
            polygon = make_valid(polygon)
            if polygon.geom_type == 'MultiPolygon':
                return [p for p in polygon.geoms if p.area > 0.1]
            elif polygon.geom_type != 'Polygon':
                return []
        return [polygon]