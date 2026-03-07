import numpy as np
from shapely.geometry import Polygon, LineString, Point, box
from shapely.ops import split, unary_union

from shapely.geometry.polygon import orient
from shapely.validation import make_valid
import math

class ConcaveDecomposer:
    """
    Implementation of Phase 2: Concavity Detection and Decomposition.
    Based on Sections 2.3 and 2.4 of the paper by Li et al. (2023).
    """

    @staticmethod
    def decompose(polygon: Polygon, heading_angle_deg: float, depth: int = 0):
        """
        Recursive main function.
        Verifies if the polygon has concavities 'Type 2' that obstruct the flight
        at the given angle. If there are any, cuts the polygon and processes the parts.
        
        :return: List of convex polygons (or safe to fly).
        """
        if depth > 50:
            print("Max Recursion Depth Reached. Returning original polygon.")
            return [polygon]

        # Convert angle to radians for trigonometric calculations
        heading_rad = np.radians(heading_angle_deg)
        
        # 1. Get coordinates
        coords = list(polygon.exterior.coords)
        if coords[0] == coords[-1]:
            coords = coords[:-1]
        n = len(coords)
        
        # 2. Find the FIRST concave vertex that is "Type 2" (obstructive).
        # Per Li et al.: Type 2 means the sweep-line topology (number of
        # segments) changes at this vertex, forcing the drone to exit the
        # work area during a U-turn without decomposition.
        for i in range(n):
            if ConcaveDecomposer._is_concave_topology_mapping(coords, i):
                if ConcaveDecomposer._is_type_2(polygon, coords[i], heading_rad):
                    # --- CUTTING PHASE ---
                    sub_polygons = ConcaveDecomposer._split_polygon_at_vertex(
                        polygon, coords[i], heading_rad
                    )

                    valid_subs = [s for s in sub_polygons
                                  if s.area > 0.1]

                    # A cut is degenerate if it produces no large piece,
                    # OR if it just returned the exact same thing (1 piece, same topology).
                    # But 1 piece IS valid if a hole was merged to the exterior!
                    
                    if not valid_subs:
                        continue
                        
                    is_degenerate = False
                    if len(valid_subs) == 1:
                        v = valid_subs[0]
                        # Check if topology improved (fewer holes)
                        if len(list(v.interiors)) >= len(list(polygon.interiors)):
                            is_degenerate = True  # nothing changed or got worse
                        if v.area < 0.95 * polygon.area:
                            is_degenerate = True  # lost too much area
                            
                    else:
                        # For >1 pieces, at least one must be reasonably large
                        if sum(s.area for s in valid_subs) < 0.95 * polygon.area:
                            is_degenerate = True

                    if is_degenerate:
                        continue  # cut was degenerate, try next vertex

                    # Recurse on each piece using the original flight heading.
                    # Re-orient to CCW: shapely split() does not guarantee winding order,
                    # and a CW polygon would invert the concavity cross-product test.
                    result = []
                    for sub in sub_polygons:
                        sub = orient(sub, sign=1.0)  # ensure CCW
                        result.extend(
                            ConcaveDecomposer.decompose(sub, heading_angle_deg, depth + 1)
                        )
                    return result

        # 3. Check interior ring (obstacle) vertices for Type 2.
        # Connect the hole to the exterior via a thin channel, turning
        # the obstacle into exterior concavities that recursion will cut.
        for interior in polygon.interiors:
            hole_coords = list(interior.coords)
            if hole_coords[0] == hole_coords[-1]:
                hole_coords = hole_coords[:-1]

            for vertex in hole_coords:
                if ConcaveDecomposer._is_type_2(polygon, vertex, heading_rad):
                    sub_polygons = ConcaveDecomposer._connect_hole_to_exterior(
                        polygon, vertex, heading_rad
                    )

                    valid_subs = [s for s in sub_polygons if s.area > 0.1]
                    if not valid_subs:
                        continue

                    # Check for improvement: must reduce total holes
                    holes_before = len(list(polygon.interiors))
                    holes_after = sum(len(list(s.interiors)) for s in valid_subs)
                    if holes_after >= holes_before and len(valid_subs) == 1:
                        continue

                    result = []
                    for sub in valid_subs:
                        sub = orient(sub, sign=1.0)
                        result.extend(
                            ConcaveDecomposer.decompose(sub, heading_angle_deg, depth + 1)
                        )
                    return result

        # If no obstructive concavity was found, the polygon is ready
        if not polygon.is_valid:
            polygon = make_valid(polygon)
            if polygon.geom_type == 'MultiPolygon':
                return [p for p in polygon.geoms if p.area > 0.1]
            elif polygon.geom_type != 'Polygon':
                return []
        return [polygon]

    @staticmethod
    def _is_concave_topology_mapping(coords, i):
        """
        Detects if vertex i is concave using 'Topology Mapping' (Eq. 8-10).
        """
        n = len(coords)
        curr_p = np.array(coords[i])
        prev_p = np.array(coords[(i - 1) % n])
        next_p = np.array(coords[(i + 1) % n])

        # Paper Section 2.3: Projective lines L1 and L2
        # The paper defines projections based on slope. 
        # Robust simplification equivalent to the paper: Cross Product.
        # The paper uses topological mapping to mathematically demonstrate what the cross product does.
        # We implement the vector logic which is computationall stable.
        
        vec_prev = prev_p - curr_p
        vec_next = next_p - curr_p
        
        # Cross product 2D: (x1*y2 - x2*y1)
        # 
        cross_prod = vec_next[0] * vec_prev[1] - vec_next[1] * vec_prev[0]
        
        # In Shapely/GIS (CCW order), a negative cross indicates a right turn (concavity)
        # NOTE: We assume the polygon is ordered CCW (Counter-Clockwise).
        return cross_prod < -1e-3  # Tolerance increased to avoid noise in almost collinear vertices

    @staticmethod
    def _is_type_2(polygon: Polygon, vertex_coords, heading_rad: float) -> bool:
        """
        Determines if a concave vertex is Type 2 (Obstructive) per Fig. 5 and
        Section 2.4 of Li et al. (2023).

        Definitive criterion: a vertex is Type 2 if and only if the NUMBER of
        sweep-line segments that intersect the polygon CHANGES when crossing
        through the vertex in the advancing direction.

        - 1 segment  ->  1 segment : Type 1 (just changes width, no split/merge)
        - 1 segment  ->  2 segments: Type 2 (new sub-region appears / merges)

        This approach is exact and handles all edge cases, including
        axis-aligned polygons where projection-based tests give zero and fail.
        """
        eps = 0.5  # metres; large enough to avoid floating-point noise
        ray = 1e5

        heading = np.array([np.cos(heading_rad), np.sin(heading_rad)])
        advancing = np.array([-np.sin(heading_rad), np.cos(heading_rad)])

        def _n_segments(pt):
            """Count sweep-line segments through pt."""
            sweep = LineString([
                (pt[0] - ray * heading[0], pt[1] - ray * heading[1]),
                (pt[0] + ray * heading[0], pt[1] + ray * heading[1]),
            ])
            inter = sweep.intersection(polygon)
            if inter.is_empty:
                return 0
            if inter.geom_type == 'LineString':
                return 1
            if inter.geom_type == 'MultiLineString':
                return len(list(inter.geoms))
            # GeometryCollection or other
            return sum(1 for g in inter.geoms if g.geom_type == 'LineString')

        curr = np.array(vertex_coords)
        n_before = _n_segments(curr - eps * advancing)
        n_after  = _n_segments(curr + eps * advancing)

        return n_before != n_after


    @staticmethod
    def _connect_hole_to_exterior(polygon: Polygon, vertex_coords, heading_rad):
        """
        Connects an interior ring (obstacle) to the exterior boundary by
        erasing a thin channel along the heading direction.  The channel
        runs from the hole vertex to the nearest boundary hit.

        The resulting polygon has one fewer hole (absorbed into the
        exterior as a concavity) which recursion will decompose.
        """
        from shapely.geometry import MultiPolygon

        ray_len = 1e6
        vx, vy = vertex_coords[0], vertex_coords[1]
        hx = np.cos(heading_rad)
        hy = np.sin(heading_rad)

        # Direction INTO the polygon (away from hole centre)
        eps = max(0.5, polygon.length * 0.002)
        interior_sign = None
        for sign in (+1.0, -1.0):
            test_pt = Point(vx + sign * eps * hx, vy + sign * eps * hy)
            if polygon.contains(test_pt):
                interior_sign = sign
                break

        if interior_sign is None:
            return [polygon]

        dx = interior_sign * hx
        dy = interior_sign * hy

        # Cast ray to find first boundary hit
        far_pt = Point(vx + ray_len * dx, vy + ray_len * dy)
        ray_line = LineString([(vx, vy), (far_pt.x, far_pt.y)])

        boundary_lines = [polygon.exterior] + list(polygon.interiors)
        hit_pt = None
        min_dist = ray_len
        origin = Point(vx, vy)

        for ring in boundary_lines:
            inter = ray_line.intersection(ring)
            if inter.is_empty:
                continue
            if inter.geom_type == 'Point':
                candidates = [inter]
            elif inter.geom_type == 'MultiPoint':
                candidates = list(inter.geoms)
            elif inter.geom_type in ('LineString', 'MultiLineString'):
                continue
            else:
                candidates = [g for g in getattr(inter, 'geoms', [])
                              if g.geom_type == 'Point']

            for pt in candidates:
                d = pt.distance(origin)
                if d > 1e-6 and d < min_dist:
                    min_dist = d
                    hit_pt = pt

        if hit_pt is None:
            return [polygon]

        # Build a thin channel from slightly inside the hole to slightly
        # past the hit boundary.
        ext = 0.1
        p1 = Point(vx - ext * dx, vy - ext * dy)
        p2 = Point(hit_pt.x + ext * dx, hit_pt.y + ext * dy)

        channel_line = LineString([p1, p2])
        channel = channel_line.buffer(0.01, cap_style=2)  # flat cap, 1cm wide

        result = polygon.difference(channel)

        if isinstance(result, MultiPolygon):
            return list(result.geoms)
        elif isinstance(result, Polygon):
            return [result]
        else:
            return [polygon]

    @staticmethod
    def _split_polygon_at_vertex(polygon: Polygon, vertex_coords, heading_rad):
        """
        Cuts the polygon with a ONE-SIDED RAY from the T2 vertex inward,
        parallel to the heading (sweep direction).

        The ray:
          - Originates at the concave vertex (T2).
          - Points toward the polygon interior (direction determined
            by which side of the heading the interior lies on).
          - Terminates at the FIRST boundary wall it hits.

        This avoids the guillotine (bidirectional) cut that would split the
        polygon on both sides of the vertex.
        """
        ray_len = 1e6  # effectively infinite

        vx, vy = vertex_coords[0], vertex_coords[1]
        hx = np.cos(heading_rad)
        hy = np.sin(heading_rad)

        # 1. Find which direction of the heading points INTO the polygon interior.
        eps = max(0.5, polygon.length * 0.002)
        interior_sign = None
        for sign in (+1.0, -1.0):
            test_pt = Point(vx + sign * eps * hx, vy + sign * eps * hy)
            if polygon.contains(test_pt):
                interior_sign = sign
                break

        if interior_sign is None:
            # Vertex is not strictly on the boundary — degenerate, skip
            return []

        dx = interior_sign * hx
        dy = interior_sign * hy

        # 2. Cast the ray into the polygon and find the FIRST hit on the boundary.
        far_pt = Point(vx + ray_len * dx, vy + ray_len * dy)
        ray_line = LineString([(vx, vy), (far_pt.x, far_pt.y)])

        # Intersect with the exterior ring (the wall we want to stop at).
        # Also check interior rings (holes/obstacles) so the ray stops there too.
        boundary_lines = [polygon.exterior] + list(polygon.interiors)
        hit_pt = None
        min_dist = ray_len
        origin = Point(vx, vy)

        for ring in boundary_lines:
            inter = ray_line.intersection(ring)
            if inter.is_empty:
                continue
            # Collect candidate points from the intersection geometry.
            # Skip LineString results: these are collinear overlaps (ray runs
            # parallel to a boundary edge) — they are NOT transverse crossings
            # and would produce a degenerate cut that GEOS cannot split.
            if inter.geom_type == 'Point':
                candidates = [inter]
            elif inter.geom_type == 'MultiPoint':
                candidates = list(inter.geoms)
            elif inter.geom_type in ('LineString', 'MultiLineString'):
                continue  # collinear — skip entirely
            else:
                # GeometryCollection — extract only Point sub-geometries
                candidates = [g for g in getattr(inter, 'geoms', [])
                              if g.geom_type == 'Point']

            for pt in candidates:
                d = pt.distance(origin)
                if d > 1e-6 and d < min_dist:
                    min_dist = d
                    hit_pt = pt

        if hit_pt is None:
            return []

        # 3. Build the cut line and perform a topological split.
        #
        #    To ensure GEOS/Shapely actually splits the space (or connects a hole
        #    to the exterior), the line MUST STRICTLY CROSS the boundaries.
        #    Stopping exactly at `hit_pt` causes a dangle, not a split.
        epsilon = 1e-3
        
        # Origin slightly outside the polygon, destination slightly inside the obstacle
        p1 = Point(vx - epsilon * dx, vy - epsilon * dy)
        p2 = Point(hit_pt.x + epsilon * dx, hit_pt.y + epsilon * dy)
        
        def _attempt_split(cut_p2: Point):
            cut_line = LineString([p1, cut_p2])
            try:
                coll = split(polygon, cut_line)
                return [g for g in coll.geoms if isinstance(g, Polygon)]
            except Exception:
                return []
                
        polys = _attempt_split(p2)
        
        # Check if the split failed (1 piece returned with the same topology & area).
        # This happens ONLY if the cut line exactly overlaps a boundary edge 
        # (collinear degenerate case, GEOS ignores it).
        is_degenerate = False
        if len(polys) == 1:
            diff_holes = len(list(polys[0].interiors)) - len(list(polygon.interiors))
            diff_area = abs(polys[0].area - polygon.area)
            if diff_holes == 0 and diff_area < 1e-6:
                is_degenerate = True

        if is_degenerate or not polys:
            # Collinear failure. Rotate the ray infinitesimaly to force a 
            # transverse intersection across the obstacle wall.
            angle_perturbation = 0.01  # ~0.6 degrees — enough for a transverse crossing
            new_angle = heading_rad + angle_perturbation
            new_dx = interior_sign * np.cos(new_angle)
            new_dy = interior_sign * np.sin(new_angle)
            
            # The length of the original ray was dist(vx, hit_pt)
            ray_dist = math.hypot(hit_pt.x - vx, hit_pt.y - vy)
            perturbed_p2 = Point(
                vx + (ray_dist + epsilon) * new_dx, 
                vy + (ray_dist + epsilon) * new_dy
            )
            polys = _attempt_split(perturbed_p2)

        return polys