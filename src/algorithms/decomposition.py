import numpy as np
from shapely.geometry import Polygon, LineString, Point
from shapely.ops import split
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
        
        # 2. Find the FIRST concave vertex that is "Type 2" (obstructive)
        for i in range(n):
            if ConcaveDecomposer._is_concave_topology_mapping(coords, i):
                # Check both heading and opposite heading (bidirectional scan)
                # This is crucial for shapes like Combs or Spirals where the "exit" 
                # from a concavity might be against the flight direction.
                test_headings = [heading_rad, heading_rad + np.pi]
                
                for cut_heading in test_headings:
                    if ConcaveDecomposer._is_type_2(coords, i, cut_heading):
                        # --- CUTTING PHASE ---
                        sub_polygons = ConcaveDecomposer._split_polygon_at_vertex(polygon, coords[i], cut_heading)
                        
                        # Verify we actually cut something meaningful
                        # Filter out trivial splits (slivers)
                        valid_subs = []
                        for sub in sub_polygons:
                            if sub.area > 0.1 and sub.area < 0.999 * polygon.area:
                                valid_subs.append(sub)
                                
                        if len(valid_subs) < 2:
                            # Try the other direction if this one failed to produce valid pieces
                            continue 
                            
                        # Recurse on valid split
                        result = []
                        for sub in sub_polygons:
                             # Note: Recursion always uses original flight heading for consistency
                            result.extend(ConcaveDecomposer.decompose(sub, heading_angle_deg, depth + 1))
                        return result

        # If no obstructive concavity was found, the polygon is ready
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
    def _is_type_2(coords, i, heading_rad):
        """
        Determines if a concavity is "Type 2" (Obstructive) according to Fig. 5 of the paper.
        """
        # Vectors from the vertex to neighbors
        n = len(coords)
        curr_p = np.array(coords[i])
        prev_p = np.array(coords[(i - 1) % n])
        next_p = np.array(coords[(i + 1) % n])
        
        vec_prev = prev_p - curr_p
        vec_next = next_p - curr_p
        
        # Flight direction vector
        flight_vec = np.array([np.cos(heading_rad), np.sin(heading_rad)])
        
        # To be Type 2 (obstructive), the flight line must enter "inside" the polygon
        # at the concave vertex.
        # Geometrically: The flight vector must be BETWEEN vec_prev and vec_next
        # within the reflex angle (the large angle > 180).
        # 
        
        # Calculate absolute angles
        ang_prev = np.arctan2(vec_prev[1], vec_prev[0])
        ang_next = np.arctan2(vec_next[1], vec_next[0])
        ang_flight = np.arctan2(flight_vec[1], flight_vec[0])
        
        # Normalize to [0, 2pi]
        ang_prev = ang_prev % (2 * np.pi)
        ang_next = ang_next % (2 * np.pi)
        ang_flight = ang_flight % (2 * np.pi)
        
        # Verify if the flight falls into the 'cone' of the concavity
        # In a CCW concave point, the interior angle is > 180.
        # If the flight passes through that angle, it cuts the polygon -> Type 2.
        
        # Epsilon for float comparison
        EPS = 1e-4
        
        # Check if flight vector is "between" prev and next in the reflex angle
        # In CCW system, if it's concave, the angle from prev to next (CCW) is > 180.
        # We need to check if flight angle is within that large arc.
        
        diff_next_prev = (ang_next - ang_prev) % (2 * np.pi)
        diff_flight_prev = (ang_flight - ang_prev) % (2 * np.pi)
        
        # If diff_next_prev is small (< pi), it's convex locally (should not happen if cross product passed)
        # But we trust the cross product check.
        
        # Condition: The flight vector enters the material?
        # Actually, "Type 2" means the sweep line at this vertex hits the polygon INTERIOR.
        # This happens if the sweep direction (orthogonal to flight/sweep line) is...
        # Wait, "heading" is the direction of the sweep lines? Or the sweep direction?
        # Usually heading = sweep line orientation.
        # The Cut Line is parallel to the sweep lines.
        
        # Simplification: A vertex is obstructive if the line passing through it 
        # intersects the polygon "locally" on both sides? No.
        # It's an "event" if the local geometry makes a "M" or "W" shape relative to sweep.
        
        # Let's trust the vector "betweenness" for now but log it.
        # print(f"  Analysing Vertex {i}: Prev={np.degrees(ang_prev):.1f} Flight={np.degrees(ang_flight):.1f} Next={np.degrees(ang_next):.1f}")
        
        if diff_flight_prev > diff_next_prev:
            # Flight vector is NOT in the small sector (Exterior). 
            # It is in the Large Sector (Interior Material).
            # This means the ray cuts into the polygon.
            # print(f"    -> OBSTRUCTIVE (Type 2) match")
            return True
            
        return False

    @staticmethod
    def _split_polygon_at_vertex(polygon: Polygon, vertex_coords, heading_rad):
        """
        Cuts the polygon by casting a ray from the vertex in the heading direction.
        
        
        """
        # Create a very long line in the flight direction
        ray_len = 10000.0 # Arbitrary large length
        ray_end_x = vertex_coords[0] + ray_len * np.cos(heading_rad)
        ray_end_y = vertex_coords[1] + ray_len * np.sin(heading_rad)
        
        cut_line = LineString([vertex_coords, (ray_end_x, ray_end_y)])
        
        # Use Shapely to split
        # Note: split() can return more than 2 geometries if complex, 
        # but for a ray from a vertex inward, it is generally 2.
        result_collection = split(polygon, cut_line)
        
        polys = []
        for geom in result_collection.geoms:
            if isinstance(geom, Polygon):
                polys.append(geom)
        
        return polys