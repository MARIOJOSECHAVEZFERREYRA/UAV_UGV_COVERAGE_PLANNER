import numpy as np
from shapely.geometry import Polygon, LineString, MultiLineString, Point
from shapely import affinity
from typing import List, Tuple

class BoustrophedonPlanner:
    """
    Implementation of Phase 3: Boustrophedon (Zig-Zag) Path Generation.
    Adapted to return fitness metrics according to Li et al. (2023).
    
    
    """

    def __init__(self, spray_width: float = 5.0):
        """
        :param spray_width: Effective spray width (d) in meters.
        
        """
        self.spray_width = spray_width

    def generate_path(self, polygon: Polygon, angle_deg: float) -> Tuple[List[tuple], float, float]:
        """
        Generates a coverage path for a given angle and calculates metrics.
        
        :param polygon: Work zone polygon (must be convex or a sub-zone).
        :param angle_deg: Sweep angle (heading) in degrees.
        :return: (waypoints, flight_distance_l, coverage_area_S_prime)
        """
        # 1. Rotate the polygon to align the sweep with the horizontal X-axis
        # We use the centroid to rotate and then un-rotate without losing position
        # 
        centroid = polygon.centroid
        rotated_poly = affinity.rotate(polygon, -angle_deg, origin=centroid)
        
        min_x, min_y, max_x, max_y = rotated_poly.bounds
        
        # Generate sweep lines
        lines = []
        y_current = min_y + (self.spray_width / 2)
        direction = True # True = Left -> Right
        
        # Internal metrics (in the rotated system)
        total_spray_length = 0.0
        
        while y_current < max_y:
            # Infinite sweep line
            sweepline = LineString([(min_x - 1000, y_current), (max_x + 1000, y_current)])
            intersection = sweepline.intersection(rotated_poly)
            
            if not intersection.is_empty:
                # Handle complex geometries (MultiLineString)
                if isinstance(intersection, MultiLineString):
                    segs = list(intersection.geoms)
                else:
                    segs = [intersection]
                
                # Sort segments by X coordinate (always left to right first)
                segs.sort(key=lambda s: s.coords[0][0])
                
                # If direction is Right -> Left (False), we should process segments Right -> Left
                if not direction:
                    segs.reverse()

                for seg in segs:
                    coords = list(seg.coords)
                    
                    # Calculate spray length (for S')
                    # According to Eq. 13: S' = Sum(length * d)
                    seg_len = Point(coords[0]).distance(Point(coords[-1]))
                    total_spray_length += seg_len
                    
                    # Implement Zig-Zag (reverse direction if needed)
                    if not direction:
                        coords.reverse()
                    
                    lines.append(coords)
            
            y_current += self.spray_width
            direction = not direction # Change direction for the next line

        # 2. Build Continuous Path (Join segments)
        # This is vital to calculate 'l' (actual flight distance including turns)
        continuous_path_rotated = []
        if not lines:
            return [], 0.0, 0.0

        # Current Y tracking to detect "same line" (holes)
        # segments in 'lines' are [(x_start, y), (x_end, y)] (implied, could be zig-zag reversed)
        
        # We need to access the 'y' of the segments.
        # Note: 'lines' is a list of coordinate lists.
        # Let's rebuild the loop to be smarter about holes.
        
        for i in range(len(lines)):
            segment = lines[i]
            
            # If it is not the first segment, add connection from the previous one
            if i > 0:
                prev_segment = lines[i-1]
                prev_end = prev_segment[-1]
                curr_start = segment[0]
                
                # Check if we are on the same scanline (approximate Y equality)
                # Note: rotated polygon means Y is the scan direction.
                # All points in a segment ideally have same Y, but due to floating point and rotation...
                # Actually, in 'lines', we generated them with strict Y.
                # Let's check Y difference.
                y_diff = abs(prev_end[1] - curr_start[1])
                
                if y_diff < 1e-4:
                    # Gaps on the same scanline -> Obstacle!
                    # We need to walk around the obstacle (hole)
                    safe_path = self._get_safe_boundary_path(prev_end, curr_start, rotated_poly)
                    # safe_path includes start and end points approx.
                    # We append it to the path.
                    # Note: safe_path[0] should be close to prev_end, safe_path[-1] close to curr_start.
                    continuous_path_rotated.extend(safe_path)
                else:
                    # Standard u-turn or connection to next line
                    # Direct connection
                    continuous_path_rotated.extend([prev_end, curr_start]) # Just points? No, just the gap?
                    # continuous_path_rotated already has prev_end from previous iteration? 
                    # No, we extend(segment).
                    # Wait, logic above was:
                    # if i > 0:
                    #    continuous_path_rotated.extend([curr_start]) # ERROR in previous code?
                    # The previous code was:
                    # if i > 0:
                    #    prev_end = lines[i-1][-1]
                    #    curr_start = segment[0]
                    #    (implicit direct line by just adding the next points?)
                    #    No/Yes. If we just add 'segment', it implicitly draws a line from last point to first of new segment.
                    pass
            
            # Add the current segment points
            # If we added a safe_path, it effectively bridged prev_end to curr_start.
            # So we just add the segment points. 
            # BUT: duplicates.
            # If safe_path was added: its last point is curr_start.
            # segment[0] is curr_start.
            # avoid duplication
            if i > 0 and continuous_path_rotated and continuous_path_rotated[-1] == segment[0]:
                continuous_path_rotated.extend(segment[1:])
            else:
                continuous_path_rotated.extend(segment)

        # 3. Un-rotate the complete path to return to GPS/Real coordinates
        final_waypoints = []
        flight_distance_l = 0.0
        
        # Convert list of points to LineString to rotate it all at once (more efficient)
        if len(continuous_path_rotated) > 1:
            path_line = LineString(continuous_path_rotated)
            restored_path = affinity.rotate(path_line, angle_deg, origin=centroid)
            
            # Extract coordinates
            final_waypoints = list(restored_path.coords)
            
            # Calculate 'l' (Total Flight Distance) - Eq. 11
            # flight_distance_l = restored_path.length 
            # (Shapely calculates Euclidean geodesic length correctly)
            flight_distance_l = restored_path.length
            
        elif len(continuous_path_rotated) == 1:
            # Edge case: a single point
            p = Point(continuous_path_rotated[0])
            restored_p = affinity.rotate(p, angle_deg, origin=centroid)
            final_waypoints = [restored_p.coords[0]]
            flight_distance_l = 0.0

        # 4. Calculate S' (Estimated Coverage Area) - Eq. 13
        # S' = Total spray line length * Spray width
        coverage_area_s_prime = total_spray_length * self.spray_width

        return final_waypoints, flight_distance_l, coverage_area_s_prime

    def _get_safe_boundary_path(self, start_pt: Tuple[float, float], end_pt: Tuple[float, float], polygon: Polygon) -> List[tuple]:
        """
        Finds the shortest path along the obstacle boundary between two points.
        Assumes start_pt and end_pt are on the boundary of one of the holes.
        """
        start_p = Point(start_pt)
        end_p = Point(end_pt)
        
        # 1. Identify which hole (interior ring) contains these points
        target_ring = None
        
        # Check all interiors (holes)
        for interior in polygon.interiors:
            # Check distance to ring to find the correct one (handling float errors)
            if interior.distance(start_p) < 1e-3 and interior.distance(end_p) < 1e-3:
                target_ring = interior
                break
        
        if target_ring is None:
            # Fallback: Check exterior if not found on holes (should not happen for internal obstacles)
            if polygon.exterior.distance(start_p) < 1e-3 and polygon.exterior.distance(end_p) < 1e-3:
                target_ring = polygon.exterior
            else:
                # If still not found, return direct line (safe fallback)
                return [start_pt, end_pt]

        # 2. Extract coordinates of the ring
        coords = list(target_ring.coords)
        # Ensure it's closed (first == last)
        if coords[0] != coords[-1]:
            coords.append(coords[0])
            
        # 3. Find indices of start and end points on the ring
        # Since the exact points might not be vertices, we need to project them to the ring
        # However, for simplicity in this MVP, we assume they are on the segments.
        # We project them to find their position "distance along line" (project returns distance from start)
        
        d_start = target_ring.project(start_p)
        d_end = target_ring.project(end_p)
        
        if d_start == d_end:
            return [start_pt, end_pt]

        # 4. Get two paths (Clockwise and Counter-Clockwise)
        # Shapely LinearRing is directed.
        # Path 1: start -> end (forward along the ring)
        # Path 2: end -> start (reverse along the ring, effectively the other way)
        
        total_length = target_ring.length
        
        # Ensure d_start < d_end for easier calculation, swap if needed?
        # No, order matters for direction.
        
        # Substring is tricky directly. easier to use cut logic.
        # But shapely doesn't have a direct "substring" for rings that wraps around.
        
        if d_start < d_end:
            len_forward = d_end - d_start
            len_backward = total_length - len_forward
        else:
            len_backward = d_start - d_end
            len_forward = total_length - len_backward
            
        # Choose shorter path
        # Note: Ideally we should check if the path collides with anything else, 
        # but since it's a hole boundary, both are "safe" relative to that hole.
        # We pick the shorter one.
        
        # We need to construct the actual points.
        # We can use shapely.ops.substring logic manually or simple interpolation?
        # Let's use interpolation logic for robustness.
        
        path_points = []
        
        # Decide direction
        forward_is_shorter = len_forward <= len_backward
        
        if forward_is_shorter:
            # Walk forward from d_start to d_end
            if d_start < d_end:
                 # Simple substring
                 segment = self._get_substring(target_ring, d_start, d_end)
            else:
                 # Wrap around: start -> total -> 0 -> end
                 seg1 = self._get_substring(target_ring, d_start, total_length)
                 seg2 = self._get_substring(target_ring, 0.0, d_end)
                 # Merge (removing duplicate point at join)
                 coords1 = list(seg1.coords)
                 coords2 = list(seg2.coords)
                 segment = LineString(coords1 + coords2[1:])
        else:
            # Walk "backward" (which is forward from end to start, then reversed)
            # Shortest path is the "other" way.
            # So effectively we go Start -> ... -> End in the longer direction? 
            # NO, we want to go Start -> End physically. 
            # If "forward" (ring direction) is long, we go "backward" (against ring direction).
            
            # To get points for backward walk:
            # We get the forward path from End -> Start, then reverse its coordinates.
            if d_end < d_start:
                # Simple substring end -> start
                seg_fwd = self._get_substring(target_ring, d_end, d_start)
            else:
                # Wrap: end -> total -> 0 -> start
                seg1 = self._get_substring(target_ring, d_end, total_length)
                seg2 = self._get_substring(target_ring, 0.0, d_start)
                coords1 = list(seg1.coords)
                coords2 = list(seg2.coords)
                seg_fwd = LineString(coords1 + coords2[1:])
            
            # Now reverse this geometry to go Start -> End
            coords = list(seg_fwd.coords)
            coords.reverse()
            segment = LineString(coords)
            
        return list(segment.coords)

    def _get_substring(self, line: LineString, start_dist: float, end_dist: float) -> LineString:
        # Helper to extract substring from LineString based on distance
        # Based on shapely.ops.substring but simplified/embedded to avoid version issues
        from shapely.ops import substring
        return substring(line, start_dist, end_dist)