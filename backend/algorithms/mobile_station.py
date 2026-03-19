from shapely.geometry import Polygon, Point, LineString
from shapely.ops import substring
import numpy as np

class MobileStation:
    """
    Implementation of Drone-Truck Synergy.
    Based on the geometric constraint: P_truck on the polygon boundary.
    
    [Image of drone and truck cooperative rendezvous concept]
    """

    def __init__(self, truck_speed_mps=5.0, truck_offset_m=0.0):
        self.truck_speed = truck_speed_mps # Average truck speed
        self.truck_offset_m = truck_offset_m # Distance from route to boundary

    def get_road_boundary(self, polygon: Polygon):
        """Returns the ring of the truck route (boundary + offset)"""
        if self.truck_offset_m > 0:
            limit_poly = polygon.buffer(self.truck_offset_m, join_style=2)
            return limit_poly.exterior
        return polygon.exterior

    def calculate_rendezvous(self, polygon: Polygon, p_drone_exit: tuple, truck_start_pos: tuple, ref_route: LineString = None):
        """
        Calculates the optimal rendezvous point (R_opt) and logistics.
        If ref_route is != None, uses that LineString (open path) instead of the perimeter (ring).
        """
        if ref_route:
            # OPEN CHAIN LOGIC (Linear Route)
            boundary = ref_route
            
            # CHECK STATIC MODE
            if self.truck_speed < 0.1:
                 # Truck cannot move. Rendezvous is always at truck_start_pos.
                 r_opt = Point(truck_start_pos)
                 if not boundary.distance(r_opt) < 0.1:
                      # Snap if needed (optional)
                      pass
                      
                 return r_opt, 0.0, 0.0, [truck_start_pos]
            
            point_exit = Point(p_drone_exit)
            
            # 1. R_opt (Nearest projection on the line)
            # [Image of orthogonal projection of point onto line]
            dist_projected = boundary.project(point_exit)
            r_opt = boundary.interpolate(dist_projected)
            
            # 2. Truck Route (Linear, no turns)
            start_dist = boundary.project(Point(truck_start_pos))
            target_dist = dist_projected
            
            truck_travel_dist = abs(target_dist - start_dist)
            
            # Path geometry
            if truck_travel_dist > 0.1:
                # Substring always returns in base line order
                params = sorted([start_dist, target_dist])
                path_geom = substring(boundary, params[0], params[1])
                path_final_coords = list(path_geom.coords)
                
                # Invert if we are going "backwards" relative to line definition
                if start_dist > target_dist:
                    path_final_coords = path_final_coords[::-1]
            else:
                path_final_coords = [(r_opt.x, r_opt.y)]
                
            truck_time_s = truck_travel_dist / self.truck_speed if self.truck_speed > 0 else float('inf')
            return r_opt, truck_travel_dist, truck_time_s, path_final_coords

        # CLOSED LOOP LOGIC (Perimeter)
        # Determine the truck path boundary
        boundary = self.get_road_boundary(polygon)
        
        # CHECK STATIC MODE
        if self.truck_speed < 0.1:
                # Truck cannot move. Rendezvous is always at truck_start_pos.
                r_opt = Point(truck_start_pos)
                return r_opt, 0.0, 0.0, [truck_start_pos]
                
        # 1. Find R_opt (Orthogonal projection onto the boundary)
        point_exit = Point(p_drone_exit)
        dist_projected = boundary.project(point_exit)
        r_opt = boundary.interpolate(dist_projected)
        
        # 2. Calculate Truck Route on the perimeter
        start_dist = boundary.project(Point(truck_start_pos))
        target_dist = dist_projected 
        total_len = boundary.length
        
        # Path 1: CCW (Forward in the ring)
        if start_dist <= target_dist:
            path_ccw_geom = substring(boundary, start_dist, target_dist)
        else:
            # Wrap: start->end + 0->target
            p1 = substring(boundary, start_dist, total_len)
            p2 = substring(boundary, 0, target_dist)
            coords = list(p1.coords) + list(p2.coords)
            path_ccw_geom = LineString(coords)
            
        len_ccw = path_ccw_geom.length
        
        # Path 2: CW (Backward in the ring) -> We calculate Target->Start (CCW) and reverse it
        # [Image of clockwise vs counter-clockwise path planning on ring]
        if target_dist <= start_dist:
            path_cw_rev = substring(boundary, target_dist, start_dist)
        else:
            p1 = substring(boundary, target_dist, total_len)
            p2 = substring(boundary, 0, start_dist)
            coords = list(p1.coords) + list(p2.coords)
            path_cw_rev = LineString(coords)
            
        len_cw = path_cw_rev.length
        
        # Choose the shortest path
        if len_ccw <= len_cw:
            truck_travel_dist = len_ccw
            path_final_coords = list(path_ccw_geom.coords)
        else:
            truck_travel_dist = len_cw
            # Invert coordinates to go Start->Target
            path_final_coords = list(path_cw_rev.coords)[::-1]

        # 3. Synchronization
        truck_time_s = truck_travel_dist / self.truck_speed if self.truck_speed > 0 else float('inf')
        
        return r_opt, truck_travel_dist, truck_time_s, path_final_coords

    def check_feasibility(self, truck_time_s, drone_endurance_s):
        """
        Verifies if the truck arrives before the drone falls.
        """
        # Safety margin (e.g., 1 minute)
        margin_s = 60.0 
        return truck_time_s < (drone_endurance_s - margin_s)