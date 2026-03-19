from shapely.geometry import Point, LineString
import numpy as np
from .mobile_station import MobileStation

class MissionSegmenter:
    """
    Cuts a continuous route into operable segments based on drone physics
    (Battery, Liquid) and truck logistics.
    
    
    """
    
    def __init__(self, drone_specs, mobile_station, target_rate_l_ha=20.0, work_speed_kmh=20.0, swath_width=None):
        self.specs = drone_specs
        self.station = mobile_station
        self.rate_l_ha = target_rate_l_ha
        self.speed_kmh = float(drone_specs.flight.work_speed_kmh.value) # Override with spec or param? 
        # Usually specs define MAX, but mission defines OPERATING. Lets use param or default to spec.
        if work_speed_kmh:
             self.speed_kmh = work_speed_kmh
        
        self.speed_ms = self.speed_kmh / 3.6
        
        # Swath Width: Use parameter if provided, otherwise fallback to drone specs
        if swath_width is not None:
            self.swath_width = swath_width
        elif self.specs.spray and self.specs.spray.swath_m:
            self.swath_width = (float(self.specs.spray.swath_m[0].value) + float(self.specs.spray.swath_m[1].value)) / 2
        else:
            self.swath_width = 5.0  # Default fallback
             
        # Physics Constants
        self.liters_per_meter = (self.rate_l_ha * self.swath_width) / 10000.0
        
        # Tank Capacity
        # 
        self.tank_capacity = 0.0
        if self.specs.spray and self.specs.spray.tank_l:
            self.tank_capacity = float(self.specs.spray.tank_l.value)

        # Battery / Endurance (Simplified to Time for now, could be Energy)
        # Using Hover Loaded as worst case conservative estimate for Work Time
        # Real work consumption is usually between Hover Empty and Hover Loaded.
        self.max_endurance_min = 15.0 # Default
        if self.specs.flight and self.specs.flight.flight_time_min:
             self.max_endurance_min = float(self.specs.flight.flight_time_min['hover_loaded'].value)
        self.max_endurance_s = self.max_endurance_min * 60.0


    def _is_spraying(self, p1, p2, polygon):
        """Determines if segment is spraying (inside field) or transit (outside/boundary)."""
        line = LineString([p1[:2], p2[:2]])
        mid = line.interpolate(0.5, normalized=True)
        
        # Check 1: If on Boundary -> TRANSIT (No Spray)
        # This handles obstacle avoidance paths and headland travels
        if polygon.boundary.distance(mid) < 0.1: # 10cm tolerance
            return False
            
        # Check 2: If strictly inside -> SPRAY
        # buffer(1e-9) is used to handle float precision for points *just* inside
        # but combined with boundary check above, true interior points are safe.
        return polygon.contains(mid)



    def segment_path(self, polygon, raw_path, truck_polygon=None, start_point=None, truck_route_line=None):
        """
        Segments the path with Smart Nozzle logic.
        """
        cycles = []
        current_cycle_points = []
        current_cycle_segments = [] # List of {'p1':, 'p2':, 'spraying': bool}
        
        ref_polygon_truck = truck_polygon if truck_polygon else polygon
        
        # Current state
        current_liquid = self.tank_capacity
        current_time_air = 0.0
        
        # Initial Truck Pos
        # If start_point (Home/Depot) is provided, truck starts there (projected to road).
        # Otherwise, truck starts at projection of first path point.
        init_pos = start_point if start_point else raw_path[0]
        r_start, _, _, _ = self.station.calculate_rendezvous(ref_polygon_truck, init_pos[:2], init_pos[:2], ref_route=truck_route_line) 
        truck_pos = (r_start.x, r_start.y)
        
        # Start Cycle Logic
        # Commute 1: Truck -> First Point (DEADHEADING)
        dist_commute_in = np.linalg.norm(np.array(truck_pos) - np.array(raw_path[0][:2]))
        time_commute_in = dist_commute_in / self.speed_ms
        current_time_air += time_commute_in
        # No liquid for commute
        
        last_point_added = truck_pos
        
        i = 0
        while i < len(raw_path) - 1:
            p1 = raw_path[i]
            p2 = raw_path[i+1]
            
            # 1. Analyze Segment (Spray vs Deadhead)
            # 
            is_spray = self._is_spraying(p1, p2, polygon)
            
            dist_step = np.linalg.norm(np.array(p1[:2]) - np.array(p2[:2]))
            time_step = dist_step / self.speed_ms
            
            # Liquid only if Spraying
            liq_step = (dist_step * self.liters_per_meter) if is_spray else 0.0
            
            # 2. Predict Return Cost from P2
            r_opt_p2, _, _, _ = self.station.calculate_rendezvous(ref_polygon_truck, p2[:2], truck_pos[:2], ref_route=truck_route_line)
            dist_return = np.linalg.norm(np.array(p2[:2]) - np.array([r_opt_p2.x, r_opt_p2.y]))
            time_return = dist_return / (self.speed_ms * 1.5)
            
            full_cycle_time = current_time_air + time_step + time_return + 120.0 # Safety
            full_cycle_liq = current_liquid - liq_step # Return no gasta liq
            
            CAN_DO = (full_cycle_time <= self.max_endurance_s) and (full_cycle_liq >= 0)
            
            if CAN_DO:
                # Add segment
                seg = {'p1': p1, 'p2': p2, 'spraying': is_spray}
                current_cycle_segments.append(seg)
                current_cycle_points.append(p1) # Only store starts, append end later
                
                current_liquid -= liq_step
                current_time_air += time_step
                last_point_added = p2
                i += 1
            else:
                # CUT CYCLE at P1
                # Finalize current cycle
                current_cycle_points.append(p1) # Close loop at P1
                
                # Calculate return stats
                r_opt_p1, dist_truck, _, truck_path_list = self.station.calculate_rendezvous(ref_polygon_truck, p1[:2], truck_pos[:2], ref_route=truck_route_line)
                r_point = (r_opt_p1.x, r_opt_p1.y)
                
                # Add Return Segment (DEADHEADING)
                commute_return = {'p1': p1, 'p2': r_point, 'spraying': False}
                current_cycle_segments.append(commute_return)
                
                # Add Initial Commute (Prepend) - wait, we tracked cost but didn't store segment object?
                # We need to reconstruct the FULL list of segments for the GUI.
                # The "current_cycle_segments" currently starts from P_start.
                # We need to prepend [Truck -> P_cycle_start].
                
                # Find P_cycle_start
                if current_cycle_segments:
                    p_cyc_start = current_cycle_segments[0]['p1']
                    commute_in = {'p1': truck_pos, 'p2': p_cyc_start, 'spraying': False}
                    current_cycle_segments.insert(0, commute_in)
                    
                    # Full Path Points
                    full_path = [truck_pos] + current_cycle_points + [r_point]
                else:
                    # Edge case: Can't even do first segment?
                    full_path = []

                # Save Cycle
                cycles.append({
                    "type": "work",
                    "path": full_path,
                    "segments": current_cycle_segments,
                    "visual_groups": self._compress_segments(current_cycle_segments), # NEW: Visual Optimization
                    "swath_width": self.swath_width,
                    "truck_start": truck_pos,
                    "truck_end": r_point,
                    "truck_dist": dist_truck,
                    "truck_path_coords": truck_path_list
                })
                
                # RESET & SETUP NEW CYCLE
                truck_pos = r_point
                current_liquid = self.tank_capacity
                current_time_air = 0.0
                current_cycle_segments = []
                current_cycle_points = []
                
                # Cost of entering the new cycle: Truck -> P1 (where we left off)
                dist_commute_in = np.linalg.norm(np.array(truck_pos) - np.array(p1[:2]))
                time_commute_in = dist_commute_in / self.speed_ms
                current_time_air += time_commute_in
                
                # Do not increment i, retry P1->P2 in the new cycle
                
        # Final Cycle
        if current_cycle_segments:
             p_last = current_cycle_segments[-1]['p2']
             # Calculate R_FINAL using the custom route if available
             r_end, dist_truck_final, _, truck_path_list_final = self.station.calculate_rendezvous(ref_polygon_truck, p_last[:2], truck_pos[:2], ref_route=truck_route_line)
             r_end_point = (r_end.x, r_end.y)
             
             # Return Segment (Land at R_FINAL)
             commute_return = {'p1': p_last, 'p2': r_end_point, 'spraying': False}
             current_cycle_segments.append(commute_return)
             
             # Prepend Start Commute
             p_cyc_start = current_cycle_segments[0]['p1']
             commute_in = {'p1': truck_pos, 'p2': p_cyc_start, 'spraying': False}
             current_cycle_segments.insert(0, commute_in)
             
             # Points
             # Note: current_cycle_points already tracked spray points. 
             # We just need to ensure the full path reflects the return.
             # FIX: Include the end point of the last segment (p_last) to avoid cutting the corner
             full_path = [truck_pos] + current_cycle_points
             if p_last != full_path[-1]: 
                 full_path.append(p_last)
             full_path.append(r_end_point)
             
             cycles.append({
                    "type": "work",
                    "path": full_path,
                    "segments": current_cycle_segments,
                    "visual_groups": self._compress_segments(current_cycle_segments), # NEW: Visual Optimization
                    "swath_width": self.swath_width,
                    "truck_start": truck_pos,
                    "truck_end": r_end_point, 
                    "truck_dist": dist_truck_final,
                    "truck_path_coords": truck_path_list_final
             })
             
        return cycles

    def _compress_segments(self, segments):
        """
        Compresses adjacent segments of same type into continuous visual groups.
        
        """
        if not segments: return []
        
        groups = []
        current_path = [segments[0]['p1'], segments[0]['p2']]
        current_type = segments[0]['spraying']
        
        for i in range(1, len(segments)):
            s = segments[i]
            # Check continuity and type
            # Assuming p1 == prev_p2 (continuity)
            if s['spraying'] == current_type:
                current_path.append(s['p2'])
            else:
                # Flush
                groups.append({
                    'path': current_path,
                    'is_spraying': current_type
                })
                # Start new
                current_path = [s['p1'], s['p2']]
                current_type = s['spraying']
                
        # Flush last
        groups.append({
            'path': current_path,
            'is_spraying': current_type
        })
        return groups