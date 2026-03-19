from shapely.geometry import Point
import numpy as np

class MissionAnalyzer:
    """
    Analyzes and compares missions (Static vs Mobile) and generates logistic plans.
    """
    
    @staticmethod
    def simulate_static_mission(segmenter_class, polygon, raw_path, drone_specs, mobile_station_class):
        """
        Simulates the same mission but with the fixed station at the starting point.
        Returns: (cycles, total_dist_air, total_time)
        
        
        """
        # 1. Define "Truck Path" as a static point (minimum buffer at the start)
        p_start = raw_path[0]
        start_point = Point(p_start[:2])
        # Small buffer to prevent shapely errors, but effectively a point
        static_region = start_point.buffer(0.1) 
        
        # 2. Instantiate a new segmenter to not pollute the original
        # Assume we can instantiate a new one with the specs
        segmenter = segmenter_class(drone_specs, mobile_station_class(truck_speed_mps=0.1)) # Irrelevant speed
        
        # 3. Run segmentation forcing the "truck_polygon" to be the static point
        cycles = segmenter.segment_path(polygon, raw_path, truck_polygon=static_region)
        
        # 4. Calculate metrics
        total_dist = 0.0
        total_time = 0.0
        
        for c in cycles:
            path = c['path']
            # Sum Euclidean distances
            d = 0
            for i in range(len(path)-1):
                d += np.linalg.norm(np.array(path[i][:2]) - np.array(path[i+1][:2]))
            total_dist += d
            
        return cycles, total_dist

    @staticmethod
    def calculate_comprehensive_metrics(cycles, polygon, drone_specs):
        """
        Calculates detailed metrics for the report:
        - Productivity (ha/h)
        - Actual Dosage (L/ha)
        - Flight vs Idle Times (min)
        """
        total_dist = 0
        spray_dist = 0
        deadhead_dist = 0
        
        # 1. Distances
        for c in cycles:
            segments = c.get('segments', [])
            if segments:
                for s in segments:
                     d = np.linalg.norm(np.array(s['p1'][:2]) - np.array(s['p2'][:2]))
                     total_dist += d
                     if s['spraying']:
                         spray_dist += d
                     else:
                         deadhead_dist += d
            else:
                # Fallback simple
                path = c.get('path', [])
                for i in range(len(path)-1):
                    total_dist += np.linalg.norm(np.array(path[i][:2]) - np.array(path[i+1][:2]))

        # 2. Area
        area_m2 = polygon.area
        area_ha = area_m2 / 10000.0
        
        # 3. Times
        # Use working speed for everything as a conservative approximation,
        # or separate if we have velocity profiles in the future.
        work_speed_ms = float(drone_specs.flight.work_speed_kmh.value) / 3.6
        flight_time_sec = total_dist / work_speed_ms
        flight_time_min = flight_time_sec / 60.0
        
        # Estimation of Reload Times
        # Assume X min per stop?
        reload_time_min = len(cycles) * 5.0 # 5 min per reload cycle?
        total_op_time_min = flight_time_min + reload_time_min
        
        # 4. Productivity
        prod_ha_hr = 0
        if total_op_time_min > 0:
            prod_ha_hr = area_ha / (total_op_time_min / 60.0)
            
        # 5. Dosage
        # Total Liters = Spray Dist (m) * Swath (m) * (L/m2 ?) No...
        # Better: Dosage = (FlowRate * SprayTime) / Area ?
        # Or simpler: Use tank * cycles? Not exact.
        # Use the rate configured in the segmenter if possible or infer it.
        # Dosage = (Total Volume Applied) / (Crop Area)
        
        # Applied Volume:
        # Sum of (Spray Dist / Speed) * FlowRate
        # 
        flow_l_min = float(drone_specs.spray.max_flow_l_min.value) # Assume max flow? Or regulated?
        # In theory the segmenter adjusts speed/flow to meet target.
        # Assume we apply what the area requires.
        # But we can calculate it "bottom-up":
        spray_time_min = (spray_dist / work_speed_ms) / 60.0
        total_vol_l = spray_time_min * flow_l_min # This assumes constant max flow.
        
        real_dosage = 0
        if area_ha > 0:
            real_dosage = total_vol_l / area_ha
            
        return {
            "area_ha": area_ha,
            "flight_time_min": flight_time_min,
            "total_op_time_min": total_op_time_min,
            "productivity_ha_hr": prod_ha_hr,
            "real_dosage_l_ha": real_dosage,
            "spray_dist_km": spray_dist / 1000.0,
            "dead_dist_km": deadhead_dist / 1000.0,
            "efficiency_ratio": (spray_dist / total_dist * 100) if total_dist else 0
        }

    @staticmethod
    def compare_missions(mobile_cycles, static_cycles):
        """
        Generates comparative metrics.
        """
        def get_metrics(cycles):
            total_dist = 0
            flight_time = 0 
            deadhead_dist = 0
            spray_dist = 0
            truck_dist = 0
            
            for c in cycles:
                # Drone Path
                path = c['path']
                segments = c.get('segments', [])
                if segments:
                    for s in segments:
                         d = np.linalg.norm(np.array(s['p1'][:2]) - np.array(s['p2'][:2]))
                         total_dist += d
                         if s['spraying']:
                             spray_dist += d
                         else:
                             deadhead_dist += d
                
                # Truck Path
                t_path = c.get('truck_path_coords', [])
                if t_path and len(t_path) > 1:
                    for i in range(len(t_path)-1):
                        p1 = np.array(t_path[i][:2])
                        p2 = np.array(t_path[i+1][:2])
                        truck_dist += np.linalg.norm(p1 - p2)

            return total_dist, deadhead_dist, spray_dist, truck_dist

        m_total, m_dead, m_spray, m_truck = get_metrics(mobile_cycles)
        s_total, s_dead, s_spray, s_truck = get_metrics(static_cycles) # s_truck is usually 0
        
        # Savings
        dead_savings_km = (s_dead - m_dead) / 1000.0
        eff_improvement = 0.0
        if s_total > 0:
             # Efficiency = Spray / Total
             eff_mobile = m_spray / m_total if m_total else 0
             eff_static = s_spray / s_total if s_total else 0
             eff_improvement = (eff_mobile - eff_static) * 100.0
             
        return {
            "mobile_dead_km": m_dead / 1000.0,
            "static_dead_km": s_dead / 1000.0,
            "mobile_truck_km": m_truck / 1000.0,
            "static_truck_km": s_truck / 1000.0,
            "savings_km": dead_savings_km, # Drone savings
            "efficiency_gain_pct": eff_improvement,
            "efficiency_static_pct": (s_spray / s_total * 100) if s_total else 0,
            "efficiency_mobile_pct": (m_spray / m_total * 100) if m_total else 0
        }

    @staticmethod
    def plan_logistics(mobile_cycles, drone_specs):
        """
        Generates the resource plan.
        """
        tank_l = float(drone_specs.spray.tank_l.value)
        
        # Battery estimation
        # Total flight time vs Charge time
        # 
        # This is complex, let's use a simple heuristic proposed by the user:
        # "Charge Rotation". We need to know how long a flight lasts and how long it takes to charge.
        flight_time_min = 15.0 # Default
        charge_time_min = 30.0 # Default
        
        if drone_specs.flight.flight_time_min:
             flight_time_min = float(drone_specs.flight.flight_time_min['hover_loaded'].value) # Conservative
        
        if drone_specs.battery and drone_specs.battery.charge_time_min:
             charge_time_min = float(drone_specs.battery.charge_time_min.value)
             
        # Ratio: If I fly 10 min and charge 30 min, I need 3 batteries charging while I fly 1.
        # Total packs = 1 (flying) + ceil(Charge / Flight)
        packs_needed = 1 + int(np.ceil(charge_time_min / flight_time_min))
        
        # Stops table
        stops = []
        total_liter_mix = 0.0
        
        # Work speed for calculations
        work_speed_ms = float(drone_specs.flight.work_speed_kmh.value) / 3.6
        flow_l_min = float(drone_specs.spray.max_flow_l_min.value)
        
        for i, cycle in enumerate(mobile_cycles):
            stop_type = "Start" if i == 0 else f"Stop {i}"
            
            # --- PRECISE CONSUMPTION CALCULATION ---
            # Sum segment distance with spraying=True
            segments = cycle.get('segments', [])
            spray_dist_m = 0
            if segments:
                for s in segments:
                    if s['spraying']:
                        d = np.linalg.norm(np.array(s['p1'][:2]) - np.array(s['p2'][:2]))
                        spray_dist_m += d
            else:
                # Fallback: assume the whole path is spray (worst case)
                path = cycle['path']
                for j in range(len(path)-1):
                    spray_dist_m += np.linalg.norm(np.array(path[j][:2]) - np.array(path[j+1][:2]))

            # Time spraying (min)
            spray_time_min = (spray_dist_m / work_speed_ms) / 60.0
            
            # Liters consumed
            # Assuming constant flow:
            liters_consumed = spray_time_min * flow_l_min
            
            # Clamp to tank size (just in case)
            if liters_consumed > tank_l: liters_consumed = tank_l
            
            total_liter_mix += liters_consumed
            
            # Generate actionable description
            # "Bat. Swap + Refill 15.4L"
            action_desc = f"Bat. Swap + Refill {liters_consumed:.1f}L"
            if i == 0:
                action_desc = f"Fill Tank ({liters_consumed:.1f}L)"
            
            stops.append({
                "name": stop_type,
                "action": action_desc,
                "notes": f"Coverage: {spray_dist_m:.0f}m" 
            })
            
        return {
            "battery_packs": packs_needed,
            "total_mix_l": total_liter_mix, 
            "stops": stops
        }