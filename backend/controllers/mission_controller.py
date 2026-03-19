from datetime import datetime
from shapely.geometry import Polygon, LineString, MultiPolygon, Point
from ..algorithms.strategy import StrategyFactory
from ..algorithms.margin import MarginReducer
from ..algorithms.segmentation import MissionSegmenter
from ..algorithms.mobile_station import MobileStation
from ..algorithms.analysis import MissionAnalyzer
from ..data import DroneDB

class MissionController:
    """
    Controller responsible for orchestrating the mission planning process.
    Handles geometry validation, spec management, optimization, and segmentation.
    Uses Strategy Pattern for optimization logic.
    """
    
    def __init__(self):
        self.last_result = None

    def run_mission_planning(self, polygon_points, drone_name, overrides, 
                             truck_route_points=None, truck_offset=0.0, 
                             use_mobile_station=True, strategy_name="genetic",
                             precalculated_path=None, obstacle_polygons=None):
        """
        Executes the full mission planning workflow.
        
        Args:
            polygon_points (list): List of (x, y) tuples for the field boundary.
            drone_name (str): Name of the drone model.
            overrides (dict): Dictionary of UI overrides (swath, tank, speed, etc.).
            truck_route_points (list, optional): List of (x, y) for manual truck route.
            truck_offset (float): Offset for truck route snapping.
            use_mobile_station (bool): Whether to calculate mobile station logistics.
            strategy_name (str): optimization strategy ("genetic" or "simple").
            precalculated_path (LineString, optional): Existing path to reuse (skips optimization).
            obstacle_polygons (list, optional): List of list of (x, y) tuples for holes.
            
        Returns:
            dict: Mission results containing geometry, cycles, metrics, and compatibility info.
        """
        
        # 1. Geometry Validation
        if len(polygon_points) < 3:
            raise ValueError("Polygon must have at least 3 points.")
            
        # Construct Polygon with Holes (if any)
        holes = obstacle_polygons if obstacle_polygons else []
        polygon = Polygon(shell=polygon_points, holes=holes)
        
        if not polygon.is_valid:
            polygon = polygon.buffer(0)
            
        # Sanitize Polygon (Fix side location conflicts & micro-segments)
        try:
            if not polygon.is_valid:
                cleaned = polygon.buffer(0)
                if cleaned.geom_type == 'MultiPolygon':
                    # Keep largest area (Filter out noise/islands)
                    cleaned = max(cleaned.geoms, key=lambda p: p.area)
                polygon = cleaned
            
            # Simplify to remove micro-segments (<1cm) which cause topology exceptions
            polygon = polygon.simplify(0.01, preserve_topology=True)
            
            # Final check
            if not polygon.is_valid:
                 polygon = polygon.buffer(0)
        except Exception as e:
            print(f"Warning: Error sanitizing polygon: {e}")

        # 2. Spec Management (Overrides)
        import copy
        specs = copy.deepcopy(DroneDB.get_specs(drone_name))
        
        # Apply Overrides
        if 'tank' in overrides and specs.spray:
            specs.spray.tank_l.value = overrides['tank']
            
        if 'speed' in overrides and specs.flight:
            specs.flight.work_speed_kmh.value = overrides['speed'] * 3.6
            
        real_swath = overrides.get('swath', 5.0)
        if not 'swath' in overrides and specs.spray and specs.spray.swath_m:
            min_s = float(specs.spray.swath_m[0].value)
            max_s = float(specs.spray.swath_m[1].value)
            real_swath = (min_s + max_s) / 2.0

        # 3. Safety Margin
        margin_h = real_swath / 2.0
        
        try:
            safe_polygon = MarginReducer.shrink(polygon, margin_h=margin_h)
        except Exception:
            raise ValueError("Field too small for safety margin.")

        # 4. Truck Route Processing
        truck_route_line = None
        
        # Smart Snap Logic
        if truck_route_points and len(truck_route_points) >= 2:
            if truck_offset > 0.1:
                try:
                    # Create buffered shell from the field boundary
                    field_shell = polygon.buffer(truck_offset, join_style=2)
                    shell_linear = field_shell.exterior
                    
                    snapped_points = []
                    for pt in truck_route_points:
                        p_obj = Point(pt)
                        proj_dist = shell_linear.project(p_obj)
                        snapped_pt = shell_linear.interpolate(proj_dist)
                        snapped_points.append((snapped_pt.x, snapped_pt.y))
                    
                    truck_route_line = LineString(snapped_points)
                except Exception as e:
                    print(f"Snap failed: {e}. Using raw points.")
                    truck_route_line = LineString(truck_route_points)
            else:
                truck_route_line = LineString(truck_route_points)
        else:
            # Auto Mode: Generate boundary for visualization only
            if truck_offset > 0.1:
                try:
                    field_shell = polygon.buffer(truck_offset, join_style=2)
                    truck_route_line = field_shell.exterior # LinearRing
                except:
                    pass

        # 5. Route Optimization (STRATEGY PATTERN)
        best_path = None
        best_angle = 0
        gen_stats = []
        
        if precalculated_path:
            print("Using PRE-CALCULATED flight path (Skipping Optimization)")
            best_path = precalculated_path
            # Angle unknown/irrelevant if reusing path, or we could pass it. 
            # For now assume 0 or keep previous.
        else:
            optimizer = StrategyFactory.get_strategy(strategy_name)
            print(f"Optimization Strategy: {strategy_name}")
            
            opt_result = optimizer.optimize(
                safe_polygon,
                swath_width=real_swath,
            )
            
            best_angle = opt_result['angle']
            best_path = LineString(opt_result['path']) if opt_result['path'] else None
            gen_stats = opt_result.get('gen_stats', [])
        
        if not best_path:
             raise ValueError("Could not generate flight path with current settings.")

        # 6. Mission Segmentation (Logistics)
        # Calculate max flow based on overrides
        app_rate = overrides.get('app_rate', 10.0)
        speed_ms = overrides.get('speed', 5.0)
        speed_kmh = speed_ms * 3.6
        calc_flow_l_min = (app_rate * speed_kmh * real_swath) / 600.0
        
        # Patch specs flow for segmenter
        if specs.spray:
            specs.spray.max_flow_l_min.value = calc_flow_l_min

        # Run Segmentation (Mobile vs Static)
        
        # A. Mobile Calculation
        mission_cycles = []
        full_metrics = {}
        comparison_metrics = {}
        resource_data = {}
        
        if use_mobile_station:

            # A. Mobile Calculation
            station_speed = 5.0 # Default truck speed
            station = MobileStation(truck_speed_mps=station_speed)
            segmenter = MissionSegmenter(specs, station, target_rate_l_ha=app_rate, work_speed_kmh=speed_kmh, swath_width=real_swath)
            
            mission_cycles = segmenter.segment_path(
                polygon=safe_polygon, 
                raw_path=list(best_path.coords),
                truck_polygon=polygon, # Use OUTER polygon for truck/logistics
                truck_route_line=truck_route_line
            )
            
            # B. Static Calculation (for comparison)
            if truck_route_line:
                home_point = truck_route_line.coords[0]
            else:
                home_point = best_path.coords[0]
                
            static_station = MobileStation(truck_speed_mps=0)
            static_segmenter = MissionSegmenter(specs, static_station, target_rate_l_ha=app_rate, work_speed_kmh=speed_kmh, swath_width=real_swath)
            
            static_cycles = static_segmenter.segment_path(
                polygon=safe_polygon,
                raw_path=list(best_path.coords),
                start_point=home_point
            )
            
            # 7. Generate Real Metrics via MissionAnalyzer
            full_metrics = MissionAnalyzer.calculate_comprehensive_metrics(mission_cycles, polygon, specs)
            
            comparison_metrics = MissionAnalyzer.compare_missions(mission_cycles, static_cycles)
            
            resource_data = MissionAnalyzer.plan_logistics(mission_cycles, specs)

        # Pack results
        return {
            "polygon": polygon,
            "safe_polygon": safe_polygon,
            "mission_cycles": mission_cycles,
            "static_cycles": static_cycles if use_mobile_station else None, # Return static for comparison toggles
            "truck_route_line": truck_route_line,
            "metrics": full_metrics,
            "comparison": comparison_metrics,
            "resources": resource_data,
            "best_angle": best_angle,
            "best_path": best_path,
            "gen_stats": gen_stats,
        }
