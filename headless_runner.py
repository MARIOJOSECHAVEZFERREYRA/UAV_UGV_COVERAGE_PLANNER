
import sys
import os
import json
from shapely.geometry import Polygon

# Add src to path
sys.path.append(os.path.join(os.path.dirname(__file__), 'src'))

from controllers.mission_controller import MissionController
from data.field_io import FieldIO

def main():
    print("Running Headless Planner Verification (MVC Architecture)...")
    
    # 1. Load 3D Map
    map_file = os.path.join(os.path.dirname(__file__), 'data', 'test_map_3d.json')
    if not os.path.exists(map_file):
        print(f"Test map not found: {map_file}")
        return
        
    try:
        poly = FieldIO.load_field(map_file)
            
    except Exception as e:
        print(f"Error loading map: {e}")
        return
    
    print(f"Polygon Area: {poly.area:.2f} m2")
    if poly.has_z:
        print("SUCCESS: 3D Coordinates detected in Polygon.")
    else:
        print("WARNING: Polygon is 2D.")

    # 2. Setup Mission Controller
    controller = MissionController()
    
    # Define simple overrides (mocking UI input)
    overrides = {
        'swath': 5.0,
        'tank': 10.0,
        'speed': 5.0, # m/s
        'app_rate': 20.0 # L/ha
    }
    drone_name = "DJI Agras T30" # Default drone
    
    print("\n--- Running Mission Planning (Genetic) ---")
    try:
        # 3. Executing Mission Plan
        result = controller.run_mission_planning(
            polygon_points=list(poly.exterior.coords),
            drone_name=drone_name,
            overrides=overrides,
            use_mobile_station=True,
            strategy_name="genetic"
        )
        
        # 4. Analyze Results
        print("\nOptimization Complete.")
        print(f"Best Angle: {result.get('best_angle', 0):.2f}°")
        
        metrics = result.get('metrics', {}) # Define EARLY for verification use
        
        cycles = result.get('mission_cycles', [])
        print(f"Total Cycles: {len(cycles)}")
        
        if 'best_path' in result:
             print(f"SUCCESS: best_path present")
        else:
             print("FAILURE: best_path NOT found")
        
        metrics = result.get('metrics', {})
        print(f"Total Dead Distance: {metrics.get('dead_dist_km', 0):.2f} km")
        print(f"Efficiency: {metrics.get('efficiency_ratio', 0) * 100:.1f}%")
        
        resources = result.get('resources', {})
        print(f"Battery Packs Required: {resources.get('battery_packs', 0)}")
        
        comparison = result.get('comparison', {})
        print("\nComparison (Mobile vs Static):")
        print(f"  Mobile Dead Dist: {comparison.get('mobile_dead_km', 0):.2f} km")
        print(f"  Static Dead Dist: {comparison.get('static_dead_km', 0):.2f} km")
        print(f"  Savings: {comparison.get('savings_km', 0):.2f} km")
        
        print("\nSUCCESS: Headless run completed using MVC architecture.")
        
    except Exception as e:
        print(f"FAILURE: Mission planning failed: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()
