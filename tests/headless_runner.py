"""
Headless Mission Planner — runs the full MVC pipeline without GUI.
Useful for verifying the controller + optimization + logistics chain.
"""

import sys
import os
import argparse
import time

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../src')))

from controllers.mission_controller import MissionController
from visual_base import load_json, resolve_json_path
from drone_picker import pick_drone, get_swath_from_drone
from utils.ga_stats import plot_3d_convergence, save_gen_stats_json


def main():
    parser = argparse.ArgumentParser(description='Headless mission planner')
    parser.add_argument('json', nargs='?', help='Path to field JSON file')
    parser.add_argument('--drone', default=None, help='Drone model (skips picker)')
    parser.add_argument('--swath', type=float, default=None, help='Override swath (m)')
    parser.add_argument('--tank', type=float, default=None, help='Override tank (L)')
    parser.add_argument('--speed', type=float, default=5.0, help='m/s')
    parser.add_argument('--app-rate', type=float, default=20.0, help='L/ha')
    parser.add_argument('--strategy', default='genetic', choices=['genetic', 'simple'])
    parser.add_argument('--save-stats', action='store_true', help='Save GA stats to data/test_results/ga_stats.json')
    args = parser.parse_args()

    # 1. Select drone
    drone_name = args.drone or pick_drone()

    # 2. Derive swath from drone (or override)
    swath = args.swath if args.swath is not None else get_swath_from_drone(drone_name)

    # 3. Derive tank from drone (or override)
    from data import DroneDB
    spec = DroneDB.get_specs(drone_name)
    tank = args.tank if args.tank is not None else (float(spec.spray.tank_l.value) if spec and spec.spray else 10.0)

    # 4. Select field
    json_path = resolve_json_path(args.json)
    polygon, name = load_json(json_path)

    print(f"\nField: {name}")
    print(f"Area: {polygon.area:.1f} m²  |  Vertices: {len(polygon.exterior.coords) - 1}")
    print(f"Drone: {drone_name}  |  Swath: {swath:.1f} m  |  Tank: {tank:.1f} L  |  Strategy: {args.strategy}")

    controller = MissionController()
    overrides = {
        'swath': swath,
        'tank': tank,
        'speed': args.speed,
        'app_rate': args.app_rate,
    }

    print(f"\n{'='*60}")
    print(f"Running Mission Planning ({args.strategy})...")
    print(f"{'='*60}\n")

    try:
        t0 = time.perf_counter()
        result = controller.run_mission_planning(
            polygon_points=list(polygon.exterior.coords)[:-1],
            drone_name=drone_name,
            overrides=overrides,
            use_mobile_station=True,
            strategy_name=args.strategy,
        )

        elapsed = time.perf_counter() - t0
        print(f"\nTime: {elapsed:.2f}s")
        print(f"Best Angle: {result.get('best_angle', 0):.2f}°")

        cycles = result.get('mission_cycles', [])
        print(f"Total Cycles: {len(cycles)}")

        if result.get('best_path'):
            print(f"Path: {len(list(result['best_path'].coords))} waypoints")

        metrics = result.get('metrics', {})
        print(f"Dead Distance: {metrics.get('dead_dist_km', 0):.2f} km")
        print(f"Efficiency: {metrics.get('efficiency_ratio', 0) * 100:.1f}%")

        resources = result.get('resources', {})
        print(f"Battery Packs: {resources.get('battery_packs', 0)}")

        comparison = result.get('comparison', {})
        if comparison:
            print(f"\nMobile vs Static:")
            print(f"  Mobile Dead: {comparison.get('mobile_dead_km', 0):.2f} km")
            print(f"  Static Dead: {comparison.get('static_dead_km', 0):.2f} km")
            print(f"  Savings:     {comparison.get('savings_km', 0):.2f} km")

        # Save JSON immediately (before blocking plt.show)
        gen_stats = result.get('gen_stats', [])
        if not gen_stats:
            gen_stats = metrics.get('gen_stats', [])
        if args.save_stats and gen_stats:
            save_gen_stats_json(gen_stats, name, drone_name, tag="headless")

        print(f"\n{'='*60}")
        print(f"Headless run completed successfully.")
        print(f"{'='*60}\n")

        if gen_stats:
            plot_3d_convergence(gen_stats, name)

    except Exception as e:
        import traceback
        traceback.print_exc()
        print(f"\nFAILED: {e}")
        sys.exit(1)


if __name__ == '__main__':
    main()
