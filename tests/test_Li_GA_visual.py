"""
Visual test for Li et al. (2023) Genetic Algorithm.
Runs the full pipeline: Margin Reduction (Phase 1) -> GA with internal
Decomposition (Phase 2) + Boustrophedon (Phase 3) + Heading Optimization (Phase 4).

Swath and margin are derived from the selected drone:
  - swath = max swath from drone specs
  - margin h = swath / 2 (Li Sec 2.1: half the spraying width)
Both can be overridden via CLI flags.
"""

import sys
import time
import os
import argparse
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
import numpy as np
from shapely.geometry import LineString

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../src')))

from algorithms.li_GA import LiGeneticOptimizer
from algorithms.path_planner import BoustrophedonPlanner
from algorithms.margin import MarginReducer
from visual_base import load_json, draw_poly, setup_dark_ax, BG, resolve_json_path
from drone_picker import pick_drone, get_swath_from_drone
from utils.ga_stats import plot_3d_convergence, save_gen_stats_json


def run_li_ga(polygon, swath_width=5.0, pop_size=200, generations=300):
    """Run the Li GA and return results."""
    planner = BoustrophedonPlanner(spray_width=swath_width)
    optimizer = LiGeneticOptimizer(
        planner,
        pop_size=pop_size,
        generations=generations,
    )
    best_angle, best_path, best_metrics = optimizer.optimize(polygon)
    return best_angle, best_path, best_metrics


def visualize(original_polygon, safe_polygon, path_coords, metrics,
              name, drone_name, swath, margin_h, elapsed):
    """Visualize original field, margin-reduced field, path, and metrics."""
    fig = plt.figure(figsize=(14, 8), facecolor=BG)
    gs = GridSpec(2, 2, figure=fig, width_ratios=[2, 1], hspace=0.35, wspace=0.3,
                  left=0.05, right=0.95, top=0.93, bottom=0.07)

    # -- Main map --
    ax_map = fig.add_subplot(gs[:, 0])
    setup_dark_ax(ax_map)

    if margin_h > 0:
        draw_poly(ax_map, original_polygon, fc='#7f8c8d', alpha=0.08, ec='#7f8c8d', lw=1)
        ax_map.plot([], [], color='#7f8c8d', lw=1, label=f'Original (h={margin_h:.1f}m)')

    draw_poly(ax_map, safe_polygon, fc='#2ecc71', alpha=0.15, ec='#2ecc71')
    ax_map.plot([], [], color='#2ecc71', lw=2, label='Work area')

    if path_coords:
        path_line = LineString(path_coords)
        px, py = path_line.xy
        ax_map.plot(px, py, color='#3498db', lw=1.2, alpha=0.85, label='Flight path')
        ax_map.plot(px[0], py[0], 'o', color='#2ecc71', ms=8, zorder=10, label='Start')
        ax_map.plot(px[-1], py[-1], 's', color='#e74c3c', ms=8, zorder=10, label='End')

    angle = metrics['angle']
    ax_map.set_title(f"{name}  |  {drone_name}  |  ψ = {angle:.3f}°",
                     color='white', fontsize=14, pad=10)
    ax_map.set_aspect('equal')
    ax_map.legend(loc='upper right', fontsize=9, facecolor='#1a1a2e', edgecolor='#444',
                  labelcolor='white')

    # -- Metrics panel --
    ax_info = fig.add_subplot(gs[0, 1])
    ax_info.axis('off')
    ax_info.set_facecolor(BG)

    flight_dist = metrics['l']
    s_prime = metrics['s_prime']
    extra_cov = metrics['extra_coverage_pct']
    fitness = metrics['fitness']
    original_area = original_polygon.area
    safe_area = safe_polygon.area

    info_text = (
        f"Drone:             {drone_name}\n"
        f"Swath d:           {swath:.1f} m\n"
        f"Margin h:          {margin_h:.1f} m (d/2)\n"
        f"Heading angle:     {angle:.3f}°\n"
        f"Fitness (Eq.15):   {fitness:.4f}\n"
        f"Flight distance:   {flight_dist:.1f} m\n"
        f"Coverage area S':  {s_prime:.1f} m²\n"
        f"Original area:     {original_area:.1f} m²\n"
        f"Work area S:       {safe_area:.1f} m²\n"
        f"Extra coverage ε:  {extra_cov:.2f}%\n"
        f"Waypoints:         {len(path_coords)}\n"
        f"Time:              {elapsed:.2f}s"
    )
    ax_info.text(0.05, 0.95, info_text, transform=ax_info.transAxes,
                 fontsize=10, color='white', verticalalignment='top',
                 fontfamily='monospace',
                 bbox=dict(boxstyle='round,pad=0.5', fc='#1a1a2e', ec='#444'))

    # -- Coverage bar --
    ax_bar = fig.add_subplot(gs[1, 1])
    setup_dark_ax(ax_bar)
    coverage_ratio = s_prime / safe_area if safe_area > 0 else 0
    bars = ax_bar.barh(['Coverage\nS\'/S'], [coverage_ratio], color='#2ecc71', height=0.4)
    ax_bar.axvline(1.0, color='#e74c3c', ls='--', lw=1.5, label='Ideal (100%)')
    ax_bar.set_xlim(0, max(1.3, coverage_ratio + 0.1))
    ax_bar.set_xlabel('Ratio', color='#bdc3c7')
    ax_bar.tick_params(axis='y', colors='white')
    ax_bar.legend(fontsize=9, facecolor='#1a1a2e', edgecolor='#444', labelcolor='white')

    for bar, val in zip(bars, [coverage_ratio]):
        ax_bar.text(bar.get_width() + 0.02, bar.get_y() + bar.get_height() / 2,
                    f'{val:.2%}', va='center', color='white', fontsize=11)

    fig.canvas.mpl_connect('key_press_event', lambda e: plt.close('all') if e.key == 'q' else None)
    plt.show()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Visual test for Li et al. (2023) GA')
    parser.add_argument('json', nargs='?', help='Path to field JSON file')
    parser.add_argument('--drone', type=str, default=None, help='Drone name (skips picker)')
    parser.add_argument('--swath', type=float, default=None, help='Override swath width (m)')
    parser.add_argument('--margin', type=float, default=None, help='Override margin h (m)')
    parser.add_argument('--pop', type=int, default=200, help='Population size (default: 200)')
    parser.add_argument('--gens', type=int, default=300, help='Generations (default: 300)')
    parser.add_argument('--save-stats', action='store_true', help='Save GA stats to data/test_results/ga_stats.json')
    args = parser.parse_args()

    # 1. Select drone
    drone_name = args.drone or pick_drone()

    # 2. Derive swath from drone (or override)
    swath = args.swath if args.swath is not None else get_swath_from_drone(drone_name)

    # 3. Derive margin: h = d/2 (Li Sec 2.1) unless overridden
    margin_h = args.margin if args.margin is not None else swath / 2.0

    # 4. Select field
    json_path = resolve_json_path(args.json)
    original_polygon, name = load_json(json_path)

    print(f"\nDrone: {drone_name}  |  Swath d={swath:.1f} m  |  Margin h={margin_h:.1f} m")

    # Phase 1: Margin Reduction (Li Sec 2.1)
    if margin_h > 0:
        safe_polygon = MarginReducer.shrink(original_polygon, margin_h)
        if safe_polygon.is_empty:
            print(f"ERROR: Margin h={margin_h}m makes the field disappear. Try smaller value.")
            sys.exit(1)
        print(f"Phase 1 — Margin reduction: h={margin_h:.1f} m")
        print(f"  Original: {original_polygon.area:.1f} m²  ->  Work area: {safe_polygon.area:.1f} m²")
    else:
        safe_polygon = original_polygon
        print("Phase 1 — Margin reduction: SKIPPED (h=0)")

    print(f"\nField: {name}")
    print(f"Work area: {safe_polygon.area:.1f} m²  |  Vertices: {len(safe_polygon.exterior.coords) - 1}")
    print(f"Pop: {args.pop}  |  Gens: {args.gens}")
    print(f"{'='*60}")
    print(f"Phases 2-4 — Decomposition + Path + GA Optimization...")
    print(f"{'='*60}\n")

    t0 = time.perf_counter()
    best_angle, best_path, best_metrics = run_li_ga(
        safe_polygon,
        swath_width=swath,
        pop_size=args.pop,
        generations=args.gens,
    )

    elapsed = time.perf_counter() - t0
    print(f"\n{'='*60}")
    print(f"Time: {elapsed:.2f}s")
    print(f"Result: ψ = {best_angle:.3f}°  |  L = {best_metrics['l']:.1f} m  |  ε = {best_metrics['extra_coverage_pct']:.2f}%")
    print(f"{'='*60}\n")

    # Save JSON immediately (before blocking plt.show calls)
    gen_stats = best_metrics.get('gen_stats', [])
    if args.save_stats and gen_stats:
        save_gen_stats_json(gen_stats, name, drone_name, tag="li_ga")

    visualize(original_polygon, safe_polygon, best_path, best_metrics,
              name, drone_name, swath, margin_h, elapsed)

    if gen_stats:
        plot_3d_convergence(gen_stats, name)
