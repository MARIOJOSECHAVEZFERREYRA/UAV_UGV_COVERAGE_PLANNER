"""Brute-force grid search over the boustrophedon heading angle.

For a fixed polygon + swath the fitness landscape in the angle dimension
is tiny (180 integer values) and the GA that used to drive this step was
ceremony: it spent hundreds of generations converging to what a simple
exhaustive scan finds in one pass. This optimizer evaluates every
integer angle in [0, 180) with the full pipeline (decompose, sweep,
assemble, deadhead estimation) and returns the one that minimizes total
flight distance `L = spray_m + ferry_m + deadhead_m`.

The best angle is re-assembled in 'full' sequencer mode so its ferries
are obstacle-aware geodesic routes rather than the euclidean-fast
approximation used during the scan.

Backwards-compatible: the return dict matches SweepAngleOptimizer's
shape so downstream callers (StrategyFactory, MissionPlanner) need no
changes.
"""

import math
import time

from shapely.geometry import Polygon
from shapely.ops import unary_union
from shapely import affinity

from ..coverage.decomposition import ConcaveDecomposer
from ..coverage.path_assembler import PathAssembler
from ..simulation.mission_simulator import simulate_mission_with_rendezvous


def _build_obstacle_union(polygon: Polygon):
    if polygon.interiors:
        return unary_union([Polygon(h.coords) for h in polygon.interiors])
    return None


class GridSearchOptimizer:
    """Exhaustive angle scan optimizer.

    Replaces the genetic algorithm with a deterministic 180-angle search.
    Each angle is evaluated with the full CPP pipeline and the minimum
    total flight distance wins.
    """

    def __init__(self, planner, angle_step: int = 1,
                 energy_model=None, rendezvous_planner=None, w_rv: float = 0.5,
                 cycle_penalty_s: float = 0.0):
        if angle_step < 1:
            raise ValueError("angle_step must be >= 1")
        self.planner = planner
        self.angle_step = int(angle_step)
        self.energy_model = energy_model
        self.rendezvous_planner = rendezvous_planner
        self.w_rv = float(w_rv)
        # Optional per-extra-cycle penalty added to mission_time. Default 0
        # assumes hot-swap batteries + instant refill → cycles penalize
        # naturally via their extra deadhead distance. Set > 0 if you
        # want the optimizer to prefer fewer cycles beyond that natural
        # bias (units are seconds but the value is a tuning weight, not
        # a physical measurement).
        self.cycle_penalty_s = float(cycle_penalty_s)

        # Mission mode flags mirroring SweepAngleOptimizer so the strategy
        # layer does not need to branch differently for grid vs genetic.
        self._rv_enabled = (
            energy_model is not None and rendezvous_planner is not None
        )
        self._static_deadhead_enabled = (
            energy_model is not None and rendezvous_planner is None
        )

    # ------------------------------------------------------------------
    # Deadhead estimation (shared with SweepAngleOptimizer rationale)
    # ------------------------------------------------------------------

    def _estimate_static_deadhead(self, route_segments, base_point):
        """Euclidean walk mirroring MissionSegmenter.segment_path.

        Returns (deadhead_total_m, n_cycles). n_cycles counts the number
        of closed mission cycles, which equals `cycle_breaks + 1` — the
        +1 is the always-present first cycle that opens with the entry
        deadhead. n_cycles is needed by the mission-time fitness to
        penalize refill/recharge overhead.
        """
        em = self.energy_model
        drone = em.drone
        bx, by = float(base_point[0]), float(base_point[1])

        e_rem = em.usable_energy_wh()
        q_rem = float(drone.mass_tank_full_kg)
        deadhead_total = 0.0
        cycle_empty = True
        cycle_breaks = 0
        uav_x, uav_y = bx, by

        if route_segments:
            first_path = route_segments[0].get('path', [])
            if first_path:
                fx, fy = float(first_path[0][0]), float(first_path[0][1])
                d_entry = math.hypot(fx - bx, fy - by)
                e_rem = max(0.0, e_rem - em.energy_transit(d_entry, q_rem))
                uav_x, uav_y = fx, fy

        for seg in route_segments:
            seg_type = seg.get('segment_type', 'ferry')
            dist = float(seg.get('distance_m', 0.0))
            path = seg.get('path', [])
            if dist < 1e-9 or len(path) < 2:
                continue

            p1x, p1y = float(path[0][0]), float(path[0][1])
            p2x, p2y = float(path[-1][0]), float(path[-1][1])

            if seg_type == 'sweep':
                e_step = em.energy_straight(dist, q_rem)
                q_step = em.reagent_consumed(dist)
            else:
                e_step = em.energy_transit(dist, q_rem)
                q_step = 0.0

            d_after_to_base = math.hypot(p2x - bx, p2y - by)
            can_do = em.feasible_after_segment_static(
                e_rem, q_rem, e_step, q_step, d_after_to_base
            )

            if can_do or cycle_empty:
                e_rem -= e_step
                q_rem = max(0.0, q_rem - q_step)
                uav_x, uav_y = p2x, p2y
                cycle_empty = False
            else:
                d_back = math.hypot(uav_x - bx, uav_y - by)
                d_fwd = math.hypot(p1x - bx, p1y - by)
                deadhead_total += d_back + d_fwd
                cycle_breaks += 1
                e_rem = em.usable_energy_wh()
                q_rem = float(drone.mass_tank_full_kg)
                e_rem = max(0.0, e_rem - em.energy_transit(d_fwd, q_rem))
                if seg_type == 'sweep':
                    e_step = em.energy_straight(dist, q_rem)
                    q_step = em.reagent_consumed(dist)
                else:
                    e_step = em.energy_transit(dist, q_rem)
                    q_step = 0.0
                e_rem -= e_step
                q_rem = max(0.0, q_rem - q_step)
                uav_x, uav_y = p2x, p2y
                cycle_empty = False

        n_cycles = cycle_breaks + 1 if route_segments else 0
        return deadhead_total, n_cycles

    # ------------------------------------------------------------------
    # Mission-time fitness
    # ------------------------------------------------------------------

    def _compute_mission_time(self, spray_m, ferry_m, deadhead_m, n_cycles, n_turns=0):
        """Convert distance + turn count to total mission time in seconds.

        Flight time pondera cada segmento por su velocidad real:
        sprays a cruise speed, ferries/deadheads a max transit speed.
        Añade un costo fijo por cada turn entre sweep rows (tomado de
        `drone.turn_duration_s`, default 10 s) porque el drone decelera,
        gira 180° y vuelve a acelerar — ese tiempo no está en la
        distancia pura. El overhead por ciclos extras se aplica solo si
        el caller pasó `cycle_penalty_s > 0`; default 0 asume hot-swap
        battery + refill instantáneo.

        Cuando no hay energy_model (tests unitarios), usa valores
        default para mantener el fitness monótono con la distancia.
        """
        if self.energy_model is not None:
            drone = self.energy_model.drone
            v_cruise = float(getattr(drone, 'speed_cruise_ms', 5.0))
            v_max = float(getattr(drone, 'speed_max_ms', v_cruise * 1.5))
            turn_s = float(getattr(drone, 'turn_duration_s', 10.0))
        else:
            v_cruise = 5.0
            v_max = 10.0
            turn_s = 10.0

        v_cruise = max(v_cruise, 0.1)
        v_max = max(v_max, 0.1)

        flight_time_s = (
            spray_m / v_cruise +
            ferry_m / v_max +
            deadhead_m / v_max +
            n_turns * turn_s
        )
        cycle_penalty = self.cycle_penalty_s * max(n_cycles - 1, 0)
        mission_time_s = flight_time_s + cycle_penalty
        return mission_time_s, flight_time_s

    # ------------------------------------------------------------------
    # Single-angle evaluation
    # ------------------------------------------------------------------

    def _evaluate_angle(self, angle_deg, polygon, target_area_S,
                        obstacle_union, base_point):
        """Run the full pipeline for one angle and return metrics."""
        swath = self.planner.spray_width

        sub_polygons = ConcaveDecomposer.decompose(
            polygon, angle_deg,
            channel_width=swath, min_swath=swath,
        )
        if not sub_polygons:
            return None

        rotation_origin = polygon.centroid
        rotated_whole = affinity.rotate(polygon, -angle_deg, origin=rotation_origin)
        global_y_origin = rotated_whole.bounds[1]

        all_sweep_segments = []
        total_spray_distance = 0.0
        total_s_prime = 0.0
        total_turn_count = 0

        for cell_id, sub_poly in enumerate(sub_polygons):
            planner_result = self.planner.generate_path(
                sub_poly, angle_deg,
                global_y_origin=global_y_origin,
                rotation_origin=rotation_origin,
                obstacles=obstacle_union,
            )
            sweep_segments = planner_result.get('sweep_segments', [])
            metrics = planner_result.get('metrics', {})

            for s in sweep_segments:
                s['cell_id'] = cell_id
            all_sweep_segments.extend(sweep_segments)

            total_spray_distance += float(metrics.get('spray_distance_m', 0.0))
            total_s_prime += float(metrics.get('coverage_area_m2', 0.0))
            total_turn_count += int(metrics.get('turn_count', 0))

        if not all_sweep_segments:
            return None

        # Fast sequencer mode for scan — winner gets re-assembled with 'full'
        # mode at the end of optimize().
        assembler = PathAssembler(
            sub_polygons, original_polygon=polygon,
            sequencer_mode='fast', base_point=base_point,
        )
        assembly_result = assembler.assemble_connected(all_sweep_segments)
        route_segments = assembly_result.get('route_segments', [])
        combined_path = assembly_result.get('combined_path', [])
        route_distances = assembly_result.get('distances', {})

        ferry_m = float(route_distances.get('ferry_m', 0.0))

        # Deadhead: base→first + last→base + intermediate cycles
        deadhead_m = 0.0
        n_cycles = 1  # always at least 1 cycle if we have any route
        if base_point is not None and route_segments:
            bp = (float(base_point[0]), float(base_point[1]))
            _, d_entry = assembler.find_connection(bp, route_segments[0]['path'][0])
            _, d_exit = assembler.find_connection(route_segments[-1]['path'][-1], bp)
            deadhead_m += d_entry + d_exit
            if self._static_deadhead_enabled:
                intermediate, n_cycles = self._estimate_static_deadhead(route_segments, bp)
                deadhead_m += intermediate

        # Fitness is total mission time — weights spray/ferry/deadhead by
        # their real flight speeds, adds turn_duration_s per 180° turn,
        # and applies an optional cycle_penalty per extra cycle.
        mission_time_s, flight_time_s = self._compute_mission_time(
            total_spray_distance, ferry_m, deadhead_m, n_cycles,
            n_turns=total_turn_count,
        )
        total_l = mission_time_s

        # Optional rendezvous simulation (dynamic missions)
        rv_feasible = True
        rv_wait = 0.0
        rv_time = 0.0
        rv_count = 0
        if self._rv_enabled and route_segments:
            sim = simulate_mission_with_rendezvous(
                route_segments=route_segments,
                energy_model=self.energy_model,
                rendezvous_planner=self.rendezvous_planner,
                assembler=assembler,
            )
            rv_feasible = sim['feasible']
            rv_wait = sim['total_wait_uav']
            rv_time = sim['total_time']
            rv_count = sim['n_rendezvous']

        return {
            'angle': angle_deg,
            'l': total_l,
            'mission_time_s': mission_time_s,
            'flight_time_s': flight_time_s,
            'n_cycles': n_cycles,
            's_prime': total_s_prime,
            'spray_m': float(total_spray_distance),
            'ferry_m': ferry_m,
            'deadhead_m': deadhead_m,
            'combined_path': combined_path,
            'route_segments': route_segments,
            'sub_polygons': sub_polygons,
            'all_sweep_segments': all_sweep_segments,
            'planner_metrics': {
                'spray_distance_m': float(total_spray_distance),
                'coverage_area_m2': float(total_s_prime),
                'turn_count': int(total_turn_count),
            },
            'route_distances': {
                'sweep_m': float(route_distances.get('sweep_m', 0.0)),
                'ferry_m': ferry_m,
                'total_m': float(route_distances.get('total_m', 0.0)),
            },
            'rv_feasible': rv_feasible,
            'rv_wait': rv_wait,
            'rv_time': rv_time,
            'rv_count': rv_count,
        }

    # ------------------------------------------------------------------
    # Public entry
    # ------------------------------------------------------------------

    def optimize(self, polygon: Polygon, base_point=None) -> dict:
        target_area_S = polygon.area
        obstacle_union = _build_obstacle_union(polygon)

        t0 = time.perf_counter()
        best = None
        gen_stats = []
        evaluated = 0
        infeasible = 0

        for angle in range(0, 180, self.angle_step):
            result = self._evaluate_angle(
                angle, polygon, target_area_S, obstacle_union, base_point,
            )
            if result is None:
                continue

            evaluated += 1
            if self._rv_enabled and not result.get('rv_feasible', True):
                infeasible += 1
                continue

            if best is None or result['l'] < best['l']:
                best = result

            gen_stats.append({
                'gen': evaluated,
                'mean_fitness': 1.0 / result['l'] if result['l'] > 0 else 0.0,
                'mean_angle': float(angle),
                'mean_l': float(result['l']),
            })

        if best is None:
            if self._rv_enabled:
                raise ValueError(
                    "GridSearchOptimizer: all angles produced infeasible "
                    "rendezvous missions."
                )
            raise ValueError(
                "GridSearchOptimizer: no angle produced a valid route."
            )

        # Re-assemble the winning angle with the full geodesic sequencer.
        best = self._reassemble_full(best, polygon, obstacle_union, base_point)

        elapsed = time.perf_counter() - t0
        target_area = polygon.area
        extra_cov = abs(best['s_prime'] - target_area) / target_area * 100.0

        best['fitness'] = 1.0 / best['l'] if best['l'] > 0 else 0.0
        best['extra_coverage_pct'] = extra_cov
        best['gen_stats'] = gen_stats

        mission_min = best['l'] / 60.0
        print(
            f"GridSearchOptimizer done in {elapsed:.2f}s "
            f"({evaluated} angles evaluated, {infeasible} infeasible). "
            f"Best angle: {best['angle']}° | "
            f"mission={mission_min:.1f}min ({best['n_cycles']} cycles) | "
            f"spray={best['spray_m']:.0f}m  ferry={best['ferry_m']:.0f}m  "
            f"deadhead={best['deadhead_m']:.0f}m"
        )
        return best

    def _reassemble_full(self, best, polygon, obstacle_union, base_point):
        """Re-run assembly for the best angle with 'full' sequencer mode."""
        sub_polygons = best['sub_polygons']
        all_sweep_segments = best['all_sweep_segments']

        assembler = PathAssembler(
            sub_polygons, original_polygon=polygon,
            sequencer_mode='full', base_point=base_point,
        )
        assembly_result = assembler.assemble_connected(all_sweep_segments)
        route_segments = assembly_result.get('route_segments', [])

        best = dict(best)
        best['route_segments'] = route_segments
        best['combined_path'] = assembly_result.get('combined_path', [])
        best['route_distances'] = assembly_result.get('distances', {})
        best['ferry_m'] = float(best['route_distances'].get('ferry_m', 0.0))

        if base_point is not None and route_segments:
            bp = (float(base_point[0]), float(base_point[1]))
            _, d_entry = assembler.find_connection(bp, route_segments[0]['path'][0])
            _, d_exit = assembler.find_connection(route_segments[-1]['path'][-1], bp)
            dh = d_entry + d_exit
            n_cycles = 1
            if self._static_deadhead_enabled:
                intermediate, n_cycles = self._estimate_static_deadhead(route_segments, bp)
                dh += intermediate
            best['deadhead_m'] = dh
            best['n_cycles'] = n_cycles
            turns = int(best.get('planner_metrics', {}).get('turn_count', 0))
            mission_time_s, flight_time_s = self._compute_mission_time(
                best['spray_m'], best['ferry_m'], dh, n_cycles, n_turns=turns,
            )
            best['l'] = mission_time_s
            best['mission_time_s'] = mission_time_s
            best['flight_time_s'] = flight_time_s

        return best
