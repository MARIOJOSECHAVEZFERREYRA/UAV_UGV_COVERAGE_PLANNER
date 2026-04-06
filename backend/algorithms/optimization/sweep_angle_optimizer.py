"""
GA-based sweep angle optimizer for coverage path planning.

Finds the optimal boustrophedon heading angle for a given polygon using a
genetic algorithm (Li et al. 2023, Aerospace 10, 755 — Section 2.5).

"""

import random
import numpy as np
from shapely.geometry import Polygon
from shapely.ops import unary_union
from shapely import affinity

from ..polygon.decomposition import ConcaveDecomposer
from ..polygon.path_planner import BoustrophedonPlanner
from ..polygon.path_assembler import PathAssembler
from ..rendezvous.mission_simulator import simulate_mission_with_rendezvous


def build_obstacle_union(polygon: Polygon):
    """Return a unary union of all interior holes, or None if there are none."""
    if polygon.interiors:
        return unary_union([Polygon(h.coords) for h in polygon.interiors])
    return None


class SweepAngleOptimizer:
    """
    GA-based optimizer that finds the best boustrophedon sweep angle for a polygon.
    """

    def __init__(self, planner, pop_size=200, generations=300, crossover_rate=0.4,
                 mutation_rate=0.05, mutation_range=15, min_diversity=20,
                 early_stopping_patience=50, energy_model=None,
                 rendezvous_planner=None, w_rv=0.5,
    ):
        self.planner = planner
        self.pop_size = pop_size
        self.generations = generations
        self.crossover_rate = crossover_rate
        self.mutation_rate = mutation_rate
        self.mutation_range = mutation_range
        self.min_diversity = min_diversity
        self.early_stopping_patience = early_stopping_patience
        self.energy_model = energy_model
        self.rendezvous_planner = rendezvous_planner
        self.w_rv = float(w_rv)
        # Modo de simulacion: True cuando se dispone de drone + UGV
        self._rv_enabled = energy_model is not None and rendezvous_planner is not None

    def _evaluate_angle(self,angle_deg: int,polygon: Polygon,target_area_S: float,assembler: PathAssembler = None,obstacle_union=None,base_point=None,
    ) -> dict:
        """
        Run the full CPP pipeline for a candidate heading angle and return
        route-aware metrics.
        """
        sub_polygons = ConcaveDecomposer.decompose(polygon, angle_deg,
                                                    channel_width=self.planner.spray_width)

        rotation_origin = polygon.centroid
        rotated_whole = affinity.rotate(polygon, -angle_deg, origin=rotation_origin)
        global_y_origin = rotated_whole.bounds[1]

        all_sweep_segments = []
        total_spray_distance = 0.0
        total_s_prime = 0.0
        total_turn_count = 0

        for sub_poly in sub_polygons:
            planner_result = self.planner.generate_path(
                sub_poly,
                angle_deg,
                global_y_origin=global_y_origin,
                rotation_origin=rotation_origin,
                obstacles=obstacle_union,
            )

            sweep_segments = planner_result.get("sweep_segments", [])
            metrics = planner_result.get("metrics", {})

            if sweep_segments:
                all_sweep_segments.extend(sweep_segments)

            total_spray_distance += float(metrics.get("spray_distance_m", 0.0))
            total_s_prime += float(metrics.get("coverage_area_m2", 0.0))
            total_turn_count += int(metrics.get("turn_count", 0))

        if assembler is None:
            assembler = PathAssembler(polygon)

        assembly_result = assembler.assemble_connected(all_sweep_segments)
        route_segments = assembly_result.get("route_segments", [])
        combined_path = assembly_result.get("combined_path", [])
        route_distances = assembly_result.get("distances", {})

        total_l = float(route_distances.get("total_m", 0.0))

        # Include round-trip deadhead cost so the fitness reflects full mission cost,
        # not just infield path length. base_point is outside the polygon; the assembler
        # falls through to the boundary-walk path, which is correct for that case.
        if base_point is not None and route_segments:
            base_pt = (float(base_point[0]), float(base_point[1]))
            _, d_entry = assembler.safe_connection(base_pt, route_segments[0]["path"][0])
            _, d_exit = assembler.safe_connection(route_segments[-1]["path"][-1], base_pt)
            total_l += d_entry + d_exit

        # Simulacion de mision con rendezvous (solo si hay drone + UGV configurados)
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
            "angle": angle_deg,
            "l": total_l,
            "s_prime": total_s_prime,
            "combined_path": combined_path,
            "route_segments": route_segments,
            "planner_metrics": {
                "spray_distance_m": float(total_spray_distance),
                "coverage_area_m2": float(total_s_prime),
                "turn_count": int(total_turn_count),
            },
            "route_distances": {
                "sweep_m": float(route_distances.get("sweep_m", 0.0)),
                "ferry_m": float(route_distances.get("ferry_m", 0.0)),
                "total_m": float(route_distances.get("total_m", 0.0)),
            },
            # Metricas de rendezvous (0/True cuando rv no esta habilitado)
            "rv_feasible": rv_feasible,
            "rv_wait": rv_wait,
            "rv_time": rv_time,
            "rv_count": rv_count,
        }

    @staticmethod
    def _compute_fitness(l_norm, s_prime, target_area_S, rv_penalty_norm=0.0, w_rv=0.0):
        """
        Fitness function:
            f = ( l_norm + |S' - S| / S + w_rv * rv_penalty_norm )^{-1}

        El termino rv_penalty_norm penaliza el tiempo de espera del UAV,
        normalizado de la misma forma que l_norm. w_rv=0 reproduce el
        comportamiento original sin rendezvous.
        """
        if target_area_S <= 0:
            return 1e-10

        extra_coverage = abs(s_prime - target_area_S) / target_area_S
        denominator = l_norm + extra_coverage + w_rv * rv_penalty_norm

        if denominator < 1e-12:
            return 1e10

        return 1.0 / denominator

    @staticmethod
    def _tournament_selection(population: list[int], fitness_values: list[float], k: int = 3) -> int:
        """Tournament selection: pick k random individuals, return the best."""
        indices = random.sample(range(len(population)), k)
        best = max(indices, key=lambda i: fitness_values[i])
        return population[best]

    @staticmethod
    def _blend_crossover(p1: int, p2: int):
        """
        Blend crossover respecting the circular [0, 180) topology.
        Uses the shortest arc between parents so children stay near both parents.
        """
        diff = (p2 - p1 + 90) % 180 - 90
        alpha = random.random()
        offset = round(alpha * diff)
        c1 = (p1 + offset) % 180
        c2 = (p1 - diff + offset) % 180
        return c1, c2

    def _mutate(self, angle: int) -> int:
        """Uniform mutation: perturb angle by a random offset."""
        if random.random() < self.mutation_rate:
            delta = random.randint(-self.mutation_range, self.mutation_range)
            return (angle + delta) % 180
        return angle

    def _evaluate_population(self,population: list[int],polygon: Polygon,target_area_S: float,cache: dict,assembler: PathAssembler = None,obstacle_union=None,base_point=None,
    ):
        """
        Evaluate all individuals using a cache keyed by integer angle.
        """
        raw_metrics = []

        for angle in population:
            if angle not in cache:
                cache[angle] = self._evaluate_angle(
                    angle,
                    polygon,
                    target_area_S,
                    assembler=assembler,
                    obstacle_union=obstacle_union,
                    base_point=base_point,
                )
            raw_metrics.append(cache[angle])

        all_l = np.array([m["l"] for m in raw_metrics], dtype=float)
        l2_norm = np.sqrt(np.sum(all_l ** 2))
        if l2_norm < 1e-12:
            l2_norm = 1.0

        l_norms = all_l / l2_norm

        if self._rv_enabled:
            # Normalizar tiempos de espera del UAV igual que se normaliza l
            # Las soluciones infactibles reciben un valor muy alto antes de normalizar
            rv_waits = []
            for m in raw_metrics:
                if m.get('rv_feasible', True):
                    rv_waits.append(m.get('rv_wait', 0.0))
                else:
                    rv_waits.append(1e9)
            all_rv_wait = np.array(rv_waits, dtype=float)
            rv_l2 = np.sqrt(np.sum(all_rv_wait ** 2))
            if rv_l2 < 1e-12:
                rv_l2 = 1.0
            rv_norms = all_rv_wait / rv_l2

            fitness_values = []
            for i in range(len(raw_metrics)):
                if not raw_metrics[i].get('rv_feasible', True):
                    # Solucion infactible: fitness minimo para que el GA la descarte
                    fitness_values.append(1e-10)
                else:
                    f = self._compute_fitness(
                        l_norms[i],
                        raw_metrics[i]["s_prime"],
                        target_area_S,
                        rv_penalty_norm=rv_norms[i],
                        w_rv=self.w_rv,
                    )
                    fitness_values.append(f)
        else:
            fitness_values = [
                self._compute_fitness(l_norms[i], raw_metrics[i]["s_prime"], target_area_S)
                for i in range(len(raw_metrics))
            ]

        return raw_metrics, fitness_values, all_l

    def optimize(self, polygon: Polygon, base_point=None) -> dict:
        """
        Execute the GA.
        """
        target_area_S = polygon.area
        eval_cache = {}

        assembler = PathAssembler(polygon)
        obstacle_union = build_obstacle_union(polygon)

        population = [random.randint(0, 179) for _ in range(self.pop_size)]

        best_solution = None
        best_fitness = -1.0
        no_improvement_count = 0
        gen_stats = []

        for gen in range(self.generations):
            raw_metrics, fitness_values, all_l = self._evaluate_population(
                population,
                polygon,
                target_area_S,
                eval_cache,
                assembler=assembler,
                obstacle_union=obstacle_union,
                base_point=base_point,
            )

            gen_stats.append({
                "gen": gen + 1,
                "mean_fitness": float(np.mean(fitness_values)),
                "mean_angle": float(np.mean(population)),
                "mean_l": float(np.mean(all_l)),
            })

            prev_best_fitness = best_fitness

            for i, f in enumerate(fitness_values):
                if f > best_fitness:
                    best_fitness = f
                    m = raw_metrics[i]
                    extra_cov = abs(m["s_prime"] - target_area_S) / target_area_S * 100.0

                    best_solution = {
                        "angle": m["angle"],
                        "fitness": f,
                        "l": m["l"],
                        "s_prime": m["s_prime"],
                        "extra_coverage_pct": extra_cov,
                        "route_segments": m.get("route_segments", []),
                        "combined_path": m.get("combined_path", []),
                        "planner_metrics": m.get("planner_metrics", {}),
                        "route_distances": m.get("route_distances", {}),
                        # Metricas de rendezvous (presentes solo cuando rv esta habilitado)
                        "rv_feasible": m.get("rv_feasible", True),
                        "rv_wait": m.get("rv_wait", 0.0),
                        "rv_time": m.get("rv_time", 0.0),
                        "rv_count": m.get("rv_count", 0),
                    }

            selected = [
                self._tournament_selection(population, fitness_values, k=3)
                for _ in range(self.pop_size)
            ]

            kids = []
            i = 0
            while i < len(selected) - 1:
                p1 = selected[i]
                p2 = selected[i + 1]

                if random.random() < self.crossover_rate:
                    c1, c2 = self._blend_crossover(p1, p2)
                else:
                    c1, c2 = p1, p2

                kids.append(self._mutate(c1))
                kids.append(self._mutate(c2))
                i += 2

            if i < len(selected):
                kids.append(self._mutate(selected[i]))

            elite = population[int(np.argmax(fitness_values))]
            population = kids[:self.pop_size]
            population[0] = elite

            if len(set(population)) < self.min_diversity:
                n_inject = self.pop_size // 5
                inject_indices = random.sample(range(1, self.pop_size), n_inject)
                for idx in inject_indices:
                    population[idx] = random.randint(0, 179)

            if best_fitness > prev_best_fitness:
                no_improvement_count = 0
            else:
                no_improvement_count += 1

            if no_improvement_count >= self.early_stopping_patience:
                print(f"  Early stopping at gen {gen + 1}")
                break

            if (gen + 1) % 25 == 0 and best_solution is not None:
                s = gen_stats[-1]
                print(
                    f"  Gen {gen + 1}/{self.generations} | "
                    f"Best: {best_fitness:.4f} @ {best_solution['angle']}° | "
                    f"Pop mean: fit={s['mean_fitness']:.4f}  "
                    f"angle={s['mean_angle']:.1f}°  "
                    f"L={s['mean_l']:.0f}m"
                )

        # Re-seleccionar el mejor desde el cache completo usando normalizacion L2 fija.
        # El problema con el tracking dentro del loop es que cada generacion normaliza
        # por la norma L2 de su propia poblacion, haciendo los fitness entre generaciones
        # incomparables. Aqui se normaliza por la norma L2 de TODAS las soluciones
        # evaluadas a lo largo de toda la ejecucion del GA, que es un denominador fijo.
        all_cached = list(eval_cache.values())

        if not all_cached:
            raise ValueError("SweepAngleOptimizer failed to find a valid solution.")

        all_l_final = np.array([m["l"] for m in all_cached], dtype=float)
        l2_final = float(np.sqrt(np.sum(all_l_final ** 2)))
        if l2_final < 1e-12:
            l2_final = 1.0

        if self._rv_enabled:
            rv_waits_final = [
                m.get("rv_wait", 0.0) if m.get("rv_feasible", True) else 1e9
                for m in all_cached
            ]
            rv_l2_final = float(np.sqrt(np.sum(np.array(rv_waits_final, dtype=float) ** 2)))
            if rv_l2_final < 1e-12:
                rv_l2_final = 1.0
        else:
            rv_waits_final = [0.0] * len(all_cached)
            rv_l2_final = 1.0

        best_abs_fitness = -1.0
        best_solution = None

        for idx, m in enumerate(all_cached):
            if self._rv_enabled and not m.get("rv_feasible", True):
                continue

            l_norm = m["l"] / l2_final
            rv_norm = rv_waits_final[idx] / rv_l2_final if self._rv_enabled else 0.0

            f = self._compute_fitness(
                l_norm, m["s_prime"], target_area_S,
                rv_penalty_norm=rv_norm, w_rv=self.w_rv,
            )

            if f > best_abs_fitness:
                best_abs_fitness = f
                extra_cov = abs(m["s_prime"] - target_area_S) / target_area_S * 100.0
                best_solution = {
                    "angle": m["angle"],
                    "fitness": f,
                    "l": m["l"],
                    "s_prime": m["s_prime"],
                    "extra_coverage_pct": extra_cov,
                    "route_segments": m.get("route_segments", []),
                    "combined_path": m.get("combined_path", []),
                    "planner_metrics": m.get("planner_metrics", {}),
                    "route_distances": m.get("route_distances", {}),
                    "rv_feasible": m.get("rv_feasible", True),
                    "rv_wait": m.get("rv_wait", 0.0),
                    "rv_time": m.get("rv_time", 0.0),
                    "rv_count": m.get("rv_count", 0),
                }

        if best_solution is None:
            raise ValueError("SweepAngleOptimizer failed to find a valid solution.")

        print(
            f"SweepAngleOptimizer done. Best angle: {best_solution['angle']}°, "
            f"fitness: {best_abs_fitness:.4f} (fixed-norm), "
            f"flight dist: {best_solution['l']:.1f} m, "
            f"extra coverage: {best_solution['extra_coverage_pct']:.2f}%"
        )

        best_solution["gen_stats"] = gen_stats
        return best_solution