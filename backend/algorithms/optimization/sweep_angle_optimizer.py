"""
GA-based sweep angle optimizer for coverage path planning.

Finds the optimal boustrophedon heading angle for a given polygon using a
genetic algorithm (Li et al. 2023, Aerospace 10, 755 — Section 2.5).

Search space: integer angles [0, 180) — exploits 180° boustrophedon symmetry.
Fitness (Eq. 15): f = (l_norm + |S' - S| / S)^{-1}
"""

import random
import numpy as np
from shapely.geometry import Polygon
from shapely.ops import unary_union
from shapely import affinity
from typing import List, Tuple

from ..polygon.decomposition import ConcaveDecomposer
from ..polygon.path_planner import BoustrophedonPlanner
from ..polygon.path_assembler import PathAssembler


def build_obstacle_union(polygon: Polygon):
    """Return a unary union of all interior holes, or None if there are none."""
    if polygon.interiors:
        return unary_union([Polygon(h.coords) for h in polygon.interiors])
    return None


class SweepAngleOptimizer:
    """
    GA-based optimizer that finds the best boustrophedon sweep angle for a polygon.

    Parameters (adapted from Li et al. 2023, Table 1):
        pop_size: population size (alpha)
        generations: max generations (delta)
        crossover_rate: crossover probability (eta = 0.4)
        mutation_rate: probability of mutating an individual
        mutation_range: max degrees to perturb during mutation
        min_diversity: minimum unique angles in population before diversity injection
        early_stopping_patience: generations without improvement before stopping
    """

    def __init__(
        self,
        planner: BoustrophedonPlanner,
        pop_size: int = 200,
        generations: int = 300,
        crossover_rate: float = 0.4,
        mutation_rate: float = 0.05,
        mutation_range: int = 15,
        min_diversity: int = 20,
        early_stopping_patience: int = 50,
    ):
        self.planner = planner
        self.pop_size = pop_size
        self.generations = generations
        self.crossover_rate = crossover_rate
        self.mutation_rate = mutation_rate
        self.mutation_range = mutation_range
        self.min_diversity = min_diversity
        self.early_stopping_patience = early_stopping_patience

    # ------------------------------------------------------------------ #
    #                        Evaluation (Eq. 11-15)                       #
    # ------------------------------------------------------------------ #

    def _evaluate_angle(
        self, angle_deg: int, polygon: Polygon, target_area_S: float,
        assembler: PathAssembler = None, obstacle_union=None,
    ) -> dict:
        """
        Run the full CPP pipeline for a candidate heading angle and return
        raw metrics: flight distance l (Eq. 11) and coverage area S' (Eq. 13).
        """
        sub_polygons = ConcaveDecomposer.decompose(polygon, angle_deg)

        rotation_origin = polygon.centroid
        rotated_whole = affinity.rotate(polygon, -angle_deg, origin=rotation_origin)
        global_y_origin = rotated_whole.bounds[1]

        total_l = 0.0
        total_s_prime = 0.0
        all_sub_paths: list[list[tuple]] = []

        for sub_poly in sub_polygons:
            path, l, s_prime, _ = self.planner.generate_path(
                sub_poly, angle_deg,
                global_y_origin=global_y_origin,
                rotation_origin=rotation_origin,
                obstacles=obstacle_union,
            )
            total_l += l
            total_s_prime += s_prime
            if path:
                all_sub_paths.append(path)

        if assembler is None:
            assembler = PathAssembler(polygon)
        combined_path, ferry_dist = assembler.assemble_connected(all_sub_paths)
        total_l += ferry_dist

        return {
            "angle": angle_deg,
            "l": total_l,
            "s_prime": total_s_prime,
            "path": combined_path,
            "sweep_segments": assembler.sweep_segments,
            "ferry_segments": assembler.ferry_segments,
        }

    @staticmethod
    def _compute_fitness(
        l_norm: float, s_prime: float, target_area_S: float
    ) -> float:
        """
        Fitness function — Eq. 15:
            f_fitness = ( l_norm + |S' - S| / S )^{-1}

        Where l_norm is the L2-normalized flight distance (Eq. 14).
        """
        if target_area_S <= 0:
            return 1e-10
        extra_coverage = abs(s_prime - target_area_S) / target_area_S
        denominator = l_norm + extra_coverage
        if denominator < 1e-12:
            return 1e10
        return 1.0 / denominator

    # ------------------------------------------------------------------ #
    #                     GA Operators                                     #
    # ------------------------------------------------------------------ #

    @staticmethod
    def _tournament_selection(
        population: list[int], fitness_values: list[float], k: int = 3
    ) -> int:
        """Tournament selection: pick k random individuals, return the best."""
        indices = random.sample(range(len(population)), k)
        best = max(indices, key=lambda i: fitness_values[i])
        return population[best]

    @staticmethod
    def _blend_crossover(p1: int, p2: int) -> Tuple[int, int]:
        """
        Blend crossover respecting the circular [0, 180) topology.
        Uses the shortest arc between parents so children stay near both
        parents regardless of wraparound (e.g. 5° and 175°).
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

    # ------------------------------------------------------------------ #
    #                    Evaluation Cache                                  #
    # ------------------------------------------------------------------ #

    def _evaluate_population(
        self, population: list[int], polygon: Polygon,
        target_area_S: float, cache: dict,
        assembler: PathAssembler = None, obstacle_union=None,
    ) -> Tuple[list[dict], list[float], np.ndarray]:
        """
        Evaluate all individuals using a cache keyed by integer angle.
        Returns (raw_metrics, fitness_values, all_l) — all_l is the flight
        distance array used downstream for gen stats without re-extraction.
        """
        raw_metrics = []
        for angle in population:
            if angle not in cache:
                cache[angle] = self._evaluate_angle(
                    angle, polygon, target_area_S,
                    assembler=assembler, obstacle_union=obstacle_union,
                )
            raw_metrics.append(cache[angle])

        all_l = np.array([m["l"] for m in raw_metrics])
        l2_norm = np.sqrt(np.sum(all_l**2))
        if l2_norm < 1e-12:
            l2_norm = 1.0
        l_norms = all_l / l2_norm

        fitness_values = [
            self._compute_fitness(l_norms[i], raw_metrics[i]["s_prime"], target_area_S)
            for i in range(len(raw_metrics))
        ]
        return raw_metrics, fitness_values, all_l

    # ------------------------------------------------------------------ #
    #                        Main Loop (Algorithm 1)                      #
    # ------------------------------------------------------------------ #

    def optimize(
        self, polygon: Polygon
    ) -> Tuple[float, List[tuple], dict]:
        """
        Execute the GA.

        :param polygon: The operating area polygon (after margin reduction).
        :return: (best_angle, best_path, best_metrics)
        """
        target_area_S = polygon.area
        eval_cache: dict = {}

        assembler = PathAssembler(polygon)
        obstacle_union = build_obstacle_union(polygon)

        population = [random.randint(0, 179) for _ in range(self.pop_size)]

        best_solution = None
        best_fitness = -1.0
        no_improvement_count = 0
        gen_stats: list[dict] = []

        for gen in range(self.generations):
            # --- 1. Evaluate ---
            raw_metrics, fitness_values, all_l = self._evaluate_population(
                population, polygon, target_area_S, eval_cache,
                assembler=assembler, obstacle_union=obstacle_union,
            )

            # --- Stats (reuse all_l from evaluation) ---
            gen_stats.append({
                'gen': gen + 1,
                'mean_fitness': float(np.mean(fitness_values)),
                'mean_angle': float(np.mean(population)),
                'mean_l': float(np.mean(all_l)),
            })

            # --- 2. Track global best ---
            prev_best_fitness = best_fitness
            for i, f in enumerate(fitness_values):
                if f > best_fitness:
                    best_fitness = f
                    m = raw_metrics[i]
                    extra_cov = abs(m["s_prime"] - target_area_S) / target_area_S * 100
                    best_solution = {
                        "angle": m["angle"],
                        "fitness": f,
                        "l": m["l"],
                        "s_prime": m["s_prime"],
                        "extra_coverage_pct": extra_cov,
                        "path": m["path"],
                        "sweep_segments": m.get("sweep_segments", []),
                        "ferry_segments": m.get("ferry_segments", []),
                    }

            # --- 3. Selection (tournament, k=3) ---
            selected = [
                self._tournament_selection(population, fitness_values, k=3)
                for _ in range(self.pop_size)
            ]

            # --- 4. Crossover and Mutation ---
            kids = []
            i = 0
            while i < len(selected) - 1:
                p1, p2 = selected[i], selected[i + 1]
                if random.random() < self.crossover_rate:
                    c1, c2 = self._blend_crossover(p1, p2)
                else:
                    c1, c2 = p1, p2
                kids.append(self._mutate(c1))
                kids.append(self._mutate(c2))
                i += 2
            if i < len(selected):
                kids.append(self._mutate(selected[i]))

            # --- 5. Generational replacement with elitism ---
            elite = population[int(np.argmax(fitness_values))]
            population = kids[:self.pop_size]
            population[0] = elite

            # --- 6. Diversity injection ---
            if len(set(population)) < self.min_diversity:
                n_inject = self.pop_size // 5
                inject_indices = random.sample(range(1, self.pop_size), n_inject)
                for idx in inject_indices:
                    population[idx] = random.randint(0, 179)

            # --- 7. Early stopping ---
            if best_fitness > prev_best_fitness:
                no_improvement_count = 0
            else:
                no_improvement_count += 1
            if no_improvement_count >= self.early_stopping_patience:
                print(f"  Early stopping at gen {gen + 1}")
                break

            if (gen + 1) % 25 == 0:
                s = gen_stats[-1]
                print(
                    f"  Gen {gen+1}/{self.generations} | "
                    f"Best: {best_fitness:.4f} @ {best_solution['angle']}° | "
                    f"Pop mean: fit={s['mean_fitness']:.4f}  "
                    f"angle={s['mean_angle']:.1f}°  "
                    f"L={s['mean_l']:.0f}m"
                )

        print(
            f"SweepAngleOptimizer done. Best angle: {best_solution['angle']}°, "
            f"fitness: {best_fitness:.4f}, "
            f"flight dist: {best_solution['l']:.1f} m, "
            f"extra coverage: {best_solution['extra_coverage_pct']:.2f}%"
        )
        best_solution["gen_stats"] = gen_stats
        return best_solution["angle"], best_solution["path"], best_solution
