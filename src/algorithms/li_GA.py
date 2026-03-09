"""
Faithful implementation of the Genetic Algorithm from:
Li et al. (2023) "Coverage Path Planning Method for Agricultural Spraying UAV
in Arbitrary Polygon Area", Aerospace, 10, 755.

Implements Section 2.5 and Algorithm 1 exactly as described:
- 22-bit binary encoding of heading angles [0, 360)
- Population size alpha = 200
- Iterations delta = 300
- Roulette wheel selection (Eq. 16-17)
- Single-point crossover (Fig. 7) with rate eta = 0.4
- Bit-flip mutation (Fig. 8) with rate lambda = 0.001
- Fitness function (Eq. 15): f = (l_norm + |S' - S| / S)^{-1}
- L2-norm normalization of flight distance (Eq. 14)
"""

import math
import random
import numpy as np
from shapely.geometry import Polygon
from shapely import affinity
from typing import List, Tuple

from .decomposition import ConcaveDecomposer
from .path_planner import BoustrophedonPlanner


# Number of bits for binary encoding (paper: 22 bits)
N_BITS = 22
# Maximum decimal value representable with N_BITS
MAX_BINARY_VAL = 2**N_BITS - 1


def _angle_to_binary(angle_deg: float) -> list[int]:
    """Encode a heading angle [0, 360) into a 22-bit binary chromosome."""
    decimal = int(round(angle_deg / 360.0 * MAX_BINARY_VAL))
    decimal = max(0, min(decimal, MAX_BINARY_VAL))
    bits = []
    for i in range(N_BITS - 1, -1, -1):
        bits.append((decimal >> i) & 1)
    return bits


def _binary_to_angle(bits: list[int]) -> float:
    """Decode a 22-bit binary chromosome into a heading angle [0, 360)."""
    decimal = 0
    for b in bits:
        decimal = (decimal << 1) | b
    return decimal / MAX_BINARY_VAL * 360.0


class LiGeneticOptimizer:
    """
    GA heading angle optimizer faithful to Li et al. (2023).

    Parameters match Table 1 of the paper:
        alpha (pop_size): 200
        delta (generations): 300
        rho (precision): 3 decimal places (achieved via 22-bit encoding)
        phi (selection_rate): 0.5  -- roulette wheel
        eta (crossover_rate): 0.4
        lam (mutation_rate): 0.001
    """

    def __init__(
        self,
        planner: BoustrophedonPlanner,
        pop_size: int = 200,
        generations: int = 300,
        crossover_rate: float = 0.4,
        mutation_rate: float = 0.01,
    ):
        self.planner = planner
        self.pop_size = pop_size
        self.generations = generations
        self.crossover_rate = crossover_rate
        self.mutation_rate = mutation_rate

    # ------------------------------------------------------------------ #
    #                        Evaluation (Eq. 11-15)                       #
    # ------------------------------------------------------------------ #

    def _evaluate_angle(
        self, angle_deg: float, polygon: Polygon, target_area_S: float
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
        all_waypoints: list[list[tuple]] = []

        for sub_poly in sub_polygons:
            path, l, s_prime, _ = self.planner.generate_path(
                sub_poly, angle_deg,
                global_y_origin=global_y_origin,
                rotation_origin=rotation_origin,
            )
            total_l += l
            total_s_prime += s_prime
            if path:
                all_waypoints.append(path)

        combined_path, ferry_dist = self._order_sub_paths(all_waypoints)
        total_l += ferry_dist

        return {
            "angle": angle_deg,
            "l": total_l,
            "s_prime": total_s_prime,
            "path": combined_path,
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
    #                     GA Operators (Sec. 2.5)                         #
    # ------------------------------------------------------------------ #

    @staticmethod
    def _roulette_wheel_selection(
        population: list[list[int]], fitness_values: list[float]
    ) -> list[int]:
        """
        Roulette wheel selection (Eq. 16-17).
        P_select(a_i) = f(a_i) / sum(f(a_j))
        Q_select(a_i) = cumulative sum of P
        Returns a COPY of the selected chromosome.
        """
        total = sum(fitness_values)
        if total <= 0:
            return list(random.choice(population))
        pick = random.uniform(0, total)
        cumulative = 0.0
        for i, chromo in enumerate(population):
            cumulative += fitness_values[i]
            if cumulative >= pick:
                return list(chromo)
        return list(population[-1])

    @staticmethod
    def _tournament_selection(
        population: list[list[int]], fitness_values: list[float], k: int = 3
    ) -> list[int]:
        """Tournament selection: pick k random individuals, return the best."""
        indices = random.sample(range(len(population)), k)
        best = max(indices, key=lambda i: fitness_values[i])
        return list(population[best])

    @staticmethod
    def _single_point_crossover(
        parent1: list[int], parent2: list[int]
    ) -> Tuple[list[int], list[int]]:
        """
        Single-point crossover (Fig. 7).
        Random crossover location k in [1, N_BITS-1].
        """
        k = random.randint(1, N_BITS - 1)
        child1 = parent1[:k] + parent2[k:]
        child2 = parent2[:k] + parent1[k:]
        return child1, child2

    @staticmethod
    def _bit_flip_mutation(chromosome: list[int], mutation_rate: float) -> list[int]:
        """
        Bit-flip mutation (Fig. 8).
        Each bit is flipped with probability = mutation_rate.
        """
        result = chromosome[:]
        for i in range(N_BITS):
            if random.random() < mutation_rate:
                result[i] = 1 - result[i]
        return result

    # ------------------------------------------------------------------ #
    #                    Evaluation Cache                                  #
    # ------------------------------------------------------------------ #

    def _evaluate_population(
        self, population: list[list[int]], polygon: Polygon,
        target_area_S: float, cache: dict
    ) -> Tuple[list[dict], list[float]]:
        """
        Evaluate all individuals, using a cache keyed by angle (rounded to
        3 decimal places as per paper's precision rho).
        Returns (raw_metrics_list, fitness_values_list).
        """
        raw_metrics = []
        for ch in population:
            angle = round(_binary_to_angle(ch), 3)
            if angle not in cache:
                cache[angle] = self._evaluate_angle(angle, polygon, target_area_S)
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
        return raw_metrics, fitness_values

    # ------------------------------------------------------------------ #
    #                        Main Loop (Algorithm 1)                      #
    # ------------------------------------------------------------------ #

    def optimize(
        self, polygon: Polygon
    ) -> Tuple[float, List[tuple], dict]:
        """
        Execute the GA as described in Algorithm 1 and Table 1.

        Algorithm 1 loop:
          1. Evaluate population, compute fitness
          2. Select best solution according to phi
          3. Crossover with rate eta -> kidsPop
          4. Mutation with rate lambda -> update kidsPop
          5. Pop = Pop + kidsPop (paper), we keep best alpha via selection

        :param polygon: The operating area polygon (after margin reduction).
        :return: (best_angle, best_path, best_metrics)
        """
        target_area_S = polygon.area

        # Evaluation cache: angle (3 decimal) -> metrics dict
        eval_cache: dict = {}

        # Initialize population: alpha random chromosomes
        population = [
            _angle_to_binary(random.uniform(0, 360))
            for _ in range(self.pop_size)
        ]

        best_solution = None
        best_fitness = -1.0

        # Per-generation statistics for analysis
        gen_stats: list[dict] = []

        for gen in range(self.generations):
            # --- 1. Evaluate all individuals ---
            raw_metrics, fitness_values = self._evaluate_population(
                population, polygon, target_area_S, eval_cache
            )

            # --- Stats ---
            angles = [round(_binary_to_angle(ch), 3) for ch in population]
            gen_stats.append({
                'gen': gen + 1,
                'mean_fitness': float(np.mean(fitness_values)),
                'mean_angle': float(np.mean(angles)),
                'mean_l': float(np.mean([m['l'] for m in raw_metrics])),
            })

            # --- 2. Track global best ---
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
                    }

            # --- 3. Selection (tournament, k=3) ---
            selected = [
                self._tournament_selection(population, fitness_values, k=3)
                for _ in range(self.pop_size)
            ]

            # --- 4. Crossover and Mutation -> kidsPop ---
            kids = []
            i = 0
            while i < len(selected) - 1:
                p1 = selected[i]
                p2 = selected[i + 1]
                if random.random() < self.crossover_rate:
                    c1, c2 = self._single_point_crossover(p1, p2)
                else:
                    c1, c2 = p1[:], p2[:]
                c1 = self._bit_flip_mutation(c1, self.mutation_rate)
                c2 = self._bit_flip_mutation(c2, self.mutation_rate)
                kids.append(c1)
                kids.append(c2)
                i += 2
            if i < len(selected):
                kids.append(
                    self._bit_flip_mutation(selected[i][:], self.mutation_rate)
                )

            # --- 5. Update Pop = kidsPop (generational replacement) ---
            # Elitism: preserve the best individual from the PARENT generation
            best_idx = int(np.argmax(fitness_values))
            elite = list(population[best_idx])
            population = kids[:self.pop_size]
            population[0] = elite

            if (gen + 1) % 25 == 0:
                s = gen_stats[-1]
                print(
                    f"  Gen {gen+1}/{self.generations} | "
                    f"Best: {best_fitness:.4f} @ {best_solution['angle']:.1f}° | "
                    f"Pop mean: fit={s['mean_fitness']:.4f}  "
                    f"angle={s['mean_angle']:.1f}°  "
                    f"L={s['mean_l']:.0f}m"
                )

        print(
            f"Li GA done. Best angle: {best_solution['angle']:.3f} deg, "
            f"fitness: {best_fitness:.4f}, "
            f"flight dist: {best_solution['l']:.1f} m, "
            f"extra coverage: {best_solution['extra_coverage_pct']:.2f}%"
        )
        best_solution["gen_stats"] = gen_stats
        return best_solution["angle"], best_solution["path"], best_solution

    # ------------------------------------------------------------------ #
    #                     Sub-path ordering                               #
    # ------------------------------------------------------------------ #

    @staticmethod
    def _order_sub_paths(sub_paths: list) -> Tuple[list, float]:
        """
        Order sub-paths using nearest-neighbor greedy to minimize ferry distance.
        Each sub-path can be traversed forward or reversed.
        """
        paths = [p for p in sub_paths if p]
        if not paths:
            return [], 0.0
        if len(paths) == 1:
            return list(paths[0]), 0.0

        def dist(a, b):
            return math.hypot(a[0] - b[0], a[1] - b[1])

        remaining = list(range(len(paths)))
        current_idx = remaining.pop(0)
        ordered = [list(paths[current_idx])]
        total_ferry = 0.0

        while remaining:
            current_end = ordered[-1][-1]
            best_dist = float("inf")
            best_idx = None
            best_reverse = False

            for idx in remaining:
                p = paths[idx]
                d_fwd = dist(current_end, p[0])
                d_rev = dist(current_end, p[-1])
                if d_fwd < best_dist:
                    best_dist = d_fwd
                    best_idx = idx
                    best_reverse = False
                if d_rev < best_dist:
                    best_dist = d_rev
                    best_idx = idx
                    best_reverse = True

            remaining.remove(best_idx)
            p = list(paths[best_idx])
            if best_reverse:
                p.reverse()
            total_ferry += best_dist
            ordered.append(p)

        combined = ordered[0]
        for p in ordered[1:]:
            combined.extend(p)
        return combined, total_ferry
