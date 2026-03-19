# evaluate_li_ga.py
#
# Benchmark del SweepAngleOptimizer contra búsqueda exhaustiva (0..359).
# Mide:
# - exactitud frente al óptimo global
# - número real de evaluaciones caras
# - diversidad poblacional por generación
# - estabilidad entre seeds
#
# Ajusta los imports de tu proyecto antes de usarlo.

import io
import os
import sys
import random
import statistics
from dataclasses import dataclass, field
from contextlib import redirect_stdout

# Allow running directly: python3 backend/algorithms/evaluate_li_ga.py
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", ".."))

import numpy as np
from shapely.geometry import Polygon

from backend.algorithms.sweep_angle_optimizer import SweepAngleOptimizer, build_obstacle_union
from backend.algorithms.path_assembler import PathAssembler


@dataclass
class GAProbe:
    expensive_calls: int = 0
    visited_angles: set = field(default_factory=set)
    population_uniques: list = field(default_factory=list)
    populations: list = field(default_factory=list)


def attach_probe(ga: SweepAngleOptimizer) -> GAProbe:
    """
    Instrumenta una instancia del GA sin tocar tu código fuente.
    Cuenta:
    - cuántas veces se llamó realmente a _evaluate_angle (evaluación cara)
    - qué ángulos únicos aparecieron
    - diversidad de la población por generación
    """
    probe = GAProbe()

    original_evaluate_angle = ga._evaluate_angle
    original_evaluate_population = ga._evaluate_population

    def wrapped_evaluate_angle(
        angle_deg, polygon, target_area_S, assembler=None, obstacle_union=None
    ):
        probe.expensive_calls += 1
        probe.visited_angles.add(angle_deg)
        return original_evaluate_angle(
            angle_deg,
            polygon,
            target_area_S,
            assembler=assembler,
            obstacle_union=obstacle_union,
        )

    def wrapped_evaluate_population(
        population, polygon, target_area_S, cache, assembler=None, obstacle_union=None
    ):
        probe.population_uniques.append(len(set(population)))
        probe.populations.append(tuple(population))
        return original_evaluate_population(
            population,
            polygon,
            target_area_S,
            cache,
            assembler=assembler,
            obstacle_union=obstacle_union,
        )

    ga._evaluate_angle = wrapped_evaluate_angle
    ga._evaluate_population = wrapped_evaluate_population
    return probe



def brute_force_reference(ga: SweepAngleOptimizer, polygon: Polygon):
    """
    Evalúa los 360 ángulos y construye una referencia exacta dentro
    del espacio discreto de tu implementación.
    """
    target_area_S = polygon.area
    assembler = PathAssembler(polygon)
    obstacle_union = build_obstacle_union(polygon)

    results = []
    for angle in range(180):
        m = ga._evaluate_angle(
            angle,
            polygon,
            target_area_S,
            assembler=assembler,
            obstacle_union=obstacle_union,
        )
        results.append(m)

    all_l = np.array([m["l"] for m in results], dtype=float)
    l2_norm = float(np.sqrt(np.sum(all_l ** 2)))
    if l2_norm < 1e-12:
        l2_norm = 1.0

    for m, l in zip(results, all_l):
        l_norm = float(l / l2_norm)
        coverage_error = abs(m["s_prime"] - target_area_S) / target_area_S
        global_fitness = ga._compute_fitness(l_norm, m["s_prime"], target_area_S)

        m["coverage_error"] = coverage_error
        m["global_fitness"] = global_fitness

    ranked = sorted(results, key=lambda x: x["global_fitness"], reverse=True)
    best = ranked[0]
    rank_by_angle = {m["angle"]: i + 1 for i, m in enumerate(ranked)}
    by_angle = {m["angle"]: m for m in results}

    return best, by_angle, rank_by_angle


def first_generation_containing_angle(populations, angle):
    generation = None
    for i, pop in enumerate(populations, start=1):
        if angle in pop:
            generation = i
            break
    return generation


def run_one_seed(factory, polygon: Polygon, seed: int, oracle_best, oracle_by_angle, oracle_rank):
    random.seed(seed)
    np.random.seed(seed)

    ga = factory()
    probe = attach_probe(ga)

    # silenciar prints del optimize() para que el benchmark no ensucie salida
    with redirect_stdout(io.StringIO()):
        best_angle, best_path, best_metrics = ga.optimize(polygon)

    target_area_S = polygon.area
    this_angle_oracle = oracle_by_angle[best_angle]

    distance_gap_pct = 0.0
    if oracle_best["l"] > 1e-12:
        distance_gap_pct = 100.0 * (best_metrics["l"] - oracle_best["l"]) / oracle_best["l"]

    this_cov_error = abs(best_metrics["s_prime"] - target_area_S) / target_area_S
    oracle_cov_error = oracle_best["coverage_error"]
    coverage_gap_pct_points = 100.0 * (this_cov_error - oracle_cov_error)

    first_gen_with_oracle = first_generation_containing_angle(
        probe.populations, oracle_best["angle"]
    )

    result = {
        "seed": seed,
        "ga_best_angle": best_angle,
        "ga_best_distance": best_metrics["l"],
        "ga_cov_error_pct": 100.0 * this_cov_error,
        "ga_extra_coverage_pct": best_metrics["extra_coverage_pct"],
        "oracle_best_angle": oracle_best["angle"],
        "oracle_best_distance": oracle_best["l"],
        "oracle_cov_error_pct": 100.0 * oracle_cov_error,
        "hit_global_optimum": int(best_angle == oracle_best["angle"]),
        "oracle_rank_of_ga_angle": oracle_rank[best_angle],
        "distance_gap_pct": distance_gap_pct,
        "coverage_gap_pct_points": coverage_gap_pct_points,
        "expensive_calls": probe.expensive_calls,
        "unique_expensive_angles": len(probe.visited_angles),
        "mean_population_diversity": statistics.mean(probe.population_uniques),
        "final_population_diversity": probe.population_uniques[-1],
        "first_gen_containing_oracle_angle": first_gen_with_oracle,
        "best_path_size": len(best_path) if best_path is not None else 0,
        "last_mean_fitness": best_metrics["gen_stats"][-1]["mean_fitness"],
        "last_mean_angle": best_metrics["gen_stats"][-1]["mean_angle"],
        "last_mean_l": best_metrics["gen_stats"][-1]["mean_l"],
        # para inspección más fina:
        "oracle_fitness_of_ga_angle": this_angle_oracle["global_fitness"],
        "oracle_cov_error_of_ga_angle": this_angle_oracle["coverage_error"],
    }

    return result


def summarize(results):
    n = len(results)

    hit_rate = sum(r["hit_global_optimum"] for r in results) / n
    mean_rank = statistics.mean(r["oracle_rank_of_ga_angle"] for r in results)
    mean_distance_gap = statistics.mean(r["distance_gap_pct"] for r in results)
    mean_unique_angles = statistics.mean(r["unique_expensive_angles"] for r in results)
    mean_expensive_calls = statistics.mean(r["expensive_calls"] for r in results)
    mean_diversity = statistics.mean(r["mean_population_diversity"] for r in results)

    print("\n===== RESUMEN =====")
    print(f"Corridas: {n}")
    print(f"Tasa de acierto exacto del óptimo global: {100*hit_rate:.1f}%")
    print(f"Ranking medio del ángulo hallado por GA: {mean_rank:.2f}")
    print(f"Gap medio de distancia vs óptimo: {mean_distance_gap:.3f}%")
    print(f"Ángulos caros únicos evaluados (media): {mean_unique_angles:.1f}")
    print(f"Llamadas caras reales a _evaluate_angle (media): {mean_expensive_calls:.1f}")
    print(f"Diversidad media de población: {mean_diversity:.1f}")

    print("\n===== DETALLE POR SEED =====")
    for r in results:
        print(
            f"seed={r['seed']:>2} | "
            f"GA={r['ga_best_angle']:>3}° | "
            f"oracle={r['oracle_best_angle']:>3}° | "
            f"hit={r['hit_global_optimum']} | "
            f"rank={r['oracle_rank_of_ga_angle']:>3} | "
            f"dist_gap={r['distance_gap_pct']:>7.3f}% | "
            f"unique={r['unique_expensive_angles']:>3} | "
            f"div_final={r['final_population_diversity']:>3}"
        )


def benchmark_ga(factory, polygon: Polygon, seeds=20):
    """
    factory: función sin argumentos que devuelve una instancia NUEVA del GA
    polygon: shapely Polygon
    seeds: cantidad de corridas con seeds distintas
    """
    oracle_ga = factory()
    oracle_best, oracle_by_angle, oracle_rank = brute_force_reference(oracle_ga, polygon)

    print("===== ORÁCULO EXHAUSTIVO =====")
    print(
        f"Mejor ángulo exacto: {oracle_best['angle']}° | "
        f"distancia={oracle_best['l']:.3f} | "
        f"coverage_error={100*oracle_best['coverage_error']:.4f}% | "
        f"global_fitness={oracle_best['global_fitness']:.6f}"
    )

    results = []
    for seed in range(seeds):
        row = run_one_seed(factory, polygon, seed, oracle_best, oracle_by_angle, oracle_rank)
        results.append(row)

    summarize(results)
    return results, oracle_best


# ----------------------------------------------------------------------
# EJEMPLO DE USO
# ----------------------------------------------------------------------
if __name__ == "__main__":
    import json
    from backend.algorithms.path_planner import BoustrophedonPlanner

    FIELD_JSON = os.path.join(
        os.path.dirname(__file__), "..", "..", "data", "test_fields", "stress_tests", "irregular_peninsula.json"
    )

    with open(FIELD_JSON) as f:
        data = json.load(f)

    polygon = Polygon(data["boundary"])
    planner = BoustrophedonPlanner(spray_width=5.0)

    factory = lambda: SweepAngleOptimizer(
        planner=planner,
        pop_size=200,
        generations=300,
        crossover_rate=0.4,
        mutation_rate=0.05,
        mutation_range=15,
    )

    results, oracle_best = benchmark_ga(factory, polygon, seeds=20)