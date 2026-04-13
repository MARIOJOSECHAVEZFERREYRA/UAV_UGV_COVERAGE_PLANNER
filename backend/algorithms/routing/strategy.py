from abc import ABC, abstractmethod
from shapely.geometry import Polygon

from .sweep_angle_optimizer import SweepAngleOptimizer, build_obstacle_union
from ..coverage.path_planner import BoustrophedonPlanner
from ..coverage.path_assembler import PathAssembler


class MissionPlannerStrategy(ABC):
    @abstractmethod
    def optimize(self, polygon, swath_width, base_point=None,
                 energy_model=None, rendezvous_planner=None, w_rv=0.5):
        pass


class GeneticStrategy(MissionPlannerStrategy):
    def optimize(self, polygon, swath_width, base_point=None,
                 energy_model=None, rendezvous_planner=None, w_rv=0.5):
        planner = BoustrophedonPlanner(spray_width=swath_width)

        num_vertices = len(list(polygon.exterior.coords))
        poly_area = polygon.area

        if num_vertices <= 8 and poly_area <= 50000:
            params = {"pop_size": 200, "generations": 300}
        elif num_vertices <= 15 and poly_area <= 200000:
            params = {"pop_size": 150, "generations": 200}
        else:
            params = {"pop_size": 100, "generations": 150}

        optimizer = SweepAngleOptimizer(
            planner,
            pop_size=params["pop_size"],
            generations=params["generations"],
            early_stopping_patience=50,
            energy_model=energy_model,
            rendezvous_planner=rendezvous_planner,
            w_rv=w_rv,
        )

        best_solution = optimizer.optimize(polygon, base_point=base_point)

        return {
            "angle": best_solution.get("angle", 0.0),
            "route_segments": best_solution.get("route_segments", []),
            "combined_path": best_solution.get("combined_path", []),
            "planner_metrics": best_solution.get("planner_metrics", {}),
            "route_distances": best_solution.get("route_distances", {}),
            "rv_count": best_solution.get("rv_count", 0),
            "rv_wait": best_solution.get("rv_wait", 0.0),
            "metrics": {
                "fitness": best_solution.get("fitness"),
                "l": best_solution.get("l"),
                "s_prime": best_solution.get("s_prime"),
                "extra_coverage_pct": best_solution.get("extra_coverage_pct"),
            },
            "gen_stats": best_solution.get("gen_stats", []),
        }


class SimpleGridStrategy(MissionPlannerStrategy):
    def optimize(self, polygon, swath_width, base_point=None,
                 energy_model=None, rendezvous_planner=None, w_rv=0.5):
        planner = BoustrophedonPlanner(spray_width=swath_width)
        obstacle_union = build_obstacle_union(polygon)
        ugv_poly = rendezvous_planner.ugv_polyline if rendezvous_planner else None

        candidates = []

        for angle in [0.0, 90.0]:
            planner_result = planner.generate_path(
                polygon,
                angle,
                obstacles=obstacle_union,
            )

            sweep_segments = planner_result.get("sweep_segments", [])
            planner_metrics = planner_result.get("metrics", {})

            if not sweep_segments:
                continue

            assembler = PathAssembler(polygon, base_point=base_point)
            assembly_result = assembler.assemble_connected(sweep_segments)

            route_segments = assembly_result.get("route_segments", [])
            combined_path = assembly_result.get("combined_path", [])
            route_distances = assembly_result.get("distances", {})

            base_cost = 0.0
            if base_point is not None and route_segments:
                bp = (float(base_point[0]), float(base_point[1]))
                _, d_entry = assembler.find_connection(bp, route_segments[0]["path"][0])
                _, d_exit = assembler.find_connection(route_segments[-1]["path"][-1], bp)
                base_cost = d_entry + d_exit

            candidates.append({
                "angle": angle,
                "route_segments": route_segments,
                "combined_path": combined_path,
                "planner_metrics": planner_metrics,
                "route_distances": route_distances,
                "base_cost": base_cost,
                "metrics": {
                    "l": route_distances.get("total_m", 0.0),
                    "s_prime": planner_metrics.get("coverage_area_m2", 0.0),
                    "n_turns": planner_metrics.get("turn_count", 0),
                },
            })

        if not candidates:
            return {
                "angle": 0.0,
                "route_segments": [],
                "combined_path": [],
                "planner_metrics": {},
                "route_distances": {},
                "rv_count": 0,
                "rv_wait": 0.0,
                "metrics": {},
                "gen_stats": [],
            }

        best = min(candidates, key=lambda x: x["route_distances"].get("total_m", float("inf")) + x.get("base_cost", 0.0))

        return {
            "angle": best["angle"],
            "route_segments": best["route_segments"],
            "combined_path": best["combined_path"],
            "planner_metrics": best["planner_metrics"],
            "route_distances": best["route_distances"],
            "rv_count": 0,
            "rv_wait": 0.0,
            "metrics": best["metrics"],
            "gen_stats": [],
        }


class StrategyFactory:
    @staticmethod
    def get_strategy(name: str) -> MissionPlannerStrategy:
        if name.lower() == "genetic":
            return GeneticStrategy()
        elif name.lower() == "simple":
            return SimpleGridStrategy()
        else:
            raise ValueError(f"Unknown strategy: {name}")