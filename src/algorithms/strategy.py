from abc import ABC, abstractmethod
from shapely.geometry import Polygon
from .genetic_optimizer import GeneticOptimizer
from .path_planner import BoustrophedonPlanner


class MissionPlannerStrategy(ABC):
    @abstractmethod
    def optimize(self, polygon: Polygon, swath_width: float) -> dict:
        pass

class GeneticStrategy(MissionPlannerStrategy):
    def optimize(self, polygon: Polygon, swath_width: float) -> dict:
        planner = BoustrophedonPlanner(spray_width=swath_width)

        num_vertices = len(list(polygon.exterior.coords))
        poly_area = polygon.area

        if num_vertices <= 8 and poly_area <= 50000:
            params = {'pop_size': 200, 'generations': 300, 'angle_discretization': 5.0}
        elif num_vertices <= 15 and poly_area <= 200000:
            params = {'pop_size': 150, 'generations': 200, 'angle_discretization': 5.0}
        else:
            params = {'pop_size': 100, 'generations': 150, 'angle_discretization': 10.0}

        optimizer = GeneticOptimizer(
            planner,
            pop_size=params['pop_size'],
            generations=params['generations'],
            angle_discretization=params['angle_discretization'],
            enable_caching=True,
            enable_early_stopping=True,
            early_stopping_patience=50,
        )

        best_angle, best_path, metrics = optimizer.optimize(polygon)

        return {
            'path': best_path,
            'angle': best_angle,
            'metrics': metrics,
            'gen_stats': metrics.get('gen_stats', []),
        }

class SimpleGridStrategy(MissionPlannerStrategy):
    def optimize(self, polygon: Polygon, swath_width: float) -> dict:
        planner = BoustrophedonPlanner(spray_width=swath_width)

        candidates = []
        for angle in [0.0, 90.0]:
            path, l, s_prime, turns = planner.generate_path(polygon, angle)
            candidates.append({
                'angle': angle,
                'path': path,
                'l': l,
                'metrics': {'angle': angle, 'l': l, 's_prime': s_prime, 'n_turns': turns}
            })

        valid = [c for c in candidates if c['path']]
        if not valid:
            return {'path': [], 'angle': 0.0, 'metrics': {}}

        best = min(valid, key=lambda x: x['l'])
        return {
            'path': best['path'],
            'angle': best['angle'],
            'metrics': best['metrics']
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
