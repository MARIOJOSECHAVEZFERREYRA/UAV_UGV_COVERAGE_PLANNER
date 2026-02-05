from abc import ABC, abstractmethod
from shapely.geometry import Polygon, LineString
import math
from .genetic_optimizer import GeneticOptimizer
from .genetic_optimizer import GeneticOptimizer
from .path_planner import BoustrophedonPlanner
from .decomposition import ConcaveDecomposer

class MissionPlannerStrategy(ABC):
    """
    Abstract Base Class for optimization strategies.
    Defines the contract for any algorithm that generates a flight path.
    """
    @abstractmethod
    def optimize(self, polygon: Polygon, swath_width: float, truck_route: LineString = None) -> dict:
        """
        Executes the optimization algorithm.
        
        Args:
            polygon (Polygon): The safe field boundary.
            swath_width (float): Effective spray width in meters.
            truck_route (LineString, optional): Constraint for logistics.

        Returns:
            dict: {
                'path': List[tuple],      # The optimized flight path points
                'angle': float,           # The selected angle
                'metrics': dict           # Performance metrics (fitness, distance, etc)
            }
        """
        pass

class GeneticStrategy(MissionPlannerStrategy):
    """
    Uses a Genetic Algorithm to find the optimal flight angle.
    Best for complex polygons.
    """
    def optimize(self, polygon: Polygon, swath_width: float, truck_route: LineString = None) -> dict:
        # Instantiate Planner and Optimizer
        planner = BoustrophedonPlanner(spray_width=swath_width)
        
        # Adaptive parameters based on complexity
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
            enable_parallelization=False 
        )
        
        best_angle, best_path, metrics = optimizer.optimize(polygon, truck_route=truck_route)
        
        return {
            'path': best_path,
            'angle': best_angle,
            'metrics': metrics
        }

class SimpleGridStrategy(MissionPlannerStrategy):
    """
    Fast strategy that checks only 0° and 90° angles.
    Useful for quick previews or very simple rectangular fields.
    """
    def optimize(self, polygon: Polygon, swath_width: float, truck_route: LineString = None) -> dict:
        planner = BoustrophedonPlanner(spray_width=swath_width)
        
        candidates = []
        for angle in [0.0, 90.0]:
            path, l, s_prime = planner.generate_path(polygon, angle)
            # Simple fitness: minimize distance (l)
            # Or coverage error?
            candidates.append({
                'angle': angle,
                'path': path,
                'l': l,
                'metrics': {'angle': angle, 'l': l, 's_prime': s_prime}
            })
            
        # Select best (min l) - assuming simple coverage maximization is met
        # If path is empty, l will be 0. We should penalize empty paths.
        valid = [c for c in candidates if c['path']]
        if not valid:
            # Return empty
             return {
                'path': [],
                'angle': 0.0,
                'metrics': {}
            }
            
        best = min(valid, key=lambda x: x['l'])
        
        return {
            'path': best['path'],
            'angle': best['angle'],
            'metrics': best['metrics']
        }

class DecompositionStrategy(MissionPlannerStrategy):
    """
    Formal Strategy: Cellular Decomposition.
    Splits the field into convex cells to handle obstacles topologically.
    """
    def optimize(self, polygon: Polygon, swath_width: float, truck_route: LineString = None) -> dict:
        # 1. Decompose (Fixed Angle 0 for now)
        # In a full valid implementation, we might optimize this angle too.
        angle = 0.0
        
        # Decompose using the existing Algorithm Phase 2
        sub_polygons = ConcaveDecomposer.decompose(polygon, angle)
        
        full_path = []
        metrics = {'angle': angle, 'cell_count': len(sub_polygons)}
        
        planner = BoustrophedonPlanner(spray_width=swath_width)
        
        # 2. Plan each cell
        # Simple chaining (Order of decomposition)
        for sub in sub_polygons:
            # Skip tiny cells
            if sub.area < 1.0: continue
            
            path, _, _ = planner.generate_path(sub, angle)
            if not path: continue
            
            # Connection Logic (Naive Direct)
            # Optimization: Connect end of prev to closest start of next? 
            # Or just append. BoustrophedonPlanner returns a list of points.
            
            full_path.extend(path)
            
        return {
            'path': LineString(full_path) if len(full_path) > 1 else None,
            'angle': angle,
            'metrics': metrics
        }

class StrategyFactory:
    """
    Factory to create strategies based on name.
    """
    @staticmethod
    def get_strategy(name: str) -> MissionPlannerStrategy:
        if name.lower() == "genetic":
            return GeneticStrategy()
        elif name.lower() == "simple":
            return SimpleGridStrategy()
        elif name.lower() == "decomposition":
            return DecompositionStrategy()
        else:
            raise ValueError(f"Unknown strategy: {name}")
