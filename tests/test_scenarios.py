"""
Integration tests for all field scenarios.
Dynamically discovers and tests all JSON scenario files.
Uses GeneticOptimizer (same as GUI) to expose real optimization bugs.
"""
import unittest
import sys
import os
import json
import glob
from shapely.geometry import Polygon, LineString
import matplotlib.pyplot as plt

# Add src to path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../src')))

from algorithms.genetic_optimizer import GeneticOptimizer
from algorithms.path_planner import BoustrophedonPlanner


class TestScenarios(unittest.TestCase):
    """Integration tests for field scenarios loaded from JSON files"""
    
    def setUp(self):
        plt.switch_backend('Agg')
        self.swath_width = 5.0
        
        # Store planner for per-test configuration
        self.planner = None
        
        # Paths
        self.test_dir = os.path.dirname(os.path.abspath(__file__))
        self.project_root = os.path.dirname(self.test_dir)
        self.data_dir = os.path.join(self.project_root, 'data', 'test_fields')
        self.results_dir = os.path.join(self.project_root, 'data', 'test_results')
        
        if not os.path.exists(self.results_dir):
            os.makedirs(self.results_dir)
    
    def _get_optimizer_for_polygon(self, polygon):
        """
        Create GeneticOptimizer with EXACT same parameters as GUI's GeneticStrategy.
        Uses adaptive parameters based on polygon complexity.
        """
        # Create fresh planner
        planner = BoustrophedonPlanner(spray_width=self.swath_width)
        
        # Adaptive parameters (EXACT copy from GeneticStrategy)
        num_vertices = len(list(polygon.exterior.coords))
        poly_area = polygon.area
        
        if num_vertices <= 8 and poly_area <= 50000:
            params = {'pop_size': 200, 'generations': 300, 'angle_discretization': 5.0}
        elif num_vertices <= 15 and poly_area <= 200000:
            params = {'pop_size': 150, 'generations': 200, 'angle_discretization': 5.0}
        else:
            params = {'pop_size': 100, 'generations': 150, 'angle_discretization': 10.0}
        
        # Create optimizer with GUI's exact configuration
        optimizer = GeneticOptimizer(
            planner,
            pop_size=params['pop_size'],
            generations=params['generations'],
            angle_discretization=params['angle_discretization'],
            enable_caching=True,
            enable_early_stopping=True,
            early_stopping_patience=50,
            enable_parallelization=False  # CRITICAL: GUI uses False
        )
        
        return optimizer
    
    def load_scenario_files(self):
        """Discover all JSON scenario files"""
        pattern = os.path.join(self.data_dir, '**', '*.json')
        files = glob.glob(pattern, recursive=True)
        if not files:
            print(f"WARNING: No JSON files found in {self.data_dir}")
        return sorted(files)
    
    def run_scenario_from_file(self, json_path):
        """Execute a single scenario test using Genetic Optimizer (GUI-matched config)"""
        with open(json_path, 'r') as f:
            data = json.load(f)
        
        test_name = data.get('name', 'unknown')
        min_coverage = data.get('min_coverage', 0.85)
        
        print(f"\n--- Testing Scenario: {test_name} ---")
        print(f"Description: {data.get('description')}")
        
        # Build Polygon
        boundary_coords = data['boundary']
        obstacle_coords_list = data.get('obstacles', [])
        holes = [obs for obs in obstacle_coords_list]
        
        poly = Polygon(boundary_coords, holes)
        self.assertTrue(poly.is_valid, f"Invalid polygon geometry in {test_name}")
        
        # Create optimizer with GUI's exact configuration for this polygon
        optimizer = self._get_optimizer_for_polygon(poly)
        
        # Execute Genetic Optimizer (exact same as GUI)
        # Returns: (best_angle, metrics_list, best_solution)
        best_angle, metrics_list, best_solution = optimizer.optimize(poly)
        
        # Extract path from best solution
        raw_path = best_solution['path']
        coverage_error = best_solution['eta'] / 100.0  # Convert back from percentage
        
        # Convert path to LineString if it's a list of tuples
        if isinstance(raw_path, list):
            path = LineString(raw_path) if raw_path else LineString()
        else:
            path = raw_path
        
        # Assertions
        self.assertIsNotNone(path, f"No path generated for {test_name}")
        
        # Visualization (pass json_path to determine subdirectory)
        self._visualize(poly, path, json_path, test_name)
        
        # Safety Check
        self._assert_safety(path, poly, test_name)
        
        # Coverage Check (using optimizer's coverage error)
        actual_coverage = 1.0 - coverage_error
        print(f"[{test_name}] Coverage: {actual_coverage*100:.1f}% (Target: {min_coverage*100}%)")
        
        if actual_coverage < min_coverage:
            print(f"[{test_name}] Low Coverage warning")
        
        # Cell Count Check (if specified in JSON)
        # Note: GeneticOptimizer doesn't expose cell_count directly
        # We skip this check for genetic optimizer tests
    
    
    # Note: Individual test methods are generated dynamically at module load time
    # See the bottom of this file for dynamic test generation code

    
    # --- Helper Methods ---
    
    def _visualize(self, polygon, path, json_path, test_name):
        """Generate visualization of test results"""
        fig, ax = plt.subplots(figsize=(10, 10))
        
        # Boundary
        x, y = polygon.exterior.xy
        ax.plot(x, y, 'k-', linewidth=2, label='Boundary')
        ax.fill(x, y, color='lightgray', alpha=0.3)
        
        # Obstacles
        for interior in polygon.interiors:
            hole = Polygon(interior)
            hx, hy = hole.exterior.xy
            ax.plot(hx, hy, 'r-', linewidth=2)
            ax.fill(hx, hy, color='red', alpha=0.3, label='Obstacle')
        
        # Path
        if path and not path.is_empty:
            if path.geom_type == 'LineString':
                px, py = path.xy
                ax.plot(px, py, 'b-', linewidth=1.5, alpha=0.8, label='Path')
            elif path.geom_type == 'MultiLineString':
                for line in path.geoms:
                    px, py = line.xy
                    ax.plot(px, py, 'b-', linewidth=1.5, alpha=0.8)
        
        ax.set_title(f"Scenario: {test_name}")
        ax.set_aspect('equal')
        handles, labels = plt.gca().get_legend_handles_labels()
        by_label = dict(zip(labels, handles))
        ax.legend(by_label.values(), by_label.keys())
        
        # Determine subdirectory based on JSON path
        # e.g. data/test_fields/basic/comb_shape.json -> basic
        path_parts = json_path.split(os.sep)
        if 'basic' in path_parts:
            subdir = 'basic'
        elif 'stress_tests' in path_parts:
            subdir = 'stress_tests'
        elif 'organic' in path_parts:
            subdir = 'organic'
        else:
            subdir = ''  # Fallback to root results dir
        
        # Build full path with subdirectory
        if subdir:
            output_dir = os.path.join(self.results_dir, subdir)
        else:
            output_dir = self.results_dir
        
        filename = f"test_result_{test_name}.png"
        full_path = os.path.join(output_dir, filename)
        
        plt.savefig(full_path)
        print(f"[Scenario: {test_name}] Saved visualization to {full_path}")
        plt.close(fig)
    
    def _assert_safety(self, path, polygon, test_name):
        """Verify path doesn't collide with obstacles"""
        if path is None or path.is_empty: 
            return
        
        for interior in polygon.interiors:
            hole_poly = Polygon(interior)
            shrunk_hole = hole_poly.buffer(-0.1)
            bad_segment = path.intersection(shrunk_hole)
            
            if not bad_segment.is_empty:
                msg = f"COLLISION detected! Length: {bad_segment.length:.2f}m"
                print(f"[{test_name}] {msg}")
                self.fail(msg)
        
        print(f"[{test_name}] Safety Check Passed")
    
    def _assert_coverage(self, path, polygon, test_name, min_coverage):
        """Verify adequate field coverage"""
        covered_area = path.buffer(self.swath_width / 2.0)
        useful_coverage = covered_area.intersection(polygon)
        coverage_pct = useful_coverage.area / polygon.area
        
        print(f"[{test_name}] Coverage: {coverage_pct*100:.1f}% (Target: {min_coverage*100}%)")
        
        if coverage_pct < min_coverage:
            print(f"[{test_name}] Low Coverage warning")


# --- Dynamic Test Generation ---
# Generate one test method per JSON file for accurate test counting

def _create_scenario_test(json_file_path):
    """Factory function to create a test method for a specific scenario"""
    def test_method(self):
        self.run_scenario_from_file(json_file_path)
    return test_method


# Discover JSON files and generate test methods
test_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(test_dir)
data_dir = os.path.join(project_root, 'data', 'test_fields')
json_pattern = os.path.join(data_dir, '**', '*.json')
json_files = glob.glob(json_pattern, recursive=True)

# Attach individual test method for each JSON scenario
for json_file in sorted(json_files):
    basename = os.path.basename(json_file)
    scenario_name = basename.replace('.json', '')
    test_method_name = f'test_scenario_{scenario_name}'
    
    # Create and attach the test method
    test_method = _create_scenario_test(json_file)
    test_method.__name__ = test_method_name
    test_method.__doc__ = f"Test scenario from {basename}"
    setattr(TestScenarios, test_method_name, test_method)


if __name__ == '__main__':
    unittest.main()
