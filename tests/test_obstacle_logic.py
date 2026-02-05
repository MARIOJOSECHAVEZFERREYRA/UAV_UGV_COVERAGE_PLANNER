import unittest
import numpy as np
from shapely.geometry import Polygon, LineString, Point
import matplotlib.pyplot as plt
import sys
import os

# Add src to path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../src')))

from algorithms.path_planner import BoustrophedonPlanner

class TestObstacleAvoidance(unittest.TestCase):
    def test_hole_avoidance(self):
        # 1. Create a 100x100 field with a 20x20 hole in the center
        # Field: (0,0) to (100,100)
        exterior = [(0, 0), (100, 0), (100, 100), (0, 100), (0, 0)]
        # Hole: (40,40) to (60,60)
        hole = [(40, 40), (60, 40), (60, 60), (40, 60), (40, 40)]
        # Shapely holes are counter-clockwise usually, but for interiors it's often CW.
        # Shapely doesn't strictly enforce it but it's good practice.
        # Let's just define it.
        
        poly = Polygon(exterior, [hole])
        
        # 2. Plan path
        # Spray with 10m -> lines at y=5, 15, ..., 95.
        # The hole is at y=40 to 60.
        # So lines at y=45 and y=55 should be cut.
        planner = BoustrophedonPlanner(spray_width=10.0)
        waypoints, length, area = planner.generate_path(poly, angle_deg=0.0)
        
        print(f"Generated {len(waypoints)} waypoints.")
        
        # 3. Visualize
        self.visualize(poly, waypoints, "obstacle_test.png")
        
        # 4. Assertions
        # Check if the path crosses the hole
        path_line = LineString(waypoints)
        hole_poly = Polygon(hole)
        
        # Intersection should be (almost) empty or just touching boundaries
        # Note: Boustrophedon lines touch the boundary of the hole.
        # Crossing strictly INSIDE is bad.
        # intersection area should be 0.
        
        intersection = path_line.intersection(hole_poly)
        # It's line vs polygon. Result is part of line inside.
        
        # Allow small tolerance for boundary touches
        if not intersection.is_empty:
            print("Intersection type:", intersection.geom_type)
            print("Intersection length:", intersection.length)
            # If implementation is correct, we hug the boundary, so we might "touch" it.
            # But we shouldn't have significant length INSIDE.
            # Actually, hugging the boundary means we ARE on the boundary.
            # So intersection length might be non-zero if the boundary matches exactly.
            # But the hole polygon includes its boundary.
            # The critical check is that we don't go THROUGH the hole.
            
            # Let's check if we have segments strictly inside.
            # Easier: Check if path intersects a slightly smaller version of the hole.
            buffered_hole = hole_poly.buffer(-0.1) # Shrink hole slightly
            strict_intersection = path_line.intersection(buffered_hole)
            
            self.assertTrue(strict_intersection.is_empty, "Path crosses strictly through the obstacle!")
            
    def visualize(self, poly, waypoints, filename):
        fig, ax = plt.subplots(figsize=(10, 10))
        
        # Plot Polygon
        x, y = poly.exterior.xy
        ax.plot(x, y, color='green', linewidth=3, label='Field')
        
        # Plot Holes
        for interior in poly.interiors:
            x, y = interior.xy
            ax.plot(x, y, color='red', linewidth=3, label='Obstacle')
            # Fill hole
            ax.fill(x, y, color='red', alpha=0.3)
            
        # Plot Path
        if waypoints:
            wx, wy = zip(*waypoints)
            ax.plot(wx, wy, color='blue', linewidth=1, marker='.', markersize=2, label='Path')
            
            # Highlight start/end
            ax.plot(wx[0], wy[0], 'go', label='Start')
            ax.plot(wx[-1], wy[-1], 'rx', label='End')
            
        ax.set_aspect('equal')
        ax.legend()
        plt.title("Obstacle Avoidance Test")
        plt.savefig(filename)
        print(f"Plot saved to {filename}")

if __name__ == '__main__':
    unittest.main()
