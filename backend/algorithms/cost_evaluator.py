from shapely.geometry import Polygon, Point, LineString
import numpy as np

class RouteCostEvaluator:
    """
    Cooperative cost evaluator (Drone + Truck).
    Decoupled for use in GA (Genetic Algorithms) and manual validation.
    """

    @staticmethod
    def calculate_perimeter_distance(polygon: Polygon, p1: tuple, p2: tuple) -> float:
        """
        Calculates the shortest distance traveling along the EXTERIOR of the polygon.
        Assumes the truck moves along the boundary (or a boundary offset).
        
        
        """
        ring = polygon.exterior
        
        # Project points to the ring (to ensure they are on the boundary)
        d1 = ring.project(Point(p1))
        d2 = ring.project(Point(p2))
        
        # Linear distance along the ring
        dist_linear = abs(d1 - d2)
        total_length = ring.length
        
        # The shortest distance on a ring is min(arc, total - arc)
        shortest_dist = min(dist_linear, total_length - dist_linear)
        
        return shortest_dist

    @staticmethod
    def calculate_total_truck_cost(polygon: Polygon, drone_path_segments: list) -> float:
        """
        Calculates the total truck cost by summing the movements necessary
        to connect the drone flight segments.
        
        :param drone_path_segments: List of point lists [[p_start1, ..., p_end1], [p_start2, ..., p_end2]]
                                    where each sublist is a route within a sub-polygon.
        :return: Total distance traveled by the truck (meters)
        """
        if len(drone_path_segments) < 2:
            return 0.0
            
        total_truck_dist = 0.0
        
        # Iterate between the end of one segment and the start of the next
        # 
        for i in range(len(drone_path_segments) - 1):
            segment_current = drone_path_segments[i]
            segment_next = drone_path_segments[i+1]
            
            if not segment_current or not segment_next:
                continue
                
            p_end_current = segment_current[-1]
            p_start_next = segment_next[0]
            
            dist = RouteCostEvaluator.calculate_perimeter_distance(polygon, p_end_current, p_start_next)
            total_truck_dist += dist
            
        return total_truck_dist