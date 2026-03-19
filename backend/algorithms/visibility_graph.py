import math
import heapq
from shapely.geometry import Polygon, Point, LineString
from shapely.ops import substring
from collections import defaultdict
from typing import List, Tuple, Dict

class VisibilityGraph:
    """
    Constructs a Visibility Graph for a given polygon (including holes).
    Provides A* search to find the shortest collision-free path between points.
    """

    def __init__(self, polygon: Polygon):
        self.polygon = polygon
        # Buffer slightly inwards to avoid floating point precision issues on boundaries
        self.safe_polygon = polygon.buffer(-1e-6)
        # However, for pure line-of-sight checks, checking intersection with interior/exterior is safer
        self.vertices = self._extract_vertices(polygon)
        self.graph = self._build_graph(self.vertices)
        
    def _extract_vertices(self, polygon: Polygon) -> List[Tuple[float, float]]:
        """Extracts unique vertices from the exterior and all holes."""
        vertices = set()
        
        # Add exterior vertices
        for coord in polygon.exterior.coords[:-1]:  # Exclude last point (duplicate of first)
            vertices.add(tuple(coord))
            
        # Add holes vertices
        for hole in polygon.interiors:
            for coord in hole.coords[:-1]:
                vertices.add(tuple(coord))
                
        return list(vertices)

    def _is_visible(self, p1: Tuple[float, float], p2: Tuple[float, float]) -> bool:
        """
        Checks if line segment between p1 and p2 is fully inside the polygon
        (doesn't cross the exterior ring or intersect any holes).
        """
        line = LineString([p1, p2])
        
        # A line is valid if it lies entirely within the polygon
        # To avoid precision issues on the exact boundary, we use the interior
        # and intersection logic.
        
        # If the line is purely on the boundary, it's safe.
        # If it crosses outside, it's unsafe.
        
        # Shapely's `contains` might fail for lines perfectly on the boundary.
        # `within` is similar.
        # A safer check: the line must only intersect the polygon, and the intersection 
        # must equal the line itself (allowing for minor floating point diffs).
        
        # Fast check: midpoint must be inside
        mid_x = (p1[0] + p2[0]) / 2.0
        mid_y = (p1[1] + p2[1]) / 2.0
        if not self.safe_polygon.contains(Point(mid_x, mid_y)):
             # Even if midpoint is inside, it could cross out and back in.
             # but if midpoint is outside, it's definitely invalid.
             # Wait, a true boundary segment's midpoint might be slightly outside safe_polygon.
             if not self.polygon.contains(Point(mid_x, mid_y)):
                 return False

        # True geometric check: Does the line segment strictly lie within the polygon interior/boundary?
        # True if length of line inside polygon equals total length of line
        geom_intersection = self.polygon.intersection(line)
        return abs(geom_intersection.length - line.length) < 1e-6

    def _build_graph(self, vertices: List[Tuple[float, float]]) -> Dict[Tuple[float, float], Dict[Tuple[float, float], float]]:
        """Builds the adjacency list for the visibility graph."""
        graph = defaultdict(dict)
        n = len(vertices)
        
        for i in range(n):
            for j in range(i + 1, n):
                v1, v2 = vertices[i], vertices[j]
                if self._is_visible(v1, v2):
                    dist = math.hypot(v1[0] - v2[0], v1[1] - v2[1])
                    graph[v1][v2] = dist
                    graph[v2][v1] = dist
                    
        return graph

    def _add_query_point(self, pt: Tuple[float, float]) -> None:
        """Temporarily adds a start or end point to the graph."""
        if pt in self.graph:
            return
            
        self.vertices.append(pt)
        for v in self.vertices[:-1]: # Don't check against self
            if self._is_visible(pt, v):
                dist = math.hypot(pt[0] - v[0], pt[1] - v[1])
                self.graph[pt][v] = dist
                self.graph[v][pt] = dist

    def _remove_query_point(self, pt: Tuple[float, float], was_in_graph: bool) -> None:
        """Removes a temporarily added point from the graph to restore state."""
        if was_in_graph:
            return
            
        if pt in self.vertices:
            self.vertices.remove(pt)
        
        if pt in self.graph:    
            neighbors = list(self.graph[pt].keys())
            for n in neighbors:
                if pt in self.graph[n]:
                    del self.graph[n][pt]
            del self.graph[pt]

    def find_shortest_path(self, start: Tuple[float, float], end: Tuple[float, float]) -> Tuple[List[Tuple[float, float]], float]:
        """
        Uses A* search to find the shortest path between start and end.
        Returns: (path_waypoints, total_distance)
        """
        # Quick check: if direct line of sight is clear, just return it
        if self._is_visible(start, end):
            dist = math.hypot(start[0] - end[0], start[1] - end[1])
            return [start, end], dist

        start_was_in = start in self.graph
        end_was_in = end in self.graph

        self._add_query_point(start)
        self._add_query_point(end)

        try:
            path, dist = self._astar(start, end)
            return path, dist
        except ValueError:
            # Fallback: walk along the polygon boundary instead of straight line
            path, dist = self._boundary_walk_fallback(start, end)
            return path, dist
        finally:
            # Cleanup temporary nodes
            self._remove_query_point(start, start_was_in)
            self._remove_query_point(end, end_was_in)


    def _boundary_walk_fallback(self, start: Tuple[float, float], end: Tuple[float, float]) -> Tuple[List[Tuple[float, float]], float]:
        """Walk along the exterior boundary when A* finds no path."""
        ring_line = LineString(self.polygon.exterior.coords)
        d_start = ring_line.project(Point(start))
        d_end = ring_line.project(Point(end))
        total_len = ring_line.length

        if abs(d_start - d_end) < 1e-6:
            dist = math.hypot(start[0] - end[0], start[1] - end[1])
            return [start, end], dist

        # Compute both directions, pick shorter
        if d_start < d_end:
            fwd_len = d_end - d_start
            bwd_len = total_len - fwd_len
        else:
            bwd_len = d_start - d_end
            fwd_len = total_len - bwd_len

        if fwd_len <= bwd_len:
            if d_start < d_end:
                seg = substring(ring_line, d_start, d_end)
            else:
                seg1 = substring(ring_line, d_start, total_len)
                seg2 = substring(ring_line, 0, d_end)
                seg = LineString(list(seg1.coords) + list(seg2.coords)[1:])
        else:
            if d_end < d_start:
                seg_fwd = substring(ring_line, d_end, d_start)
            else:
                seg1 = substring(ring_line, d_end, total_len)
                seg2 = substring(ring_line, 0, d_start)
                seg_fwd = LineString(list(seg1.coords) + list(seg2.coords)[1:])
            coords = list(seg_fwd.coords)
            coords.reverse()
            seg = LineString(coords)

        path_coords = list(seg.coords)
        path_coords[0] = start
        path_coords[-1] = end
        return path_coords, seg.length

    def _astar(self, start: Tuple[float, float], goal: Tuple[float, float]) -> Tuple[List[Tuple[float, float]], float]:
        """A* algorithm implementation."""
        def heuristic(a: Tuple[float, float], b: Tuple[float, float]) -> float:
            return math.hypot(b[0] - a[0], b[1] - a[1])

        # Priority queue: (f_score, node)
        open_set = [(0, start)]
        
        came_from = {}
        
        # Cost from start along best known path
        g_score = defaultdict(lambda: float('inf'))
        g_score[start] = 0
        
        # Estimated total cost from start to goal through y
        f_score = defaultdict(lambda: float('inf'))
        f_score[start] = heuristic(start, goal)

        while open_set:
            _, current = heapq.heappop(open_set)

            if current == goal:
                # Reconstruct path
                path = []
                total_dist = g_score[goal]
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                path.reverse()
                return path, total_dist

            for neighbor, weight in self.graph[current].items():
                tentative_g_score = g_score[current] + weight

                if tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f = tentative_g_score + heuristic(neighbor, goal)
                    f_score[neighbor] = f
                    
                    # Add to open set if not already there
                    # We can just push it, heapq will sort it. Duplicate nodes in pq are fine since 
                    # we always pop the lowest f_score and g_score handles duplicates effectively.
                    heapq.heappush(open_set, (f, neighbor))

        raise ValueError("No valid path found in visibility graph")
