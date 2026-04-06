import math
import heapq
from collections import defaultdict

from shapely.geometry import Polygon, Point, LineString
from shapely.ops import substring


class VisibilityGraph:
    """
    Visibility graph for a polygon with optional holes.
    """

    def __init__(self, polygon: Polygon):
        self.polygon = polygon
        self.vertices = self._extract_vertices(polygon)
        self.graph = self._build_graph(self.vertices)

    @staticmethod
    def _pts_equal(a, b, tol=1e-6):
        return abs(a[0] - b[0]) < tol and abs(a[1] - b[1]) < tol

    def _extract_vertices(self, polygon):
        """Extract unique vertices from exterior and all holes, deterministically."""
        vertices = set()

        for coord in polygon.exterior.coords[:-1]:
            vertices.add((float(coord[0]), float(coord[1])))

        for hole in polygon.interiors:
            for coord in hole.coords[:-1]:
                vertices.add((float(coord[0]), float(coord[1])))

        return sorted(vertices)

    def _is_point_usable(self, pt):
        """Point must lie in or on the polygon boundary."""
        return self.polygon.covers(Point(pt))

    def _is_visible(self, p1, p2):
        """
        True if the segment p1->p2 lies entirely within or on the boundary
        of the polygon and does not pass through holes.
        """
        p1 = (float(p1[0]), float(p1[1]))
        p2 = (float(p2[0]), float(p2[1]))

        if self._pts_equal(p1, p2):
            return self._is_point_usable(p1)

        if not self._is_point_usable(p1) or not self._is_point_usable(p2):
            return False

        line = LineString([p1, p2])

        # Fast reject: midpoint must also be covered by the polygon
        mid = line.interpolate(0.5, normalized=True)
        if not self.polygon.covers(mid):
            return False

        # Main geometric test: every point of the line must be covered
        if self.polygon.covers(line):
            return True

        # Numerical fallback
        geom_intersection = self.polygon.intersection(line)
        return abs(geom_intersection.length - line.length) < 1e-7

    def _build_graph(self, vertices):
        """Build adjacency list of visible vertex pairs."""
        graph = defaultdict(dict)
        n = len(vertices)

        for i in range(n):
            for j in range(i + 1, n):
                v1 = vertices[i]
                v2 = vertices[j]
                if self._is_visible(v1, v2):
                    dist = math.hypot(v1[0] - v2[0], v1[1] - v2[1])
                    graph[v1][v2] = dist
                    graph[v2][v1] = dist

        return graph

    def _add_query_point(self, pt):
        """
        Temporarily add a start/end point to the graph.
        Returns True if it was added, False if it already existed.
        """
        pt = (float(pt[0]), float(pt[1]))

        if pt in self.vertices:
            return False

        if not self._is_point_usable(pt):
            raise ValueError("Query point lies outside the polygon.")

        self.vertices.append(pt)
        self.graph.setdefault(pt, {})

        for v in self.vertices[:-1]:
            if self._is_visible(pt, v):
                dist = math.hypot(pt[0] - v[0], pt[1] - v[1])
                self.graph[pt][v] = dist
                self.graph[v][pt] = dist

        return True

    def _remove_query_point(self, pt, was_added):
        """Remove a temporarily added query point and restore graph state."""
        pt = (float(pt[0]), float(pt[1]))

        if not was_added:
            return

        if pt in self.vertices:
            self.vertices.remove(pt)

        if pt in self.graph:
            neighbors = list(self.graph[pt].keys())
            for n in neighbors:
                if pt in self.graph[n]:
                    del self.graph[n][pt]
            del self.graph[pt]

    @staticmethod
    def _shortest_ring_walk(ring_line, d_start, d_end, start_pt, end_pt):
        """Build the shortest walk along a ring between two projected distances."""
        total_len = ring_line.length

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
        path_coords[0] = start_pt
        path_coords[-1] = end_pt
        return path_coords

    def _boundary_walk_fallback(self, start, end):
        """
        Walk along the exterior boundary only if both query points can be connected
        safely to their projected points on that boundary.
        """
        ring_line = LineString(self.polygon.exterior.coords)

        d_start = ring_line.project(Point(start))
        d_end = ring_line.project(Point(end))

        start_on_ring = tuple(ring_line.interpolate(d_start).coords[0])
        end_on_ring = tuple(ring_line.interpolate(d_end).coords[0])

        if not self._is_visible(start, start_on_ring):
            raise ValueError("No valid fallback path from start to boundary.")

        if not self._is_visible(end_on_ring, end):
            raise ValueError("No valid fallback path from boundary to end.")

        ring_path = self._shortest_ring_walk(
            ring_line,
            d_start,
            d_end,
            start_on_ring,
            end_on_ring,
        )

        full_path = [start]

        for pt in ring_path:
            if not self._pts_equal(full_path[-1], pt):
                full_path.append(pt)

        if not self._pts_equal(full_path[-1], end):
            full_path.append(end)

        total_distance = 0.0
        for i in range(len(full_path) - 1):
            total_distance += math.hypot(
                full_path[i + 1][0] - full_path[i][0],
                full_path[i + 1][1] - full_path[i][1],
            )

        return full_path, total_distance

    def find_shortest_path(self, start, end):
        """
        Find a shortest collision-free path from start to end.

        Returns:
            (path_waypoints, total_distance)
        """
        start = (float(start[0]), float(start[1]))
        end = (float(end[0]), float(end[1]))

        if not self._is_point_usable(start):
            raise ValueError("Start point lies outside the polygon.")

        if not self._is_point_usable(end):
            raise ValueError("End point lies outside the polygon.")

        if self._is_visible(start, end):
            dist = math.hypot(start[0] - end[0], start[1] - end[1])
            return [start, end], dist

        start_added = self._add_query_point(start)
        end_added = self._add_query_point(end)

        try:
            return self._astar(start, end)
        except ValueError:
            return self._boundary_walk_fallback(start, end)
        finally:
            self._remove_query_point(start, start_added)
            self._remove_query_point(end, end_added)

    def _astar(self, start, goal):
        """A* search on the visibility graph."""
        if start not in self.graph or goal not in self.graph:
            raise ValueError("Start or goal is not present in the graph.")

        def heuristic(a, b):
            return math.hypot(b[0] - a[0], b[1] - a[1])

        open_set = [(heuristic(start, goal), start)]
        came_from = {}

        g_score = defaultdict(lambda: float("inf"))
        g_score[start] = 0.0

        while open_set:
            _, current = heapq.heappop(open_set)

            if current == goal:
                path = [goal]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                path.reverse()
                return path, g_score[goal]

            for neighbor, weight in self.graph[current].items():
                tentative_g = g_score[current] + weight

                if tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score = tentative_g + heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score, neighbor))

        raise ValueError("No valid path found in visibility graph")