"""Shortest path in ℝ² \\ holes.

Reduced visibility graph over hole vertices + A* with euclidean
heuristic. Used by PathAssembler for both ferries and deadheads.
"""

import heapq
import math
from collections import defaultdict

from shapely.geometry import LineString, Point, Polygon
from shapely.ops import unary_union
from shapely.prepared import prep


_COORD_QUANT = 1_000_000_000  # dedup vertices at 9 decimals
_POINT_TOL = 1e-9


def _quant(pt):
    return (round(pt[0] * _COORD_QUANT) / _COORD_QUANT,
            round(pt[1] * _COORD_QUANT) / _COORD_QUANT)


def _pts_equal(a, b, tol=1e-6):
    return abs(a[0] - b[0]) < tol and abs(a[1] - b[1]) < tol


class GeodesicSolver:
    """Shortest collision-free path between two points.

    Construction is O(V²) visibility tests (V = hole vertices, usually
    small). Each query is O(1) when the direct line is clear, otherwise
    A* on the visibility graph.
    """

    def __init__(self, holes=None):
        cleaned = [h for h in (holes or []) if h is not None and not h.is_empty]
        if cleaned:
            self._holes_union = unary_union(cleaned)
            self._prepared_holes = prep(self._holes_union)
        else:
            self._holes_union = None
            self._prepared_holes = None

        self._vertices = self._extract_hole_vertices(cleaned)
        self._base_graph = self._build_visibility_graph(self._vertices)

    @staticmethod
    def _extract_hole_vertices(holes):
        seen = set()
        out = []
        for h in holes:
            geoms = h.geoms if h.geom_type == 'MultiPolygon' else [h]
            for g in geoms:
                for ring in (g.exterior, *g.interiors):
                    for coord in list(ring.coords)[:-1]:
                        q = _quant(coord)
                        if q not in seen:
                            seen.add(q)
                            out.append((float(coord[0]), float(coord[1])))
        return out

    def _build_visibility_graph(self, vertices):
        graph = defaultdict(dict)
        n = len(vertices)
        for i in range(n):
            graph[vertices[i]]  # ensure node exists
            for j in range(i + 1, n):
                a, b = vertices[i], vertices[j]
                if self._is_visible(a, b):
                    d = math.hypot(a[0] - b[0], a[1] - b[1])
                    graph[a][b] = d
                    graph[b][a] = d
        return graph

    def _is_visible(self, p1, p2):
        """True if the open segment p1-p2 never enters a hole interior.

        Skimming a boundary or touching a vertex is allowed; only
        crossings with interior overlap are rejected.
        """
        if self._holes_union is None:
            return True
        if _pts_equal(p1, p2):
            return True

        line = LineString([p1, p2])
        if not self._prepared_holes.intersects(line):
            return True

        inter = line.intersection(self._holes_union)
        if inter.is_empty:
            return True

        # Interior overlap is empty iff the intersection is entirely
        # on the hole boundary (safe skim, no crossing).
        interior_overlap = inter.difference(self._holes_union.boundary)
        return interior_overlap.is_empty

    def shortest_path(self, start, end):
        """Return (path, distance) from start to end, avoiding holes.

        Endpoints in the result are exactly the inputs. When an endpoint
        lands numerically on or inside a hole (float drift from clipped
        sweeps), it is snapped just outside and retried; last resort
        returns the direct euclidean line with a warning.
        """
        start = (float(start[0]), float(start[1]))
        end = (float(end[0]), float(end[1]))

        if _pts_equal(start, end):
            return [start, end], 0.0

        if self._is_visible(start, end):
            return [start, end], math.hypot(end[0] - start[0], end[1] - start[1])

        try:
            return self._astar(start, end)
        except RuntimeError:
            pass

        # Retry with endpoints snapped just outside any containing hole.
        s_snap = self._snap_outside_holes(start)
        e_snap = self._snap_outside_holes(end)
        if s_snap != start or e_snap != end:
            try:
                path, dist = self._astar(s_snap, e_snap)
                # Insert the original endpoints as short hops at each
                # end; replacing the snapped points would create a line
                # from start to the A* path's second point that may
                # cross a hole.
                if path:
                    if tuple(start) != tuple(path[0]):
                        dist += math.hypot(
                            start[0] - path[0][0], start[1] - path[0][1],
                        )
                        path = [start] + path
                    if tuple(end) != tuple(path[-1]):
                        dist += math.hypot(
                            end[0] - path[-1][0], end[1] - path[-1][1],
                        )
                        path = path + [end]
                return path, dist
            except RuntimeError:
                pass

        # Last resort: direct line with a warning instead of raising.
        import warnings
        warnings.warn(
            "GeodesicSolver: A* failed from {} to {}; "
            "falling back to direct euclidean line".format(start, end),
            stacklevel=2,
        )
        return [start, end], math.hypot(end[0] - start[0], end[1] - start[1])

    _SNAP_EPSILON = 1e-4

    def _snap_outside_holes(self, pt):
        """Nudge `pt` just outside any hole that contains or touches it.

        Points strictly outside (distance > epsilon) are kept as-is.
        On-boundary points are snapped outward because visibility rays
        from a boundary point can all graze the interior.
        """
        if self._holes_union is None:
            return pt
        from shapely.geometry import Point
        p = Point(pt)
        if p.distance(self._holes_union) > self._SNAP_EPSILON:
            return pt

        boundary = self._holes_union.boundary
        from shapely.ops import nearest_points
        _, nearest = nearest_points(p, boundary)
        nx, ny = nearest.x, nearest.y
        dx = nx - pt[0]
        dy = ny - pt[1]
        norm = math.hypot(dx, dy)

        if norm < 1e-12:
            # pt lies exactly on the boundary; probe 8 cardinal
            # directions for the first that lands strictly outside.
            for step_dx, step_dy in ((1, 0), (-1, 0), (0, 1), (0, -1),
                                     (1, 1), (-1, 1), (1, -1), (-1, -1)):
                eps = self._SNAP_EPSILON * 10.0
                cand = (pt[0] + step_dx * eps, pt[1] + step_dy * eps)
                if Point(cand).distance(self._holes_union) > self._SNAP_EPSILON:
                    return cand
            return (pt[0] + self._SNAP_EPSILON, pt[1])

        ux = dx / norm
        uy = dy / norm
        return (nx + ux * self._SNAP_EPSILON, ny + uy * self._SNAP_EPSILON)

    def _astar(self, start, end):
        def h(pt):
            return math.hypot(end[0] - pt[0], end[1] - pt[1])

        # Adjacency is lazy: start/end get visibility-tested on demand
        # against the base graph.
        neighbors_of = {}

        def neighbors(node):
            if node in neighbors_of:
                return neighbors_of[node]
            if node is start or node == start:
                vis = {}
                for v in self._vertices:
                    if self._is_visible(start, v):
                        vis[v] = math.hypot(v[0] - start[0], v[1] - start[1])
                if self._is_visible(start, end):
                    vis[end] = math.hypot(end[0] - start[0], end[1] - start[1])
                neighbors_of[node] = vis
                return vis
            if node is end or node == end:
                vis = {}
                for v in self._vertices:
                    if self._is_visible(end, v):
                        vis[v] = math.hypot(v[0] - end[0], v[1] - end[1])
                neighbors_of[node] = vis
                return vis
            # Hole vertex: base neighbors + direct-to-end visibility.
            vis = dict(self._base_graph.get(node, {}))
            if self._is_visible(node, end):
                vis[end] = math.hypot(end[0] - node[0], end[1] - node[1])
            neighbors_of[node] = vis
            return vis

        open_set = [(h(start), 0.0, start)]
        came_from = {}
        g_score = {start: 0.0}
        closed = set()

        while open_set:
            _, g_cur, cur = heapq.heappop(open_set)
            if cur in closed:
                continue
            if cur == end:
                path = [end]
                while path[-1] in came_from:
                    path.append(came_from[path[-1]])
                path.reverse()
                path[0] = start
                path[-1] = end
                return path, g_cur
            closed.add(cur)

            for nb, w in neighbors(cur).items():
                if nb in closed:
                    continue
                tentative = g_cur + w
                if tentative < g_score.get(nb, math.inf):
                    g_score[nb] = tentative
                    came_from[nb] = cur
                    heapq.heappush(open_set, (tentative + h(nb), tentative, nb))

        raise RuntimeError(
            "GeodesicSolver: no path from {} to {} (endpoint "
            "likely enclosed by holes).".format(start, end)
        )
