"""Assemble sweep segments into a connected mission route.

Thin facade over GeodesicSolver (shortest path) and SweepSequencer
(sweep ordering). Preserves the public contract of the previous
cell-based implementation so callers (SweepAngleOptimizer, StrategyFactory,
RendezvousPlanner, pipeline_visual) do not need to change.

Navigation space is ℝ² minus the polygon's holes — the drone is free to
fly over the margin strip and outside the field, only physical obstacles
(interior rings) matter for path clearance.
"""

from shapely.geometry import LineString, Polygon
from shapely.ops import unary_union

from .geodesic_solver import GeodesicSolver
from .sweep_sequencer import SweepSequencer


class PathAssembler:
    """Facade: sequences sweeps and emits typed route segments with ferries.

    Constructor preserves the previous signature for caller compatibility.
    The sub_polygons argument is accepted (legacy shape used by the optimizer
    and visualizer) but only consulted as a source of holes when
    original_polygon is not provided.
    """

    def __init__(self, sub_polygons, original_polygon=None,
                 sequencer_mode='full', base_point=None):
        holes = self._extract_holes(sub_polygons, original_polygon)
        self._solver = GeodesicSolver(holes=holes)

        # Precompute cell adjacency when we have multiple sub_polygons.
        # The sequencer uses it to prefer topologically adjacent cells
        # during NN, which keeps consecutive mission cycles contiguous.
        cell_adjacency = self._build_cell_adjacency(sub_polygons)

        self._sequencer = SweepSequencer(
            self._solver,
            mode=sequencer_mode,
            base_point=base_point,
            cell_adjacency=cell_adjacency,
        )

    # ------------------------------------------------------------------
    # Cell adjacency
    # ------------------------------------------------------------------

    @staticmethod
    def _build_cell_adjacency(sub_polygons):
        """Return a dict mapping cell index to the set of adjacent cell indices.

        Two cells are adjacent when their boundaries share a line segment
        of positive length (not just a point). Returns None if sub_polygons
        is not a list of multiple polygons.
        """
        if not isinstance(sub_polygons, (list, tuple)):
            return None
        if len(sub_polygons) < 2:
            return None

        adj = {i: set() for i in range(len(sub_polygons))}
        for i in range(len(sub_polygons)):
            pi = sub_polygons[i]
            if not isinstance(pi, Polygon):
                continue
            for j in range(i + 1, len(sub_polygons)):
                pj = sub_polygons[j]
                if not isinstance(pj, Polygon):
                    continue
                shared = pi.boundary.intersection(pj.boundary)
                if shared.is_empty:
                    continue
                length = getattr(shared, 'length', 0.0)
                if length > 1e-6:
                    adj[i].add(j)
                    adj[j].add(i)
        return adj

    # ------------------------------------------------------------------
    # Hole extraction
    # ------------------------------------------------------------------

    @staticmethod
    def _extract_holes(sub_polygons, original_polygon):
        """Return a list of Polygon objects representing physical obstacles.

        Priority:
          1. original_polygon.interiors (authoritative — these are the
             safe_polygon holes set by MissionPlanner).
          2. sub_polygons if it is a single Polygon with interiors.
          3. Empty list — no obstacles.
        """
        source = None
        if original_polygon is not None:
            source = original_polygon
        elif isinstance(sub_polygons, Polygon):
            source = sub_polygons

        if source is None:
            return []

        if source.geom_type == 'MultiPolygon':
            interiors = []
            for g in source.geoms:
                interiors.extend(list(g.interiors))
        else:
            interiors = list(source.interiors)

        return [Polygon(ring) for ring in interiors if len(ring.coords) >= 4]

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def find_connection(self, start, end):
        """Shortest obstacle-aware path between two points.

        Returns (path_waypoints, distance_meters). path[0] and path[-1] are
        exactly the inputs. The path never crosses a hole interior.
        """
        return self._solver.shortest_path(start, end)

    def assemble_connected(self, sweep_segments):
        """Sequence sweeps and interleave ferry connections.

        Returns a dict with the legacy shape:
            {
                "route_segments": [{segment_type, spraying, path, distance_m}, ...],
                "combined_path": [(x, y), ...],
                "distances": {"sweep_m", "ferry_m", "total_m"},
            }

        route_segments[0] is always a sweep. Ferries are inserted between
        consecutive sweeps whenever the geodesic path between them has
        non-zero length.
        """
        valid = [s for s in (sweep_segments or [])
                 if s and len(s.get('path', [])) >= 2]

        if not valid:
            return {
                'route_segments': [],
                'combined_path': [],
                'distances': {'sweep_m': 0.0, 'ferry_m': 0.0, 'total_m': 0.0},
            }

        ordered = self._sequencer.sequence(valid)

        route_segments = []
        sweep_m = 0.0
        ferry_m = 0.0

        for i, sw in enumerate(ordered):
            if i > 0:
                prev_end = route_segments[-1]['path'][-1]
                cur_start = sw['path'][0]
                ferry_path, ferry_d = self._solver.shortest_path(prev_end, cur_start)
                if ferry_d > 1e-9 and len(ferry_path) >= 2:
                    # Ferry inherits destination cell_id so the segmenter sees
                    # a clean cell transition at the boundary.
                    ferry_rec = self._segment_record(ferry_path, 'ferry', False)
                    ferry_rec['cell_id'] = sw.get('cell_id')
                    route_segments.append(ferry_rec)
                    ferry_m += ferry_d

            sweep_record = self._segment_record(sw['path'], 'sweep', True)
            sweep_record['cell_id'] = sw.get('cell_id')
            route_segments.append(sweep_record)
            sweep_m += sweep_record['distance_m']

        return {
            'route_segments': route_segments,
            'combined_path': self._segments_to_path(route_segments),
            'distances': {
                'sweep_m': float(sweep_m),
                'ferry_m': float(ferry_m),
                'total_m': float(sweep_m + ferry_m),
            },
        }

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _pts_equal(a, b, tol=1e-6):
        return abs(a[0] - b[0]) < tol and abs(a[1] - b[1]) < tol

    @staticmethod
    def _segment_record(path_coords, segment_type, spraying):
        coords = [tuple(p) for p in path_coords]
        return {
            'segment_type': segment_type,
            'spraying': spraying,
            'path': coords,
            'distance_m': float(LineString(coords).length),
        }

    @staticmethod
    def _segments_to_path(route_segments):
        if not route_segments:
            return []
        path = list(route_segments[0]['path'])
        for seg in route_segments[1:]:
            p = seg['path']
            if path and p and PathAssembler._pts_equal(path[-1], p[0]):
                path.extend(p[1:])
            else:
                path.extend(p)
        return path
