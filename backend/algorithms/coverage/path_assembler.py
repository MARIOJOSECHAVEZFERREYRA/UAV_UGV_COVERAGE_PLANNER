"""Facade that sequences sweeps and stitches them with obstacle-aware ferries.

Composes a GeodesicSolver (shortest path in ℝ² \\ holes) with a
SweepSequencer (cell-aware sweep ordering). Navigation ignores the field
boundary — only interior rings (real obstacles) constrain paths.
"""

from shapely.geometry import LineString, Polygon

from .geodesic_solver import GeodesicSolver
from .sweep_sequencer import SweepSequencer


class PathAssembler:
    """Sequence sweeps and interleave ferries to form the mission route."""

    def __init__(self, sub_polygons, original_polygon=None,
                 sequencer_mode='full', base_point=None):
        holes = self._extract_holes(sub_polygons, original_polygon)
        self._solver = GeodesicSolver(holes=holes)

        # Adjacency lets the sequencer prefer topologically connected
        # cells, keeping consecutive mission cycles contiguous.
        cell_adjacency = self._build_cell_adjacency(sub_polygons)

        self._sequencer = SweepSequencer(
            self._solver,
            mode=sequencer_mode,
            base_point=base_point,
            cell_adjacency=cell_adjacency,
        )

    @staticmethod
    def _build_cell_adjacency(sub_polygons):
        """Map cell index -> set of neighbors sharing a boundary segment.

        Returns None for single-polygon or non-list input.
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

    @staticmethod
    def _extract_holes(sub_polygons, original_polygon):
        """Return Polygon objects for every obstacle the drone must avoid.

        Prefers original_polygon.interiors (authoritative), falls back
        to sub_polygons interiors when it is a single Polygon.
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

    def find_connection(self, start, end):
        """Obstacle-aware shortest path between two points.

        Returns (path, distance). Endpoints are preserved exactly.
        """
        return self._solver.shortest_path(start, end)

    def assemble_connected(self, sweep_segments):
        """Emit the ordered route: sweeps + interleaved ferry connections.

        route_segments[0] is always a sweep; ferries appear only when
        consecutive sweeps need an obstacle-aware jump.
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
                    # Ferry inherits the destination's cell_id so the
                    # segmenter sees a clean transition.
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
