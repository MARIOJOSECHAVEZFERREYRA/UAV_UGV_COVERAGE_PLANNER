import math

import pytest
from shapely.geometry import LineString, Polygon

from backend.algorithms.coverage.geodesic_solver import GeodesicSolver


def _path_crosses_hole(path, hole):
    """True iff any interior portion of the path lies strictly inside the hole.

    Skimming a hole edge or touching a vertex is allowed; entering the
    interior is not. Uses the same boundary-aware test as GeodesicSolver.
    """
    line = LineString(path)
    inter = line.intersection(hole)
    if inter.is_empty:
        return False
    interior = inter.difference(hole.boundary)
    return not interior.is_empty


def test_no_holes_is_direct_line():
    solver = GeodesicSolver(holes=None)
    path, d = solver.shortest_path((0, 0), (10, 10))
    assert path == [(0.0, 0.0), (10.0, 10.0)]
    assert d == pytest.approx(math.sqrt(200))


def test_empty_holes_list_is_direct():
    solver = GeodesicSolver(holes=[])
    path, d = solver.shortest_path((0, 0), (10, 10))
    assert len(path) == 2
    assert d == pytest.approx(math.sqrt(200))


def test_line_does_not_cross_hole_stays_direct():
    hole = Polygon([(5, 5), (6, 5), (6, 6), (5, 6)])
    solver = GeodesicSolver(holes=[hole])
    # Line from (0,0) to (0,10) is far from the hole — direct.
    path, d = solver.shortest_path((0, 0), (0, 10))
    assert path == [(0.0, 0.0), (0.0, 10.0)]
    assert d == pytest.approx(10.0)


def test_line_crosses_rectangular_hole_finds_detour():
    hole = Polygon([(4, 0), (6, 0), (6, 10), (4, 10)])
    solver = GeodesicSolver(holes=[hole])
    path, d = solver.shortest_path((0, 5), (10, 5))
    assert path[0] == (0.0, 5.0)
    assert path[-1] == (10.0, 5.0)
    assert not _path_crosses_hole(path, hole)
    assert len(path) >= 3
    # Detour goes around the bar — shorter than walking the full perimeter.
    assert d < 25.0


def test_multiple_holes():
    h1 = Polygon([(3, 3), (4, 3), (4, 4), (3, 4)])
    h2 = Polygon([(6, 6), (7, 6), (7, 7), (6, 7)])
    solver = GeodesicSolver(holes=[h1, h2])
    path, d = solver.shortest_path((0, 0), (10, 10))
    assert path[0] == (0.0, 0.0)
    assert path[-1] == (10.0, 10.0)
    assert not _path_crosses_hole(path, h1)
    assert not _path_crosses_hole(path, h2)
    assert d >= math.sqrt(200)


def test_same_start_end_distance_zero():
    solver = GeodesicSolver(holes=None)
    path, d = solver.shortest_path((5, 5), (5, 5))
    assert d == 0.0


def test_path_endpoints_are_exact():
    solver = GeodesicSolver(holes=None)
    s = (0.123456, 7.891011)
    e = (9.87654, 3.21098)
    path, _ = solver.shortest_path(s, e)
    assert path[0] == s
    assert path[-1] == e


def test_outside_points_with_hole_between():
    # base_point outside a "field" with a hole forcing detour
    hole = Polygon([(4, 4), (6, 4), (6, 6), (4, 6)])
    solver = GeodesicSolver(holes=[hole])
    path, d = solver.shortest_path((-5, 5), (15, 5))
    assert path[0] == (-5.0, 5.0)
    assert path[-1] == (15.0, 5.0)
    assert not _path_crosses_hole(path, hole)


def test_grazing_a_hole_vertex_is_allowed():
    # A line that only touches a corner of a hole should be accepted as direct.
    hole = Polygon([(5, 5), (6, 5), (6, 6), (5, 6)])
    solver = GeodesicSolver(holes=[hole])
    # Line from (0,5) to (5,5) ends exactly at a hole vertex — grazing.
    path, d = solver.shortest_path((0, 5), (5, 5))
    assert d == pytest.approx(5.0)
