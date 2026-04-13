import math

import pytest
from shapely.geometry import LineString, Polygon

from backend.algorithms.coverage.path_assembler import PathAssembler


SIMPLE_SQUARE = Polygon([(0, 0), (20, 0), (20, 20), (0, 20)])
SQUARE_WITH_HOLE = Polygon(
    [(0, 0), (20, 0), (20, 20), (0, 20)],
    holes=[[(8, 8), (12, 8), (12, 12), (8, 12)]],
)


def _sweep(a, b):
    return {
        'path': [a, b],
        'segment_type': 'sweep',
        'spraying': True,
        'distance_m': math.hypot(b[0] - a[0], b[1] - a[1]),
    }


def test_empty_sweeps_returns_empty_route():
    pa = PathAssembler(SIMPLE_SQUARE)
    r = pa.assemble_connected([])
    assert r['route_segments'] == []
    assert r['combined_path'] == []
    assert r['distances'] == {'sweep_m': 0.0, 'ferry_m': 0.0, 'total_m': 0.0}


def test_route_segments_start_with_sweep():
    pa = PathAssembler(SIMPLE_SQUARE)
    sweeps = [_sweep((0, i), (20, i)) for i in (2, 4, 6)]
    r = pa.assemble_connected(sweeps)
    assert r['route_segments'][0]['segment_type'] == 'sweep'


def test_distances_sum_consistent():
    pa = PathAssembler(SIMPLE_SQUARE)
    sweeps = [_sweep((0, i), (20, i)) for i in (2, 4, 6)]
    r = pa.assemble_connected(sweeps)
    d = r['distances']
    assert d['total_m'] == pytest.approx(d['sweep_m'] + d['ferry_m'])


def test_all_sweeps_present_in_output():
    pa = PathAssembler(SIMPLE_SQUARE)
    sweeps = [_sweep((0, i), (20, i)) for i in (2, 4, 6)]
    r = pa.assemble_connected(sweeps)
    n_sweeps = sum(1 for s in r['route_segments'] if s['segment_type'] == 'sweep')
    assert n_sweeps == 3


def test_ferries_interleaved_between_sweeps():
    pa = PathAssembler(SIMPLE_SQUARE)
    # 3 sweeps that don't share endpoints → 2 ferries between them
    sweeps = [
        _sweep((0, 2), (20, 2)),
        _sweep((0, 4), (20, 4)),
        _sweep((0, 6), (20, 6)),
    ]
    r = pa.assemble_connected(sweeps)
    types = [s['segment_type'] for s in r['route_segments']]
    # Sweep, ferry, sweep, ferry, sweep (maybe — sequencer may flip)
    assert types.count('sweep') == 3
    assert types.count('ferry') >= 1


def test_combined_path_no_duplicate_seam():
    pa = PathAssembler(SIMPLE_SQUARE)
    sweeps = [_sweep((0, 2), (20, 2)), _sweep((0, 4), (20, 4))]
    r = pa.assemble_connected(sweeps)
    cp = r['combined_path']
    # No two consecutive points should be equal
    for i in range(len(cp) - 1):
        assert cp[i] != cp[i + 1], f"duplicate seam at index {i}"


def test_find_connection_endpoints_exact():
    pa = PathAssembler(SIMPLE_SQUARE)
    path, d = pa.find_connection((0, 0), (5, 5))
    assert path[0] == (0.0, 0.0)
    assert path[-1] == (5.0, 5.0)
    assert d == pytest.approx(math.sqrt(50))


def test_find_connection_outside_point():
    pa = PathAssembler(SIMPLE_SQUARE)
    # base_point outside the field
    path, d = pa.find_connection((-5, -5), (5, 5))
    assert path[0] == (-5.0, -5.0)
    assert path[-1] == (5.0, 5.0)


def test_find_connection_avoids_hole():
    pa = PathAssembler(SQUARE_WITH_HOLE)
    path, d = pa.find_connection((0, 10), (20, 10))
    line = LineString(path)
    hole = Polygon([(8, 8), (12, 8), (12, 12), (8, 12)])
    interior = line.intersection(hole).difference(hole.boundary)
    assert interior.is_empty, f"path enters hole interior: {interior}"


def test_assemble_connected_sweep_order_respects_holes_in_full_mode():
    pa = PathAssembler(SQUARE_WITH_HOLE, sequencer_mode='full')
    sweeps = [_sweep((1, 5), (19, 5)), _sweep((1, 15), (19, 15))]
    r = pa.assemble_connected(sweeps)
    # Verify no emitted ferry crosses the hole interior
    hole = Polygon([(8, 8), (12, 8), (12, 12), (8, 12)])
    for seg in r['route_segments']:
        if seg['segment_type'] != 'ferry':
            continue
        line = LineString(seg['path'])
        interior = line.intersection(hole).difference(hole.boundary)
        assert interior.is_empty, f"ferry crosses hole: {seg['path']}"


def test_constructor_accepts_list_of_cells_like_optimizer():
    # SweepAngleOptimizer passes sub_polygons + original_polygon
    cells = [
        Polygon([(0, 0), (10, 0), (10, 20), (0, 20)]),
        Polygon([(10, 0), (20, 0), (20, 20), (10, 20)]),
    ]
    pa = PathAssembler(cells, original_polygon=SIMPLE_SQUARE)
    r = pa.assemble_connected([])
    assert r['distances']['total_m'] == 0.0


def test_segment_record_has_expected_keys():
    pa = PathAssembler(SIMPLE_SQUARE)
    sweeps = [_sweep((0, 2), (20, 2)), _sweep((0, 4), (20, 4))]
    r = pa.assemble_connected(sweeps)
    for seg in r['route_segments']:
        assert set(seg.keys()) >= {'segment_type', 'spraying', 'path', 'distance_m'}
        assert seg['segment_type'] in ('sweep', 'ferry')
        assert isinstance(seg['spraying'], bool)
        assert isinstance(seg['distance_m'], float)
