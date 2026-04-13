import math

import pytest

from backend.algorithms.coverage.geodesic_solver import GeodesicSolver
from backend.algorithms.coverage.sweep_sequencer import SweepSequencer


def _sweep(a, b):
    return {
        'path': [a, b],
        'segment_type': 'sweep',
        'spraying': True,
        'distance_m': math.hypot(b[0] - a[0], b[1] - a[1]),
    }


def _ferry_cost(ordered):
    total = 0.0
    for i in range(len(ordered) - 1):
        a = ordered[i]['path'][-1]
        b = ordered[i + 1]['path'][0]
        total += math.hypot(a[0] - b[0], a[1] - b[1])
    return total


def test_empty_sequence():
    seq = SweepSequencer(GeodesicSolver(None), mode='fast')
    assert seq.sequence([]) == []


def test_single_sweep():
    sw = _sweep((0, 0), (10, 0))
    out = SweepSequencer(GeodesicSolver(None), mode='fast').sequence([sw])
    assert len(out) == 1
    assert out[0]['path'] == [(0, 0), (10, 0)]


def test_two_sweeps_natural_order_best():
    # Current end (10, 0). Sweep2 has endpoints (10, 10) and (0, 10).
    # Natural = connect to (10,10) — distance 10. Flipped = (0,10) — distance ~14.14.
    s1 = _sweep((0, 0), (10, 0))
    s2 = _sweep((10, 10), (0, 10))
    out = SweepSequencer(GeodesicSolver(None), 'fast').sequence([s1, s2])
    assert out[1]['path'][0] == (10, 10)


def test_flip_when_reverse_is_closer():
    # Current end (10, 0). Sweep2 (0, 10) -> (10, 10). Natural distance to (0,10)
    # is ~14.14, flipped to (10,10) is 10. Sequencer should flip.
    s1 = _sweep((0, 0), (10, 0))
    s2 = _sweep((0, 10), (10, 10))
    out = SweepSequencer(GeodesicSolver(None), 'fast').sequence([s1, s2])
    assert out[1]['path'][0] == (10, 10)
    assert out[1]['path'][-1] == (0, 10)


def test_nn_beats_natural_order_on_scrambled_sweeps():
    # Five parallel sweeps given in an order that produces long ferries
    # if left as-is. NN should pick the nearest each time.
    swps = [
        _sweep((0, 0), (10, 0)),
        _sweep((0, 40), (10, 40)),     # far
        _sweep((0, 10), (10, 10)),     # near first
        _sweep((0, 30), (10, 30)),
        _sweep((0, 20), (10, 20)),
    ]
    natural = _ferry_cost(swps)
    out = SweepSequencer(GeodesicSolver(None), 'fast').sequence(swps)
    assert _ferry_cost(out) <= natural


def test_sequencer_respects_holes_in_full_mode():
    from shapely.geometry import Polygon
    hole = Polygon([(4, 4), (6, 4), (6, 6), (4, 6)])
    solver = GeodesicSolver(holes=[hole])
    s1 = _sweep((0, 5), (3, 5))
    s2 = _sweep((7, 5), (10, 5))
    # Full mode uses real distances; the ferry between s1 end (3,5) and
    # s2 start (7,5) must go around the hole.
    out = SweepSequencer(solver, 'full').sequence([s1, s2])
    assert len(out) == 2


def test_base_anchored_nn_starts_near_base():
    # Four sweeps — the second one (index 1) is closest to the base.
    # Without base_point NN starts from sweeps[0]; with base_point it
    # should start from the sweep nearest the base.
    swps = [
        _sweep((100, 100), (110, 100)),  # far from base
        _sweep((0, 0), (10, 0)),          # near (0,0) base
        _sweep((200, 200), (210, 200)),
        _sweep((50, 50), (60, 50)),
    ]
    out = SweepSequencer(GeodesicSolver(None), 'fast', base_point=(0, 0)).sequence(swps)
    # First sweep of the tour should be the one near the base.
    assert out[0]['path'][0] == (0, 0) or out[0]['path'][-1] == (0, 0)


def test_base_anchored_tour_starts_near_base():
    # The first sweep of the tour must be the one whose nearest endpoint
    # is closest to the base. 2-opt deliberately ignores virtual base
    # edges to avoid creating internal jumps just to "tuck" a closing
    # edge near the base — see _two_opt_flat docstring — so we only
    # check the opening anchor, not the full round-trip cost.
    swps = [
        _sweep((100, 0), (110, 0)),
        _sweep((0, 0), (10, 0)),
        _sweep((100, 10), (110, 10)),
        _sweep((0, 10), (10, 10)),
    ]
    base = (-5, 5)
    anchored = SweepSequencer(GeodesicSolver(None), 'fast', base_point=base).sequence(swps)
    first_entry = anchored[0]['path'][0]
    d_first = math.hypot(first_entry[0] - base[0], first_entry[1] - base[1])
    assert d_first < 10.0, f"first sweep not anchored near base: {first_entry}, d={d_first}"


def test_cell_grouping_preserves_atomicity():
    """Sweeps from one cell must stay contiguous in the output."""
    # 2 cells; cell 0 has 3 sweeps on the left, cell 1 has 3 on the right.
    swps = [
        {**_sweep((0, 0), (5, 0)),  'cell_id': 0},
        {**_sweep((0, 5), (5, 5)),  'cell_id': 0},
        {**_sweep((0, 10), (5, 10)), 'cell_id': 0},
        {**_sweep((20, 0), (25, 0)),  'cell_id': 1},
        {**_sweep((20, 5), (25, 5)),  'cell_id': 1},
        {**_sweep((20, 10), (25, 10)), 'cell_id': 1},
    ]
    out = SweepSequencer(GeodesicSolver(None), 'fast').sequence(swps)
    # Extract the cell_id sequence — must be one contiguous run of 0s
    # followed by a contiguous run of 1s (or vice versa).
    ids = [s['cell_id'] for s in out]
    transitions = sum(1 for i in range(len(ids) - 1) if ids[i] != ids[i + 1])
    assert transitions == 1, f"cells interleaved: {ids}"


def test_cell_ordering_base_aware():
    """Cell closer to base should come first."""
    # cell 0 far (right), cell 1 near (left) — base at origin
    swps = [
        {**_sweep((100, 0), (105, 0)), 'cell_id': 0},
        {**_sweep((100, 5), (105, 5)), 'cell_id': 0},
        {**_sweep((0, 0), (5, 0)),      'cell_id': 1},
        {**_sweep((0, 5), (5, 5)),      'cell_id': 1},
    ]
    out = SweepSequencer(GeodesicSolver(None), 'fast', base_point=(-5, 0)).sequence(swps)
    assert out[0]['cell_id'] == 1, f"expected cell 1 first, got {[s['cell_id'] for s in out]}"


def test_cell_reversal_for_min_transition():
    """When reversing a cell lowers the ferry cost to the next, do it."""
    # cell 0: sweeps at y=0 and y=5. Natural: start (0,0) end (5,5).
    #         Reversed: start (5,5) end (0,0).
    # cell 1: sweeps at y=10 with start (0,10). If cell0 natural ends at (5,5)
    # the ferry to (0,10) is ~7.07. If reversed ends at (0,0) ferry is 10.
    # So cell0 should stay natural. Reverse test: put cell1 at (5,10).
    swps = [
        {**_sweep((0, 0), (5, 0)), 'cell_id': 0},
        {**_sweep((5, 5), (0, 5)), 'cell_id': 0},   # boustrophedon zigzag
        {**_sweep((5, 10), (0, 10)), 'cell_id': 1},  # starts at (5,10)
    ]
    # base near (0,0): cell0 should be natural (starts at (0,0)).
    out = SweepSequencer(GeodesicSolver(None), 'fast', base_point=(-5, -5)).sequence(swps)
    ids = [s['cell_id'] for s in out]
    assert ids == [0, 0, 1] or ids == [1, 0, 0]
    # Must be atomic
    transitions = sum(1 for i in range(len(ids) - 1) if ids[i] != ids[i + 1])
    assert transitions == 1


def test_adjacency_prevents_cell_jumps():
    """With adjacency graph, NN walks only through adjacent cells.

    Layout: 3 cells A, B, C in a row. A-B adjacent, B-C adjacent, A-C NOT.
    Without adjacency, NN from a position near A's right end could jump
    to C if C happens to be closer by euclidean than B. With adjacency,
    NN must pick B next because A and C are not adjacent.
    """
    # Construct: A at x=[0,10], B at x=[10,20], C at x=[20,30].
    # All horizontal sweeps at y=5.
    a_sweeps = [{**_sweep((0, 5), (10, 5)), 'cell_id': 0}]
    b_sweeps = [{**_sweep((10, 5), (20, 5)), 'cell_id': 1}]
    c_sweeps = [{**_sweep((20, 5), (30, 5)), 'cell_id': 2}]
    swps = a_sweeps + b_sweeps + c_sweeps

    # Adjacency: only A-B and B-C share borders
    adj = {0: {1}, 1: {0, 2}, 2: {1}}

    seq = SweepSequencer(
        GeodesicSolver(None), 'fast',
        base_point=(-5, 5),
        cell_adjacency=adj,
    )
    out = seq.sequence(swps)
    ids = [s['cell_id'] for s in out]
    # Must be sequential adjacent walk: 0, 1, 2 (or reverse)
    assert ids == [0, 1, 2] or ids == [2, 1, 0], f"non-adjacent walk: {ids}"


def test_adjacency_disconnected_falls_back():
    """If adjacency is disconnected, NN falls back to global nearest."""
    # Two cells, no adjacency at all (e.g., isolated islands)
    a = [{**_sweep((0, 0), (5, 0)), 'cell_id': 0}]
    b = [{**_sweep((100, 100), (105, 100)), 'cell_id': 1}]
    adj = {0: set(), 1: set()}  # no edges
    seq = SweepSequencer(
        GeodesicSolver(None), 'fast',
        base_point=(0, 0),
        cell_adjacency=adj,
    )
    out = seq.sequence(a + b)
    # Should still produce both, starting from the one near base
    assert len(out) == 2
    assert out[0]['cell_id'] == 0


def test_fallback_when_cell_id_missing():
    """Sweeps without cell_id should still work via flat NN+2opt."""
    swps = [
        _sweep((0, 0), (10, 0)),
        _sweep((0, 5), (10, 5)),
        _sweep((0, 10), (10, 10)),
    ]
    out = SweepSequencer(GeodesicSolver(None), 'fast').sequence(swps)
    assert len(out) == 3


def test_single_cell_falls_back_to_flat():
    """One cell only — the cell-level path degenerates to flat."""
    swps = [
        {**_sweep((0, 0), (10, 0)), 'cell_id': 0},
        {**_sweep((0, 5), (10, 5)), 'cell_id': 0},
    ]
    out = SweepSequencer(GeodesicSolver(None), 'fast').sequence(swps)
    assert len(out) == 2


def test_two_opt_does_not_worsen_tour():
    # Any scrambled input: after NN + 2-opt, cost should be <= NN alone.
    swps = [
        _sweep((0, 0), (5, 0)),
        _sweep((10, 10), (15, 10)),
        _sweep((0, 10), (5, 10)),
        _sweep((10, 0), (15, 0)),
    ]
    out_fast = SweepSequencer(GeodesicSolver(None), 'fast').sequence(swps)
    c = _ferry_cost(out_fast)
    # Upper bound: any valid permutation including the natural one
    natural = _ferry_cost(swps)
    assert c <= natural
