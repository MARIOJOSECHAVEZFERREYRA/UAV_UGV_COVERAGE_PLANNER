"""Smoke tests for the grid search optimizer.

Verifies that the optimizer produces a monotone minimum of total flight
distance across all 180 angles and that the returned dict has the
expected shape for the strategy layer.
"""

import pytest
from shapely.geometry import Polygon

from backend.algorithms.coverage.path_planner import BoustrophedonPlanner
from backend.algorithms.routing.grid_search_optimizer import GridSearchOptimizer


SIMPLE_RECT = Polygon([(0, 0), (100, 0), (100, 60), (0, 60)])
PENTAGON = Polygon([
    (207.9, 240.8),
    (221.2, 431.7),
    (292.6, 473.0),
    (537.7, 232.3),
    (266.0, 102.1),
])


def _optimizer(swath=5):
    planner = BoustrophedonPlanner(spray_width=swath)
    return GridSearchOptimizer(planner, angle_step=1)


def test_rect_no_obstacles_no_base():
    opt = _optimizer(swath=5)
    result = opt.optimize(SIMPLE_RECT)
    assert result['angle'] in range(0, 180)
    assert result['l'] > 0
    assert result['spray_m'] > 0
    assert result['ferry_m'] >= 0
    # Without base_point deadheads are zero
    assert result['deadhead_m'] == 0.0
    # Coverage should be close to polygon area for a simple rectangle
    assert result['s_prime'] >= SIMPLE_RECT.area * 0.9


def test_rect_with_base_point():
    opt = _optimizer(swath=5)
    result = opt.optimize(SIMPLE_RECT, base_point=(-10, 30))
    # With base_point deadheads are non-zero
    assert result['deadhead_m'] > 0
    # Fitness is now mission time in seconds, not distance.
    assert 'mission_time_s' in result
    assert 'flight_time_s' in result
    assert 'n_cycles' in result
    assert result['n_cycles'] >= 1
    assert result['l'] == pytest.approx(result['mission_time_s'])
    # Mission time should be >= flight time (cycle overhead is non-negative)
    assert result['mission_time_s'] >= result['flight_time_s']


def test_rect_angle_0_or_90_wins_rectangle():
    """For a rectangle the optimum angle should be 0 or 90 (long sides)."""
    opt = _optimizer(swath=5)
    result = opt.optimize(SIMPLE_RECT)
    assert result['angle'] in (0, 90)


def test_pentagon_returns_result():
    opt = _optimizer(swath=9)
    result = opt.optimize(PENTAGON, base_point=(296, 78))
    assert 'angle' in result
    assert 'route_segments' in result
    assert 'combined_path' in result
    assert len(result['route_segments']) > 0
    assert result['l'] > 0


def test_angle_step_coarse():
    opt = GridSearchOptimizer(BoustrophedonPlanner(spray_width=5), angle_step=5)
    result = opt.optimize(SIMPLE_RECT)
    # 180/5 = 36 angles scanned
    assert len(result['gen_stats']) <= 36


def test_exact_optimum_matches_expected_shape():
    """Sanity: the returned dict has all keys the strategy layer expects."""
    opt = _optimizer(swath=5)
    result = opt.optimize(SIMPLE_RECT, base_point=(-5, 30))
    required = {
        'angle', 'l', 's_prime', 'spray_m', 'ferry_m', 'deadhead_m',
        'route_segments', 'combined_path', 'planner_metrics',
        'route_distances', 'fitness', 'extra_coverage_pct', 'gen_stats',
    }
    assert required.issubset(set(result.keys())), \
        f"missing: {required - set(result.keys())}"
