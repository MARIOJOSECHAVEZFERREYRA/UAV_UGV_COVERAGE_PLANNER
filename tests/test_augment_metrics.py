"""
Tests for DynamicMissionController._augment_metrics().

Verifies that rendezvous metrics are extracted from mission_cycles
(actual segmented mission) and NOT from opt_result (GA optimizer estimate).
"""

import pytest
from backend.controllers.mission_controller import DynamicMissionController


@pytest.fixture
def ctrl():
    return DynamicMissionController(ugv_polyline=[(0.0, 0.0), (100.0, 0.0)])


def test_augment_metrics_counts_only_cycles_with_rv_wait(ctrl):
    # Two cycles have rv_wait_s; last cycle has none (mission complete, no rv).
    cycles = [
        {"rv_wait_s": 30.0},
        {"rv_wait_s": 0.0},
        {},
    ]
    metrics = {}
    ctrl._augment_metrics(metrics, opt_result={}, mission_cycles=cycles)
    assert metrics["rv_n_rendezvous"] == 2


def test_augment_metrics_sums_wait_time_correctly(ctrl):
    cycles = [
        {"rv_wait_s": 30.0},
        {"rv_wait_s": 90.0},
        {},
    ]
    metrics = {}
    ctrl._augment_metrics(metrics, opt_result={}, mission_cycles=cycles)
    assert metrics["rv_wait_min"] == pytest.approx((30.0 + 90.0) / 60.0)


def test_augment_metrics_ignores_opt_result(ctrl):
    # Even when opt_result has rv_count/rv_wait, the real values win.
    cycles = [{"rv_wait_s": 60.0}, {}]
    metrics = {}
    ctrl._augment_metrics(
        metrics,
        opt_result={"rv_count": 99, "rv_wait": 9999.0},
        mission_cycles=cycles,
    )
    assert metrics["rv_n_rendezvous"] == 1
    assert metrics["rv_wait_min"] == pytest.approx(1.0)


def test_augment_metrics_precalculated_route_with_rv(ctrl):
    # Simulates precalculated_route_segments path: opt_result is empty.
    # Metrics must still come from mission_cycles.
    cycles = [{"rv_wait_s": 120.0}, {"rv_wait_s": 0.0}, {}]
    metrics = {}
    ctrl._augment_metrics(metrics, opt_result={}, mission_cycles=cycles)
    assert metrics["rv_n_rendezvous"] == 2
    assert metrics["rv_wait_min"] == pytest.approx(120.0 / 60.0)


def test_augment_metrics_no_rendezvous_at_all(ctrl):
    # All cycles have no rv_wait_s (e.g. mission completed in one cycle).
    cycles = [{}]
    metrics = {}
    ctrl._augment_metrics(metrics, opt_result={}, mission_cycles=cycles)
    assert metrics["rv_n_rendezvous"] == 0
    assert metrics["rv_wait_min"] == pytest.approx(0.0)


def test_rv_n_rendezvous_equals_cycles_with_rv_wait_s(ctrl):
    """
    Consistency invariant: rv_n_rendezvous must equal exactly the number
    of cycles that carry rv_wait_s, regardless of their wait value.
    """
    cycles = [
        {"rv_wait_s": 0.0},
        {"rv_wait_s": 120.0},
        {"rv_wait_s": 45.0},
        {},   # last cycle: no rendezvous
    ]
    metrics = {}
    ctrl._augment_metrics(metrics, opt_result={}, mission_cycles=cycles)
    cycles_with_rv = sum(1 for c in cycles if 'rv_wait_s' in c)
    assert metrics["rv_n_rendezvous"] == cycles_with_rv


def test_rv_wait_min_is_zero_when_no_rv(ctrl):
    # Explicit check: no rv_wait_s keys -> 0.0, not a division-by-zero.
    metrics = {}
    ctrl._augment_metrics(metrics, opt_result={}, mission_cycles=[{}, {}])
    assert metrics["rv_wait_min"] == 0.0
