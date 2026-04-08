from types import SimpleNamespace

import pytest

from backend.controllers.mission_controller import MissionController


def make_drone(**overrides):
    values = {
        "name": "Test Drone",
        "spray_swath_min_m": 6.0,
        "spray_swath_max_m": 9.0,
        "app_rate_default_l_ha": 12.0,
        "app_rate_min_l_ha": 4.0,
        "app_rate_max_l_ha": 18.0,
        "speed_cruise_ms": 5.0,
        "speed_max_ms": 8.0,
    }
    values.update(overrides)
    return SimpleNamespace(**values)


def test_resolve_operational_params_uses_drone_app_rate_default():
    ctrl = MissionController()

    _, app_rate, speed_kmh, calc_flow_l_min, margin_h = ctrl._resolve_operational_params(
        make_drone(),
        {},
    )

    assert app_rate == 12.0
    assert speed_kmh == pytest.approx(18.0)
    assert calc_flow_l_min == pytest.approx((12.0 * 18.0 * 9.0) / 600.0)
    assert margin_h == pytest.approx(4.5)


def test_resolve_operational_params_validates_drone_specific_app_rate_range():
    ctrl = MissionController()
    drone = make_drone(app_rate_min_l_ha=7.0, app_rate_max_l_ha=9.0)

    with pytest.raises(ValueError, match="\\[7\\.0, 9\\.0\\].*Test Drone"):
        ctrl._resolve_operational_params(drone, {"app_rate": 10.0})
