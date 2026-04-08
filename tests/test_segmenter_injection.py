"""
Integration tests for MissionSegmenter energy_model injection.

Verifies that:
  - When an energy_model is passed explicitly, MissionSegmenter uses exactly
    that object (identity, not just equality).
  - When no energy_model is passed, MissionSegmenter creates its own valid
    DroneEnergyModel from the drone spec.
"""

from types import SimpleNamespace

from backend.algorithms.drone.energy_model import DroneEnergyModel
from backend.algorithms.rendezvous.segmentation import MissionSegmenter


def _make_drone():
    return SimpleNamespace(
        name="DJI Agras T30",
        num_rotors=6,
        mass_empty_kg=26.4,
        mass_battery_kg=10.1,
        mass_tank_full_kg=30.0,
        battery_capacity_wh=1502.2,
        battery_voltage_v=51.8,
        battery_reserve_pct=20.0,
        battery_charge_time_min=10.0,
        power_hover_empty_w=4396.0,
        power_hover_full_w=11556.0,
        speed_cruise_ms=5.0,
        speed_max_ms=10.0,
        speed_vertical_ms=3.0,
        turn_duration_s=10.0,
        turn_power_factor=1.1,
        spray_flow_rate_lpm=8.0,
        spray_swath_min_m=4.0,
        spray_swath_max_m=9.0,
        spray_height_m=2.5,
        spray_pump_power_w=200.0,
        service_time_s=120.0,
    )


def test_segmenter_uses_injected_energy_model():
    """MissionSegmenter must use the exact energy_model instance passed in."""
    drone = _make_drone()
    model = DroneEnergyModel(drone)
    segmenter = MissionSegmenter(drone, energy_model=model)
    assert segmenter.energy_model is model


def test_segmenter_creates_own_energy_model_when_none():
    """MissionSegmenter must create a valid DroneEnergyModel when none is provided."""
    drone = _make_drone()
    segmenter = MissionSegmenter(drone)
    assert segmenter.energy_model is not None
    assert isinstance(segmenter.energy_model, DroneEnergyModel)
