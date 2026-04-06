"""
Tests for DroneEnergyModel (backend/algorithms/drone/energy_model.py).
Covers only the physics/energy model itself - no planner, simulator, or frontend.

Reference drone: DJI Agras T30
  m_empty_total = 26.4 + 10.1 = 36.5 kg
  power_hover_empty_w = 4396 W
  hover power formula: P = 4396 * (m / 36.5)^1.5  (Actuator Disk Theory)
"""

import pytest
from types import SimpleNamespace
from backend.algorithms.drone.energy_model import DroneEnergyModel


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

@pytest.fixture
def drone():
    """DJI Agras T30 spec as a plain namespace (mirrors SQLAlchemy Drone columns)."""
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
        spray_swath_m=9.0,
        spray_height_m=2.5,
        spray_pump_power_w=200.0,
        service_time_s=120.0,
    )


@pytest.fixture
def model(drone):
    """DroneEnergyModel using default liquid density of 1.0 kg/L."""
    return DroneEnergyModel(drone)


# ---------------------------------------------------------------------------
# Helpers (not tests) - pre-computed reference values for the T30
# ---------------------------------------------------------------------------

# m_empty_total = 26.4 + 10.1 = 36.5 kg
M_EMPTY_TOTAL = 36.5

# P_hover(m) = 4396 * (m / 36.5)^1.5
def _hover_ref(m):
    return 4396.0 * (m / M_EMPTY_TOTAL) ** 1.5


# ---------------------------------------------------------------------------
# TestPhysicsBase
# ---------------------------------------------------------------------------

class TestPhysicsBase:

    def test_instant_mass_full_tank(self, model):
        # 26.4 + 10.1 + 30.0 * 1.0 = 66.5 kg
        assert model.instant_mass(30.0) == pytest.approx(66.5, rel=0.01)

    def test_instant_mass_empty_tank(self, model):
        # 26.4 + 10.1 + 0 = 36.5 kg
        assert model.instant_mass(0.0) == pytest.approx(36.5, rel=0.01)

    def test_instant_mass_half_tank(self, model):
        # 26.4 + 10.1 + 15.0 = 51.5 kg
        assert model.instant_mass(15.0) == pytest.approx(51.5, rel=0.01)

    def test_hover_power_empty(self, model):
        # ratio = 36.5/36.5 = 1.0 -> P = 4396 * 1.0^1.5 = 4396 W
        assert model.hover_power(0.0) == pytest.approx(4396.0, rel=0.01)

    def test_hover_power_full(self, model):
        # m = 66.5, ratio = 66.5/36.5 = 1.8219
        # 1.8219^1.5 ≈ 2.4589 -> P ≈ 4396 * 2.4589 ≈ 10808 W
        expected = _hover_ref(66.5)
        assert model.hover_power(30.0) == pytest.approx(expected, rel=0.01)

    def test_hover_power_decreases_as_tank_empties(self, model):
        # Power must decrease monotonically as reagent is consumed
        p_full = model.hover_power(30.0)
        p_half = model.hover_power(15.0)
        p_empty = model.hover_power(0.0)
        assert p_full > p_half > p_empty

    def test_cruise_power_equals_hover(self, model):
        # At low agricultural speeds, drag is negligible; cruise == hover
        for reagent in (0.0, 15.0, 30.0):
            assert model.cruise_power(reagent) == pytest.approx(
                model.hover_power(reagent), rel=0.01
            )

    def test_spray_power_adds_pump(self, model):
        # P_spray = P_cruise + 200 W pump
        for reagent in (0.0, 15.0, 30.0):
            expected = model.cruise_power(reagent) + 200.0
            assert model.spray_power(reagent) == pytest.approx(expected, rel=0.01)


# ---------------------------------------------------------------------------
# TestEnergyCosts
# ---------------------------------------------------------------------------

class TestEnergyCosts:

    def test_energy_straight_basic(self, model):
        # 100 m at full tank (30 L):
        #   t = 100/5 = 20 s
        #   P_spray = hover_power(30) + 200 ≈ 10808 + 200 = 11008 W
        #   E = 11008 * 20 / 3600 ≈ 61.16 Wh
        p_spray = _hover_ref(66.5) + 200.0
        expected = p_spray * (100.0 / 5.0) / 3600.0
        assert model.energy_straight(100.0, 30.0) == pytest.approx(expected, rel=0.01)

    def test_energy_straight_longer_costs_more(self, model):
        # 200 m must cost more than 100 m at same fuel load
        assert model.energy_straight(200.0, 15.0) > model.energy_straight(100.0, 15.0)

    def test_energy_straight_heavier_costs_more(self, model):
        # Full tank (higher mass) means higher power -> more energy for same segment
        assert model.energy_straight(100.0, 30.0) > model.energy_straight(100.0, 0.0)

    def test_energy_turn_180(self, model):
        # 180 deg turn at full tank (30 L):
        #   effective_duration = 10 * (180/180) = 10 s
        #   P_turn = hover_power(30) * 1.1 ≈ 10808 * 1.1 = 11889 W
        #   E = 11889 * 10 / 3600 ≈ 33.025 Wh
        p_turn = _hover_ref(66.5) * 1.1
        expected = p_turn * 10.0 / 3600.0
        assert model.energy_turn(180.0, 30.0) == pytest.approx(expected, rel=0.01)

    def test_energy_turn_90_is_half_of_180(self, model):
        # Scale factor is angle/180, so 90 deg = half cost of 180 deg
        e180 = model.energy_turn(180.0, 15.0)
        e90 = model.energy_turn(90.0, 15.0)
        assert e90 == pytest.approx(e180 / 2.0, rel=0.01)

    def test_energy_turn_heavier_costs_more(self, model):
        # Higher mass -> higher hover power -> more turn energy
        assert model.energy_turn(180.0, 30.0) > model.energy_turn(180.0, 0.0)

    def test_energy_transit_uses_max_speed(self, model):
        # Transit uses speed_max (10 m/s), not speed_cruise (5 m/s).
        # For 100 m at full tank:
        #   t_transit = 100/10 = 10 s  (half the time of a cruise segment)
        #   E_transit = hover_power(30) * 10 / 3600 ≈ 30.025 Wh
        #   E_cruise_speed = hover_power(30) * 20 / 3600 ≈ 60.05 Wh (hypothetical)
        # The transit result must be half the cruise-speed result (no pump, double speed)
        p_cruise = _hover_ref(66.5)
        expected_transit = p_cruise * (100.0 / 10.0) / 3600.0
        assert model.energy_transit(100.0, 30.0) == pytest.approx(expected_transit, rel=0.01)

    def test_energy_landing_takeoff(self, model):
        # height = 2.5 m, vert_speed = 3 m/s
        # descent_time = ascent_time = 2.5/3 = 0.8333 s  -> total = 1.6667 s
        # E = hover_power(30) * 1.6667 / 3600
        t_total = 2.0 * (2.5 / 3.0)
        expected = _hover_ref(66.5) * t_total / 3600.0
        assert model.energy_landing_takeoff(30.0) == pytest.approx(expected, rel=0.01)


# ---------------------------------------------------------------------------
# TestReagent
# ---------------------------------------------------------------------------

class TestReagent:

    def test_reagent_consumed_basic(self, model):
        # 100 m at 5 m/s -> t = 20 s
        # Q = 8 L/min * 20 s / 60 = 2.6667 L
        expected = 8.0 * (100.0 / 5.0) / 60.0
        assert model.reagent_consumed(100.0) == pytest.approx(expected, rel=0.01)

    def test_reagent_consumed_proportional_to_distance(self, model):
        # Linear with distance: doubling distance doubles reagent use
        q100 = model.reagent_consumed(100.0)
        q200 = model.reagent_consumed(200.0)
        assert q200 == pytest.approx(2.0 * q100, rel=0.01)

    def test_full_tank_depletion(self, model):
        # Solve: 30 L = 8 * (d / 5) / 60  =>  d = 30 * 300 / 8 = 1125 m
        # Verify that consuming 30 L takes approximately 1125 m
        expected_distance = 1125.0
        consumed = model.reagent_consumed(expected_distance)
        assert consumed == pytest.approx(30.0, rel=0.01)


# ---------------------------------------------------------------------------
# TestCanContinue
# ---------------------------------------------------------------------------

class TestCanContinue:

    def _threshold(self, model, distance_m, reagent_l):
        """Exact energy threshold used by can_continue."""
        transit = model.energy_transit(distance_m, reagent_l)
        lto = model.energy_landing_takeoff(reagent_l)
        reserve = model.usable_energy_wh() * 0.10
        return transit + lto + reserve

    def test_can_continue_with_full_resources(self, model):
        # Full battery (usable = 1201.76 Wh) and 30 L, 100 m away -> True
        usable = model.usable_energy_wh()
        assert model.can_continue(usable, 30.0, 100.0) is True

    def test_cannot_continue_no_reagent(self, model):
        # reagent = 0 L must return False regardless of available energy
        usable = model.usable_energy_wh()
        assert model.can_continue(usable, 0.0, 100.0) is False

    def test_cannot_continue_low_energy(self, model):
        # Only 10 Wh left with 500 m rendezvous -> clearly not enough
        assert model.can_continue(10.0, 15.0, 500.0) is False

    def test_can_continue_barely_enough(self, model):
        # Provide exactly the required threshold -> True
        distance_m = 100.0
        reagent_l = 15.0
        threshold = self._threshold(model, distance_m, reagent_l)
        assert model.can_continue(threshold, reagent_l, distance_m) is True

    def test_cannot_continue_just_below_threshold(self, model):
        # One Wh below the exact threshold -> False
        distance_m = 100.0
        reagent_l = 15.0
        threshold = self._threshold(model, distance_m, reagent_l)
        assert model.can_continue(threshold - 1.0, reagent_l, distance_m) is False


# ---------------------------------------------------------------------------
# TestEdgeCases
# ---------------------------------------------------------------------------

class TestEdgeCases:

    def test_zero_distance_segment(self, model):
        # 0 m straight segment -> 0 Wh and 0 L
        assert model.energy_straight(0.0, 15.0) == pytest.approx(0.0, abs=1e-9)
        assert model.reagent_consumed(0.0) == pytest.approx(0.0, abs=1e-9)

    def test_very_short_segment(self, model):
        # 1 m segment -> very small but strictly positive energy and reagent
        assert model.energy_straight(1.0, 15.0) > 0.0
        assert model.reagent_consumed(1.0) > 0.0

    def test_turn_zero_degrees(self, model):
        # 0-degree turn -> 0 Wh (effective_duration = 0)
        assert model.energy_turn(0.0, 15.0) == pytest.approx(0.0, abs=1e-9)

    def test_hover_power_nonnegative(self, model):
        # Hover power must be >= 0 for any physically valid reagent level
        for reagent in (0.0, 0.001, 1.0, 15.0, 30.0):
            assert model.hover_power(reagent) >= 0.0

    def test_usable_energy_less_than_total_capacity(self, model):
        # After applying 20% reserve, usable energy < total capacity
        # usable = 1502.2 * 0.8 = 1201.76 Wh
        assert model.usable_energy_wh() < model.drone.battery_capacity_wh
        assert model.usable_energy_wh() == pytest.approx(1201.76, rel=0.01)
