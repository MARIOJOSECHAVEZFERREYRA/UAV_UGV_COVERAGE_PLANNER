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
        spray_swath_min_m=4.0,
        spray_swath_max_m=9.0,
        spray_height_m=2.5,
        spray_pump_power_w=200.0,
        service_time_s=120.0,
        # Semiempirical kinematic extension (not official DJI spec)
        accel_horizontal_ms2=1.5,
        decel_horizontal_ms2=1.5,
        power_accel_factor=1.15,
        power_decel_factor=1.05,
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
        # 100 m at full tank (30 L), three-phase profile:
        #   v_c=5 m/s, a_acc=a_dec=1.5 m/s²
        #   d_acc = 5²/(2·1.5) = 8.333 m  <  100 m  → cruise phase exists
        #   t_acc = 5/1.5 = 10/3 s,  t_dec = 10/3 s
        #   d_cruise = 100 - 16.667 = 83.333 m,  t_cruise = 50/3 s
        #   E = P_spray * (1.15·10/3 + 50/3 + 1.05·10/3) / 3600
        #     = P_spray * 24 / 3600
        p_spray = _hover_ref(66.5) + 200.0
        expected = p_spray * 24.0 / 3600.0
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
        # Transit uses speed_max (10 m/s), three-phase profile, no pump power.
        # For 100 m at full tank, a=1.5 m/s²:
        #   d_acc = 10²/(2·1.5) = 33.333 m  <  100 m  → cruise phase exists
        #   t_acc = 10/1.5 = 20/3 s,  t_dec = 20/3 s
        #   d_cruise = 100 - 66.667 = 33.333 m,  t_cruise = 10/3 s
        #   E = P_cruise * (1.15·20/3 + 10/3 + 1.05·20/3) / 3600
        #     = P_cruise * 18 / 3600
        p_cruise = _hover_ref(66.5)
        expected_transit = p_cruise * 18.0 / 3600.0
        assert model.energy_transit(100.0, 30.0) == pytest.approx(expected_transit, rel=0.01)

    def test_energy_landing_takeoff(self, model):
        # height = 2.5 m, vert_speed = 3 m/s
        # descent_time = ascent_time = 2.5/3 = 0.8333 s  -> total = 1.6667 s
        # E = hover_power(30) * 1.6667 / 3600
        t_total = 2.0 * (2.5 / 3.0)
        expected = _hover_ref(66.5) * t_total / 3600.0
        assert model.energy_landing_takeoff(30.0) == pytest.approx(expected, rel=0.01)


# ---------------------------------------------------------------------------
# TestStraightProfile  (semiempirical kinematic extension)
# ---------------------------------------------------------------------------

class TestStraightProfile:
    """
    Unit tests for _straight_profile and the three-phase energy model.
    Validates the semiempirical extension; not an official manufacturer spec.
    """

    def test_long_segment_has_cruise_phase(self, model):
        # 100 m spray segment: d_acc+d_dec = 16.667 m < 100 m → cruise exists
        t_acc, t_cruise, t_dec, v_peak = model._straight_profile(
            100.0, 5.0, 1.5, 1.5
        )
        assert t_cruise > 0.0
        assert v_peak == pytest.approx(5.0, rel=0.001)

    def test_short_segment_no_cruise_phase(self, model):
        # 10 m at v=5, a=1.5: d_acc+d_dec = 16.667 m > 10 m → no cruise
        t_acc, t_cruise, t_dec, v_peak = model._straight_profile(
            10.0, 5.0, 1.5, 1.5
        )
        assert t_cruise == pytest.approx(0.0, abs=1e-12)
        assert v_peak < 5.0

    def test_short_segment_distance_conservation(self, model):
        # Reconstructed distance from v_peak must equal input distance
        d = 10.0
        t_acc, t_cruise, t_dec, v_peak = model._straight_profile(d, 5.0, 1.5, 1.5)
        d_reconstructed = 0.5 * v_peak * t_acc + 0.5 * v_peak * t_dec
        assert d_reconstructed == pytest.approx(d, rel=0.001)

    def test_long_segment_distance_conservation(self, model):
        # For long segment, sum of phase distances must equal total
        d = 100.0
        v = 5.0
        a = 1.5
        t_acc, t_cruise, t_dec, v_peak = model._straight_profile(d, v, a, a)
        d_acc = 0.5 * v_peak * t_acc
        d_dec = 0.5 * v_peak * t_dec
        d_cruise = v_peak * t_cruise
        assert d_acc + d_cruise + d_dec == pytest.approx(d, rel=0.001)

    def test_zero_distance_profile(self, model):
        t_acc, t_cruise, t_dec, v_peak = model._straight_profile(0.0, 5.0, 1.5, 1.5)
        assert t_acc == pytest.approx(0.0, abs=1e-12)
        assert t_cruise == pytest.approx(0.0, abs=1e-12)
        assert t_dec == pytest.approx(0.0, abs=1e-12)
        assert v_peak == pytest.approx(0.0, abs=1e-12)

    def test_three_phase_energy_greater_than_constant_speed(self, model):
        # Acc/dec factors > 1 mean 3-phase costs MORE than naive constant-speed model
        d = 100.0
        reagent = 15.0
        p_spray = model.spray_power(reagent)
        naive_wh = p_spray * (d / model.drone.speed_cruise_ms) / 3600.0
        assert model.energy_straight(d, reagent) > naive_wh

    def test_transit_three_phase_energy_greater_than_constant_speed(self, model):
        d = 100.0
        reagent = 15.0
        p_cruise = model.cruise_power(reagent)
        naive_wh = p_cruise * (d / model.drone.speed_max_ms) / 3600.0
        assert model.energy_transit(d, reagent) > naive_wh

    def test_time_straight_longer_than_constant_speed(self, model):
        # Three-phase total time > constant-speed time (acc/dec add overhead)
        d = 100.0
        t_3phase = model.time_straight(d)
        t_naive = d / model.drone.speed_cruise_ms
        assert t_3phase > t_naive

    def test_time_transit_longer_than_constant_speed(self, model):
        d = 100.0
        t_3phase = model.time_transit(d)
        t_naive = d / model.drone.speed_max_ms
        assert t_3phase > t_naive

    def test_transit_energy_below_straight_same_conditions(self, model):
        # energy_transit has no pump load; energy_straight adds spray_pump_power_w.
        # For same distance and reagent, transit must cost less than straight.
        for d in (50.0, 100.0, 500.0):
            assert model.energy_transit(d, 15.0) < model.energy_straight(d, 15.0)

    def test_short_segment_energy_numerical(self, model):
        # 10 m spray segment: d < d_acc + d_dec = 16.667 m -> no cruise
        #   a = 1.5 m/s², v_peak = sqrt(1.5 * 10) = sqrt(15) ≈ 3.873 m/s
        #   t_acc = v_peak / 1.5,  t_dec = v_peak / 1.5
        #   E = (k_acc * t_acc + k_dec * t_dec) * P_spray(15) / 3600
        import math
        v_peak = math.sqrt(1.5 * 10.0)
        t_acc = v_peak / 1.5
        t_dec = v_peak / 1.5
        p_spray = model.spray_power(15.0)
        expected = (1.15 * p_spray * t_acc + 1.05 * p_spray * t_dec) / 3600.0
        assert model.energy_straight(10.0, 15.0) == pytest.approx(expected, rel=0.001)


# ---------------------------------------------------------------------------
# TestReagent
# ---------------------------------------------------------------------------

class TestReagent:

    def test_reagent_consumed_basic(self, model):
        # 100 m, three-phase profile: time_straight(100) = 10/3 + 50/3 + 10/3 = 70/3 s
        # Q = 8 L/min * (70/3) s / 60 = 8 * 70 / 180 ≈ 3.111 L
        # Fixed-flow-rate pump assumption: reagent = flow * actual_flight_time / 60
        expected = 8.0 * model.time_straight(100.0) / 60.0
        assert model.reagent_consumed(100.0) == pytest.approx(expected, rel=0.001)

    def test_reagent_consumed_uses_three_phase_time(self, model):
        # Fixed-flow pump: Q = spray_flow_rate_lpm * time_straight(d) / 60
        # Consistent with energy_straight which also integrates over three-phase time.
        for d in (10.0, 50.0, 100.0, 500.0):
            expected = model.drone.spray_flow_rate_lpm * model.time_straight(d) / 60.0
            assert model.reagent_consumed(d) == pytest.approx(expected, rel=0.001)

    def test_reagent_consumed_greater_than_naive(self, model):
        # Three-phase time > d/v_cruise, so reagent > naive constant-speed estimate
        naive = model.drone.spray_flow_rate_lpm * (100.0 / model.drone.speed_cruise_ms) / 60.0
        assert model.reagent_consumed(100.0) > naive

    def test_reagent_consumed_monotone(self, model):
        # More distance -> more reagent (monotone, not necessarily linear)
        assert model.reagent_consumed(200.0) > model.reagent_consumed(100.0)
        assert model.reagent_consumed(100.0) > model.reagent_consumed(50.0)


# ---------------------------------------------------------------------------
# TestCanContinue
# ---------------------------------------------------------------------------

class TestCanContinue:

    def _threshold(self, model, distance_m, reagent_l):
        """Exact energy threshold used by can_continue (reserve = reserve_wh_mobile = 20%)."""
        transit = model.energy_transit(distance_m, reagent_l)
        lto = model.energy_landing_takeoff(reagent_l)
        reserve = model.reserve_wh_mobile()
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

    def test_cannot_continue_infinite_distance(self, model):
        # No reachable rendezvous (dist = inf) -> always False
        usable = model.usable_energy_wh()
        assert model.can_continue(usable, 30.0, float('inf')) is False


# ---------------------------------------------------------------------------
# TestFeasibilityMethods
# ---------------------------------------------------------------------------

class TestFeasibilityMethods:
    """
    Tests for feasible_after_segment_static and feasible_after_segment_dynamic.

    Both methods answer: "if I execute this segment now, will I still be able
    to reach a service point afterwards?"
    """

    # --- feasible_after_segment_static ---

    def test_static_feasible_full_resources(self, model):
        # Full battery, full tank, short segment, close base -> True
        usable = model.usable_energy_wh()
        energy_step = model.energy_straight(50.0, 30.0)
        liq_step = 50.0 * (8.0 / (5.0 * 60.0))   # 50 m at cruise
        assert model.feasible_after_segment_static(
            usable, 30.0, energy_step, liq_step, 100.0
        ) is True

    def test_static_infeasible_no_liquid(self, model):
        # liq_step exceeds available liquid -> liquid_after < 0 -> False
        usable = model.usable_energy_wh()
        assert model.feasible_after_segment_static(
            usable, 0.001, 0.0, 0.002, 0.0
        ) is False

    def test_static_infeasible_base_unreachable(self, model):
        # Almost no battery left, huge return distance -> False
        assert model.feasible_after_segment_static(
            10.0, 15.0, 0.0, 0.0, 50000.0
        ) is False

    def test_static_barely_feasible(self, model):
        # Provide just above the required threshold -> True
        dist_to_base = 200.0
        reagent_l = 15.0
        e_service = model.energy_to_service_static(dist_to_base, reagent_l)
        reserve = model.reserve_wh_static()
        energy_step = model.energy_transit(50.0, reagent_l)
        # Add a small epsilon to avoid floating-point subtraction going below reserve
        energy_rem = energy_step + e_service + reserve + 1e-6
        assert model.feasible_after_segment_static(
            energy_rem, reagent_l, energy_step, 0.0, dist_to_base
        ) is True

    def test_static_just_below_threshold(self, model):
        dist_to_base = 200.0
        reagent_l = 15.0
        e_service = model.energy_to_service_static(dist_to_base, reagent_l)
        reserve = model.reserve_wh_static()
        energy_step = model.energy_transit(50.0, reagent_l)
        energy_rem = energy_step + e_service + reserve - 0.01
        assert model.feasible_after_segment_static(
            energy_rem, reagent_l, energy_step, 0.0, dist_to_base
        ) is False

    # --- feasible_after_segment_dynamic ---

    def test_dynamic_feasible_full_resources(self, model):
        usable = model.usable_energy_wh()
        energy_step = model.energy_straight(50.0, 30.0)
        liq_step = 50.0 * (8.0 / (5.0 * 60.0))
        assert model.feasible_after_segment_dynamic(
            usable, 30.0, energy_step, liq_step, 100.0
        ) is True

    def test_dynamic_infeasible_no_rv(self, model):
        # No feasible rendezvous -> dist_to_rv = inf -> False
        usable = model.usable_energy_wh()
        assert model.feasible_after_segment_dynamic(
            usable, 30.0, 0.0, 0.0, float('inf')
        ) is False

    def test_dynamic_infeasible_no_liquid(self, model):
        # liq_step exceeds available liquid -> liquid_after < 0 -> False
        usable = model.usable_energy_wh()
        assert model.feasible_after_segment_dynamic(
            usable, 0.001, 0.0, 0.002, 50.0
        ) is False

    def test_dynamic_includes_lto_vs_static(self, model):
        # For the same distance, dynamic threshold is strictly higher than static
        # because it adds energy_landing_takeoff on top of transit.
        dist = 200.0
        reagent_l = 15.0
        e_static = model.energy_to_service_static(dist, reagent_l)
        e_dynamic = model.energy_to_service_dynamic(dist, reagent_l)
        assert e_dynamic > e_static
        assert e_dynamic == pytest.approx(
            e_static + model.energy_landing_takeoff(reagent_l), rel=0.01
        )

    def test_dynamic_barely_feasible(self, model):
        dist_to_rv = 200.0
        reagent_l = 15.0
        e_service = model.energy_to_service_dynamic(dist_to_rv, reagent_l)
        reserve = model.reserve_wh_mobile()
        energy_step = model.energy_transit(50.0, reagent_l)
        energy_rem = energy_step + e_service + reserve
        assert model.feasible_after_segment_dynamic(
            energy_rem, reagent_l, energy_step, 0.0, dist_to_rv
        ) is True

    def test_dynamic_just_below_threshold(self, model):
        dist_to_rv = 200.0
        reagent_l = 15.0
        e_service = model.energy_to_service_dynamic(dist_to_rv, reagent_l)
        reserve = model.reserve_wh_mobile()
        energy_step = model.energy_transit(50.0, reagent_l)
        energy_rem = energy_step + e_service + reserve - 0.01
        assert model.feasible_after_segment_dynamic(
            energy_rem, reagent_l, energy_step, 0.0, dist_to_rv
        ) is False


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
