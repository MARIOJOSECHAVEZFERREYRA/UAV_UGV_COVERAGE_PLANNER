class DroneEnergyModel:
    """
    Physics-based energy model for agricultural spray drones.

    Implements Actuator Disk Theory for non-linear hover power scaling
    with mass, and derives all energy, reagent, and time costs from
    first principles given a Drone ORM instance.
    """

    def __init__(self, drone, liquid_density_kg_l=1.0):
        """
        Parameters
        ----------
        drone : Drone
            SQLAlchemy Drone model instance with all spec columns populated.
        liquid_density_kg_l : float
            Density of spray liquid in kg/L. Default 1.0 (aqueous pesticide solution).
        """
        self.drone = drone
        self.liquid_density = liquid_density_kg_l

    # ------------------------------------------------------------------
    # MODELO FISICO BASE
    # ------------------------------------------------------------------

    def instant_mass(self, reagent_remaining_l):
        """
        Instantaneous drone mass in kg.

        m(t) = mass_empty_kg + mass_battery_kg + reagent_remaining_l * density
        """
        reagent_mass = reagent_remaining_l * self.liquid_density
        return self.drone.mass_empty_kg + self.drone.mass_battery_kg + reagent_mass

    def hover_power(self, reagent_remaining_l):
        """
        Instantaneous hover power in watts derived from Actuator Disk Theory.

        P_hover(m) = P_hover_empty * (m(t) / m_empty_total)^(3/2)

        The 3/2 exponent comes from momentum theory: thrust T = mg, and
        power P = T * v_induced where v_induced ~ sqrt(T). Hence P ~ T^(3/2) ~ m^(3/2).
        """
        m_empty_total = self.drone.mass_empty_kg + self.drone.mass_battery_kg
        m_current = self.instant_mass(reagent_remaining_l)
        mass_ratio = m_current / m_empty_total
        return self.drone.power_hover_empty_w * (mass_ratio ** 1.5)

    def cruise_power(self, reagent_remaining_l):
        """
        Power during straight-line cruise in watts.

        P_cruise = P_hover(m)

        Valid simplification for low agricultural speeds (~5 m/s) where
        parasitic drag power is negligible compared to hover (induced) power.
        """
        return self.hover_power(reagent_remaining_l)

    def spray_power(self, reagent_remaining_l):
        """
        Total power during active spraying in watts.

        P_spray = P_cruise(m) + spray_pump_power_w

        Adds the spray pump electrical load to the flight power.
        """
        return self.cruise_power(reagent_remaining_l) + self.drone.spray_pump_power_w

    # ------------------------------------------------------------------
    # COSTOS ENERGETICOS POR SEGMENTO
    # ------------------------------------------------------------------

    def energy_straight(self, distance_m, reagent_remaining_l):
        """
        Energy in Wh consumed flying a straight spray segment.

        E = P_spray(m) * (distance_m / speed_cruise_ms) / 3600
        """
        time_s = distance_m / self.drone.speed_cruise_ms
        energy_ws = self.spray_power(reagent_remaining_l) * time_s
        return energy_ws / 3600.0

    def energy_turn(self, angle_deg, reagent_remaining_l):
        """
        Energy in Wh consumed during a turn.

        E_turn = P_hover(m) * turn_power_factor * turn_duration_s * (angle_deg / 180) / 3600

        Models the three-phase turn (brake -> rotate -> accelerate). The
        turn_power_factor accounts for increased power above pure hover during
        the maneuver. Cost scales linearly with angle so a 90 deg turn costs
        half a 180 deg turn.
        """
        effective_duration = self.drone.turn_duration_s * (angle_deg / 180.0)
        power_during_turn = self.hover_power(reagent_remaining_l) * self.drone.turn_power_factor
        energy_ws = power_during_turn * effective_duration
        return energy_ws / 3600.0

    def energy_transit(self, distance_m, reagent_remaining_l):
        """
        Energy in Wh consumed during a transit (no spraying) leg.

        E_transit = P_cruise(m) * (distance_m / speed_max_ms) / 3600

        Uses speed_max_ms because the drone can fly faster when not spraying.
        """
        time_s = distance_m / self.drone.speed_max_ms
        energy_ws = self.cruise_power(reagent_remaining_l) * time_s
        return energy_ws / 3600.0

    # ------------------------------------------------------------------
    # COSTOS DE REAGENTE
    # ------------------------------------------------------------------

    def reagent_consumed(self, distance_m):
        """
        Reagent consumed in liters over a spray segment.

        dQ = spray_flow_rate_lpm * (distance_m / speed_cruise_ms) / 60

        Divides by 60 to convert flow from L/min to L/s before multiplying
        by the segment duration in seconds.
        """
        time_s = distance_m / self.drone.speed_cruise_ms
        return self.drone.spray_flow_rate_lpm * time_s / 60.0

    # ------------------------------------------------------------------
    # COSTOS DE TIEMPO
    # ------------------------------------------------------------------

    def time_straight(self, distance_m):
        """
        Time in seconds to fly a straight spray segment.

        T = distance_m / speed_cruise_ms
        """
        return distance_m / self.drone.speed_cruise_ms

    def time_turn(self, angle_deg):
        """
        Time in seconds to complete a turn.

        T = turn_duration_s * (angle_deg / 180)
        """
        return self.drone.turn_duration_s * (angle_deg / 180.0)

    def time_transit(self, distance_m):
        """
        Time in seconds to complete a transit leg without spraying.

        T = distance_m / speed_max_ms
        """
        return distance_m / self.drone.speed_max_ms

    # ------------------------------------------------------------------
    # CONDICIONES DE RECURSO
    # ------------------------------------------------------------------

    def usable_energy_wh(self):
        """
        Usable battery energy in Wh after subtracting the reserve margin.

        E_usable = battery_capacity_wh * (1 - battery_reserve_pct / 100)
        """
        return self.drone.battery_capacity_wh * (1.0 - self.drone.battery_reserve_pct / 100.0)

    def energy_landing_takeoff(self, reagent_remaining_l):
        """Energy required to land and take off at the rendezvous point."""
        height = self.drone.spray_height_m
        vertical_speed = self.drone.speed_vertical_ms
        descent_time = height / vertical_speed
        ascent_time = height / vertical_speed
        power = self.hover_power(reagent_remaining_l)
        energy = power * (descent_time + ascent_time) / 3600
        return energy


    def can_continue(self, energy_remaining_wh, reagent_remaining_l, distance_to_rendezvous_m):
        transit_energy = self.energy_transit(distance_to_rendezvous_m, reagent_remaining_l)
        landing_takeoff_energy = self.energy_landing_takeoff(reagent_remaining_l)
        reserve_energy = self.usable_energy_wh() * 0.10
        required_energy = transit_energy + landing_takeoff_energy + reserve_energy

        if energy_remaining_wh < required_energy:
            return False
        if reagent_remaining_l <= 0:
            return False
        return True