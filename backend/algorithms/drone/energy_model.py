class DroneEnergyModel:
    """
    Physics-based energy model for agricultural spray drones.
    """

    def __init__(self, drone, liquid_density_kg_l=1.0):
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
    # PERFIL CINEMATICO RECTILINEO (helper interno)
    # ------------------------------------------------------------------

    def _straight_profile(self, distance_m, v_target_ms, a_acc_ms2, a_dec_ms2):
        """
        Three-phase kinematic profile for a straight segment: acc / cruise / dec.

        This is a semiempirical extension of the nonlinear energy model.
        It is NOT an official manufacturer specification.

        Parameters
        ----------
        distance_m  : segment length (m)
        v_target_ms : desired cruise speed (m/s)
        a_acc_ms2   : horizontal acceleration (m/s²)
        a_dec_ms2   : horizontal deceleration (m/s²)

        Returns
        -------
        (t_acc, t_cruise, t_dec, v_peak)
        - v_peak == v_target_ms when a cruise phase exists
        - t_cruise == 0 when segment is too short to reach v_target_ms
        """
        d_acc_full = v_target_ms ** 2 / (2.0 * a_acc_ms2)
        d_dec_full = v_target_ms ** 2 / (2.0 * a_dec_ms2)

        if distance_m >= d_acc_full + d_dec_full:
            v_peak = v_target_ms
            t_acc = v_peak / a_acc_ms2
            t_dec = v_peak / a_dec_ms2
            d_cruise = distance_m - d_acc_full - d_dec_full
            t_cruise = d_cruise / v_peak
        else:
            # Segment too short to reach v_target; solve for v_peak such that
            # d = v_peak² / (2·a_acc) + v_peak² / (2·a_dec)
            v_peak = (2.0 * distance_m / (1.0 / a_acc_ms2 + 1.0 / a_dec_ms2)) ** 0.5
            t_acc = v_peak / a_acc_ms2
            t_dec = v_peak / a_dec_ms2
            t_cruise = 0.0

        return t_acc, t_cruise, t_dec, v_peak

    # ------------------------------------------------------------------
    # COSTOS ENERGETICOS POR SEGMENTO
    # ------------------------------------------------------------------

    def energy_straight(self, distance_m, reagent_remaining_l):
        """
        Energy in Wh consumed flying a straight spray segment.

        Applies a three-phase kinematic profile (acc / cruise / dec) with
        semiempirical power factors for each phase.  NOT an official DJI spec.
        """
        t_acc, t_cruise, t_dec, _ = self._straight_profile(
            distance_m,
            self.drone.speed_cruise_ms,
            self.drone.accel_horizontal_ms2,
            self.drone.decel_horizontal_ms2,
        )
        p_base = self.spray_power(reagent_remaining_l)
        energy_ws = (
            self.drone.power_accel_factor * p_base * t_acc
            + p_base * t_cruise
            + self.drone.power_decel_factor * p_base * t_dec
        )
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

        Applies a three-phase kinematic profile (acc / cruise / dec) at
        speed_max_ms with semiempirical power factors.  NOT an official DJI spec.
        """
        t_acc, t_cruise, t_dec, _ = self._straight_profile(
            distance_m,
            self.drone.speed_max_ms,
            self.drone.accel_horizontal_ms2,
            self.drone.decel_horizontal_ms2,
        )
        p_base = self.cruise_power(reagent_remaining_l)
        energy_ws = (
            self.drone.power_accel_factor * p_base * t_acc
            + p_base * t_cruise
            + self.drone.power_decel_factor * p_base * t_dec
        )
        return energy_ws / 3600.0

    # ------------------------------------------------------------------
    # COSTOS DE REAGENTE
    # ------------------------------------------------------------------

    def reagent_consumed(self, distance_m):
        """
        Reagent consumed in liters over a spray segment.

        dQ = spray_flow_rate_lpm * time_straight(distance_m) / 60
        """
        return self.drone.spray_flow_rate_lpm * self.time_straight(distance_m) / 60.0

    # ------------------------------------------------------------------
    # COSTOS DE TIEMPO
    # ------------------------------------------------------------------

    def time_straight(self, distance_m):
        """
        Time in seconds to fly a straight spray segment (three-phase profile).
        """
        t_acc, t_cruise, t_dec, _ = self._straight_profile(
            distance_m,
            self.drone.speed_cruise_ms,
            self.drone.accel_horizontal_ms2,
            self.drone.decel_horizontal_ms2,
        )
        return t_acc + t_cruise + t_dec

    def time_turn(self, angle_deg):
        """
        Time in seconds to complete a turn.

        T = turn_duration_s * (angle_deg / 180)

        Estado de integracion:
          - Fisicamente valido y testeado.
          - NO llamado por el planificador actual.
          - Ver energy_turn() para el razonamiento completo.
        """
        return self.drone.turn_duration_s * (angle_deg / 180.0)

    def time_transit(self, distance_m):
        """
        Time in seconds to complete a transit leg without spraying (three-phase profile).
        """
        t_acc, t_cruise, t_dec, _ = self._straight_profile(
            distance_m,
            self.drone.speed_max_ms,
            self.drone.accel_horizontal_ms2,
            self.drone.decel_horizontal_ms2,
        )
        return t_acc + t_cruise + t_dec

    # ------------------------------------------------------------------
    # CONDICIONES DE RECURSO
    # ------------------------------------------------------------------

    def usable_energy_wh(self):
        """
        Usable battery energy in Wh after subtracting the reserve margin.

        E_usable = battery_capacity_wh * (1 - battery_reserve_pct / 100)
        """
        return self.drone.battery_capacity_wh * (1.0 - self.drone.battery_reserve_pct / 100.0)

    def reserve_wh_static(self):
        """
        Operational reserve for static-base missions (20% of usable energy).

        Unified with the mobile reserve: both mission types hold back 20% of
        usable energy as a safety margin before declaring a segment infeasible.
        """
        return self.usable_energy_wh() * 0.20

    def reserve_wh_mobile(self):
        """
        Operational reserve for mobile-rendezvous missions (20% of usable energy).

        In dynamic mode the drone may need to fly to a rendezvous point whose
        exact location is not yet known when the check is performed.  The 20%
        reserve guarantees that enough energy remains to reach any feasible point
        on the UGV polyline after completing the current segment.
        """
        return self.usable_energy_wh() * 0.20

    def energy_landing_takeoff(self, reagent_remaining_l):
        """
        Energy in Wh required to land and take off at a service point.

        Models descent + ascent at spray height using vertical speed.
        Applied at rendezvous points where the drone physically lands on or
        near the UGV to be refueled and reloaded.
        """
        height = self.drone.spray_height_m
        vertical_speed = self.drone.speed_vertical_ms
        descent_time = height / vertical_speed
        ascent_time = height / vertical_speed
        power = self.hover_power(reagent_remaining_l)
        return power * (descent_time + ascent_time) / 3600.0

    # ------------------------------------------------------------------
    # COSTOS DE RECUPERACION (trayecto hasta el punto de servicio)
    # ------------------------------------------------------------------

    def energy_to_service_static(self, dist_m, liquid_rem):
        """
        Energy in Wh needed to return to the fixed base.

        Preserves the existing static-mode cost model: transit energy only,
        consistent with how segment_path() has always tracked the return budget.
        """
        return self.energy_transit(dist_m, liquid_rem)

    def energy_to_service_dynamic(self, dist_m, liquid_rem):
        """
        Energy in Wh needed to reach a mobile rendezvous point.

        Includes transit flight plus landing/takeoff at the rendezvous, since
        the drone physically lands on or near the UGV for servicing.
        """
        return self.energy_transit(dist_m, liquid_rem) + self.energy_landing_takeoff(liquid_rem)

    # ------------------------------------------------------------------
    # PREDICADOS DE FACTIBILIDAD POST-SEGMENTO
    # ------------------------------------------------------------------

    def feasible_after_segment_static(self, energy_rem, liquid_rem,
                                      energy_step, liq_step, dist_to_base):
        """
        True if the drone can execute the next segment and still return to the
        fixed base with enough reserve.

        Parameters
        ----------
        energy_rem  : Wh currently available.
        liquid_rem  : liters currently available.
        energy_step : Wh cost of executing the candidate segment.
        liq_step    : liters consumed by the candidate segment.
        dist_to_base: straight-line distance (m) from the segment end to base.
        """
        liquid_after = liquid_rem - liq_step
        e_service = self.energy_to_service_static(dist_to_base, max(liquid_after, 0.0))
        return (
            energy_rem - energy_step - e_service >= self.reserve_wh_static()
            and liquid_after >= 0.0
        )

    def feasible_after_segment_dynamic(self, energy_rem, liquid_rem,
                                       energy_step, liq_step, dist_to_rv):
        """
        True if the drone can execute the next segment and still reach a
        rendezvous point with enough reserve.

        Includes landing/takeoff cost at the rendezvous (physically required
        for battery swap and reagent reload on the UGV).

        Parameters
        ----------
        energy_rem : Wh currently available.
        liquid_rem : liters currently available.
        energy_step: Wh cost of executing the candidate segment.
        liq_step   : liters consumed by the candidate segment.
        dist_to_rv : UAV flight distance (m) to the best rendezvous candidate.
                     Pass float('inf') when no feasible rendezvous exists.
        """
        liquid_after = liquid_rem - liq_step
        e_service = self.energy_to_service_dynamic(dist_to_rv, max(liquid_after, 0.0))
        return (
            energy_rem - energy_step - e_service >= self.reserve_wh_mobile()
            and liquid_after >= 0.0
        )

    def can_continue(self, energy_remaining_wh, reagent_remaining_l, distance_to_rendezvous_m):
        """
        True if the drone can reach the rendezvous point *from its current
        position* (no additional segment executed).

        This answers a punctual reachability question: "given my current
        state, can I still get to a service point?" It is NOT a substitute
        for feasible_after_segment_dynamic, which also accounts for the energy
        cost of the next work segment before the recovery leg.

        Reserve used: reserve_wh_mobile() (20% of usable energy).
        """
        transit_energy = self.energy_transit(distance_to_rendezvous_m, reagent_remaining_l)
        landing_takeoff_energy = self.energy_landing_takeoff(reagent_remaining_l)
        reserve_energy = self.reserve_wh_mobile()
        required_energy = transit_energy + landing_takeoff_energy + reserve_energy
        return energy_remaining_wh >= required_energy and reagent_remaining_l > 0