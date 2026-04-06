import numpy as np
from .drone.energy_model import DroneEnergyModel


class MissionAnalyzer:
    """
    Analyzes and compares missions (Static vs Mobile) and generates logistic plans.

    All methods that need drone parameters accept a Drone ORM instance directly.
    """

    @staticmethod
    def calculate_comprehensive_metrics(cycles, polygon, drone, flow_l_min=None):
        """
        Calculates detailed metrics for the report:
        - Productivity (ha/h)
        - Actual Dosage (L/ha)
        - Flight vs Idle Times (min)

        flow_l_min: effective flow rate in L/min used during the mission.
                    Defaults to drone.spray_flow_rate_lpm if not provided.
        """
        total_dist = 0.0
        spray_dist = 0.0
        deadhead_dist = 0.0
        flight_time_sec = 0.0

        for c in cycles:
            for s in c['segments']:
                d = float(np.linalg.norm(np.array(s['p1'][:2]) - np.array(s['p2'][:2])))
                total_dist += d
                seg_type = s.get('segment_type', 'ferry')
                if seg_type == 'sweep':
                    spray_dist += d
                    # Sweeps vuelan a velocidad de aspersion
                    flight_time_sec += d / drone.speed_cruise_ms
                else:
                    deadhead_dist += d
                    # Ferries y deadheads vuelan a velocidad maxima
                    flight_time_sec += d / drone.speed_max_ms

        area_m2 = polygon.area
        area_ha = area_m2 / 10000.0

        flight_time_min = flight_time_sec / 60.0

        reload_time_min = len(cycles) * 5.0
        total_op_time_min = flight_time_min + reload_time_min

        prod_ha_hr = 0.0
        if total_op_time_min > 0:
            prod_ha_hr = area_ha / (total_op_time_min / 60.0)

        effective_flow = flow_l_min if flow_l_min is not None else drone.spray_flow_rate_lpm
        spray_time_min = (spray_dist / drone.speed_cruise_ms) / 60.0
        total_vol_l = spray_time_min * effective_flow

        real_dosage = 0
        if area_ha > 0:
            real_dosage = total_vol_l / area_ha

        return {
            "area_ha": area_ha,
            "flight_time_min": flight_time_min,
            "total_op_time_min": total_op_time_min,
            "productivity_ha_hr": prod_ha_hr,
            "real_dosage_l_ha": real_dosage,
            "spray_dist_km": spray_dist / 1000.0,
            "dead_dist_km": deadhead_dist / 1000.0,
            "efficiency_ratio": (spray_dist / total_dist * 100) if total_dist else 0
        }

    @staticmethod
    def plan_logistics(mobile_cycles, drone, flow_l_min=None):
        """
        Generates the resource plan (battery packs needed, liquid mix, stop list).

        flow_l_min: effective flow rate in L/min used during the mission.
                    Defaults to drone.spray_flow_rate_lpm if not provided.
        """
        tank_l = drone.mass_tank_full_kg  # numerically equal to liters at density 1.0

        # Flight time estimate from energy model at full tank (conservative)
        energy_model = DroneEnergyModel(drone)
        full_reagent_l = tank_l
        avg_power_w = energy_model.spray_power(full_reagent_l)
        flight_time_min = energy_model.usable_energy_wh() / avg_power_w * 60.0

        charge_time_min = drone.battery_charge_time_min

        packs_needed = 1 + int(np.ceil(charge_time_min / flight_time_min))

        effective_flow = flow_l_min if flow_l_min is not None else drone.spray_flow_rate_lpm
        work_speed_ms = drone.speed_cruise_ms

        stops = []
        total_liter_mix = 0.0

        for i, cycle in enumerate(mobile_cycles):
            stop_type = "Start" if i == 0 else "Stop {}".format(i)

            spray_dist_m = 0
            for s in cycle['segments']:
                if s['spraying']:
                    d = np.linalg.norm(np.array(s['p1'][:2]) - np.array(s['p2'][:2]))
                    spray_dist_m += d

            spray_time_min = (spray_dist_m / work_speed_ms) / 60.0
            liters_consumed = spray_time_min * effective_flow

            if liters_consumed > tank_l:
                liters_consumed = tank_l

            total_liter_mix += liters_consumed

            if i == 0:
                action_desc = "Fill Tank ({:.1f}L)".format(liters_consumed)
            else:
                action_desc = "Bat. Swap + Refill {:.1f}L".format(liters_consumed)

            stops.append({
                "name": stop_type,
                "action": action_desc,
                "notes": "Coverage: {:.0f}m".format(spray_dist_m)
            })

        return {
            "battery_packs": packs_needed,
            "total_mix_l": total_liter_mix,
            "stops": stops
        }
