from shapely.geometry import LineString
import numpy as np
from ..drone.energy_model import DroneEnergyModel


class MissionSegmenter:
    """
    Cuts a continuous route into operable cycles based on drone physics
    (battery energy, liquid tank). Each cycle departs from a static base
    point, performs work, and returns to base for recharge/refill.
    """

    def __init__(self, drone, target_rate_l_ha=20.0, work_speed_kmh=None, swath_width=None):
        self.drone = drone
        self.rate_l_ha = target_rate_l_ha
        self.energy_model = DroneEnergyModel(drone)

        if work_speed_kmh is not None:
            self.speed_kmh = work_speed_kmh
        else:
            self.speed_kmh = drone.speed_cruise_ms * 3.6
        self.speed_ms = self.speed_kmh / 3.6

        if swath_width is not None:
            self.swath_width = swath_width
        else:
            self.swath_width = drone.spray_swath_m

        self.liters_per_meter = (self.rate_l_ha * self.swath_width) / 10000.0
        self.tank_capacity = drone.mass_tank_full_kg

    def _is_spraying(self, p1, p2, polygon):
        """Returns True if the midpoint of segment p1-p2 is inside the field polygon."""
        line = LineString([p1[:2], p2[:2]])
        mid = line.interpolate(0.5, normalized=True)
        if polygon.boundary.distance(mid) < 0.1:
            return False
        return polygon.contains(mid)

    def segment_path(self, polygon, raw_path, base_point=None):
        """
        Segments the path into cycles. Each cycle starts and ends at base_point.

        Parameters
        ----------
        polygon : Shapely Polygon
            Safe field polygon used for spraying classification.
        raw_path : list of (x, y) or (x, y, z)
            Ordered flight waypoints from the planner.
        base_point : tuple (x, y), optional
            Static recharge/refill location. Defaults to the first path point.

        Returns
        -------
        list of cycle dicts with keys:
            path, segments, visual_groups, swath_width, base_point
        """
        if not raw_path or len(raw_path) < 2:
            return []

        if base_point is None:
            base_point = tuple(raw_path[0][:2])
        else:
            base_point = tuple(base_point[:2])

        usable_energy = self.energy_model.usable_energy_wh()
        reserve_wh = usable_energy * 0.05

        energy_remaining = usable_energy
        liquid_remaining = self.tank_capacity

        # Account for initial commute-in: base → raw_path[0]
        dist_init = np.linalg.norm(np.array(raw_path[0][:2]) - np.array(base_point))
        energy_remaining -= self.energy_model.energy_transit(dist_init, liquid_remaining)

        current_cycle_segments = []
        current_cycle_points = []
        cycles = []

        i = 0
        while i < len(raw_path) - 1:
            p1 = raw_path[i]
            p2 = raw_path[i + 1]

            is_spray = self._is_spraying(p1, p2, polygon)
            dist_step = np.linalg.norm(np.array(p1[:2]) - np.array(p2[:2]))
            liq_step = (dist_step * self.liters_per_meter) if is_spray else 0.0

            energy_step = (
                self.energy_model.energy_straight(dist_step, liquid_remaining)
                if is_spray
                else self.energy_model.energy_transit(dist_step, liquid_remaining)
            )

            liquid_after = liquid_remaining - liq_step
            dist_return = np.linalg.norm(np.array(p2[:2]) - np.array(base_point))
            energy_return = self.energy_model.energy_transit(dist_return, max(liquid_after, 0.0))

            CAN_DO = (
                energy_remaining - energy_step - energy_return >= reserve_wh
                and liquid_after >= 0.0
            )

            # Guard: always execute the step if cycle is empty (prevents infinite loop)
            if CAN_DO or not current_cycle_segments:
                current_cycle_segments.append({'p1': p1, 'p2': p2, 'spraying': is_spray})
                current_cycle_points.append(p1)
                energy_remaining -= energy_step
                liquid_remaining -= liq_step
                i += 1
            else:
                # Close cycle: drone returns to base from p1
                current_cycle_points.append(p1)
                current_cycle_segments.append({'p1': p1, 'p2': base_point, 'spraying': False})

                p_start = current_cycle_segments[0]['p1']
                current_cycle_segments.insert(0, {'p1': base_point, 'p2': p_start, 'spraying': False})

                full_path = [base_point] + current_cycle_points + [base_point]
                cycles.append(self._make_cycle(full_path, current_cycle_segments, base_point))

                # Reset resources for new cycle
                energy_remaining = usable_energy
                liquid_remaining = self.tank_capacity

                # Commute-in energy for new cycle: base → p1
                dist_commute = np.linalg.norm(np.array(p1[:2]) - np.array(base_point))
                energy_remaining -= self.energy_model.energy_transit(dist_commute, liquid_remaining)

                current_cycle_segments = []
                current_cycle_points = []
                # Do not increment i — retry this step in the new cycle

        # Close final cycle
        if current_cycle_segments:
            p_last = current_cycle_segments[-1]['p2']
            current_cycle_segments.append({'p1': p_last, 'p2': base_point, 'spraying': False})

            p_start = current_cycle_segments[0]['p1']
            current_cycle_segments.insert(0, {'p1': base_point, 'p2': p_start, 'spraying': False})

            full_path = [base_point] + current_cycle_points
            if not current_cycle_points or tuple(p_last[:2]) != tuple(current_cycle_points[-1][:2]):
                full_path.append(p_last)
            full_path.append(base_point)

            cycles.append(self._make_cycle(full_path, current_cycle_segments, base_point))

        return cycles

    def _make_cycle(self, full_path, segments, base_point):
        return {
            "type": "work",
            "path": full_path,
            "segments": segments,
            "visual_groups": self._compress_segments(segments),
            "swath_width": self.swath_width,
            "base_point": base_point,
        }

    def _compress_segments(self, segments):
        """Merges adjacent segments of the same type into visual groups."""
        if not segments:
            return []

        groups = []
        current_path = [segments[0]['p1'], segments[0]['p2']]
        current_type = segments[0]['spraying']

        for s in segments[1:]:
            if s['spraying'] == current_type:
                current_path.append(s['p2'])
            else:
                groups.append({'path': current_path, 'is_spraying': current_type})
                current_path = [s['p1'], s['p2']]
                current_type = s['spraying']

        groups.append({'path': current_path, 'is_spraying': current_type})
        return groups
