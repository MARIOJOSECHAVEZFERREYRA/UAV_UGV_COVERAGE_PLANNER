from shapely.geometry import Polygon, LineString

from ..algorithms.optimization.strategy import StrategyFactory
from ..algorithms.polygon.margin import MarginReducer
from ..algorithms.rendezvous.segmentation import MissionSegmenter
from ..algorithms.analysis import MissionAnalyzer
from ..db.drone import Drone


class MissionController:
    """
    Orchestrates the full mission planning pipeline.

    The UAG base point is a static location defined by the user.
    Drones depart from base, spray, and return to base when resources run low.
    """

    def __init__(self):
        self.last_result = None

    def run_mission_planning(self, db, polygon_points, drone_name, overrides,
                             base_point=None, strategy_name="genetic",
                             precalculated_path=None, obstacle_polygons=None):
        """
        Executes the full mission planning workflow.

        Args:
            db: SQLAlchemy Session for drone lookup.
            polygon_points (list): List of (x, y) tuples for the field boundary.
            drone_name (str): Name of the drone model in the drones table.
            overrides (dict): UI overrides — keys: swath, app_rate, speed.
            base_point (tuple, optional): (x, y) static recharge/refill location.
                Defaults to the first point of the optimized path.
            strategy_name (str): "genetic" or "simple".
            precalculated_path (LineString, optional): Existing path to reuse.
            obstacle_polygons (list, optional): List of list of (x, y) for holes.

        Returns:
            dict: polygon, safe_polygon, mission_cycles, metrics, resources,
                  best_angle, best_path, gen_stats.
        """

        # 1. Drone lookup
        drone = db.query(Drone).filter(Drone.name == drone_name).first()
        if drone is None:
            raise ValueError("Drone '{}' not found in database.".format(drone_name))

        # 2. Geometry validation
        if len(polygon_points) < 3:
            raise ValueError("Polygon must have at least 3 points.")

        holes = obstacle_polygons if obstacle_polygons else []
        polygon = Polygon(shell=polygon_points, holes=holes)

        if not polygon.is_valid:
            polygon = polygon.buffer(0)

        try:
            if not polygon.is_valid:
                cleaned = polygon.buffer(0)
                if cleaned.geom_type == 'MultiPolygon':
                    cleaned = max(cleaned.geoms, key=lambda p: p.area)
                polygon = cleaned
            polygon = polygon.simplify(0.01, preserve_topology=True)
            if not polygon.is_valid:
                polygon = polygon.buffer(0)
        except Exception as e:
            print("Warning: error sanitizing polygon: {}".format(e))

        # 3. Override resolution
        real_swath = overrides.get('swath', drone.spray_swath_m)
        app_rate   = overrides.get('app_rate', 10.0)
        speed_ms   = overrides.get('speed', drone.speed_cruise_ms)
        speed_kmh  = speed_ms * 3.6
        calc_flow_l_min = (app_rate * speed_kmh * real_swath) / 600.0

        # 4. Safety margin
        margin_h = real_swath / 2.0
        try:
            safe_polygon = MarginReducer.shrink(polygon, margin_h=margin_h)
        except Exception:
            raise ValueError("Field too small for safety margin.")

        # 5. Route optimization
        best_path  = None
        best_angle = 0
        gen_stats  = []

        if precalculated_path:
            best_path = precalculated_path
        else:
            optimizer = StrategyFactory.get_strategy(strategy_name)
            opt_result = optimizer.optimize(safe_polygon, swath_width=real_swath)
            best_angle = opt_result['angle']
            best_path  = LineString(opt_result['path']) if opt_result['path'] else None
            gen_stats  = opt_result.get('gen_stats', [])

        if not best_path:
            raise ValueError("Could not generate flight path with current settings.")

        # 6. Resolve base point
        if base_point is None:
            base_point = tuple(best_path.coords[0])

        # 7. Mission segmentation
        segmenter = MissionSegmenter(
            drone,
            target_rate_l_ha=app_rate,
            work_speed_kmh=speed_kmh,
            swath_width=real_swath,
        )
        mission_cycles = segmenter.segment_path(
            polygon=safe_polygon,
            raw_path=list(best_path.coords),
            base_point=base_point,
        )

        # 8. Metrics
        full_metrics  = MissionAnalyzer.calculate_comprehensive_metrics(
            mission_cycles, polygon, drone, flow_l_min=calc_flow_l_min
        )
        resource_data = MissionAnalyzer.plan_logistics(
            mission_cycles, drone, flow_l_min=calc_flow_l_min
        )

        return {
            "polygon":        polygon,
            "safe_polygon":   safe_polygon,
            "mission_cycles": mission_cycles,
            "metrics":        full_metrics,
            "resources":      resource_data,
            "best_angle":     best_angle,
            "best_path":      best_path,
            "gen_stats":      gen_stats,
        }
