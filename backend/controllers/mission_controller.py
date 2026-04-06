from shapely.geometry import Polygon, LineString, Point

from ..algorithms.optimization.strategy import StrategyFactory
from ..algorithms.polygon.margin import MarginReducer
from ..algorithms.rendezvous.segmentation import MissionSegmenter
from ..algorithms.analysis import MissionAnalyzer
from ..algorithms.drone.energy_model import DroneEnergyModel
from ..algorithms.rendezvous.rendezvous_planner import RendezvousPlanner
from ..db.drone import Drone


class MissionController:
    """
    Orchestrates the mission planning pipeline for the static-UAG case.

    Expected optimizer contract:
        {
            "angle": float,
            "route_segments": [
                {
                    "segment_type": "sweep" | "ferry",
                    "spraying": bool,
                    "path": [(x, y), ...],
                    "distance_m": float,
                },
                ...
            ],
            "combined_path": [(x, y), ...],
            "gen_stats": [...],
            "planner_metrics": {...},   # optional
            "route_distances": {...},   # optional
        }
    """

    def __init__(self):
        self.last_result = None

    def run_mission_planning(self, db, polygon_points, drone_name, overrides, base_point,
                             strategy_name="genetic", precalculated_route_segments=None,
                             obstacle_polygons=None, ugv_polyline=None,
                             ugv_speed=2.0, ugv_t_service=300.0):
        """
        Executes the full mission planning workflow.

        Cuando se proporciona ugv_polyline (lista de puntos [(x,y),...]),
        activa la co-optimizacion implicita UAV-UGV:
        - Se construye un RendezvousPlanner con la polyline del UGV.
        - El GA evalua el fitness de cada angulo simulando el rendezvous tactico.

        ugv_polyline   : lista de (x, y) en coordenadas planas (metros).
                         Si es None, se usa el flujo estatico original.
        ugv_speed      : velocidad del UGV en m/s (default 2.0).
        ugv_t_service  : duracion del servicio manual en segundos (default 300).
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
                if cleaned.geom_type == "MultiPolygon":
                    cleaned = max(cleaned.geoms, key=lambda p: p.area)
                polygon = cleaned

            polygon = polygon.simplify(0.01, preserve_topology=True)

            if not polygon.is_valid:
                polygon = polygon.buffer(0)
        except Exception as e:
            print("Warning: error sanitizing polygon: {}".format(e))

        if polygon.is_empty:
            raise ValueError("Invalid or empty polygon after sanitization.")

        # 3. Override resolution
        real_swath = overrides.get("swath", drone.spray_swath_m)
        app_rate = overrides.get("app_rate", 10.0)
        speed_ms = overrides.get("speed", drone.speed_cruise_ms)
        speed_kmh = speed_ms * 3.6
        calc_flow_l_min = (app_rate * speed_kmh * real_swath) / 600.0

        # 4. Safety margin
        margin_h = float(overrides.get("margin", real_swath / 2.0))
        try:
            safe_polygon = MarginReducer.shrink(polygon, margin_h=margin_h)
        except Exception:
            raise ValueError("Field too small for safety margin.")

        if safe_polygon.is_empty:
            raise ValueError("Safe polygon is empty after margin reduction.")

        # 5. Base point validation for static UAG workflow
        if base_point is None:
            raise ValueError("base_point is required for mission planning.")

        base_point = (float(base_point[0]), float(base_point[1]))
        base_pt = Point(base_point)

        if safe_polygon.contains(base_pt):
            raise ValueError("base_point must not be inside the safe spray area.")

        # 6. Construir infraestructura de rendezvous si el usuario proporciono polyline del UGV
        energy_model = None
        rendezvous_planner = None

        if ugv_polyline is not None and len(ugv_polyline) >= 2:
            energy_model = DroneEnergyModel(drone)
            rendezvous_planner = RendezvousPlanner(
                ugv_polyline=ugv_polyline,
                v_ugv=float(ugv_speed),
                t_service=float(ugv_t_service),
            )

        # 7. Route optimization / route acquisition
        best_angle = 0.0
        gen_stats = []
        planner_metrics = {}
        route_distances = {}
        route_segments = []
        combined_path = []
        rv_count_opt = 0
        rv_wait_opt_s = 0.0

        if precalculated_route_segments is not None:
            route_segments = list(precalculated_route_segments)
            combined_path = self._combine_segment_paths(route_segments)
        else:
            optimizer = StrategyFactory.get_strategy(strategy_name)
            opt_result = optimizer.optimize(
                safe_polygon,
                swath_width=real_swath,
                base_point=base_point,
                obstacle_polygons=holes if holes else None,
                energy_model=energy_model,
                rendezvous_planner=rendezvous_planner,
            )

            best_angle = float(opt_result.get("angle", 0.0))
            route_segments = opt_result.get("route_segments", [])
            combined_path = opt_result.get("combined_path", [])
            gen_stats = opt_result.get("gen_stats", [])
            planner_metrics = opt_result.get("planner_metrics", {})
            route_distances = opt_result.get("route_distances", {})

            if rendezvous_planner is not None:
                rv_count_opt = int(opt_result.get("rv_count", 0))
                rv_wait_opt_s = float(opt_result.get("rv_wait", 0.0))

        if not route_segments:
            raise ValueError("Could not generate typed route segments with current settings.")

        if not combined_path:
            combined_path = self._combine_segment_paths(route_segments)

        best_path = None
        if combined_path and len(combined_path) >= 2:
            best_path = LineString(combined_path)

        # 8. Mission segmentation
        segmenter = MissionSegmenter(
            drone,
            target_rate_l_ha=app_rate,
            work_speed_kmh=speed_kmh,
            swath_width=real_swath,
        )

        if rendezvous_planner is not None:
            mission_cycles = segmenter.segment_path_mobile(
                polygon=safe_polygon,
                route_segments=route_segments,
                rendezvous_planner=rendezvous_planner,
            )
        else:
            mission_cycles = segmenter.segment_path(
                polygon=safe_polygon,
                route_segments=route_segments,
                base_point=base_point,
            )

        # 9. Metrics
        full_metrics = MissionAnalyzer.calculate_comprehensive_metrics(
            mission_cycles,
            polygon,
            drone,
            flow_l_min=calc_flow_l_min,
        )

        if rendezvous_planner is not None:
            full_metrics["rv_n_rendezvous"] = rv_count_opt
            full_metrics["rv_wait_min"] = rv_wait_opt_s / 60.0

        resource_data = MissionAnalyzer.plan_logistics(
            mission_cycles,
            drone,
            flow_l_min=calc_flow_l_min,
        )

        result = {
            "polygon": polygon,
            "safe_polygon": safe_polygon,
            "mission_cycles": mission_cycles,
            "metrics": full_metrics,
            "resources": resource_data,
            "best_angle": best_angle,
            "best_path": best_path,
            "route_segments": route_segments,
            "gen_stats": gen_stats,
            "planner_metrics": planner_metrics,
            "route_distances": route_distances,
        }

        self.last_result = result
        return result

    @staticmethod
    def _combine_segment_paths(route_segments):
        """
        Flatten ordered typed segments into one combined path for display/metrics only.
        This is a presentation helper, not a semantic source of truth.
        """
        if not route_segments:
            return []

        combined = []

        for seg in route_segments:
            path = seg.get("path", [])
            if not path:
                continue

            if not combined:
                combined.extend(path)
            else:
                if MissionController._pts_equal(combined[-1], path[0]):
                    combined.extend(path[1:])
                else:
                    combined.extend(path)

        return combined

    @staticmethod
    def _pts_equal(a, b, tol=1e-6):
        return abs(a[0] - b[0]) < tol and abs(a[1] - b[1]) < tol