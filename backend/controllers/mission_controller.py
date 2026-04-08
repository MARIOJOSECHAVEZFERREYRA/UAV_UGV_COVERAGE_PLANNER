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
Base orchestrator for mission planning. Subclasses define rendezvous
infrastructure and segmentation strategy through hook methods.

Responsibility map:
    MissionController        — orchestrates the planning pipeline, creates the
                            shared DroneEnergyModel, and wires dependencies.
    StaticMissionController  — static mission mode; base_point is the permanent
                            logistics base.
    DynamicMissionController — dynamic mission mode; builds RendezvousPlanner,
                            and treats base_point only as the UAV's initial
                            takeoff position.
    DroneEnergyModel         — physics model: energy costs and feasibility
                            predicates (feasible_after_segment_*).
    RendezvousPlanner        — rendezvous logic: given UAV/UGV states, selects
                            the best feasible service point on the UGV route.
    MissionSegmenter         — splits the route into mission cycles using the
                            energy model and, in dynamic mode, the
                            RendezvousPlanner.

Optimizer contract:
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
        "planner_metrics": {...},
        "route_distances": {...},
    }
"""

    def __init__(self):
        self.last_result = None

    def run_mission_planning(self, db, polygon_points, drone_name, overrides, base_point, strategy_name="genetic", precalculated_route_segments=None, obstacle_polygons=None):
        
        # 1. Get the drone info and build the single energy model used across the mission.
        drone, energy_model = self._load_drone_and_energy_model(db, drone_name)
        
        # 2. Geometry validation + sanitization
        polygon, holes = self._build_polygon(polygon_points, obstacle_polygons)
        
        # 3. Override mission parameters from frontend panel
        real_swath, app_rate, speed_kmh, calc_flow_l_min, margin_h = \
            self._resolve_operational_params(drone, overrides)

        # 4. Safety margin
        safe_polygon = self._build_safe_polygon(polygon, margin_h)

        # 5. Base point validation
        base_point = self._validate_base_point(base_point, safe_polygon)

        # 6. Rendezvous infrastructure
        #Set up rendezvous components for dynamic missions.
        #Static mode returns (None, None).
        opt_energy_model, rendezvous_planner = self._init_rendezvous(drone, energy_model)

        # 7. Route optimization / route acquisition
        #Get the mission route either from precomputed segments or by running the selected optimizer strategy.
        route_segments, combined_path, best_angle, gen_stats, planner_metrics, \
            route_distances, opt_result = self._acquire_route(
                strategy_name, safe_polygon, real_swath, base_point,
                opt_energy_model, rendezvous_planner, precalculated_route_segments,
            )

        best_path = self._build_best_path(combined_path)

        # 8. Mission segmentation
        #Build the segmenter and split the route into mission cycles.
        #The subclass chooses static or dynamic segmentation.
        segmenter = self._build_segmenter(drone, app_rate, speed_kmh, real_swath, energy_model)
        mission_cycles = self._segment(segmenter, safe_polygon, route_segments, base_point, rendezvous_planner)

        # 9. Metrics
        full_metrics, resource_data = self._compute_metrics_and_resources(
            mission_cycles, polygon, drone, calc_flow_l_min, opt_result,
        )

        result = self._build_result(
            polygon, safe_polygon, mission_cycles, full_metrics, resource_data,
            best_angle, best_path, route_segments, gen_stats, planner_metrics, route_distances,
        )
        self.last_result = result
        return result

    # ------------------------------------------------------------------
    # Pipeline helpers
    # ------------------------------------------------------------------

    def _load_drone_and_energy_model(self, db, drone_name):
        drone = db.query(Drone).filter(Drone.name == drone_name).first()
        if drone is None:
            raise ValueError("Drone '{}' not found in database.".format(drone_name))
        return drone, DroneEnergyModel(drone)

    def _build_polygon(self, polygon_points, obstacle_polygons=None):
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

        return polygon, holes

    def _resolve_operational_params(self, drone, overrides):
        real_swath = overrides.get("swath", drone.spray_swath_max_m)
        app_rate = overrides.get("app_rate", drone.app_rate_default_l_ha)
        speed_ms = overrides.get("speed", drone.speed_cruise_ms)
        margin_h = float(overrides.get("margin", real_swath / 2.0))

        if real_swath <= 0:
            raise ValueError("Spray width must be greater than 0.")
        if real_swath < drone.spray_swath_min_m or real_swath > drone.spray_swath_max_m:
            raise ValueError(
                f"Spray width {real_swath:.1f} m out of range "
                f"[{drone.spray_swath_min_m:.1f}, {drone.spray_swath_max_m:.1f}] for drone '{drone.name}'."
            )

        if app_rate <= 0:
            raise ValueError("Application rate must be greater than 0.")
        if app_rate < drone.app_rate_min_l_ha or app_rate > drone.app_rate_max_l_ha:
            raise ValueError(
                f"Application rate {app_rate:.1f} L/ha out of range "
                f"[{drone.app_rate_min_l_ha:.1f}, {drone.app_rate_max_l_ha:.1f}] "
                f"for drone '{drone.name}'."
            )

        if speed_ms <= 0:
            raise ValueError("Speed must be greater than 0.")
        if speed_ms > drone.speed_max_ms:
            raise ValueError(
                f"Speed {speed_ms:.1f} m/s exceeds drone max ({drone.speed_max_ms:.1f} m/s)."
            )

        if margin_h < 0:
            raise ValueError("Margin must be >= 0.")

        speed_kmh = speed_ms * 3.6
        calc_flow_l_min = (app_rate * speed_kmh * real_swath) / 600.0
        return real_swath, app_rate, speed_kmh, calc_flow_l_min, margin_h

    def _build_safe_polygon(self, polygon, margin_h):
        try:
            safe_polygon = MarginReducer.shrink(polygon, margin_h=margin_h)
        except Exception:
            raise ValueError("Field too small for safety margin.")

        if safe_polygon.is_empty:
            raise ValueError("Safe polygon is empty after margin reduction.")

        return safe_polygon

    def _validate_base_point(self, base_point, safe_polygon):
        if base_point is None:
            raise ValueError("base_point is required for mission planning.")

        base_point = (float(base_point[0]), float(base_point[1]))

        if safe_polygon.contains(Point(base_point)):
            raise ValueError("base_point must not be inside the safe spray area.")

        return base_point

    def _acquire_route(self, strategy_name, safe_polygon, real_swath, base_point, opt_energy_model, rendezvous_planner, precalculated_route_segments):
        best_angle = 0.0
        gen_stats = []
        planner_metrics = {}
        route_distances = {}
        opt_result = {}

        if precalculated_route_segments is not None:

            route_segments = list(precalculated_route_segments)
            combined_path = self._combine_segment_paths(route_segments)
        else:
            optimizer = StrategyFactory.get_strategy(strategy_name)
            opt_result = optimizer.optimize(
                safe_polygon,
                swath_width=real_swath,
                base_point=base_point,
                energy_model=opt_energy_model,
                rendezvous_planner=rendezvous_planner,
            )

            best_angle = float(opt_result.get("angle", 0.0))
            route_segments = opt_result.get("route_segments", [])
            combined_path = opt_result.get("combined_path", [])
            gen_stats = opt_result.get("gen_stats", [])
            planner_metrics = opt_result.get("planner_metrics", {})
            route_distances = opt_result.get("route_distances", {})

        if not route_segments:
            raise ValueError("Could not generate typed route segments with current settings.")

        if not combined_path:
            combined_path = self._combine_segment_paths(route_segments)

        return (route_segments, combined_path, best_angle, gen_stats,
                planner_metrics, route_distances, opt_result)

    def _build_best_path(self, combined_path):
        if combined_path and len(combined_path) >= 2:
            return LineString(combined_path)
        return None

    def _build_segmenter(self, drone, app_rate, speed_kmh, real_swath, energy_model):
        return MissionSegmenter(
            drone,
            target_rate_l_ha=app_rate,
            work_speed_kmh=speed_kmh,
            swath_width=real_swath,
            energy_model=energy_model,
        )

    def _compute_metrics_and_resources(self, mission_cycles, polygon, drone, calc_flow_l_min, opt_result):
        full_metrics = MissionAnalyzer.calculate_comprehensive_metrics(
            mission_cycles,
            polygon,
            drone,
            flow_l_min=calc_flow_l_min,
        )
        self._augment_metrics(full_metrics, opt_result, mission_cycles)

        resource_data = MissionAnalyzer.plan_logistics(
            mission_cycles,
            drone,
            flow_l_min=calc_flow_l_min,
        )
        return full_metrics, resource_data

    def _build_result(self, polygon, safe_polygon, mission_cycles, full_metrics, resource_data, best_angle, best_path, route_segments,gen_stats, planner_metrics, route_distances):
        return {
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

    # ------------------------------------------------------------------
    # Subclass hooks for static and dynamic missions
    # ------------------------------------------------------------------

    def _init_rendezvous(self, drone, energy_model):
        """Hook for mode-specific rendezvous setup."""
        return None, None

    def _segment(self, segmenter, safe_polygon, route_segments, base_point, rendezvous_planner):
        """Hook for mode-specific mission segmentation."""
        raise NotImplementedError

    def _augment_metrics(self, full_metrics, opt_result, mission_cycles):
        """Hook for mode-specific metric augmentation."""
        pass

    # ------------------------------------------------------------------
    # Static utilities
    # ------------------------------------------------------------------

    @staticmethod
    #merges paths
    def _combine_segment_paths(route_segments):
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
    #decides whether the boundary point between two paths is already duplicated
    def _pts_equal(a, b, tol=1e-6):
        return abs(a[0] - b[0]) < tol and abs(a[1] - b[1]) < tol


class StaticMissionController(MissionController):#
    """
    base_point is the permanent logistics base: the drone departs from it,
    returns to it between every cycle, and the segmenter uses it as the fixed
    service reference when evaluating feasibility and closing cycle boundaries.
    """

    def _init_rendezvous(self, drone, energy_model):
        # Pass energy_model to the optimizer so it can estimate per-cycle
        # deadhead costs during fitness evaluation (static deadhead mode).
        return energy_model, None

    #ignores rendezvous_planner
    def _segment(self, segmenter, safe_polygon, route_segments, base_point, rendezvous_planner):
        return segmenter.segment_path(
            polygon=safe_polygon,
            route_segments=route_segments,
            base_point=base_point,
        )


class DynamicMissionController(MissionController):
    """
    base_point is the UAV's initial takeoff position only. It is NOT a
    permanent logistics base: once the mission starts, all service stops
    occur at rendezvous points selected dynamically on the UGV polyline by
    RendezvousPlanner. The UAV never returns to base_point mid-mission.

    ugv_polyline  : list of (x, y) in flat-plane coordinates (meters).
    ugv_speed     : UGV speed in m/s.
    ugv_t_service : manual service duration in seconds.
    """

    def __init__(self, ugv_polyline, ugv_speed=2.0, ugv_t_service=300.0):
        super().__init__()
        self._ugv_polyline = ugv_polyline
        self._ugv_speed = float(ugv_speed)
        self._ugv_t_service = float(ugv_t_service)

    def _init_rendezvous(self, drone, energy_model):
        # Reuse the shared energy_model created by the base controller so the
        # GA optimizer and the segmenter work with the same physics instance.
        rendezvous_planner = RendezvousPlanner(
            ugv_polyline=self._ugv_polyline,
            v_ugv=self._ugv_speed,
            t_service=self._ugv_t_service,
        )
        return energy_model, rendezvous_planner

    def _segment(self, segmenter, safe_polygon, route_segments, base_point, rendezvous_planner):
        return segmenter.segment_path_mobile(
            polygon=safe_polygon,
            route_segments=route_segments,
            rendezvous_planner=rendezvous_planner,
        )

    def _augment_metrics(self, full_metrics, opt_result, mission_cycles):
        # Extraer métricas de rendezvous de la misión realmente segmentada.
        # Cada ciclo cerrado por segment_path_mobile() tiene 'rv_wait_s';
        # el último ciclo (misión completa) no tiene esa clave.
        rv_waits = [c['rv_wait_s'] for c in mission_cycles if 'rv_wait_s' in c]
        full_metrics["rv_n_rendezvous"] = len(rv_waits)
        full_metrics["rv_wait_min"] = sum(rv_waits) / 60.0 if rv_waits else 0.0
