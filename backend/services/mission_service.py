"""Mission planning service — uses MissionController for full pipeline."""
import json
import traceback

from sqlalchemy.orm import Session

from ..controllers.mission_controller import MissionController
from ..db.mission import Mission, MissionStatus, Waypoint, WaypointType
from ..schemas.mission import MissionCreate


def create_mission(db: Session, payload: MissionCreate) -> Mission:
    mission = Mission(
        name=payload.name,
        status=MissionStatus.pending,
        field_geojson=payload.field.model_dump_json(),
        spray_width=payload.spray_width,
        strategy=payload.strategy,
        drone_name=payload.drone_name,
    )
    db.add(mission)
    db.commit()
    db.refresh(mission)
    return mission


def compute_mission(db: Session, mission: Mission) -> Mission:
    mission.status = MissionStatus.running
    db.commit()

    try:
        field = json.loads(mission.field_geojson)
        exterior = [tuple(p) for p in field["coordinates"]]
        holes    = [[tuple(p) for p in ring] for ring in field.get("obstacles", [])]

        raw_base = field.get("base_point")
        base_point = tuple(raw_base) if raw_base else None

        drone_name = mission.drone_name or "DJI Agras T30"

        controller = MissionController()
        result = controller.run_mission_planning(
            db=db,
            polygon_points=exterior,
            drone_name=drone_name,
            overrides={"swath": mission.spray_width},
            base_point=base_point,
            strategy_name=mission.strategy,
            obstacle_polygons=holes if holes else None,
        )

        mission_cycles = result.get("mission_cycles", [])
        best_path      = result.get("best_path")
        safe_polygon   = result.get("safe_polygon")

        # Extract waypoints from cycles
        waypoints = []
        seq = 0
        prev_pt = None

        for cycle in mission_cycles:
            for group in cycle.get("visual_groups", []):
                wp_type = WaypointType.sweep if group["is_spraying"] else WaypointType.ferry
                for pt in group["path"]:
                    xy = (float(pt[0]), float(pt[1]))
                    if prev_pt == xy:
                        continue
                    waypoints.append(Waypoint(
                        mission_id=mission.id,
                        sequence=seq,
                        x=xy[0], y=xy[1],
                        waypoint_type=wp_type,
                    ))
                    prev_pt = xy
                    seq += 1

            # Base point at end of each cycle (drone returns here to recharge/refill)
            cycle_base = cycle.get("base_point")
            if cycle_base:
                base_xy = (float(cycle_base[0]), float(cycle_base[1]))
                if prev_pt != base_xy:
                    waypoints.append(Waypoint(
                        mission_id=mission.id,
                        sequence=seq,
                        x=base_xy[0], y=base_xy[1],
                        waypoint_type=WaypointType.base,
                    ))
                    prev_pt = base_xy
                    seq += 1

        db.add_all(waypoints)

        mission.best_angle     = result.get("best_angle")
        mission.n_cycles       = len(mission_cycles)
        mission.total_distance = float(best_path.length) if best_path else None
        mission.coverage_area  = float(safe_polygon.area) if safe_polygon else None
        mission.metrics_json   = json.dumps(result.get("metrics", {}))
        mission.status         = MissionStatus.completed

    except Exception as exc:
        traceback.print_exc()
        mission.status = MissionStatus.failed
        mission.error_message = str(exc)

    db.commit()
    db.refresh(mission)
    return mission


def get_mission(db: Session, mission_id: int) -> Mission | None:
    return db.get(Mission, mission_id)


def list_missions(db: Session, skip: int = 0, limit: int = 50) -> list[Mission]:
    return db.query(Mission).order_by(Mission.created_at.desc()).offset(skip).limit(limit).all()
