"""Mission planning service — uses MissionController for full pipeline."""
import json
import traceback

from fastapi import HTTPException
from shapely.geometry import Polygon, Point
from sqlalchemy.orm import Session

from ..controllers.mission_controller import MissionController
from ..db.mission import Mission, MissionStatus, Waypoint, WaypointType
from ..schemas.mission import MissionCreate


def validate_mission_request(payload: MissionCreate) -> None:
    field = payload.field
    exterior = [tuple(p) for p in field.coordinates]
    holes = [[tuple(p) for p in ring] for ring in field.obstacles]
    base_point = tuple(field.base_point)

    polygon = Polygon(shell=exterior, holes=holes)
    if not polygon.is_valid:
        polygon = polygon.buffer(0)

    if polygon.is_empty:
        raise HTTPException(status_code=422, detail="Invalid field polygon.")

    base_pt = Point(base_point)

    # Static UAG rule for now:
    # base point must not be inside the spraying field interior.
    if polygon.contains(base_pt):
        raise HTTPException(
            status_code=422,
            detail="base_point must be outside the spray polygon or on its boundary."
        )


def create_mission(db: Session, payload: MissionCreate) -> Mission:
    overrides = {"app_rate": payload.app_rate}
    if payload.cruise_speed_ms is not None:
        overrides["speed"] = payload.cruise_speed_ms
    if payload.margin_m is not None:
        overrides["margin"] = payload.margin_m

    mission = Mission(
        name=payload.name,
        status=MissionStatus.pending,
        field_geojson=payload.field.model_dump_json(),
        spray_width=payload.spray_width,
        strategy=payload.strategy,
        drone_name=payload.drone_name,
        overrides_json=json.dumps(overrides),
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
        holes = [[tuple(p) for p in ring] for ring in field.get("obstacles", [])]
        base_point = tuple(field["base_point"])

        # Campos UGV opcionales (presentes solo en modo rendezvous movil)
        raw_ugv = field.get("ugv_polyline")
        ugv_polyline = [tuple(p) for p in raw_ugv] if raw_ugv and len(raw_ugv) >= 2 else None
        ugv_speed = float(field.get("ugv_speed", 2.0))
        ugv_t_service = float(field.get("ugv_t_service", 300.0))

        drone_name = mission.drone_name or "DJI Agras T30"

        controller = MissionController()
        stored_overrides = {}
        if mission.overrides_json:
            try:
                stored_overrides = json.loads(mission.overrides_json)
            except Exception:
                pass

        overrides = {"swath": mission.spray_width, **stored_overrides}

        result = controller.run_mission_planning(
            db=db,
            polygon_points=exterior,
            drone_name=drone_name,
            overrides=overrides,
            base_point=base_point,
            strategy_name=mission.strategy,
            obstacle_polygons=holes if holes else None,
            ugv_polyline=ugv_polyline,
            ugv_speed=ugv_speed,
            ugv_t_service=ugv_t_service,
        )

        mission_cycles = result.get("mission_cycles", [])
        best_path = result.get("best_path")
        safe_polygon = result.get("safe_polygon")

        def serialize_point(p):
            """Safe conversion of point-like objects to [x, y] list."""
            try:
                if hasattr(p, "__getitem__"):
                    return [float(p[0]), float(p[1])]
                if hasattr(p, "x") and hasattr(p, "y"):
                    return [float(p.x), float(p.y)]
                return [0.0, 0.0]
            except (IndexError, AttributeError, TypeError):
                return [0.0, 0.0]

        def serialize_cycle(cycle):
            """Convert cycle dict to JSON-serializable format."""
            segments = []
            for seg in cycle.get("segments", []):
                try:
                    p1 = seg.get("p1")
                    p2 = seg.get("p2")
                    segments.append({
                        "p1": serialize_point(p1),
                        "p2": serialize_point(p2),
                        "spraying": bool(seg.get("spraying", False)),
                        "segment_type": seg.get("segment_type"),
                    })
                except Exception as e:
                    print(f"Error serializing segment: {e}, seg={seg}")
                    continue

            base = cycle.get("base_point")
            return {
                "segments": segments,
                "base_point": serialize_point(base),
                "swath_width": float(cycle.get("swath_width", 0.0)),
            }

        try:
            mission.mission_cycles_json = json.dumps(
                [serialize_cycle(cycle) for cycle in mission_cycles]
            )
        except Exception as e:
            print(f"Error serializing mission_cycles: {e}")
            mission.mission_cycles_json = None

        # Extract waypoints from cycles using explicit segment_type only.
        waypoints = []
        seq = 0
        prev_pt = None

        for cycle_idx, cycle in enumerate(mission_cycles):
            cycle_segments = cycle.get("segments", [])

            for seg in cycle_segments:
                p1 = (float(seg["p1"][0]), float(seg["p1"][1]))
                p2 = (float(seg["p2"][0]), float(seg["p2"][1]))
                explicit_type = seg.get("segment_type")

                seg_type = _map_segment_type(explicit_type)

                # Add waypoint for p1 (with dedup and type update at boundaries)
                if prev_pt == p1:
                    if waypoints:
                        waypoints[-1].waypoint_type = seg_type
                        waypoints[-1].cycle_index = cycle_idx
                else:
                    waypoints.append(
                        Waypoint(
                            mission_id=mission.id,
                            sequence=seq,
                            x=p1[0],
                            y=p1[1],
                            waypoint_type=seg_type,
                            cycle_index=cycle_idx,
                        )
                    )
                    seq += 1
                    prev_pt = p1

                # Add waypoint for p2
                if prev_pt != p2:
                    waypoints.append(
                        Waypoint(
                            mission_id=mission.id,
                            sequence=seq,
                            x=p2[0],
                            y=p2[1],
                            waypoint_type=seg_type,
                            cycle_index=cycle_idx,
                        )
                    )
                    seq += 1
                    prev_pt = p2

            # Base point at end of cycle
            cycle_base = tuple(cycle.get("base_point", [0, 0]))
            if prev_pt != cycle_base:
                waypoints.append(
                    Waypoint(
                        mission_id=mission.id,
                        sequence=seq,
                        x=cycle_base[0],
                        y=cycle_base[1],
                        waypoint_type=WaypointType.base,
                        cycle_index=cycle_idx,
                    )
                )
                seq += 1
                prev_pt = cycle_base

        db.add_all(waypoints)

        mission.best_angle = result.get("best_angle")
        mission.n_cycles = len(mission_cycles)
        mission.total_distance = float(best_path.length) if best_path else None
        mission.coverage_area = float(safe_polygon.area) if safe_polygon else None

        metrics_dict = dict(result.get("metrics") or {})
        if safe_polygon and not safe_polygon.is_empty:
            try:
                metrics_dict["_safe_polygon"] = [list(c) for c in safe_polygon.exterior.coords]
            except Exception:
                pass
        mission.metrics_json = json.dumps(metrics_dict)
        mission.status = MissionStatus.completed

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
    return (
        db.query(Mission)
        .order_by(Mission.created_at.desc())
        .offset(skip)
        .limit(limit)
        .all()
    )


def _map_segment_type(explicit_type):
    """
    Translate algorithm-level segment_type to DB/API WaypointType.

    This service does not infer geometry semantics.
    Segment type must be decided upstream in the algorithms layer.
    """
    if explicit_type == "sweep":
        return WaypointType.sweep
    if explicit_type == "ferry":
        return WaypointType.ferry
    if explicit_type == "deadhead":
        return WaypointType.deadhead

    raise ValueError(f"Unknown or missing segment_type: {explicit_type}")