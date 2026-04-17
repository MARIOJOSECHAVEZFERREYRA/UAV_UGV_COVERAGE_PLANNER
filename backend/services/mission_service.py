"""Mission planning service — uses MissionPlanner for full pipeline."""
import json
import traceback

from fastapi import HTTPException
from shapely.geometry import Polygon, Point
from sqlalchemy.orm import Session

from .mission_planner import StaticMissionPlanner, DynamicMissionPlanner
from ..models.mission_model import Mission, MissionStatus, Waypoint, WaypointType
from ..schemas.mission import MissionCreate


MIN_FIELD_AREA_M2    = 100   # 10×10 m — degenerate below this
MIN_OBSTACLE_AREA_M2 = 1     # 1 m²


def validate_mission_request(payload: MissionCreate) -> None:
    field = payload.field
    exterior = [tuple(p) for p in field.coordinates]
    holes = [[tuple(p) for p in ring] for ring in field.obstacles]
    base_point = tuple(field.base_point)

    # --- Exterior polygon ---
    if len(exterior) < 3:
        raise HTTPException(status_code=422, detail="Field polygon requires at least 3 vertices.")

    ext_poly = Polygon(exterior)
    if not ext_poly.is_valid:
        ext_poly = ext_poly.buffer(0)
    if ext_poly.is_empty:
        raise HTTPException(status_code=422, detail="Field polygon is empty or degenerate.")
    if ext_poly.area < MIN_FIELD_AREA_M2:
        raise HTTPException(
            status_code=422,
            detail=f"Field area ({ext_poly.area:.1f} m²) is below minimum ({MIN_FIELD_AREA_M2} m²)."
        )

    # --- Obstacles ---
    obstacle_polys = []
    for i, hole in enumerate(holes):
        if len(hole) < 3:
            raise HTTPException(status_code=422, detail=f"Obstacle {i+1} requires at least 3 vertices.")
        obs_poly = Polygon(hole)
        if not obs_poly.is_valid:
            obs_poly = obs_poly.buffer(0)
        if obs_poly.is_empty:
            raise HTTPException(status_code=422, detail=f"Obstacle {i+1} is empty or degenerate.")
        if obs_poly.area < MIN_OBSTACLE_AREA_M2:
            raise HTTPException(
                status_code=422,
                detail=f"Obstacle {i+1} area ({obs_poly.area:.2f} m²) is below minimum ({MIN_OBSTACLE_AREA_M2} m²)."
            )
        if not ext_poly.contains(obs_poly):
            raise HTTPException(
                status_code=422,
                detail=f"Obstacle {i+1} is not fully contained within the field boundary."
            )
        for j, prev in enumerate(obstacle_polys):
            if obs_poly.intersects(prev):
                raise HTTPException(
                    status_code=422,
                    detail=f"Obstacles {j+1} and {i+1} overlap or touch each other."
                )
        obstacle_polys.append(obs_poly)

    # --- Combined polygon (field with holes) ---
    full_poly = Polygon(shell=exterior, holes=holes)
    if not full_poly.is_valid:
        full_poly = full_poly.buffer(0)
    if full_poly.is_empty or full_poly.area < MIN_FIELD_AREA_M2:
        raise HTTPException(
            status_code=422,
            detail="Field geometry after subtracting obstacles is too small or degenerate."
        )

    # --- Base point ---
    base_pt = Point(base_point)
    if full_poly.contains(base_pt):
        raise HTTPException(
            status_code=422,
            detail="base_point must be outside the spray polygon or on its boundary."
        )


def create_mission(db: Session, payload: MissionCreate) -> Mission:
    overrides = {}
    if payload.app_rate is not None:
        overrides["app_rate"] = payload.app_rate
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

        stored_overrides = {}
        if mission.overrides_json:
            try:
                stored_overrides = json.loads(mission.overrides_json)
            except Exception:
                pass

        overrides = {"swath": mission.spray_width, **stored_overrides}

        if ugv_polyline:
            controller = DynamicMissionPlanner(
                ugv_polyline=ugv_polyline,
                ugv_speed=ugv_speed,
                ugv_t_service=ugv_t_service,
            )
        else:
            controller = StaticMissionPlanner()

        result = controller.run_mission_planning(
            db=db,
            polygon_points=exterior,
            drone_name=drone_name,
            overrides=overrides,
            base_point=base_point,
            strategy_name=mission.strategy,
            obstacle_polygons=holes if holes else None,
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
            result = {
                "segments": segments,
                "base_point": serialize_point(base),
                "swath_width": float(cycle.get("swath_width", 0.0)),
            }
            if "rv_wait_s" in cycle:
                result["rv_wait_s"] = float(cycle["rv_wait_s"])
            return result

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

                # Dedup at same coordinate: update type within the same
                # cycle, but never overwrite across cycle boundaries (the
                # base waypoint must remain as separator so the frontend
                # splits deadhead runs per cycle).
                same_pt = (prev_pt == p1)
                can_merge = (same_pt and waypoints
                             and waypoints[-1].waypoint_type != WaypointType.base
                             and waypoints[-1].cycle_index == cycle_idx)
                if can_merge:
                    waypoints[-1].waypoint_type = seg_type
                else:
                    # New coordinate, or cross-cycle boundary at same
                    # coordinate (after a base waypoint): always emit a
                    # fresh waypoint so each cycle owns its endpoints.
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

            # Base waypoint at end of every cycle: unconditional insert so
            # the frontend always sees a "base" separator between cycles.
            # This prevents adjacent deadhead runs from merging.
            cycle_base = tuple(cycle.get("base_point", [0, 0]))
            if prev_pt == cycle_base and waypoints:
                waypoints[-1].waypoint_type = WaypointType.base
                waypoints[-1].cycle_index = cycle_idx
            else:
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
