"""Mission planning service — uses MissionController for full pipeline."""
import json
import math
import traceback

from shapely.geometry import Point
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

        # Store mission_cycles_json for later simulation use
        def serialize_point(p):
            """Safe conversion of point-like objects to [x, y] list."""
            try:
                if hasattr(p, "__getitem__"):
                    return [float(p[0]), float(p[1])]
                elif hasattr(p, "x") and hasattr(p, "y"):
                    return [float(p.x), float(p.y)]
                else:
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

        # Extract waypoints from cycles with proper segment type classification
        waypoints = []
        seq = 0
        prev_pt = None

        for cycle_idx, cycle in enumerate(mission_cycles):
            base_point = tuple(cycle.get("base_point", [0, 0]))
            cycle_segments = cycle.get("segments", [])

            # Process each segment in this cycle
            for seg in cycle_segments:
                p1 = (float(seg["p1"][0]), float(seg["p1"][1]))
                p2 = (float(seg["p2"][0]), float(seg["p2"][1]))
                is_spraying = seg.get("spraying", False)

                # Classify segment type
                seg_type = _classify_segment_type(
                    p1, p2, is_spraying, base_point, safe_polygon
                )

                # Add waypoint for p1 (with dedup and type update at boundaries)
                if prev_pt == p1:
                    # Boundary point: update previous waypoint's type
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


def _classify_segment_type(p1, p2, is_spraying, base_point, polygon):
    """
    Classifies a segment as spray, ferry, or deadhead.

    Rules:
    1. If is_spraying=True → WaypointType.sweep
    2. If either endpoint is the base_point (within 0.5m) → deadhead
    3. If midpoint is inside the work polygon → ferry
    4. Otherwise → deadhead
    """
    if is_spraying:
        return WaypointType.sweep

    BASE_TOL = 0.5
    p1_dist_to_base = math.hypot(p1[0] - base_point[0], p1[1] - base_point[1])
    p2_dist_to_base = math.hypot(p2[0] - base_point[0], p2[1] - base_point[1])

    if p1_dist_to_base < BASE_TOL or p2_dist_to_base < BASE_TOL:
        return WaypointType.deadhead

    # Check midpoint
    mid_x = (p1[0] + p2[0]) / 2
    mid_y = (p1[1] + p2[1]) / 2
    mid_pt = Point(mid_x, mid_y)

    if polygon.contains(mid_pt) or polygon.boundary.distance(mid_pt) < 0.1:
        return WaypointType.ferry

    return WaypointType.deadhead
