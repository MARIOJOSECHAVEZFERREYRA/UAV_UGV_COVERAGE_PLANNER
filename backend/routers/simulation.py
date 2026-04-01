import asyncio
import json

from fastapi import APIRouter, Depends, WebSocket, WebSocketDisconnect
from sqlalchemy.orm import Session

from backend.database import get_db
from backend.db.mission import MissionStatus
from backend.db.drone import Drone
from backend.services import mission_service, simulation_service
from backend.services.route_builder import UAVRouteBuilder, UGVRouteBuilder
from backend.schemas.simulation import SimulationConfig

router = APIRouter(tags=["simulation"])


@router.websocket("/simulation/{mission_id}")
async def simulation_ws(
    mission_id: int,
    ws: WebSocket,
    db: Session = Depends(get_db),
):
    """
    Opens a simulation stream for a completed mission.

    1. Loads mission + drone from DB.
    2. Rebuilds mission_cycles from stored JSON.
    3. Builds UAVRoute and UGVRoute using route builders.
    4. Streams SimulationFrame until both vehicles finish or client disconnects.

    The client may send SimulationConfig messages to adjust playback_speed.
    """
    mission = mission_service.get_mission(db, mission_id)
    if not mission or mission.status != MissionStatus.completed:
        await ws.close(code=1008)
        return

    drone = db.query(Drone).filter(Drone.name == mission.drone_name).first()
    if not drone:
        await ws.close(code=1011)
        return

    # Reconstruct mission_cycles from stored JSON
    if not mission.mission_cycles_json:
        await ws.close(code=1011)
        return

    try:
        mission_cycles = json.loads(mission.mission_cycles_json)
        if not mission_cycles:
            await ws.close(code=1011)
            return
    except (json.JSONDecodeError, TypeError, ValueError) as e:
        print(f"Error parsing mission_cycles_json: {e}")
        await ws.close(code=1011)
        return

    # Load the work polygon from field_geojson
    try:
        from shapely.geometry import Polygon
        field = json.loads(mission.field_geojson)
        coords = field["coordinates"]
        work_polygon = Polygon(coords)
    except Exception:
        await ws.close(code=1011)
        return

    # Build routes
    uav_builder = UAVRouteBuilder(drone, work_polygon)
    uav_route = uav_builder.build(mission_cycles)

    ugv_builder = UGVRouteBuilder()
    ugv_route = ugv_builder.build_static(mission_cycles, uav_route.total_duration_s)

    await ws.accept()

    # Shared mutable state for playback speed
    sim_state = simulation_service.SimulationState()

    async def recv_config():
        """Background task: listen for playback control messages from client."""
        try:
            while True:
                data = await ws.receive_text()
                cfg = SimulationConfig.model_validate_json(data)
                sim_state.playback_speed = cfg.playback_speed
        except WebSocketDisconnect:
            pass
        except Exception:
            pass

    recv_task = asyncio.create_task(recv_config())

    try:
        async for frame in simulation_service.stream_simulation(
            uav_route=uav_route,
            ugv_route=ugv_route,
            drone=drone,
            state=sim_state,
            interval_ms=200,
        ):
            await ws.send_text(frame.model_dump_json())
    except WebSocketDisconnect:
        pass
    finally:
        recv_task.cancel()
        try:
            await recv_task
        except asyncio.CancelledError:
            pass
