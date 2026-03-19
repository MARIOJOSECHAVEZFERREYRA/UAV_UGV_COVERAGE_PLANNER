from fastapi import APIRouter, Depends, WebSocket, WebSocketDisconnect
from sqlalchemy.orm import Session

from backend.database import get_db
from backend.models.mission import MissionStatus
from backend.services import mission_service, telemetry_service

router = APIRouter(tags=["telemetry"])


@router.websocket("/telemetry/{mission_id}")
async def telemetry_ws(mission_id: int, ws: WebSocket, db: Session = Depends(get_db)):
    mission = mission_service.get_mission(db, mission_id)
    if not mission or mission.status != MissionStatus.completed:
        await ws.close(code=1008)  # Policy Violation
        return

    waypoints = [(wp.x, wp.y) for wp in sorted(mission.waypoints, key=lambda w: w.sequence)]

    await ws.accept()
    try:
        async for frame in telemetry_service.stream_telemetry(waypoints):
            await ws.send_text(frame.model_dump_json())
    except WebSocketDisconnect:
        pass
