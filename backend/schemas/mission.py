from datetime import datetime
from pydantic import BaseModel, Field

from backend.db.mission import MissionStatus, WaypointType


class FieldPolygon(BaseModel):
    coordinates: list[list[float]] = Field(..., min_length=4)
    obstacles: list[list[list[float]]] = Field(default_factory=list)
    base_point: list[float] | None = Field(default=None, description="[x, y] static recharge/refill location")


class MissionCreate(BaseModel):
    name: str = Field(..., min_length=1, max_length=128)
    field: FieldPolygon
    spray_width: float = Field(default=5.0, gt=0.0, le=50.0)
    strategy: str = Field(default="genetic", pattern="^(genetic|simple)$")
    drone_name: str = Field(default="DJI Agras T30")


class WaypointOut(BaseModel):
    id: int
    sequence: int
    x: float
    y: float
    waypoint_type: WaypointType

    model_config = {"from_attributes": True}


class MissionOut(BaseModel):
    id: int
    name: str
    status: MissionStatus
    spray_width: float
    strategy: str
    drone_name: str | None
    best_angle: float | None
    total_distance: float | None
    coverage_area: float | None
    n_cycles: int | None
    metrics_json: str | None
    error_message: str | None
    created_at: datetime
    waypoints: list[WaypointOut] = []

    model_config = {"from_attributes": True}
