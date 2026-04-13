from datetime import datetime
from pydantic import BaseModel, Field

from backend.models.mission_model import MissionStatus, WaypointType



class FieldPolygon(BaseModel):
    coordinates: list[list[float]] = Field(..., min_length=4)
    obstacles: list[list[list[float]]] = Field(default_factory=list)
    base_point: list[float] = Field(
        ...,
        min_length=2,
        max_length=2,
        description="[x, y] static recharge/refill location")
    ugv_polyline: list[list[float]] | None = Field(
        default=None,
        description="Ordered [x, y] waypoints defining the UGV route. "
                    "When provided, enables mobile UGV rendezvous mode.")
    ugv_speed: float = Field(
        default=2.0,
        gt=0.0,
        description="UGV cruise speed in m/s")
    ugv_t_service: float = Field(
        default=300.0,
        gt=0.0,
        description="Manual service duration at rendezvous point in seconds")


class MissionCreate(BaseModel):
    name: str = Field(..., min_length=1, max_length=128)
    field: FieldPolygon
    spray_width: float = Field(default=5.0, gt=0.0, le=50.0)
    strategy: str = Field(default="genetic", pattern="^(genetic|simple)$")
    drone_name: str = Field(default="DJI Agras T30")
    app_rate: float | None = Field(
        default=None,
        gt=0.0,
        description="Override application rate in L/ha; None = use drone default",
    )
    cruise_speed_ms: float | None = Field(default=None, gt=0.0, description="Override cruise speed in m/s; None = use drone default")
    margin_m: float | None = Field(default=None, ge=0.0, description="Safety margin in meters; None = swath/2")


class WaypointOut(BaseModel):
    id: int
    sequence: int
    x: float
    y: float
    waypoint_type: WaypointType
    cycle_index: int = 0
    model_config = {"from_attributes": True}


class MissionOut(BaseModel):
    id: int
    name: str
    status: MissionStatus
    spray_width: float
    strategy: str
    drone_name: str | None
    overrides_json: str | None = None
    best_angle: float | None
    total_distance: float | None
    coverage_area: float | None
    n_cycles: int | None
    metrics_json: str | None
    error_message: str | None
    created_at: datetime
    waypoints: list[WaypointOut] = []
    model_config = {"from_attributes": True}
