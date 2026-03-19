from pydantic import BaseModel


class VehicleState(BaseModel):
    vehicle_id: str          # "uav" | "ugv"
    x: float
    y: float
    heading: float           # degrees
    speed: float             # m/s
    waypoint_index: int      # current target waypoint
    battery_pct: float | None = None


class TelemetryFrame(BaseModel):
    timestamp_ms: int
    vehicles: list[VehicleState]
