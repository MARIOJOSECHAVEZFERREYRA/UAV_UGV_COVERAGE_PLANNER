from enum import Enum as PyEnum
from pydantic import BaseModel


class SegmentType(str, PyEnum):
    spray = "spray"
    ferry = "ferry"
    deadhead = "deadhead"
    service = "service"


class VehicleSimState(BaseModel):
    vehicle_id: str
    x: float
    y: float
    heading: float
    speed: float
    segment_type: SegmentType
    cycle_index: int
    waypoint_index: int
    battery_pct: float
    energy_remaining_wh: float
    reagent_l: float
    pump_active: bool
    is_done: bool

    model_config = {"use_enum_values": False}


class SimulationFrame(BaseModel):
    timestamp_ms: int
    sim_time_s: float
    vehicles: list[VehicleSimState]
    playback_speed: float


class SimulationConfig(BaseModel):
    playback_speed: float = 1.0
