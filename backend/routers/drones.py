from fastapi import APIRouter, Depends
from sqlalchemy.orm import Session

from backend.database import get_db
from backend.db.drone import Drone

router = APIRouter(prefix="/drones", tags=["drones"])


@router.get("/{name}")
def get_drone(name: str, db: Session = Depends(get_db)):
    drone = db.query(Drone).filter(Drone.name == name).first()
    if not drone:
        from fastapi import HTTPException
        raise HTTPException(status_code=404, detail="Drone not found")
    return {
        "id": drone.id,
        "name": drone.name,
        "num_rotors": drone.num_rotors,
        "mass_empty_kg": drone.mass_empty_kg,
        "mass_battery_kg": drone.mass_battery_kg,
        "mass_tank_full_kg": drone.mass_tank_full_kg,
        "battery_capacity_wh": drone.battery_capacity_wh,
        "battery_voltage_v": drone.battery_voltage_v,
        "battery_reserve_pct": drone.battery_reserve_pct,
        "battery_charge_time_min": drone.battery_charge_time_min,
        "power_hover_empty_w": drone.power_hover_empty_w,
        "power_hover_full_w": drone.power_hover_full_w,
        "speed_cruise_ms": drone.speed_cruise_ms,
        "speed_max_ms": drone.speed_max_ms,
        "speed_vertical_ms": drone.speed_vertical_ms,
        "turn_duration_s": drone.turn_duration_s,
        "turn_power_factor": drone.turn_power_factor,
        "spray_flow_rate_lpm": drone.spray_flow_rate_lpm,
        "spray_swath_m": drone.spray_swath_m,
        "spray_height_m": drone.spray_height_m,
        "spray_pump_power_w": drone.spray_pump_power_w,
        "service_time_s": drone.service_time_s,
    }


@router.get("/")
def list_drones(db: Session = Depends(get_db)):
    drones = db.query(Drone).order_by(Drone.name).all()
    result = []
    for drone in drones:
        result.append({
            "name": drone.name,
            "min_swath": drone.spray_swath_m,
            "max_swath": drone.spray_swath_m,
            "default_swath": drone.spray_swath_m,
            "num_rotors": drone.num_rotors,
            "spray_flow_rate_lpm": drone.spray_flow_rate_lpm,
            "speed_cruise_ms": drone.speed_cruise_ms,
            "speed_max_ms": drone.speed_max_ms,
        })
    return result
