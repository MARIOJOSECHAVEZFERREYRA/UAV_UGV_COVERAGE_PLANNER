from fastapi import APIRouter, Depends, HTTPException
from sqlalchemy.orm import Session

from backend.database import get_db
from backend.db.drone import Drone

router = APIRouter(prefix="/drones", tags=["drones"])


def _drone_or_404(name: str, db: Session) -> Drone:
    drone = db.query(Drone).filter(Drone.name == name).first()
    if not drone:
        raise HTTPException(status_code=404, detail="Drone not found")
    return drone


@router.get("/")
def list_drones(db: Session = Depends(get_db)):
    drones = db.query(Drone).order_by(Drone.name).all()
    return [
        {
            "name": d.name,
            "num_rotors": d.num_rotors,
            "default_swath": d.spray_swath_max_m,
            "speed_cruise_ms": d.speed_cruise_ms,
            "speed_max_ms": d.speed_max_ms,
            "spray_flow_rate_lpm": d.spray_flow_rate_lpm,
            "battery_capacity_wh": d.battery_capacity_wh,
            "battery_reserve_pct": d.battery_reserve_pct,
            "service_time_s": d.service_time_s,
        }
        for d in drones
    ]


@router.get("/{name}/defaults")
def drone_mission_defaults(name: str, db: Session = Depends(get_db)):
    """
    Returns suggested default and range values for all user-configurable
    mission planning parameters, derived from the selected drone's specs.
    This is the single source of truth consumed by the frontend.
    """
    d = _drone_or_404(name, db)
    default_swath = d.spray_swath_max_m
    default_margin = round(d.spray_swath_max_m / 2.0, 2)

    return {
        # Swath — min/max come directly from persisted DB fields
        "swath_m":     default_swath,
        "spray_swath_min_m": d.spray_swath_min_m,
        "spray_swath_max_m": d.spray_swath_max_m,
        # Safety margin — suggested default only; backend rule is simply margin >= 0
        "margin_m": default_margin,
        # Application rate — fully persisted per drone
        "app_rate_l_ha":     d.app_rate_default_l_ha,
        "app_rate_min_l_ha": d.app_rate_min_l_ha,
        "app_rate_max_l_ha": d.app_rate_max_l_ha,
        # Flight speed
        "speed_ms":     d.speed_cruise_ms,
        "speed_min_ms": d.speed_cruise_ms,
        "speed_max_ms": d.speed_max_ms,
    }


@router.get("/{name}")
def get_drone(name: str, db: Session = Depends(get_db)):
    d = _drone_or_404(name, db)
    return {
        "id": d.id,
        "name": d.name,
        "num_rotors": d.num_rotors,
        "mass_empty_kg": d.mass_empty_kg,
        "mass_battery_kg": d.mass_battery_kg,
        "mass_tank_full_kg": d.mass_tank_full_kg,
        "battery_capacity_wh": d.battery_capacity_wh,
        "battery_voltage_v": d.battery_voltage_v,
        "battery_reserve_pct": d.battery_reserve_pct,
        "battery_charge_time_min": d.battery_charge_time_min,
        "power_hover_empty_w": d.power_hover_empty_w,
        "power_hover_full_w": d.power_hover_full_w,
        "speed_cruise_ms": d.speed_cruise_ms,
        "speed_max_ms": d.speed_max_ms,
        "speed_vertical_ms": d.speed_vertical_ms,
        "turn_duration_s": d.turn_duration_s,
        "turn_power_factor": d.turn_power_factor,
        "spray_flow_rate_lpm": d.spray_flow_rate_lpm,
        "spray_swath_min_m": d.spray_swath_min_m,
        "spray_swath_max_m": d.spray_swath_max_m,
        "app_rate_default_l_ha": d.app_rate_default_l_ha,
        "app_rate_min_l_ha": d.app_rate_min_l_ha,
        "app_rate_max_l_ha": d.app_rate_max_l_ha,
        "spray_height_m": d.spray_height_m,
        "spray_pump_power_w": d.spray_pump_power_w,
        "service_time_s": d.service_time_s,
    }
