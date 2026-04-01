from sqlalchemy import Column, Integer, String, Float
from backend.database import Base


class Drone(Base):
    __tablename__ = "drones"

    # Identificacion
    id = Column(Integer, primary_key=True, autoincrement=True)
    name = Column(String(128), nullable=False, unique=True)
    num_rotors = Column(Integer, nullable=False)

    # Masa (kg)
    mass_empty_kg = Column(Float, nullable=False)
    mass_battery_kg = Column(Float, nullable=False)
    mass_tank_full_kg = Column(Float, nullable=False)

    # Energia
    battery_capacity_wh = Column(Float, nullable=False)
    battery_voltage_v = Column(Float, nullable=False)
    battery_reserve_pct = Column(Float, nullable=False, default=20.0)

    # Potencias de hover (W)
    power_hover_empty_w = Column(Float, nullable=False)
    power_hover_full_w = Column(Float, nullable=False)

    # Cinematica (m/s)
    speed_cruise_ms = Column(Float, nullable=False)
    speed_max_ms = Column(Float, nullable=False)
    speed_vertical_ms = Column(Float, nullable=False, default=3.0)

    # Parametros de giro
    turn_duration_s = Column(Float, nullable=False, default=10.0)
    turn_power_factor = Column(Float, nullable=False, default=1.1)

    # Rociado
    spray_flow_rate_lpm = Column(Float, nullable=False)
    spray_swath_m = Column(Float, nullable=False)
    spray_height_m = Column(Float, nullable=False)
    spray_pump_power_w = Column(Float, nullable=False, default=200.0)

    # Operacional
    battery_charge_time_min = Column(Float, nullable=False, default=30.0)
    service_time_s = Column(Float, nullable=False, default=120.0)
