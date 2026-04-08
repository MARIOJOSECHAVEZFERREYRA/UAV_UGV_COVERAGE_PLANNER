from backend.database import SessionLocal, engine, Base                                                    
from backend.db.drone import Drone                                                                         

Base.metadata.create_all(bind=engine)                                                                      
                                                                                                             
db = SessionLocal()                                                                                        
                                                                                                             
db.add(Drone(
    name="DJI Agras T30",
    num_rotors=6,
    mass_empty_kg=26.4,
    mass_battery_kg=10.1,
    mass_tank_full_kg=30.0,
    battery_capacity_wh=1502.2,
    battery_voltage_v=51.8,
    battery_reserve_pct=20.0,
    battery_charge_time_min=10.0,
    power_hover_empty_w=4396.0,
    power_hover_full_w=11556.0,
    speed_cruise_ms=5.0,
    speed_max_ms=10.0,
    speed_vertical_ms=3.0,
    turn_duration_s=10.0,
    turn_power_factor=1.1,
    spray_flow_rate_lpm=8.0,
    spray_swath_min_m=4.0,
    spray_swath_max_m=9.0,
    spray_height_m=2.5,
    spray_pump_power_w=200.0,
    service_time_s=120.0,
    app_rate_default_l_ha=10.0,
    app_rate_min_l_ha=3.0,
    app_rate_max_l_ha=50.0,
    accel_horizontal_ms2=1.5,
    decel_horizontal_ms2=1.5,
    power_accel_factor=1.15,
    power_decel_factor=1.05,
))                                             
                                                                                                    
db.commit()                                               
db.close()
print("Done")
