from .drone_db import DroneDB, DroneSpec, FlightSpec, BatterySpec, SpraySpec, SpecValue, PhysicalSpec

DroneDB.DRONES = {
    # --- HYLIO (USA) ---
    "Hylio AG-272": DroneSpec(
        name="Hylio AG-272",
        category="spray",
        flight=FlightSpec(
            max_speed_kmh=SpecValue(48.0, "km/h", "Max operating speed", "Hylio Spec Sheet"),
            work_speed_kmh=SpecValue(48.0, "km/h", "Capable work speed", "Hylio Spec Sheet"),
            flight_time_min={
                "hover_loaded": SpecValue(8.0, "min", "Full payload (181kg takeoff)", "Hylio Spec Sheet"),
                "hover_empty": SpecValue(20.0, "min", "No payload (53kg takeoff)", "Hylio Spec Sheet")
            },
            max_wind_mps=SpecValue(11.2, "m/s", "Max sustained wind", "Hylio Spec Sheet")
        ),
        physical=PhysicalSpec(
            width_m=SpecValue(3.05, "m", "Arms unfolded (No props)", "Hylio Spec Sheet"),
            length_m=SpecValue(3.05, "m", "Arms unfolded (No props)", "Hylio Spec Sheet"),
            height_m=SpecValue(0.91, "m", "Height", "Hylio Spec Sheet"),
            weight_empty_kg=SpecValue(53.0, "kg", "Without batteries", "Hylio Spec Sheet"),
            weight_max_takeoff_kg=SpecValue(181.4, "kg", "Max Recommended Takeoff", "Hylio Spec Sheet")
        ),
        spray=SpraySpec(
            tank_l=SpecValue(68.1, "L", "18 gal Capacity", "Hylio Spec Sheet"),
            swath_m=(SpecValue(9.1, "m"), SpecValue(12.2, "m")),
            max_flow_l_min=SpecValue(15.1, "L/min", "Standard Pump Flow", "Hylio Spec Sheet"),
            application_rate_l_ha=(SpecValue(19, "L/ha"), SpecValue(50, "L/ha")),
            nozzle_count=SpecValue(16, "units", "TeeJet Nozzles", "Hylio Spec Sheet")
        ),
        battery=BatterySpec(
            model="2x 14S 42Ah LiPo",
            energy_wh=SpecValue(4351.2, "Wh", "Total System (2x 2175Wh)", "Hylio Spec Sheet"),
            capacity_ah=SpecValue(84.0, "Ah", "Total System (2x 42Ah)", "Hylio Spec Sheet"),
            voltage_v=SpecValue(51.8, "V", "Nominal 14S", "Hylio Spec Sheet"),
            charge_time_min=SpecValue(30.0, "min", "Pair charging time", "Hylio Spec Sheet"),
            hot_swap=SpecValue(True, "bool", "Supported", "Hylio Spec Sheet")
        )
    ),

    # --- DJI AGRAS SERIES (CHINA) ---
    "DJI Agras T50": DroneSpec(
        name="DJI Agras T50",
        category="spray",
        flight=FlightSpec(
            max_speed_kmh=SpecValue(36.0, "km/h", "Max flight speed (10 m/s)", "DJI Specs"),
            work_speed_kmh=SpecValue(36.0, "km/h", "Max operating speed (10 m/s)", "DJI Specs"),
            flight_time_min={
                "hover_loaded": SpecValue(7.5, "min", "Full payload spray (92kg takeoff)", "DJI Specs"),
                "hover_empty": SpecValue(18.0, "min", "Empty tank", "DJI Specs")
            },
            max_wind_mps=SpecValue(6.0, "m/s", "Wind resistance", "DJI Specs")
        ),
        physical=PhysicalSpec(
            width_m=SpecValue(2.80, "m", "Desplegado (con hélices)", "DJI Specs"),
            length_m=SpecValue(3.085, "m", "Desplegado (con hélices)", "DJI Specs"),
            height_m=SpecValue(0.82, "m", "Altura", "DJI Specs"),
            weight_empty_kg=SpecValue(39.9, "kg", "Sin batería", "DJI Specs"),
            weight_max_takeoff_kg=SpecValue(92.0, "kg", "Max takeoff (Spray)", "DJI Specs")
        ),
        spray=SpraySpec(
            tank_l=SpecValue(40.0, "L", "Rated volume (Spray)", "DJI Specs"),
            swath_m=(SpecValue(4.0, "m"), SpecValue(11.0, "m")), 
            max_flow_l_min=SpecValue(16.0, "L/min", "2 sprinklers (Standard)", "DJI Specs"),
            application_rate_l_ha=(SpecValue(10, "L/ha"), SpecValue(30, "L/ha")),
            nozzle_count=SpecValue(2, "discs", "Dual Atomized Centrifugal", "DJI Specs")
        ),
        battery=BatterySpec(
            model="DB1560 Intelligent Flight Battery",
            energy_wh=SpecValue(1566.6, "Wh", "Calculated", "DJI Specs"),
            capacity_ah=SpecValue(30.0, "Ah", "Nominal", "DJI Specs"),
            voltage_v=SpecValue(52.22, "V", "Nominal", "DJI Specs"),
            charge_time_min=SpecValue(9.0, "min", "Fast charge", "DJI Specs"),
            hot_swap=SpecValue(True, "bool", "Supported", "DJI Specs")
        )
    ),

    "DJI Agras T40": DroneSpec(
        name="DJI Agras T40",
        category="spray",
        flight=FlightSpec(
            max_speed_kmh=SpecValue(36.0, "km/h", "Max flight speed (10 m/s)", "DJI Specs"),
            work_speed_kmh=SpecValue(36.0, "km/h", "Max operating speed (10 m/s)", "DJI Specs"),
            flight_time_min={
                "hover_loaded": SpecValue(5.5, "min", "Full payload spray (90kg takeoff)", "DJI Specs"),
                "hover_empty": SpecValue(18.0, "min", "Empty tank", "DJI Specs")
            },
            max_wind_mps=SpecValue(6.0, "m/s", "Wind resistance", "DJI Specs")
        ),
        physical=PhysicalSpec(
            width_m=SpecValue(2.80, "m", "Desplegado (con hélices)", "DJI Specs"),
            length_m=SpecValue(3.085, "m", "Desplegado (con hélices)", "DJI Specs"),
            height_m=SpecValue(0.82, "m", "Altura", "DJI Specs"),
            weight_empty_kg=SpecValue(38.0, "kg", "Sin batería", "DJI Specs"),
            weight_max_takeoff_kg=SpecValue(90.0, "kg", "Peso máx despegue (Spray)", "DJI Specs")
        ),
        spray=SpraySpec(
            tank_l=SpecValue(40.0, "L", "Rated volume (Spray)", "DJI Specs"),
            swath_m=(SpecValue(4.0, "m"), SpecValue(11.0, "m")), 
            max_flow_l_min=SpecValue(12.0, "L/min", "Standard (2 sprinklers)", "DJI Specs"),
            application_rate_l_ha=(SpecValue(10, "L/ha"), SpecValue(40, "L/ha")),
            nozzle_count=SpecValue(2, "discs", "Centrifugal Sprinklers", "DJI Specs")
        ),
        battery=BatterySpec(
            model="DB1560 Intelligent Flight Battery",
            energy_wh=SpecValue(1566.6, "Wh", "Calculated", "DJI Specs"),
            capacity_ah=SpecValue(30.0, "Ah", "Nominal", "DJI Specs"),
            voltage_v=SpecValue(52.22, "V", "Nominal", "DJI Specs"),
            charge_time_min=SpecValue(9.0, "min", "Fast charge", "DJI Specs"),
            hot_swap=SpecValue(True, "bool", "Supported", "DJI Specs")
        )
    ),

    "DJI Agras T30": DroneSpec(
        name="DJI Agras T30",
        category="spray",
        flight=FlightSpec(
            max_speed_kmh=SpecValue(36.0, "km/h", "Max flight speed (10 m/s)", "DJI Specs"),
            work_speed_kmh=SpecValue(25.2, "km/h", "Max operating speed (7 m/s)", "DJI Specs"),
            flight_time_min={
                "hover_loaded": SpecValue(7.8, "min", "Full payload (66.5kg takeoff)", "DJI Specs"),
                "hover_empty": SpecValue(20.5, "min", "Empty tank (36.5kg takeoff)", "DJI Specs")
            },
            max_wind_mps=SpecValue(8.0, "m/s", "Level 5 resistance", "DJI Specs")
        ),
        physical=PhysicalSpec(
            width_m=SpecValue(2.858, "m", "Desplegado (con hélices)", "DJI Specs"),
            length_m=SpecValue(2.685, "m", "Desplegado (con hélices)", "DJI Specs"),
            height_m=SpecValue(0.790, "m", "Altura", "DJI Specs"),
            weight_empty_kg=SpecValue(26.4, "kg", "Sin batería", "DJI Specs"),
            weight_max_takeoff_kg=SpecValue(76.5, "kg", "Peso máx despegue", "DJI Specs")
        ),
        spray=SpraySpec(
            tank_l=SpecValue(30.0, "L", "Rated volume", "DJI Specs"),
            swath_m=(SpecValue(4.0, "m"), SpecValue(9.0, "m")), 
            max_flow_l_min=SpecValue(8.0, "L/min", "Max flow rate", "DJI Specs"),
            application_rate_l_ha=(SpecValue(10, "L/ha"), SpecValue(30, "L/ha")),
            nozzle_count=SpecValue(16, "units", "Standard layout", "DJI Specs")
        ),
        battery=BatterySpec(
            model="BAX501-29000mAh-51.8V",
            energy_wh=SpecValue(1502.0, "Wh", "Calculated", "DJI Specs"),
            capacity_ah=SpecValue(29.0, "Ah", "Nominal", "DJI Specs"),
            voltage_v=SpecValue(51.8, "V", "Nominal", "DJI Specs"),
            charge_time_min=SpecValue(10.0, "min", "Fast charge (7200W)", "DJI Specs"),
            hot_swap=SpecValue(True, "bool", "Supported", "DJI Specs")
        )
    ),

    "DJI Agras T25": DroneSpec(
        name="DJI Agras T25",
        category="spray",
        flight=FlightSpec(
            max_speed_kmh=SpecValue(36.0, "km/h", "Max flight speed (10 m/s)", "DJI Specs"),
            work_speed_kmh=SpecValue(36.0, "km/h", "Max operating speed (10 m/s)", "DJI Specs"),
            flight_time_min={
                "hover_loaded": SpecValue(7.5, "min", "Full payload spray (52kg takeoff)", "DJI Specs"),
                "hover_empty": SpecValue(19.0, "min", "Empty tank", "DJI Specs")
            },
            max_wind_mps=SpecValue(6.0, "m/s", "Wind resistance", "DJI Specs")
        ),
        physical=PhysicalSpec(
            width_m=SpecValue(1.790, "m", "Desplegado (con hélices)", "DJI Specs"),
            length_m=SpecValue(2.105, "m", "Desplegado (con hélices)", "DJI Specs"),
            height_m=SpecValue(0.730, "m", "Altura", "DJI Specs"),
            weight_empty_kg=SpecValue(25.5, "kg", "Sin batería", "DJI Specs"),
            weight_max_takeoff_kg=SpecValue(52.0, "kg", "Peso máx despegue (Spray)", "DJI Specs")
        ),
        spray=SpraySpec(
            tank_l=SpecValue(20.0, "L", "Rated volume (Spray)", "DJI Specs"),
            swath_m=(SpecValue(4.0, "m"), SpecValue(7.0, "m")), 
            max_flow_l_min=SpecValue(16.0, "L/min", "2 sprinklers (Standard)", "DJI Specs"),
            application_rate_l_ha=(SpecValue(10, "L/ha"), SpecValue(30, "L/ha")),
            nozzle_count=SpecValue(2, "discs", "Centrifugal Sprinklers", "DJI Specs")
        ),
        battery=BatterySpec(
            model="DB800 Intelligent Flight Battery",
            energy_wh=SpecValue(809.4, "Wh", "Calculated", "DJI Specs"),
            capacity_ah=SpecValue(15.5, "Ah", "Nominal", "DJI Specs"),
            voltage_v=SpecValue(52.22, "V", "Nominal", "DJI Specs"),
            charge_time_min=SpecValue(9.0, "min", "Fast charge", "DJI Specs"),
            hot_swap=SpecValue(True, "bool", "Supported", "DJI Specs")
        )
    ),

    # --- XAG SERIES (CHINA) ---
    "XAG P100 Pro": DroneSpec(
        name="XAG P100 Pro",
        category="spray",
        flight=FlightSpec(
            max_speed_kmh=SpecValue(49.6, "km/h", "13.8 m/s (Max Flight)", "XAG Specs"),
            work_speed_kmh=SpecValue(36.0, "km/h", "10 m/s (Operational)", "XAG Specs"),
            flight_time_min={
                "hover_loaded": SpecValue(7.5, "min", "Full payload (50kg)", "XAG Specs"), 
                "hover_empty": SpecValue(17.0, "min", "No payload", "XAG Specs")
            },
            max_wind_mps=SpecValue(6.0, "m/s", "Wind resistance", "XAG Specs")
        ),
        physical=PhysicalSpec(
            width_m=SpecValue(2.487, "m", "Desplegado (con hélices)", "XAG Specs"),
            length_m=SpecValue(2.460, "m", "Desplegado (con hélices)", "XAG Specs"),
            height_m=SpecValue(0.685, "m", "Altura", "XAG Specs"),
            weight_empty_kg=SpecValue(38.0, "kg", "Sin batería ni tanque", "XAG Specs"),
            weight_max_takeoff_kg=SpecValue(88.0, "kg", "MTOW Spray", "XAG Specs")
        ),
        spray=SpraySpec(
            tank_l=SpecValue(50.0, "L", "Rated Capacity", "XAG Specs"),
            swath_m=(SpecValue(5.0, "m"), SpecValue(10.0, "m")),
            max_flow_l_min=SpecValue(22.0, "L/min", "Dual Peristaltic Pumps", "XAG Specs"),
            application_rate_l_ha=(SpecValue(10, "L/ha"), SpecValue(40, "L/ha")),
            nozzle_count=SpecValue(2, "atomizers", "Centrifugal Atomizers", "XAG Specs")
        ),
        battery=BatterySpec(
            model="B13960S Smart Battery",
            energy_wh=SpecValue(962.0, "Wh", "962 Wh", "XAG Specs"),
            capacity_ah=SpecValue(20.0, "Ah", "@ 48.1V", "XAG Specs"),
            voltage_v=SpecValue(48.1, "V", "Nominal", "XAG Specs"),
            charge_time_min=SpecValue(11.0, "min", "Water cooling tank", "XAG Specs"),
            hot_swap=SpecValue(False, "bool", "No", "XAG Specs")
        )
    ),

    "XAG P150": DroneSpec(
        name="XAG P150",
        category="spray",
        flight=FlightSpec(
            max_speed_kmh=SpecValue(64.8, "km/h", "18 m/s (Max Flight)", "XAG Specs"),
            work_speed_kmh=SpecValue(49.6, "km/h", "13.8 m/s (Max Operation)", "XAG Specs"),
            flight_time_min={
                "hover_loaded": SpecValue(8.0, "min", "Full payload (60L)", "XAG Specs"),
                "hover_empty": SpecValue(18.0, "min", "No payload", "XAG Specs")
            },
            max_wind_mps=SpecValue(6.0, "m/s", "Wind resistance", "XAG Specs")
        ),
        physical=PhysicalSpec(
            width_m=SpecValue(3.110, "m", "Desplegado (con hélices)", "XAG Specs"),
            length_m=SpecValue(3.118, "m", "Desplegado (con hélices)", "XAG Specs"),
            height_m=SpecValue(0.764, "m", "Altura", "XAG Specs"),
            weight_empty_kg=SpecValue(54.0, "kg", "Con spray system + baterías", "XAG Specs"),
            weight_max_takeoff_kg=SpecValue(114.0, "kg", "Rated MTOW Spray", "XAG Specs")
        ),
        spray=SpraySpec(
            tank_l=SpecValue(60.0, "L", "Rated Capacity (Max 70L)", "XAG Specs"),
            swath_m=(SpecValue(5.0, "m"), SpecValue(10.0, "m")),
            max_flow_l_min=SpecValue(30.0, "L/min", "Dual Impeller Pumps", "XAG Specs"),
            application_rate_l_ha=(SpecValue(10, "L/ha"), SpecValue(60, "L/ha")),
            nozzle_count=SpecValue(2, "atomizers", "Centrifugal (Standard)", "XAG Specs")
        ),
        battery=BatterySpec(
            model="B13970 Smart Battery",
            energy_wh=SpecValue(1300.0, "Wh", "Estimado (High Capacity)", "XAG Specs"),
            voltage_v=SpecValue(48.1, "V", "Nominal", "XAG Specs"),
            capacity_ah=SpecValue(27.0, "Ah", "Estimado", "XAG Specs"),
            charge_time_min=SpecValue(10.0, "min", "Fast charge", "XAG Specs"),
            hot_swap=SpecValue(False, "bool", "No", "XAG Specs")
        )
    ),

    # --- EAVISION (SPECIALTY) ---
    "EAVISION EA-30X": DroneSpec(
        name="EAVISION EA-30X (Hercules)",
        category="spray",
        flight=FlightSpec(
            max_speed_kmh=SpecValue(36.0, "km/h", "Max flight speed (10 m/s)", "EAVISION Specs"),
            work_speed_kmh=SpecValue(25.2, "km/h", "Max operating speed (7 m/s)", "EAVISION Specs"),
            flight_time_min={
                "hover_loaded": SpecValue(8.0, "min", "Full payload (Estimated)", "Review Data"),
                "hover_empty": SpecValue(20.0, "min", "No payload", "EAVISION Specs")
            },
            max_wind_mps=SpecValue(10.0, "m/s", "Wind resistance", "EAVISION Specs")
        ),
        physical=PhysicalSpec(
            width_m=SpecValue(2.350, "m", "Desplegado", "EAVISION Specs"),
            length_m=SpecValue(2.760, "m", "Desplegado", "EAVISION Specs"),
            height_m=SpecValue(0.620, "m", "Altura (Low profile)", "EAVISION Specs"),
            weight_empty_kg=SpecValue(36.1, "kg", "Sin batería", "EAVISION Specs"),
            weight_max_takeoff_kg=SpecValue(67.1, "kg", "MTOW Spray", "EAVISION Specs")
        ),
        spray=SpraySpec(
            tank_l=SpecValue(30.0, "L", "Rated Capacity", "EAVISION Specs"),
            swath_m=(SpecValue(4.5, "m"), SpecValue(8.0, "m")),
            max_flow_l_min=SpecValue(10.0, "L/min", "Max Flow Rate", "EAVISION Specs"),
            application_rate_l_ha=(SpecValue(8, "L/ha"), SpecValue(25, "L/ha")),
            nozzle_count=SpecValue(2, "mist_nozzles", "CCMS Mist Nozzles", "EAVISION Specs")
        ),
        battery=BatterySpec(
            model="Intelligent Battery",
            energy_wh=SpecValue(1502.0, "Wh", "Aprox (29Ah @ 51.8V)", "EAVISION Specs"),
            capacity_ah=SpecValue(29.0, "Ah", "Nominal", "EAVISION Specs"),
            voltage_v=SpecValue(51.8, "V", "Nominal", "EAVISION Specs"),
            charge_time_min=SpecValue(15.0, "min", "Fast charge", "EAVISION Specs"),
            hot_swap=SpecValue(True, "bool", "Supported", "EAVISION Specs")
        )
    )
}