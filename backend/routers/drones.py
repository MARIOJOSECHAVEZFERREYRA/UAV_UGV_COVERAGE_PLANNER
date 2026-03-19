from fastapi import APIRouter
from backend.data.drone_db import DroneDB

router = APIRouter(prefix="/drones", tags=["drones"])


@router.get("/")
def list_drones():
    result = []
    for name, spec in DroneDB.DRONES.items():
        swath = spec.spray.swath_m if spec.spray else None
        min_swath = max_swath = 5.0
        if isinstance(swath, tuple) and len(swath) == 2:
            try:
                min_swath = float(swath[0].value)
                max_swath = float(swath[1].value)
            except (ValueError, AttributeError):
                pass
        result.append({
            "name": name,
            "min_swath": round(min_swath, 1),
            "max_swath": round(max_swath, 1),
            "default_swath": round(max_swath, 1),
        })
    return result
