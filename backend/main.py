from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

from backend.database import Base, engine
from backend.routers import fields, mission, telemetry, drones

# Create tables on startup
Base.metadata.create_all(bind=engine)

app = FastAPI(title="AgriSwarm Planner API", version="0.1.0")

app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:5173"],  # Vite dev server
    allow_methods=["*"],
    allow_headers=["*"],
)

app.include_router(fields.router)
app.include_router(mission.router)
app.include_router(telemetry.router)
app.include_router(drones.router)


@app.get("/health")
def health():
    return {"status": "ok"}
