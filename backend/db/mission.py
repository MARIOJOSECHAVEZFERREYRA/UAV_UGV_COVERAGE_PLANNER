from datetime import datetime, timezone
from enum import Enum as PyEnum
from sqlalchemy import DateTime, Float, ForeignKey, Integer, String, Text, Enum
from sqlalchemy.orm import Mapped, mapped_column, relationship

from backend.database import Base


class MissionStatus(str, PyEnum):
    pending = "pending"
    running = "running"
    completed = "completed"
    failed = "failed"


class WaypointType(str, PyEnum):
    sweep = "sweep"
    ferry = "ferry"
    deadhead = "deadhead"
    base = "base"


class Mission(Base):
    __tablename__ = "missions"

    id: Mapped[int] = mapped_column(Integer, primary_key=True, index=True)
    name: Mapped[str] = mapped_column(String(128), nullable=False)
    status: Mapped[MissionStatus] = mapped_column(
        Enum(MissionStatus), default=MissionStatus.pending, nullable=False
    )

    # Input: polygon stored as GeoJSON string
    field_geojson: Mapped[str] = mapped_column(Text, nullable=False)

    # Planning config
    spray_width: Mapped[float] = mapped_column(Float, default=5.0)
    strategy: Mapped[str] = mapped_column(String(32), default="genetic")
    drone_name: Mapped[str | None] = mapped_column(String(128), nullable=True)

    # Result
    best_angle: Mapped[float | None] = mapped_column(Float, nullable=True)
    total_distance: Mapped[float | None] = mapped_column(Float, nullable=True)
    coverage_area: Mapped[float | None] = mapped_column(Float, nullable=True)
    n_cycles: Mapped[int | None] = mapped_column(Integer, nullable=True)
    metrics_json: Mapped[str | None] = mapped_column(Text, nullable=True)
    mission_cycles_json: Mapped[str | None] = mapped_column(Text, nullable=True)
    error_message: Mapped[str | None] = mapped_column(Text, nullable=True)

    created_at: Mapped[datetime] = mapped_column(
        DateTime(timezone=True),
        default=lambda: datetime.now(timezone.utc),
    )

    waypoints: Mapped[list["Waypoint"]] = relationship(
        "Waypoint", back_populates="mission", cascade="all, delete-orphan"
    )


class Waypoint(Base):
    __tablename__ = "waypoints"

    id: Mapped[int] = mapped_column(Integer, primary_key=True, index=True)
    mission_id: Mapped[int] = mapped_column(
        Integer, ForeignKey("missions.id", ondelete="CASCADE"), nullable=False, index=True
    )
    sequence: Mapped[int] = mapped_column(Integer, nullable=False)
    x: Mapped[float] = mapped_column(Float, nullable=False)
    y: Mapped[float] = mapped_column(Float, nullable=False)
    waypoint_type: Mapped[WaypointType] = mapped_column(
        Enum(WaypointType), default=WaypointType.sweep, nullable=False
    )
    cycle_index: Mapped[int | None] = mapped_column(Integer, nullable=True)

    mission: Mapped["Mission"] = relationship("Mission", back_populates="waypoints")
