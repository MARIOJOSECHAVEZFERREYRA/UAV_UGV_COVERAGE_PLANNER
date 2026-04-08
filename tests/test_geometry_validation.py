"""
Tests for validate_mission_request() in mission_service.py.

Covers:
  - Valid field (no obstacles, with obstacles)
  - Field too small
  - Obstacle too small
  - Obstacle outside field
  - Obstacle straddling field boundary
  - Two overlapping obstacles
  - Two obstacles touching at a vertex/edge
  - Base point inside the field
  - Geometry degenerate after holes subtracted
"""

import pytest
from fastapi import HTTPException

from backend.services.mission_service import validate_mission_request
from backend.schemas.mission import MissionCreate, FieldPolygon

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

BASE_OUTSIDE = [-10.0, 50.0]   # outside every test polygon below

# 200×200 m field — 40 000 m², always above the 100 m² floor
LARGE_FIELD = [[0.0,0.0],[200.0,0.0],[200.0,200.0],[0.0,200.0]]

# 5×5 m field — 25 m², below the 100 m² floor
TINY_FIELD  = [[0.0,0.0],[5.0,0.0],[5.0,5.0],[0.0,5.0]]

# Obstacles that fit inside LARGE_FIELD without touching
OBS_A = [[10.0,10.0],[40.0,10.0],[40.0,40.0],[10.0,40.0]]   # 900 m²
OBS_B = [[60.0,60.0],[90.0,60.0],[90.0,90.0],[60.0,90.0]]   # 900 m²

# Obstacle that overlaps OBS_A (shares interior area)
OBS_OVERLAPS_A = [[30.0,30.0],[60.0,30.0],[60.0,60.0],[30.0,60.0]]

# Obstacle that touches OBS_A at a single vertex (190,190)
OBS_TOUCHES_A = [[40.0,40.0],[70.0,40.0],[70.0,70.0],[40.0,70.0]]

# Obstacle that straddles the right boundary of LARGE_FIELD
OBS_STRADDLE = [[190.0,10.0],[210.0,10.0],[210.0,40.0],[190.0,40.0]]

# Obstacle fully outside LARGE_FIELD
OBS_OUTSIDE  = [[300.0,300.0],[320.0,300.0],[320.0,320.0],[300.0,320.0]]

# Tiny obstacle below MIN_OBSTACLE_AREA_M2 = 1 m²
OBS_TINY     = [[10.0,10.0],[10.5,10.0],[10.5,10.5],[10.0,10.5]]   # 0.25 m²


def make_payload(
    coords=None,
    obstacles=None,
    base_point=None,
):
    return MissionCreate(
        name="Test Mission",
        field=FieldPolygon(
            coordinates=coords or LARGE_FIELD,
            obstacles=obstacles or [],
            base_point=base_point or BASE_OUTSIDE,
        ),
        spray_width=5.0,
        strategy="genetic",
        drone_name="DJI Agras T30",
        app_rate=10.0,
    )


def assert_passes(payload):
    """validate_mission_request must not raise."""
    validate_mission_request(payload)


def assert_422(payload, fragment: str = ""):
    """validate_mission_request must raise 422 with optional detail fragment."""
    with pytest.raises(HTTPException) as exc_info:
        validate_mission_request(payload)
    assert exc_info.value.status_code == 422
    if fragment:
        assert fragment.lower() in exc_info.value.detail.lower()


# ---------------------------------------------------------------------------
# Valid cases
# ---------------------------------------------------------------------------

class TestValidCases:
    def test_large_field_no_obstacles(self):
        assert_passes(make_payload())

    def test_large_field_one_obstacle(self):
        assert_passes(make_payload(obstacles=[OBS_A]))

    def test_large_field_two_non_overlapping_obstacles(self):
        assert_passes(make_payload(obstacles=[OBS_A, OBS_B]))

    def test_base_point_on_boundary_is_allowed(self):
        # Point on boundary: x=0, y=100 — on the left edge of LARGE_FIELD
        # full_poly.contains() returns False for boundary points, so no error
        assert_passes(make_payload(base_point=[0.0, 100.0]))


# ---------------------------------------------------------------------------
# Field validation
# ---------------------------------------------------------------------------

class TestFieldValidation:
    def test_field_too_small(self):
        assert_422(make_payload(coords=TINY_FIELD), "below minimum")

    def test_field_exactly_at_threshold_passes(self):
        # 11×11 = 121 m² > 100 m²
        field_just_above = [[0.0,0.0],[11.0,0.0],[11.0,11.0],[0.0,11.0]]
        assert_passes(make_payload(coords=field_just_above))

    def test_base_point_inside_field_fails(self):
        # Center of LARGE_FIELD is (100, 100)
        assert_422(make_payload(base_point=[100.0, 100.0]), "outside")


# ---------------------------------------------------------------------------
# Obstacle validation
# ---------------------------------------------------------------------------

class TestObstacleValidation:
    def test_obstacle_too_small(self):
        assert_422(make_payload(obstacles=[OBS_TINY]), "below minimum")

    def test_obstacle_outside_field(self):
        assert_422(make_payload(obstacles=[OBS_OUTSIDE]), "not fully contained")

    def test_obstacle_straddling_boundary(self):
        assert_422(make_payload(obstacles=[OBS_STRADDLE]), "not fully contained")

    def test_two_obstacles_overlapping(self):
        assert_422(make_payload(obstacles=[OBS_A, OBS_OVERLAPS_A]), "overlap")

    def test_two_obstacles_touching_at_vertex(self):
        # OBS_TOUCHES_A shares the vertex (40,40) with OBS_A
        # Shapely intersects() returns True for vertex contact → 422
        assert_422(make_payload(obstacles=[OBS_A, OBS_TOUCHES_A]), "overlap")

    def test_obstacle_order_independent(self):
        # Swapping order of invalid pair must still fail
        assert_422(make_payload(obstacles=[OBS_OVERLAPS_A, OBS_A]), "overlap")


# ---------------------------------------------------------------------------
# Degenerate combined geometry
# ---------------------------------------------------------------------------

class TestCombinedGeometry:
    def test_obstacle_nearly_fills_field_makes_combined_too_small(self):
        # Field 11×11 = 121 m². Obstacle 10×10 = 100 m² inside it.
        # Net area = 121 - 100 = 21 m² < 100 m² floor → 422.
        small_field    = [[0.0,0.0],[11.0,0.0],[11.0,11.0],[0.0,11.0]]
        near_fill_obs  = [[0.5,0.5],[10.5,0.5],[10.5,10.5],[0.5,10.5]]
        assert_422(
            make_payload(coords=small_field, obstacles=[near_fill_obs]),
            "too small",
        )
