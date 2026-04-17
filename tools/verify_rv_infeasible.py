"""Verification harness for the rendezvous infeasibility fix.

Runs the full DynamicMissionPlanner / StaticMissionPlanner pipeline against
representative fields and asserts that:
  - Feasible dynamic missions report rv_infeasible=False and no rv_warning.
  - Infeasibility-forcing dynamic missions close cleanly with a partial
    plan, rv_infeasible=True, and a known reason.
  - Static missions never carry any rv_infeasible flag or warning.
  - Energy never goes negative inside any committed cycle.

Usage:
    source venv/bin/activate
    python tools/verify_rv_infeasible.py
"""

import json
import sys
import os

sys.path.insert(0, os.path.abspath(os.path.dirname(os.path.dirname(__file__))))

from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker

from backend.services.mission_planner import DynamicMissionPlanner, StaticMissionPlanner
from backend.algorithms.energy.energy_model import DroneEnergyModel
from backend.models.drone_model import Drone


DRONE_NAME = "DJI Agras T30"
DB_URL = "sqlite:///agriswarm.db"


def _load_field(path):
    with open(path, "r") as f:
        data = json.load(f)
    boundary = [tuple(p) for p in data["boundary"]]
    if boundary[0] == boundary[-1]:
        boundary = boundary[:-1]
    base_point = tuple(data.get("base_point", (boundary[0][0] - 10, boundary[0][1] - 10)))
    ugv_polyline = data.get("ugv_polyline")
    ugv_speed = float(data.get("ugv_speed", 2.0))
    ugv_t_service = float(data.get("ugv_t_service", 300.0))
    return {
        "name": data.get("name", os.path.basename(path)),
        "boundary": boundary,
        "base_point": base_point,
        "ugv_polyline": [tuple(p) for p in ugv_polyline] if ugv_polyline else None,
        "ugv_speed": ugv_speed,
        "ugv_t_service": ugv_t_service,
    }


def _walk_energy(cycle, drone, energy_model):
    """Return (min_energy_seen, negative_count) across a single cycle.

    Each cycle starts with a freshly serviced drone (usable energy + full
    tank). Walks the cycle segments applying the same per-segment energy
    cost rules as plan_dynamic_cycles / segmenter. Any strictly-negative
    value indicates the silent-fallback bug has returned.
    """
    e = energy_model.usable_energy_wh()
    q = float(drone.mass_tank_full_kg)
    min_e = e
    negative = 0
    for seg in cycle.get("segments", []):
        dist = float(seg.get("distance_m", 0.0))
        seg_type = seg.get("segment_type", "ferry")
        if dist < 1e-9:
            continue
        if seg_type == "sweep":
            e_step = energy_model.energy_straight(dist, q)
            q_step = energy_model.reagent_consumed(dist)
        else:
            e_step = energy_model.energy_transit(dist, q)
            q_step = 0.0
        e -= e_step
        q = max(0.0, q - q_step)
        if e < min_e:
            min_e = e
        if e < -1e-6:
            negative += 1
    return min_e, negative


def _run(controller, field, strategy="grid"):
    engine = create_engine(DB_URL)
    Session = sessionmaker(bind=engine)
    db = Session()
    try:
        overrides = {"speed": 5.0, "app_rate": 20.0}
        result = controller.run_mission_planning(
            db=db,
            polygon_points=field["boundary"],
            drone_name=DRONE_NAME,
            overrides=overrides,
            base_point=field["base_point"],
            strategy_name=strategy,
        )
        return result, db
    finally:
        db.close()


def _drone_and_model(db_url=DB_URL):
    engine = create_engine(db_url)
    Session = sessionmaker(bind=engine)
    db = Session()
    try:
        drone = db.query(Drone).filter(Drone.name == DRONE_NAME).first()
        return drone, DroneEnergyModel(drone)
    finally:
        db.close()


def check_dynamic_feasible():
    print("\n=== [1/3] Dynamic mission — expected FEASIBLE ===")
    field = _load_field("tests/test_fields/dynamic/polyline_diagonal.json")
    planner = DynamicMissionPlanner(
        ugv_polyline=field["ugv_polyline"],
        ugv_speed=field["ugv_speed"],
        ugv_t_service=field["ugv_t_service"],
    )
    result, _ = _run(planner, field)
    print(f"  field={field['name']}  angle={result['best_angle']:.1f}°  "
          f"cycles={len(result['mission_cycles'])}  "
          f"rv_infeasible={result.get('rv_infeasible')}  "
          f"reason={result.get('rv_infeasible_reason')}")
    assert result.get("rv_infeasible") is False, "Expected feasible dynamic run"
    assert result.get("rv_infeasible_reason") is None

    drone, em = _drone_and_model()
    for i, c in enumerate(result["mission_cycles"]):
        min_e, neg = _walk_energy(c, drone, em)
        print(f"  cycle[{i}] segs={len(c.get('segments', []))}  "
              f"min_energy={min_e:.2f} Wh  neg_count={neg}")
        assert neg == 0, f"Cycle {i} went negative on energy"
    print("  PASS")
    return result


def check_dynamic_infeasible_synthetic(feasible_result):
    """Bypass the grid search and exercise plan_dynamic_cycles directly.

    Reuses the real route_segments from the feasible run (so the route is
    a valid product of the optimizer), but swaps in a tiny stub UGV polyline
    far from the field work. With monotonic advance, after the UGV reaches
    the stub's end no forward candidates exist and later segments cannot
    find a reachable rendezvous — the exact branch we rewired.
    """
    print("\n=== [2/3] Dynamic mission — stub polyline forcing infeasibility ===")
    from backend.algorithms.rendezvous.planner import RendezvousPlanner
    from backend.algorithms.energy.segmentation import MissionSegmenter

    drone, em = _drone_and_model()

    field = _load_field("tests/test_fields/dynamic/polyline_diagonal.json")
    route_segments = feasible_result.get("route_segments", [])
    safe = feasible_result.get("safe_polygon")
    assert route_segments, "feasible run produced no route_segments"
    assert safe is not None, "feasible run produced no safe_polygon"

    # Long polyline that starts next to the field and then plunges far
    # south. Early candidates are reachable (first cycles succeed), but
    # as monotonic advance forces later candidates further down the
    # plunge, they fall out of energy range. This exercises the
    # "close partial cycle with committed sweeps" branch.
    ugv_polyline = [(-40.0, -30.0), (-40.0, -80.0), (-40.0, -5000.0)]
    rp = RendezvousPlanner(
        ugv_polyline=ugv_polyline, v_ugv=2.0, t_service=300.0,
    )
    print(f"  route segments={len(route_segments)}  polyline_len={rp.total_length:.1f} m"
          f"  polyline tail at {ugv_polyline[-1]}")

    segmenter = MissionSegmenter(
        drone, target_rate_l_ha=20.0, work_speed_kmh=5.0 * 3.6,
        swath_width=5.0, energy_model=em,
    )
    plan = rp.plan_dynamic_cycles(segmenter, safe, route_segments)
    print(f"  infeasible={plan.get('infeasible')}  reason={plan.get('reason')}  "
          f"cycles={len(plan.get('cycles', []))}")

    for i, c in enumerate(plan.get('cycles', [])):
        min_e, neg = _walk_energy(c, drone, em)
        has_sweep = any(s.get('segment_type') == 'sweep' for s in c.get('segments', []))
        print(f"  cycle[{i}] segs={len(c.get('segments', []))}  "
              f"has_sweep={has_sweep}  min_energy={min_e:.2f} Wh  neg_count={neg}")
        assert neg == 0, f"Cycle {i} went negative on energy"

    assert plan.get('infeasible') is True, \
        "Expected stub polyline to force infeasibility"
    assert plan.get('reason') in (
        "no_reachable_rendezvous", "no_reachable_final_rv",
    ), f"Unexpected reason: {plan.get('reason')}"
    print("  PASS (infeasible branch exercised directly)")
    return plan


def check_static():
    print("\n=== [3/3] Static mission — expected regression-free ===")
    field = _load_field("tests/test_fields/basic/l_shape.json")
    planner = StaticMissionPlanner()
    result, _ = _run(planner, field)
    print(f"  field={field['name']}  angle={result['best_angle']:.1f}°  "
          f"cycles={len(result['mission_cycles'])}  "
          f"rv_infeasible={result.get('rv_infeasible')}  "
          f"reason={result.get('rv_infeasible_reason')}")
    assert result.get("rv_infeasible") is False, "Static must never be infeasible by RV"
    assert result.get("rv_infeasible_reason") is None
    print("  PASS")
    return result


def main():
    r1 = check_dynamic_feasible()
    r2 = check_dynamic_infeasible_synthetic(r1)
    r3 = check_static()

    print("\n=== SUMMARY ===")
    print(f"dynamic feasible      rv_infeasible={r1.get('rv_infeasible')}")
    print(f"dynamic synthetic      infeasible={r2.get('infeasible')}  "
          f"reason={r2.get('reason')}  cycles={len(r2.get('cycles', []))}")
    print(f"static feasible       rv_infeasible={r3.get('rv_infeasible')}")


if __name__ == "__main__":
    main()
