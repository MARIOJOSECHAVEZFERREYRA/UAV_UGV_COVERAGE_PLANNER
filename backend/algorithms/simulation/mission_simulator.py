"""Dynamic UAV-UGV mission simulator.

Mirrors the continuous-projection model of :class:`RendezvousPlanner`:

- The UGV advances monotonically along its polyline (``current_s``).
- The UAV services at the closest continuous point on the remaining
  polyline (``find_best_rendezvous``).
- After service the mission continues from the meet point. The UAV
  does NOT return to a previous work position.

Used by the GA and GridSearch optimizers to estimate the real mission
time (flight + waits + service) in dynamic-mode fitness.
"""

import math


def _make_transit_fn(energy_model, q_reagent):
    """Closure around ``energy_transit`` with a fixed reagent mass."""
    def fn(distance_m):
        return energy_model.energy_transit(distance_m, q_reagent)
    return fn


def simulate_mission_with_rendezvous(route_segments, energy_model,
                                     rendezvous_planner, assembler=None):
    """Simulate a dynamic mission and return aggregate metrics.

    Parameters
    ----------
    route_segments : list[dict]
        Typed sweep/ferry segments produced upstream.
    energy_model : DroneEnergyModel
    rendezvous_planner : RendezvousPlanner
        Continuous-projection planner; its ``v_ugv`` and ``t_service``
        are read directly, and ``find_best_rendezvous`` is used to
        probe feasibility and locate service points.
    assembler : PathAssembler, optional
        When provided, the UAV-to-rendezvous deadhead is measured with
        the obstacle-aware shortest path instead of a euclidean line.

    Returns
    -------
    dict
        ``{feasible, total_time, total_wait_uav, n_rendezvous}``.
    """
    if not route_segments:
        return {
            'feasible': False,
            'total_time': float('inf'),
            'total_wait_uav': 0.0,
            'n_rendezvous': 0,
        }

    drone = energy_model.drone
    e_max = energy_model.usable_energy_wh()
    e_reserve = energy_model.reserve_wh_mobile()
    q_max = float(drone.mass_tank_full_kg)
    v_ugv = rendezvous_planner.v_ugv
    v_uav = float(drone.speed_max_ms)
    t_service = rendezvous_planner.t_service

    # Teleport the UAV to the first route point so the sim matches
    # plan_dynamic_cycles; the opening deadhead from the polyline
    # origin is out of scope.
    first_pt = route_segments[0].get('path', [None])[0]
    if first_pt is None:
        uav_pos = rendezvous_planner.initial_point()
    else:
        uav_pos = (float(first_pt[0]), float(first_pt[1]))

    # Kinematic model: UGV is parked at current_s until the planner
    # commits to a rendezvous. It does not drift forward during drone
    # flight. UGV travel after commit is accounted for when computing
    # t_ugv_arrival below.
    current_s = 0.0
    t = 0.0
    E_rem = e_max
    Q_rem = q_max
    total_wait_uav = 0.0
    n_rendezvous = 0

    i = 0
    n = len(route_segments)
    while i < n:
        seg = route_segments[i]
        seg_type = seg.get('segment_type', 'ferry')
        path = seg.get('path', [])
        dist = float(seg.get('distance_m', 0.0))

        if len(path) < 2 or dist < 1e-9:
            i += 1
            continue
        # meeting point (updated at line current_s = rv_s below).

        if seg_type == 'sweep':
            e_seg = energy_model.energy_straight(dist, Q_rem)
            t_seg = energy_model.time_straight(dist)
            q_seg = energy_model.reagent_consumed(dist)
        else:
            e_seg = energy_model.energy_transit(dist, Q_rem)
            t_seg = energy_model.time_transit(dist)
            q_seg = 0.0

        p2 = (float(path[-1][0]), float(path[-1][1]))
        E_after = E_rem - e_seg
        Q_after = max(0.0, Q_rem - q_seg)

        # Feasibility probe for the post-segment state.
        rv_after = rendezvous_planner.find_best_rendezvous(
            uav_pos=p2,
            uav_energy_rem=E_after,
            ugv_distance_along=current_s,
            t_current=t + t_seg,
            v_uav=v_uav,
            transit_energy_fn=_make_transit_fn(energy_model, Q_after),
            e_reserve=e_reserve,
        )

        if rv_after['feasible']:
            E_rem = E_after
            Q_rem = Q_after
            t += t_seg
            uav_pos = p2
            i += 1
            continue

        # --- Service needed before this segment ---
        rv = rendezvous_planner.find_best_rendezvous(
            uav_pos=uav_pos,
            uav_energy_rem=E_rem,
            ugv_distance_along=current_s,
            t_current=t,
            v_uav=v_uav,
            transit_energy_fn=_make_transit_fn(energy_model, Q_rem),
            e_reserve=e_reserve,
        )

        if not rv['feasible']:
            return {
                'feasible': False,
                'total_time': float('inf'),
                'total_wait_uav': total_wait_uav,
                'n_rendezvous': n_rendezvous,
            }

        rv_point = rv['point']
        rv_s = rv['distance_along']

        if assembler is not None:
            _, d_uav = assembler.find_connection(uav_pos, rv_point)
        else:
            d_uav = rv['d_uav']

        e_to_rv = energy_model.energy_transit(d_uav, Q_rem)
        t_to_rv = energy_model.time_transit(d_uav)

        E_rem -= e_to_rv
        t_uav_arrival = t + t_to_rv
        d_ugv = max(0.0, rv_s - current_s)
        t_ugv_arrival = t + d_ugv / v_ugv
        wait = max(0.0, t_ugv_arrival - t_uav_arrival)

        t = max(t_uav_arrival, t_ugv_arrival) + t_service
        total_wait_uav += wait
        n_rendezvous += 1

        current_s = rv_s
        uav_pos = rv_point
        E_rem = e_max
        Q_rem = q_max
        # Retry the same segment (no i increment).

    return {
        'feasible': True,
        'total_time': t,
        'total_wait_uav': total_wait_uav,
        'n_rendezvous': n_rendezvous,
    }
