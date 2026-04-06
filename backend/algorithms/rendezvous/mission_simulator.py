"""
Simulador de mision UAV-UGV con rendezvous tactico.

Recibe los segmentos de ruta ya calculados por el optimizador de angulo
y evalua el costo real de la mision incluyendo:
  - tiempo de vuelo (aspersion y transito)
  - tiempos de servicio (rendezvous con el UGV)
  - tiempo de espera del UAV en el punto de encuentro
  - viabilidad energetica de cada rendezvous

Este modulo es invocado desde el fitness del GA para que el optimizador
pueda descubrir angulos de barrido que se sincronicen mejor con el UGV.
"""

import math


def _make_transit_fn(energy_model, q_reagent):
    """
    Construye una funcion de costo energetico de transito con reactivo fijo.
    Se usa para evitar el uso de lambdas (estilo del proyecto).

    El reactivo se captura en el momento del servicio, antes del rendezvous.
    """
    def transit_energy(distance_m):
        return energy_model.energy_transit(distance_m, q_reagent)
    return transit_energy


def simulate_mission_with_rendezvous(route_segments, energy_model,
                                     rendezvous_planner, ugv_start_distance=0.0,
                                     assembler=None):
    """
    Simula la mision completa con rendezvous tactico UAV-UGV.

    Logica de checkpoints de servicio:
    - Se verifica al final de cada segmento de tipo 'sweep' (checkpoint natural).
    - Se dispara servicio cuando reactivo <= 0 o energia <= reserva (20% de E_max).
    - Tras el servicio, el UAV regresa al punto de interrupcion antes de continuar.

    Parameters
    ----------
    route_segments : list de dict
        Segmentos tipados (sweep/ferry), tal como los produce PathAssembler.
        Cada uno tiene: segment_type, path [(x,y),...], distance_m.
    energy_model : DroneEnergyModel
        Modelo de energia del drone.
    rendezvous_planner : RendezvousPlanner
        Planificador tactico de rendezvous con el UGV.
    ugv_start_distance : float
        Distancia inicial del UGV sobre su polyline en metros.

    Returns
    -------
    dict con:
        'feasible'       : bool   — False si algún rendezvous fue infactible
        'total_time'     : float  — tiempo total de mision en segundos (inf si infactible)
        'total_wait_uav' : float  — suma de tiempos de espera del UAV en segundos
        'n_rendezvous'   : int    — numero de rendezvous realizados
    """
    if not route_segments:
        return {
            'feasible': False,
            'total_time': float('inf'),
            'total_wait_uav': 0.0,
            'n_rendezvous': 0,
        }

    drone = energy_model.drone
    v_uav_transit = float(drone.speed_max_ms)
    e_max = energy_model.usable_energy_wh()
    e_reserve = energy_model.reserve_wh_mobile()
    # Capacidad del tanque en litros (densidad 1.0 kg/L)
    q_max = float(drone.mass_tank_full_kg)

    # Estado inicial
    first_pt = route_segments[0]['path'][0]
    t = 0.0
    E_rem = e_max
    Q_rem = q_max
    uav_pos = (float(first_pt[0]), float(first_pt[1]))
    ugv_distance_along = float(ugv_start_distance)
    ugv_last_update_time = 0.0
    total_wait_uav = 0.0
    n_rendezvous = 0

    for seg in route_segments:
        seg_type = seg.get('segment_type', 'ferry')
        path = seg.get('path', [])
        dist = float(seg.get('distance_m', 0.0))

        if len(path) < 2 or dist < 1e-9:
            continue

        # Avanzar UGV al tiempo actual antes de ejecutar el segmento
        ugv_state = rendezvous_planner.advance_ugv_to_time(
            ugv_distance_along, ugv_last_update_time, t
        )
        ugv_distance_along = ugv_state['distance_along']
        ugv_last_update_time = ugv_state['last_update_time']

        # Ejecutar el segmento y consumir recursos
        if seg_type == 'sweep':
            e_seg = energy_model.energy_straight(dist, Q_rem)
            t_seg = energy_model.time_straight(dist)
            q_seg = energy_model.reagent_consumed(dist)
        else:
            # ferry, deadhead u otro transito sin aspersion
            e_seg = energy_model.energy_transit(dist, Q_rem)
            t_seg = energy_model.time_transit(dist)
            q_seg = 0.0

        E_rem -= e_seg
        Q_rem = max(0.0, Q_rem - q_seg)
        t += t_seg
        last_pt = path[-1]
        uav_pos = (float(last_pt[0]), float(last_pt[1]))

        # Avanzar UGV al tiempo actual (valido para sweep y ferry)
        ugv_state = rendezvous_planner.advance_ugv_to_time(
            ugv_distance_along, ugv_last_update_time, t
        )
        ugv_distance_along = ugv_state['distance_along']
        ugv_last_update_time = ugv_state['last_update_time']

        # Verificar si se necesita servicio:
        # - Al final de todo segmento de sweep (checkpoint natural).
        # - Al final de un ferry si la energia ya cayo bajo la reserva
        #   (un ferry largo puede dejar al UAV sin energia para el siguiente sweep).
        if seg_type == 'sweep':
            need_service = Q_rem <= 0.0 or E_rem <= e_reserve
        else:
            # Para ferries solo verificar energia; el reactivo no se consume en transito.
            need_service = E_rem <= e_reserve

        if not need_service:
            continue

        # --- Secuencia de servicio ---

        # Guardar posicion de reanudacion (el UAV volvera aqui tras el servicio)
        resume_pos = uav_pos
        # Capturar reactivo en este momento para calcular energia de transito
        q_at_service = Q_rem

        # Buscar el mejor punto de rendezvous factible
        rv = rendezvous_planner.find_best_rendezvous(
            uav_pos=uav_pos,
            uav_energy_rem=E_rem,
            ugv_distance_along=ugv_distance_along,
            t_current=t,
            v_uav=v_uav_transit,
            transit_energy_fn=_make_transit_fn(energy_model, q_at_service),
            e_reserve=e_reserve,
        )

        if not rv['feasible']:
            # Sin rendezvous factible: la solucion es inviable
            return {
                'feasible': False,
                'total_time': float('inf'),
                'total_wait_uav': total_wait_uav,
                'n_rendezvous': n_rendezvous,
            }

        # a) Volar del UAV al punto de rendezvous
        e_to_rv = energy_model.energy_transit(rv['d_uav'], q_at_service)
        E_rem -= e_to_rv
        t += rv['flight_time_uav']
        uav_pos = rv['point']

        # b) Tiempo real de encuentro: el que llegue despues define el inicio del servicio
        meet_time = max(rv['t_uav_arrival'], rv['t_ugv_arrival'])
        total_wait_uav += rv['t_wait_uav']
        t = meet_time

        # c) Fijar estado del UGV en el punto de rendezvous (ha llegado aqui)
        ugv_distance_along = rv['distance_along']
        ugv_last_update_time = t

        # d) Servicio manual; el UGV permanece detenido durante t_service
        t += rendezvous_planner.t_service
        # ugv_last_update_time se actualiza al final del servicio para que
        # advance_ugv_to_time() no lo avance durante este periodo
        ugv_last_update_time = t

        # e) Resetear recursos del UAV tras la recarga
        E_rem = e_max
        Q_rem = q_max
        n_rendezvous += 1

        # f) Retornar al punto de reanudacion para continuar la mision.
        # Si se dispone de un PathAssembler, usar la distancia real obstacle-aware
        # para que el fitness del GA refleje el costo verdadero del retorno.
        if assembler is not None:
            _, d_back = assembler.safe_connection(uav_pos, resume_pos)
        else:
            d_back = math.hypot(resume_pos[0] - uav_pos[0], resume_pos[1] - uav_pos[1])
        e_back = energy_model.energy_transit(d_back, Q_rem)
        t_back = energy_model.time_transit(d_back)

        E_rem -= e_back
        t += t_back
        uav_pos = resume_pos

    return {
        'feasible': True,
        'total_time': t,
        'total_wait_uav': total_wait_uav,
        'n_rendezvous': n_rendezvous,
    }
