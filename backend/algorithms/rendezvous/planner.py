"""
Modulo de planificacion tactica de rendezvous UAV-UGV.


"""

import math


class RendezvousPlanner:
    """
    Selecciona el mejor punto de rendezvous sobre la polyline fija del UGV.

    Garantias del UGV:
    - Movimiento monotono hacia adelante (nunca retrocede).
    - Puede detenerse y esperar en el punto de encuentro.
    - Avanza un paso por ciclo (polyline_length / est_cycles).
    """

    def __init__(self, ugv_polyline, v_ugv, t_service,
                 alpha=1.0, beta=0.5, gamma=0.3, candidate_spacing=50.0):
        """
        Parameters
        ----------
        ugv_polyline : list of (x, y)
            Ruta fija del UGV definida por el usuario, en metros (coordenadas planas).
        v_ugv : float
            Velocidad de crucero del UGV en m/s.
        t_service : float
            Duracion del servicio manual (recarga de bateria y reactivo) en segundos.
        alpha : float
            Peso de penalizacion por tiempo de espera del UAV [segundos].
        beta : float
            Peso de penalizacion por deadhead del UAV [adimensional, escala tiempo].
        gamma : float
            Peso de penalizacion por avance del UGV [adimensional, escala tiempo].
        candidate_spacing : float
            Separacion entre candidatos discretizados sobre la polyline en metros.
        """
        self.ugv_polyline = [tuple(float(c) for c in p[:2]) for p in ugv_polyline]
        self.v_ugv = max(float(v_ugv), 1e-9)
        self.t_service = float(t_service)
        self.alpha = float(alpha)
        self.beta = float(beta)
        self.gamma = float(gamma)
        self.candidate_spacing = float(candidate_spacing)

        self.total_length = self._compute_polyline_length()
        # Candidatos precalculados para evitar recomputar en cada llamada al fitness
        self._candidates = self._discretize_polyline(self.candidate_spacing)

    def initial_point(self):
        """Primer punto de la polyline del UGV."""
        return self.ugv_polyline[0]

    # ------------------------------------------------------------------
    # GEOMETRIA DE LA POLYLINE
    # ------------------------------------------------------------------

    def _compute_polyline_length(self):
        """Calcula la longitud total de la polyline del UGV en metros."""
        pts = self.ugv_polyline
        total = 0.0
        for i in range(len(pts) - 1):
            dx = pts[i + 1][0] - pts[i][0]
            dy = pts[i + 1][1] - pts[i][1]
            total += math.hypot(dx, dy)
        return total

    def _find_segment_at_distance(self, distance_along):
        """
        Retorna el indice del segmento de la polyline que contiene
        la distancia acumulada dada.
        """
        distance_along = max(0.0, min(distance_along, self.total_length))
        pts = self.ugv_polyline
        accumulated = 0.0

        for i in range(len(pts) - 1):
            p1 = pts[i]
            p2 = pts[i + 1]
            seg_len = math.hypot(p2[0] - p1[0], p2[1] - p1[1])

            # Usar <= con tolerancia para manejar el ultimo segmento correctamente
            if accumulated + seg_len >= distance_along - 1e-9:
                return i
            accumulated += seg_len

        return max(0, len(pts) - 2)

    def _interpolate_point_at_distance(self, distance_along):
        """
        Dado una distancia acumulada sobre la polyline, retorna el punto
        (x, y) interpolado linealmente.

        Satura en los extremos [0, total_length].
        """
        distance_along = max(0.0, min(distance_along, self.total_length))
        pts = self.ugv_polyline

        if len(pts) == 1:
            return pts[0]

        accumulated = 0.0
        for i in range(len(pts) - 1):
            p1 = pts[i]
            p2 = pts[i + 1]
            seg_len = math.hypot(p2[0] - p1[0], p2[1] - p1[1])

            if accumulated + seg_len >= distance_along - 1e-9:
                if seg_len < 1e-9:
                    return p1
                t = (distance_along - accumulated) / seg_len
                t = max(0.0, min(1.0, t))
                x = p1[0] + t * (p2[0] - p1[0])
                y = p1[1] + t * (p2[1] - p1[1])
                return (x, y)

            accumulated += seg_len

        return pts[-1]

    def _discretize_polyline(self, spacing):
        """
        Genera candidatos de rendezvous equiespaciados sobre la polyline.

        Siempre incluye el punto inicial y el punto final.
        Cada candidato tiene:
            {
                'point': (x, y),
                'distance_along': float,   # distancia acumulada desde el inicio
                'segment_idx': int,        # indice del segmento de la polyline
            }
        """
        if self.total_length < 1e-9 or not self.ugv_polyline:
            return []

        candidates = []

        d = 0.0
        while d <= self.total_length + 1e-9:
            d_clamped = min(d, self.total_length)
            point = self._interpolate_point_at_distance(d_clamped)
            seg_idx = self._find_segment_at_distance(d_clamped)
            candidates.append({
                'point': point,
                'distance_along': d_clamped,
                'segment_idx': seg_idx,
            })
            if d_clamped >= self.total_length:
                break
            d += spacing

        # Asegurar que el punto final este incluido si la division no lo tocó
        if candidates and candidates[-1]['distance_along'] < self.total_length - 1e-3:
            end_point = self._interpolate_point_at_distance(self.total_length)
            seg_idx = self._find_segment_at_distance(self.total_length)
            candidates.append({
                'point': end_point,
                'distance_along': self.total_length,
                'segment_idx': seg_idx,
            })

        return candidates

    # ------------------------------------------------------------------
    # SELECCION DE RENDEZVOUS
    # ------------------------------------------------------------------

    def find_best_rendezvous(self, uav_pos, uav_energy_rem, ugv_distance_along,
                             t_current, v_uav, transit_energy_fn, e_reserve):
        """
        Selecciona el candidato de rendezvous de menor costo factible.

        Factibilidad:
        1. El candidato debe estar por delante del UGV sobre la polyline.
        2. La energia para alcanzarlo debe caber dentro de uav_energy_rem - e_reserve.

        El UGV puede llegar antes y esperar; el UAV puede llegar antes y esperar.
        El costo penaliza principalmente la espera del UAV, el deadhead del UAV,
        y el avance del UGV, todos homogeneizados en segundos equivalentes:

            cost = alpha * t_wait_uav
                 + beta  * (d_uav / v_uav)
                 + gamma * (d_ugv / v_ugv)

        Parameters
        ----------
        uav_pos : (x, y)
            Posicion actual del UAV en metros.
        uav_energy_rem : float
            Energia restante del UAV en Wh.
        ugv_distance_along : float
            Distancia acumulada actual del UGV sobre la polyline en metros.
        t_current : float
            Tiempo actual de la simulacion en segundos.
        v_uav : float
            Velocidad del UAV en modo transito en m/s.
        transit_energy_fn : callable
            Funcion que recibe distance_m (float) y retorna energia en Wh.
            El caller debe capturar el reagente actual via closure o parcial.
            Ejemplo: lambda d: energy_model.energy_transit(d, reagent_remaining)
        e_reserve : float
            Reserva energetica minima requerida en Wh.

        Returns
        -------
        dict
            Si hay candidato factible:
                {
                    'feasible': True,
                    'point': (x, y),
                    'distance_along': float,
                    'd_uav': float,           # distancia de vuelo del UAV en metros
                    'd_ugv': float,           # distancia de avance del UGV en metros
                    'flight_time_uav': float, # tiempo de vuelo del UAV en segundos
                    'travel_time_ugv': float, # tiempo de viaje del UGV en segundos
                    't_uav_arrival': float,   # tiempo absoluto de llegada del UAV
                    't_ugv_arrival': float,   # tiempo absoluto de llegada del UGV
                    't_wait_uav': float,      # espera del UAV (0 si llega despues)
                    'cost': float,
                }
            Si no hay candidato factible:
                {'feasible': False}
        """
        v_uav = max(float(v_uav), 1e-9)
        uav_pos = (float(uav_pos[0]), float(uav_pos[1]))

        best = None
        best_cost = float('inf')

        for candidate in self._candidates:
            cand_point = candidate['point']
            d_along = candidate['distance_along']

            # Monotonic constraint: UGV can only move forward.
            # Skip candidates the UGV has already passed.
            if d_along < ugv_distance_along - 1e-3:
                continue

            d_uav = math.hypot(
                cand_point[0] - uav_pos[0],
                cand_point[1] - uav_pos[1],
            )

            # Energy feasibility
            energy_to_point = transit_energy_fn(d_uav)
            if energy_to_point > uav_energy_rem - e_reserve:
                continue

            flight_time_uav = d_uav / v_uav
            t_uav_arrival = t_current + flight_time_uav

            # UGV travel: only forward distance from current position
            d_ugv = max(0.0, d_along - ugv_distance_along)
            travel_time_ugv = d_ugv / self.v_ugv
            t_ugv_arrival = t_current + travel_time_ugv
            t_wait_uav = max(0.0, t_ugv_arrival - t_uav_arrival)

            # Weighted cost: balances UAV wait, UAV deadhead, UGV advance
            cost = (self.alpha * t_wait_uav
                    + self.beta * flight_time_uav
                    + self.gamma * travel_time_ugv)

            if cost < best_cost:
                best_cost = cost
                best = {
                    'feasible': True,
                    'point': cand_point,
                    'distance_along': d_along,
                    'd_uav': d_uav,
                    'd_ugv': d_ugv,
                    'flight_time_uav': flight_time_uav,
                    'travel_time_ugv': travel_time_ugv,
                    't_uav_arrival': t_uav_arrival,
                    't_ugv_arrival': t_ugv_arrival,
                    't_wait_uav': t_wait_uav,
                    'cost': cost,
                }

        if best is None:
            return {'feasible': False}

        return best

    # ------------------------------------------------------------------
    # PLANIFICACION DE CICLOS CON UGV MOVIL
    # ------------------------------------------------------------------

    def plan_dynamic_cycles(self, segmenter, polygon, route_segments):
        """
        Builds mission cycles with dynamic rendezvous for mobile UGV.

        Handles UGV time tracking and selects the best rendezvous point for
        each service stop. Inserts deadhead segments to/from rendezvous points.

        Called after coverage path planning. The segmenter provides energy/liquid
        feasibility predicates; this method owns the service stop decision.

        Parameters
        ----------
        segmenter : MissionSegmenter
            Pre-configured segmenter with drone params and energy model.
        polygon : Polygon
            Field polygon for obstacle-aware deadhead routing.
        route_segments : list[dict]
            Typed sweep/ferry segments from coverage path planner.

        Returns
        -------
        list[dict]
            Full cycles with deadheads, same format as static pipeline.
        """
        from ..coverage.path_assembler import PathAssembler

        if not route_segments:
            return []

        def _transit_fn(energy_model, q_reagent):
            def fn(distance_m):
                return energy_model.energy_transit(distance_m, q_reagent)
            return fn

        assembler = PathAssembler(polygon)
        energy_model = segmenter.energy_model
        usable_energy = energy_model.usable_energy_wh()
        reserve_wh = energy_model.reserve_wh_mobile()
        v_uav = float(segmenter.drone.speed_max_ms)

        ugv_distance_along = 0.0
        t_ugv_ref = 0.0  # mission time when ugv_distance_along was last set
        energy_remaining = usable_energy
        liquid_remaining = segmenter.tank_capacity
        t = 0.0

        cycles = []
        current_cycle_segments = []

        base_point = tuple(self.ugv_polyline[0])
        first_path = route_segments[0].get('path', [])
        uav_pos = segmenter._xy(first_path[0][:2]) if first_path else base_point

        # Opening deadhead: base (ugv_polyline[0]) → first work point
        dh_open_path, d_open = assembler.find_connection(base_point, uav_pos)
        if d_open > 1e-3:
            dh_open = segmenter._path_to_segments(dh_open_path, 'deadhead', False)
            current_cycle_segments.extend(dh_open)
            energy_remaining -= energy_model.energy_transit(d_open, liquid_remaining)
            t += energy_model.time_transit(d_open)

        i = 0
        while i < len(route_segments):
            seg = route_segments[i]
            seg_type = seg.get('segment_type', 'ferry')
            path = seg.get('path', [])
            spraying = bool(seg.get('spraying', seg_type == 'sweep'))

            if len(path) < 2:
                i += 1
                continue

            normalized_path = [segmenter._xy(pt[:2]) for pt in path]
            p2 = normalized_path[-1]
            dist = float(seg.get('distance_m', segmenter._path_length(normalized_path)))

            if dist < 1e-9:
                i += 1
                continue

            liq_step = segmenter._segment_liquid_use(seg_type, dist)
            energy_step = segmenter._segment_energy(seg_type, dist, liquid_remaining)
            liquid_after = max(0.0, liquid_remaining - liq_step)
            t_step = segmenter._segment_time(seg_type, dist)
            t_after = t + t_step

            # UGV advances at v_ugv during drone flight.  Query uses
            # the real UGV position at the time the drone would finish
            # this segment.
            ugv_s = min(
                ugv_distance_along + self.v_ugv * max(0.0, t_after - t_ugv_ref),
                self.total_length,
            )
            rv_after = self.find_best_rendezvous(
                uav_pos=p2,
                uav_energy_rem=energy_remaining - energy_step,
                ugv_distance_along=ugv_s,
                t_current=t_after,
                v_uav=v_uav,
                transit_energy_fn=_transit_fn(energy_model, liquid_after),
                e_reserve=reserve_wh,
            )

            dist_to_rv = rv_after['d_uav'] if rv_after['feasible'] else float('inf')
            can_do = energy_model.feasible_after_segment_dynamic(
                energy_remaining, liquid_remaining, energy_step, liq_step, dist_to_rv
            )

            if can_do or not current_cycle_segments:
                atomic = segmenter._path_to_segments(normalized_path, seg_type, spraying)
                current_cycle_segments.extend(atomic)
                energy_remaining -= energy_step
                liquid_remaining = liquid_after
                t += t_step
                uav_pos = p2
                i += 1
                continue

            # --- Service needed before this segment ---

            # Trim trailing non-spray segments only if the cycle already
            # contains at least one sweep.  If the cycle is purely
            # deadhead (opening transit), trimming would empty it and
            # corrupt the resume position.
            has_sweep = any(
                s.get('segment_type') == 'sweep' for s in current_cycle_segments
            )
            if has_sweep:
                while (current_cycle_segments
                       and current_cycle_segments[-1].get('segment_type') != 'sweep'):
                    dropped = current_cycle_segments.pop()
                    d_drop = dropped.get('distance_m', 0.0)
                    energy_remaining += energy_model.energy_transit(d_drop, liquid_remaining)
                    t -= segmenter._segment_time(dropped.get('segment_type', 'ferry'), d_drop)
                uav_pos = current_cycle_segments[-1]['p2']

            resume_pos = uav_pos

            ugv_s = min(
                ugv_distance_along + self.v_ugv * max(0.0, t - t_ugv_ref),
                self.total_length,
            )
            rv = self.find_best_rendezvous(
                uav_pos=uav_pos,
                uav_energy_rem=energy_remaining,
                ugv_distance_along=ugv_s,
                t_current=t,
                v_uav=v_uav,
                transit_energy_fn=_transit_fn(energy_model, liquid_remaining),
                e_reserve=reserve_wh,
            )

            if not rv['feasible']:
                # TODO: raise MissionInfeasibleError — currently falls back to
                # continuing the segment to avoid aborting the mission silently.
                atomic = segmenter._path_to_segments(normalized_path, seg_type, spraying)
                current_cycle_segments.extend(atomic)
                energy_remaining -= energy_step
                liquid_remaining = liquid_after
                t += t_step
                uav_pos = p2
                i += 1
                continue

            rv_point = rv['point']

            dh_to_rv_path, _ = assembler.find_connection(uav_pos, rv_point)
            dh_to_rv = segmenter._path_to_segments(dh_to_rv_path, 'deadhead', False)

            cycle_all_segs = current_cycle_segments + dh_to_rv
            cycle_path = segmenter.segments_to_path(cycle_all_segs)
            cycles.append({
                "type": "work",
                "path": cycle_path,
                "segments": cycle_all_segs,
                "visual_groups": segmenter.compress_segments(cycle_all_segs),
                "swath_width": segmenter.swath_width,
                "base_point": rv_point,
                "rv_wait_s": max(0.0, rv['t_wait_uav']),
            })

            t += rv['flight_time_uav']
            meet_time = max(rv['t_uav_arrival'], rv['t_ugv_arrival'])
            t = meet_time + self.t_service

            # UGV is at the RV point after service.  Reset reference
            # time so it resumes advancing from here during next cycle.
            ugv_distance_along = rv['distance_along']
            t_ugv_ref = t

            energy_remaining = usable_energy
            liquid_remaining = segmenter.tank_capacity
            uav_pos = rv_point

            dh_from_rv_path, d_back = assembler.find_connection(rv_point, resume_pos)
            e_back = energy_model.energy_transit(d_back, liquid_remaining)
            t_back = energy_model.time_transit(d_back)
            energy_remaining -= e_back
            t += t_back
            uav_pos = resume_pos

            dh_from_rv = segmenter._path_to_segments(dh_from_rv_path, 'deadhead', False)
            current_cycle_segments = dh_from_rv
            # Do NOT increment i: retry same segment with fresh resources

        if current_cycle_segments:
            # Trim trailing non-spray before closing deadhead, but only
            # if there is at least one sweep to fall back to.
            has_sweep = any(
                s.get('segment_type') == 'sweep' for s in current_cycle_segments
            )
            if has_sweep:
                while (current_cycle_segments
                       and current_cycle_segments[-1].get('segment_type') != 'sweep'):
                    dropped = current_cycle_segments.pop()
                    d_drop = dropped.get('distance_m', 0.0)
                    energy_remaining += energy_model.energy_transit(d_drop, liquid_remaining)
                    t -= segmenter._segment_time(dropped.get('segment_type', 'ferry'), d_drop)
                uav_pos = current_cycle_segments[-1]['p2']

            # Closing deadhead: last work point → final RV on UGV polyline
            ugv_s = min(
                ugv_distance_along + self.v_ugv * max(0.0, t - t_ugv_ref),
                self.total_length,
            )
            rv_final = self.find_best_rendezvous(
                uav_pos=uav_pos,
                uav_energy_rem=energy_remaining,
                ugv_distance_along=ugv_s,
                t_current=t,
                v_uav=v_uav,
                transit_energy_fn=_transit_fn(energy_model, liquid_remaining),
                e_reserve=0.0,  # final landing, no subsequent flight
            )

            if rv_final['feasible']:
                rv_point = rv_final['point']
                dh_close_path, _ = assembler.find_connection(uav_pos, rv_point)
                dh_close = segmenter._path_to_segments(
                    dh_close_path, 'deadhead', False)
                current_cycle_segments.extend(dh_close)
                final_base = rv_point
            else:
                final_base = uav_pos

            cycle_path = segmenter.segments_to_path(current_cycle_segments)
            cycles.append({
                "type": "work",
                "path": cycle_path,
                "segments": current_cycle_segments,
                "visual_groups": segmenter.compress_segments(current_cycle_segments),
                "swath_width": segmenter.swath_width,
                "base_point": final_base,
            })

        return cycles
