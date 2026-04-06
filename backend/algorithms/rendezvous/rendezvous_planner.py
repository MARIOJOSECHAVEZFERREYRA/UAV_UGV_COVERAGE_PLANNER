"""
Modulo de planificacion tactica de rendezvous UAV-UGV.

El RendezvousPlanner NO forma parte del cromosoma del GA.
Se invoca durante la evaluacion del fitness como consecuencia
de la logica de servicio, dados el estado actual del UAV y del UGV.

Arquitectura de co-optimizacion implicita:
  - Nivel estrategico: el GA decide angulos y orden de celdas.
  - Nivel tactico (este modulo): dado el estado del sistema,
    selecciona el mejor punto de encuentro factible sobre la
    polyline fija del UGV.
"""

import math


class RendezvousPlanner:
    """
    Selecciona el mejor punto de rendezvous sobre la polyline fija del UGV.

    Garantias del UGV:
    - Movimiento monotono hacia adelante (nunca retrocede).
    - Puede detenerse y esperar en el punto de encuentro.
    - Su posicion se actualiza continuamente con advance_ugv_to_time().
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
    # CINEMATICA DEL UGV
    # ------------------------------------------------------------------

    def advance_ugv_to_time(self, ugv_distance_along, last_update_time, t_now):
        """
        Actualiza la posicion del UGV avanzando desde last_update_time hasta t_now.

        El UGV:
        - Solo avanza hacia adelante (movimiento monotono).
        - Se satura al final de la polyline.
        - Si delta_t <= 0 no se mueve.

        Parameters
        ----------
        ugv_distance_along : float
            Distancia acumulada actual del UGV sobre la polyline en metros.
        last_update_time : float
            Tiempo en segundos en el que se registro el estado actual del UGV.
        t_now : float
            Tiempo actual en segundos.

        Returns
        -------
        dict
            {'distance_along': float, 'last_update_time': float}
        """
        delta_t = t_now - last_update_time

        if delta_t <= 0:
            return {
                'distance_along': ugv_distance_along,
                'last_update_time': last_update_time,
            }

        new_distance = ugv_distance_along + self.v_ugv * delta_t
        new_distance = min(new_distance, self.total_length)

        return {
            'distance_along': new_distance,
            'last_update_time': t_now,
        }

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
            d_along = candidate['distance_along']

            # Filtro monotono: solo candidatos por delante del UGV
            if d_along < ugv_distance_along - 1e-3:
                continue

            cand_point = candidate['point']

            # Distancia euclidea del UAV al candidato
            d_uav = math.hypot(
                cand_point[0] - uav_pos[0],
                cand_point[1] - uav_pos[1],
            )

            # Verificar factibilidad energetica antes de calcular tiempos
            energy_to_point = transit_energy_fn(d_uav)
            if energy_to_point > uav_energy_rem - e_reserve:
                continue

            # Tiempos de llegada
            flight_time_uav = d_uav / v_uav
            t_uav_arrival = t_current + flight_time_uav

            # Distancia que el UGV debe avanzar (siempre >= 0 por el filtro de arriba)
            d_ugv = max(0.0, d_along - ugv_distance_along)
            travel_time_ugv = d_ugv / self.v_ugv
            t_ugv_arrival = t_current + travel_time_ugv

            # Espera del UAV: solo si llega antes que el UGV
            t_wait_uav = max(0.0, t_ugv_arrival - t_uav_arrival)

            # Funcion de costo homogenea en segundos equivalentes
            cost = (
                self.alpha * t_wait_uav
                + self.beta * (d_uav / v_uav)
                + self.gamma * (d_ugv / self.v_ugv)
            )

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
