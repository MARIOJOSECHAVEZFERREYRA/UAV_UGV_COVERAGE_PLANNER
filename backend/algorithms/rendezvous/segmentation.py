import math

from shapely.geometry import LineString
import numpy as np

_SNAP_THRESHOLD = 0.6   # endpoint must be at most this fraction of d_current from base
_SNAP_LOOKAHEAD = 2     # max atomic segments to look ahead for a near-base landing

from ..drone.energy_model import DroneEnergyModel
from ..polygon.path_assembler import PathAssembler


def _make_seg_transit_fn(energy_model, q_reagent):
    """
    Devuelve una funcion de costo de transito con reactivo fijo.
    Evita el uso de lambdas (convencion del proyecto).
    """
    def transit_energy(distance_m):
        return energy_model.energy_transit(distance_m, q_reagent)
    return transit_energy


class MissionSegmenter:
    """
    Segments a typed route into operable CYCLES based on drone resources.

    Expected input route_segments:
        [
            {
                "segment_type": "sweep" | "ferry",
                "spraying": bool,
                "path": [(x, y), ...],
                "distance_m": float,   # optional
            },
            ...
        ]

    This component:
    - trusts upstream segment semantics for sweep/ferry
    - inserts obstacle-safe deadhead segments to/from the static base
    - splits the mission into cycles according to battery/liquid constraints
    """

    def __init__(self, drone, target_rate_l_ha=20.0, work_speed_kmh=None, swath_width=None,
                 energy_model=None):
        self.drone = drone
        self.rate_l_ha = target_rate_l_ha
        # Accept an externally-created DroneEnergyModel so the controller can
        # inject the shared instance. Falls back to creating one if not provided
        # (e.g. when MissionSegmenter is used directly in tests or scripts).
        self.energy_model = energy_model if energy_model is not None else DroneEnergyModel(drone)

        if work_speed_kmh is not None:
            self.speed_kmh = work_speed_kmh
        else:
            self.speed_kmh = drone.speed_cruise_ms * 3.6
        self.speed_ms = self.speed_kmh / 3.6

        if swath_width is not None:
            self.swath_width = swath_width
        else:
            self.swath_width = drone.spray_swath_max_m

        self.liters_per_meter = (self.rate_l_ha * self.swath_width) / 10000.0
        self.tank_capacity = drone.mass_tank_full_kg

    @staticmethod
    def _xy(point):
        return (float(point[0]), float(point[1]))

    @staticmethod
    def _pts_equal(a, b, tol=1e-6):
        return abs(a[0] - b[0]) < tol and abs(a[1] - b[1]) < tol

    @staticmethod
    def _path_length(path_coords):
        if not path_coords or len(path_coords) < 2:
            return 0.0

        total = 0.0
        for i in range(len(path_coords) - 1):
            p1 = path_coords[i]
            p2 = path_coords[i + 1]
            total += np.linalg.norm(np.array(p1) - np.array(p2))
        return float(total)

    def _segment_energy(self, seg_type, distance_m, liquid_remaining):
        if seg_type == "sweep":
            return self.energy_model.energy_straight(distance_m, liquid_remaining)
        return self.energy_model.energy_transit(distance_m, liquid_remaining)

    def _segment_time(self, seg_type, distance_m):
        if seg_type == "sweep":
            return self.energy_model.time_straight(distance_m)
        return self.energy_model.time_transit(distance_m)

    def _segment_liquid_use(self, seg_type, distance_m):
        if seg_type == "sweep":
            return distance_m * self.liters_per_meter
        return 0.0

    @staticmethod
    def _path_to_segments(path_coords, segment_type, spraying):
        if not path_coords or len(path_coords) < 2:
            return []

        segments = []
        for i in range(len(path_coords) - 1):
            p1 = path_coords[i]
            p2 = path_coords[i + 1]
            seg_line = LineString([p1, p2])
            segments.append({
                "p1": p1,
                "p2": p2,
                "spraying": spraying,
                "segment_type": segment_type,
                "distance_m": float(seg_line.length),
            })
        return segments

    def _expand_route_segments(self, route_segments):
        """
        Expands multi-point route segments into atomic 2-point segments while preserving type.
        """
        atomic = []

        for seg in route_segments or []:
            seg_type = seg.get("segment_type")
            if seg_type not in ("sweep", "ferry", "deadhead"):
                raise ValueError("Unknown or missing segment_type in route_segments: {}".format(seg_type))

            spraying = bool(seg.get("spraying", seg_type == "sweep"))
            path = seg.get("path", [])
            if not path or len(path) < 2:
                continue

            normalized_path = [self._xy(pt[:2]) if len(pt) >= 2 else self._xy(pt) for pt in path]
            atomic.extend(self._path_to_segments(normalized_path, seg_type, spraying))

        return atomic

    def _safe_deadhead_segments(self, assembler, start_pt, end_pt):
        path_coords, _ = assembler.safe_connection(start_pt, end_pt)
        segments = self._path_to_segments(path_coords, "deadhead", False)
        return segments, path_coords

    def _segments_to_path(self, segments):
        if not segments:
            return []

        path = [segments[0]["p1"]]
        for seg in segments:
            if not self._pts_equal(path[-1], seg["p2"]):
                path.append(seg["p2"])
        return path

    def segment_path(self, polygon, route_segments, base_point=None):
        """
        Segments the typed route into cycles. Each cycle starts and ends at base_point.
        """
        atomic_segments = self._expand_route_segments(route_segments)

        if not atomic_segments:
            return []

        if base_point is None:
            raise ValueError("base_point is required for static mission segmentation.")

        base_point = self._xy(base_point[:2])
        assembler = PathAssembler(polygon)

        usable_energy = self.energy_model.usable_energy_wh()

        energy_remaining = usable_energy
        liquid_remaining = self.tank_capacity

        cycles = []
        current_cycle_segments = []

        first_start = atomic_segments[0]["p1"]
        init_path, _ = assembler.safe_connection(base_point, first_start)
        init_dist = self._path_length(init_path)
        energy_remaining -= self.energy_model.energy_transit(init_dist, liquid_remaining)

        i = 0
        while i < len(atomic_segments):
            seg = atomic_segments[i]
            p1 = seg["p1"]
            p2 = seg["p2"]
            seg_type = seg["segment_type"]
            spraying = seg["spraying"]
            dist_step = seg.get("distance_m", self._path_length([p1, p2]))

            liq_step = self._segment_liquid_use(seg_type, dist_step)
            energy_step = self._segment_energy(seg_type, dist_step, liquid_remaining)

            return_path, _ = assembler.safe_connection(p2, base_point)
            dist_return = self._path_length(return_path)

            can_do = self.energy_model.feasible_after_segment_static(
                energy_remaining, liquid_remaining, energy_step, liq_step, dist_return
            )

            # Guard: always execute first atomic segment of an empty cycle
            if can_do or not current_cycle_segments:
                current_cycle_segments.append({
                    "p1": p1,
                    "p2": p2,
                    "spraying": spraying,
                    "segment_type": seg_type,
                    "distance_m": float(dist_step),
                })
                energy_remaining -= energy_step
                liquid_remaining -= liq_step
                i += 1
            else:
                # --- Base-side snapping ---
                # Before closing the cycle at the current position, look up to
                # _SNAP_LOOKAHEAD atomic segments ahead.  In a boustrophedon a
                # strip that ends far from the base is typically followed by a
                # short ferry + a return strip heading back toward the base.
                # If a nearby segment endpoint is significantly closer to the
                # base (< _SNAP_THRESHOLD * current distance) AND the drone can
                # afford all lookahead segments plus the return from there, snap
                # to that endpoint before closing.  This eliminates the long
                # deadheads that occur when a cycle closes on the "far" side.
                current_end = current_cycle_segments[-1]["p2"] if current_cycle_segments else base_point
                d_current = math.hypot(current_end[0] - base_point[0],
                                       current_end[1] - base_point[1])

                snapped = False
                if d_current > 1e-3:
                    e_acc = energy_remaining
                    q_acc = liquid_remaining
                    snap_buffer = []

                    for j in range(i, min(i + _SNAP_LOOKAHEAD, len(atomic_segments))):
                        look = atomic_segments[j]
                        ltype = look["segment_type"]
                        ldist = look.get("distance_m",
                                         self._path_length([look["p1"], look["p2"]]))
                        le = self._segment_energy(ltype, ldist, q_acc)
                        lq = self._segment_liquid_use(ltype, ldist)

                        e_acc -= le
                        q_acc = max(0.0, q_acc - lq)
                        snap_buffer.append((look, le, lq))

                        d_end = math.hypot(look["p2"][0] - base_point[0],
                                           look["p2"][1] - base_point[1])

                        if d_end >= d_current * _SNAP_THRESHOLD:
                            continue  # not significantly closer — keep looking

                        # Near-base candidate found.  Check feasibility:
                        # remaining resources after lookahead must cover the
                        # return flight plus the operational reserve.
                        snap_return_path, _ = assembler.safe_connection(
                            look["p2"], base_point
                        )
                        d_snap_ret = self._path_length(snap_return_path)
                        e_snap_ret = self.energy_model.energy_to_service_static(
                            d_snap_ret, max(q_acc, 0.0)
                        )
                        e_reserve = self.energy_model.reserve_wh_static()

                        if e_acc - e_snap_ret >= e_reserve and q_acc >= 0.0:
                            # Affordable snap: execute the lookahead segments
                            for snap_seg, snap_e, snap_q in snap_buffer:
                                current_cycle_segments.append({
                                    "p1": snap_seg["p1"],
                                    "p2": snap_seg["p2"],
                                    "spraying": snap_seg["spraying"],
                                    "segment_type": snap_seg["segment_type"],
                                    "distance_m": float(snap_seg.get(
                                        "distance_m",
                                        self._path_length([snap_seg["p1"], snap_seg["p2"]]),
                                    )),
                                })
                                energy_remaining -= snap_e
                                liquid_remaining = max(0.0, liquid_remaining - snap_q)
                            i += len(snap_buffer)
                            snapped = True
                        break  # stop lookahead regardless (either snapped or not affordable)

                # Close the cycle (with or without snap)
                cycle_start = current_cycle_segments[0]["p1"]
                cycle_end = current_cycle_segments[-1]["p2"]

                open_segments, _ = self._safe_deadhead_segments(assembler, base_point, cycle_start)
                close_segments, _ = self._safe_deadhead_segments(assembler, cycle_end, base_point)

                cycle_segments = open_segments + current_cycle_segments + close_segments
                full_path = self._segments_to_path(cycle_segments)
                cycles.append(self._make_cycle(full_path, cycle_segments, base_point))

                energy_remaining = usable_energy
                liquid_remaining = self.tank_capacity

                if not snapped:
                    commute_path, _ = assembler.safe_connection(base_point, p1)
                    dist_commute = self._path_length(commute_path)
                else:
                    commute_path, _ = assembler.safe_connection(base_point, atomic_segments[i]["p1"])
                    dist_commute = self._path_length(commute_path)
                energy_remaining -= self.energy_model.energy_transit(
                    dist_commute,
                    liquid_remaining,
                )

                current_cycle_segments = []

        if current_cycle_segments:
            cycle_start = current_cycle_segments[0]["p1"]
            cycle_end = current_cycle_segments[-1]["p2"]

            open_segments, _ = self._safe_deadhead_segments(assembler, base_point, cycle_start)
            close_segments, _ = self._safe_deadhead_segments(assembler, cycle_end, base_point)

            cycle_segments = open_segments + current_cycle_segments + close_segments
            full_path = self._segments_to_path(cycle_segments)
            cycles.append(self._make_cycle(full_path, cycle_segments, base_point))

        return cycles

    def segment_path_mobile(self, polygon, route_segments, rendezvous_planner):
        """
        Segmenta la ruta en ciclos usando rendezvous con UGV movil.

        En lugar de volver a una base fija, cada parada de servicio ocurre
        en el mejor punto de encuentro factible sobre la polyline del UGV.

        Logica de checkpoints de servicio:
        - Se trabaja al nivel de route_segment (sweep/ferry completo), no atomico.
        - Antes de cada segmento, se verifica si hay energia para hacerlo
          Y llegar a un rendezvous despues. Si no, se dispara servicio primero.
        - No se interrumpe un segmento a mitad.

        La estructura de cada ciclo es:
          [deadhead rv→resume_pos (solo desde ciclo 2)]
          + segmentos de trabajo
          + [deadhead ultimo_pos→rv_point]

        El base_point del ciclo es el punto de rendezvous de ese ciclo.
        El ultimo ciclo no tiene deadhead final (mision completa).

        Parameters
        ----------
        polygon : Polygon
            Poligono del campo para routing con obstaculos.
        route_segments : list de dict
            Segmentos tipados (sweep/ferry) de PathAssembler.
        rendezvous_planner : RendezvousPlanner
            Planificador tactico de encuentro UAV-UGV.

        Returns
        -------
        list de dicts de ciclo, mismo formato que segment_path().
        """
        if not route_segments:
            return []

        assembler = PathAssembler(polygon)
        usable_energy = self.energy_model.usable_energy_wh()
        # reserve_wh is used only when passing e_reserve to find_best_rendezvous;
        # the feasibility predicate (feasible_after_segment_dynamic) applies its
        # own reserve internally via DroneEnergyModel.reserve_wh_mobile().
        reserve_wh = self.energy_model.reserve_wh_mobile()
        v_uav = float(self.drone.speed_max_ms)

        ugv_distance_along = 0.0
        ugv_last_update_time = 0.0

        energy_remaining = usable_energy
        liquid_remaining = self.tank_capacity
        t = 0.0

        cycles = []
        current_cycle_segments = []

        # El UAV empieza en el primer punto de la ruta
        first_path = route_segments[0].get('path', [])
        uav_pos = self._xy(first_path[0][:2]) if first_path else (0.0, 0.0)

        i = 0
        while i < len(route_segments):
            seg = route_segments[i]
            seg_type = seg.get('segment_type', 'ferry')
            path = seg.get('path', [])
            spraying = bool(seg.get('spraying', seg_type == 'sweep'))

            if len(path) < 2:
                i += 1
                continue

            normalized_path = [self._xy(pt[:2]) for pt in path]
            p2 = normalized_path[-1]
            dist = float(seg.get('distance_m', self._path_length(normalized_path)))

            if dist < 1e-9:
                i += 1
                continue

            # Avanzar UGV al tiempo actual
            ugv_state = rendezvous_planner.advance_ugv_to_time(
                ugv_distance_along, ugv_last_update_time, t
            )
            ugv_distance_along = ugv_state['distance_along']
            ugv_last_update_time = ugv_state['last_update_time']

            # Costos del segmento actual
            liq_step = self._segment_liquid_use(seg_type, dist)
            energy_step = self._segment_energy(seg_type, dist, liquid_remaining)
            liquid_after = max(0.0, liquid_remaining - liq_step)
            t_step = self._segment_time(seg_type, dist)
            t_after = t + t_step

            # Estimar si, tras ejecutar este segmento, el UAV podra llegar a un rendezvous
            rv_after = rendezvous_planner.find_best_rendezvous(
                uav_pos=p2,
                uav_energy_rem=energy_remaining - energy_step,
                ugv_distance_along=ugv_distance_along,
                t_current=t_after,
                v_uav=v_uav,
                transit_energy_fn=_make_seg_transit_fn(self.energy_model, liquid_after),
                e_reserve=reserve_wh,
            )

            # float('inf') signals "no feasible rendezvous exists"; the predicate
            # will return False because energy_to_service_dynamic(inf, ...) = inf.
            dist_to_rv = rv_after['d_uav'] if rv_after['feasible'] else float('inf')
            can_do = self.energy_model.feasible_after_segment_dynamic(
                energy_remaining, liquid_remaining, energy_step, liq_step, dist_to_rv
            )

            # Guardia: siempre ejecutar el primer segmento de un ciclo vacio
            if can_do or not current_cycle_segments:
                atomic = self._path_to_segments(normalized_path, seg_type, spraying)
                current_cycle_segments.extend(atomic)
                energy_remaining -= energy_step
                liquid_remaining = liquid_after
                t += t_step
                uav_pos = p2
                i += 1
                continue

            # --- Servicio necesario antes de este segmento ---

            resume_pos = uav_pos

            # Avanzar UGV al tiempo actual (ya fue avanzado antes, pero confirmar)
            ugv_state = rendezvous_planner.advance_ugv_to_time(
                ugv_distance_along, ugv_last_update_time, t
            )
            ugv_distance_along = ugv_state['distance_along']
            ugv_last_update_time = ugv_state['last_update_time']

            rv = rendezvous_planner.find_best_rendezvous(
                uav_pos=uav_pos,
                uav_energy_rem=energy_remaining,
                ugv_distance_along=ugv_distance_along,
                t_current=t,
                v_uav=v_uav,
                transit_energy_fn=_make_seg_transit_fn(self.energy_model, liquid_remaining),
                e_reserve=reserve_wh,
            )

            if not rv['feasible']:
                # Fallback: no hay rendezvous factible — continuar el segmento de todos modos.
                #
                # TODO (TASK 3): este fallback viola el criterio energético centralizado
                # introducido en TASK 1 (feasible_after_segment_dynamic). El UAV puede
                # continuar ejecutando trabajo aunque ya no le quede energía suficiente
                # para llegar a ningún punto de servicio, produciendo una misión
                # físicamente no ejecutable.
                #
                # En TASK 3 decidir la política correcta entre:
                #   a) Abortar la misión y propagar un error al controlador.
                #   b) Cerrar el ciclo en el último punto conocido y detener la segmentación.
                #   c) Escalar la condición al controlador para que decida (e.g. excepción
                #      MissionInfeasibleError con el estado actual del UAV y del UGV).
                atomic = self._path_to_segments(normalized_path, seg_type, spraying)
                current_cycle_segments.extend(atomic)
                energy_remaining -= energy_step
                liquid_remaining = liquid_after
                t += t_step
                uav_pos = p2
                i += 1
                continue

            rv_point = rv['point']

            # Deadhead: posicion actual → punto de rendezvous
            dh_to_rv_path, _ = assembler.safe_connection(uav_pos, rv_point)
            dh_to_rv = self._path_to_segments(dh_to_rv_path, 'deadhead', False)

            # Cerrar ciclo actual.
            # rv es el segundo find_best_rendezvous (CONFIRMACION de servicio),
            # no el rv_after del lookahead (que solo se usó para can_do).
            # max(0.0, ...) garantiza que rv_wait_s nunca sea negativo aunque
            # haya discrepancias de punto flotante en los tiempos de llegada.
            cycle_all_segs = current_cycle_segments + dh_to_rv
            cycle_path = self._segments_to_path(cycle_all_segs)
            cycle_dict = self._make_cycle(cycle_path, cycle_all_segs, rv_point)
            cycle_dict['rv_wait_s'] = max(0.0, rv['t_wait_uav'])
            cycles.append(cycle_dict)

            # Actualizar tiempo: vuelo al rv + espera + servicio
            t += rv['flight_time_uav']
            meet_time = max(rv['t_uav_arrival'], rv['t_ugv_arrival'])
            t = meet_time
            t += rendezvous_planner.t_service

            # Fijar estado del UGV en el punto de rendezvous (detenido durante servicio)
            ugv_distance_along = rv['distance_along']
            ugv_last_update_time = t

            # Resetear recursos del UAV
            energy_remaining = usable_energy
            liquid_remaining = self.tank_capacity
            uav_pos = rv_point

            # Retornar al punto de reanudacion usando la misma ruta segura que
            # se usara en la geometria del ciclo, para que costo y trayecto sean consistentes.
            dh_from_rv_path, d_back = assembler.safe_connection(rv_point, resume_pos)
            e_back = self.energy_model.energy_transit(d_back, liquid_remaining)
            t_back = self.energy_model.time_transit(d_back)
            energy_remaining -= e_back
            t += t_back
            uav_pos = resume_pos

            # El nuevo ciclo empieza con el deadhead rv→resume_pos
            dh_from_rv = self._path_to_segments(dh_from_rv_path, 'deadhead', False)
            current_cycle_segments = dh_from_rv
            # No incrementar i: reintentar el mismo segmento con recursos frescos

        # Cerrar el ultimo ciclo (mision completa, sin deadhead final)
        if current_cycle_segments:
            cycle_path = self._segments_to_path(current_cycle_segments)
            cycles.append(self._make_cycle(cycle_path, current_cycle_segments, uav_pos))

        return cycles

    def _make_cycle(self, full_path, segments, base_point):
        return {
            "type": "work",
            "path": full_path,
            "segments": segments,
            "visual_groups": self._compress_segments(segments),
            "swath_width": self.swath_width,
            "base_point": base_point,
        }

    def _compress_segments(self, segments):
        """
        Merges adjacent segments of the same semantic type into visual groups.
        Keeps both segment_type and spraying flag for downstream rendering if needed.
        """
        if not segments:
            return []

        groups = []
        current_path = [segments[0]["p1"], segments[0]["p2"]]
        current_type = segments[0]["segment_type"]
        current_spraying = segments[0]["spraying"]

        for seg in segments[1:]:
            if seg["segment_type"] == current_type:
                current_path.append(seg["p2"])
            else:
                groups.append({
                    "path": current_path,
                    "segment_type": current_type,
                    "is_spraying": current_spraying,
                })
                current_path = [seg["p1"], seg["p2"]]
                current_type = seg["segment_type"]
                current_spraying = seg["spraying"]

        groups.append({
            "path": current_path,
            "segment_type": current_type,
            "is_spraying": current_spraying,
        })
        return groups
