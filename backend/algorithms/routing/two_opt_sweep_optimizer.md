# Sweep Order Optimizer (Greedy NN + 2-opt)

## Propósito

Decide en qué orden visitar las pasadas de barrido para minimizar distancia total de ferries.

Pipeline en dos fases:
1. **Greedy NN** — construye orden inicial eligiendo siempre la pasada no visitada más barata
2. **2-opt** — refina ese orden invirtiendo subsecuencias para eliminar cruces

## Pertenencia arquitectónica

Esta lógica pertenece en `backend/algorithms/routing/`, no en `coverage/path_assembler.py`.
Es optimización de ruta, independiente del ensamblaje de ferries.

Destino sugerido: `backend/algorithms/routing/sweep_order_optimizer.py`

---

## Fase 1 — Greedy Nearest-Neighbor

### Propósito

Construye el orden de visita `[(sweep_idx, is_reversed), ...]` eligiendo en cada paso
la pasada no visitada con menor costo combinado:

```
cost = ferry_dist(actual → candidata_entrada)
     + ugv_bias  * dist(candidata_salida, ugv_polyline)   [modo dinámico]
     + base_bias * dist(candidata_salida, base_point)      [modo estático]
```

El bias de servicio hace que los ciclos terminen cerca del punto de recarga,
reduciendo deadhead cuando el dron necesita servicio.

La selección de la primera pasada también usa proximidad al punto de servicio.

### Helpers requeridos

```python
@staticmethod
def _sweep_endpoint(path, reversed_):
    """Return the exit endpoint of a sweep given its traversal direction."""
    return path[0] if reversed_ else path[-1]

@staticmethod
def _sweep_startpoint(path, reversed_):
    """Return the entry endpoint of a sweep given its traversal direction."""
    return path[-1] if reversed_ else path[0]

def _dist_to_base(self, point):
    """Euclidean distance from point to the static base. Returns 0.0 if not configured."""
    if self._base_point is None:
        return 0.0
    return math.hypot(
        float(point[0]) - self._base_point[0],
        float(point[1]) - self._base_point[1],
    )

def _dist_to_ugv(self, point):
    """Minimum perpendicular distance from point to the UGV polyline. Returns 0.0 if not configured."""
    if not self._ugv_polyline:
        return 0.0
    px, py = float(point[0]), float(point[1])
    min_d = float('inf')
    for i in range(len(self._ugv_polyline) - 1):
        x1, y1 = self._ugv_polyline[i]
        x2, y2 = self._ugv_polyline[i + 1]
        dx, dy = x2 - x1, y2 - y1
        seg_len_sq = dx * dx + dy * dy
        if seg_len_sq < 1e-12:
            d = math.hypot(px - x1, py - y1)
        else:
            t = max(0.0, min(1.0, ((px - x1) * dx + (py - y1) * dy) / seg_len_sq))
            cx, cy = x1 + t * dx, y1 + t * dy
            d = math.hypot(px - cx, py - cy)
        if d < min_d:
            min_d = d
    return min_d
```

### Código

```python
# Constructor params needed:
#   ugv_polyline, ugv_bias=0.3, base_point=None, base_bias=0.3

remaining = list(range(len(sweeps)))

if self._ugv_polyline:
    # Dynamic mode: start from the sweep endpoint closest to UGV start.
    service_start = self._ugv_polyline[0]
    best_start_cost = float('inf')
    best_start_idx = 0
    best_start_rev = False
    for idx in remaining:
        path = sweeps[idx]["path"]
        d_fwd = math.hypot(path[0][0] - service_start[0], path[0][1] - service_start[1])
        d_rev = math.hypot(path[-1][0] - service_start[0], path[-1][1] - service_start[1])
        if d_fwd < best_start_cost:
            best_start_cost = d_fwd
            best_start_idx = idx
            best_start_rev = False
        if d_rev < best_start_cost:
            best_start_cost = d_rev
            best_start_idx = idx
            best_start_rev = True
    remaining.remove(best_start_idx)
    order = [(best_start_idx, best_start_rev)]
elif self._base_point is not None:
    # Static mode: start from the sweep entry closest to the base.
    bx, by = self._base_point
    best_start_cost = float('inf')
    best_start_idx = 0
    best_start_rev = False
    for idx in remaining:
        path = sweeps[idx]["path"]
        d_fwd = math.hypot(path[0][0] - bx, path[0][1] - by)
        d_rev = math.hypot(path[-1][0] - bx, path[-1][1] - by)
        if d_fwd < best_start_cost:
            best_start_cost = d_fwd
            best_start_idx = idx
            best_start_rev = False
        if d_rev < best_start_cost:
            best_start_cost = d_rev
            best_start_idx = idx
            best_start_rev = True
    remaining.remove(best_start_idx)
    order = [(best_start_idx, best_start_rev)]
else:
    current_idx = remaining.pop(0)
    order = [(current_idx, False)]

while remaining:
    _, last_rev = order[-1]
    last_idx, _ = order[-1]
    current_end = self._sweep_endpoint(sweeps[last_idx]["path"], last_rev)

    best_cost = float("inf")
    best_idx = None
    best_reverse = False

    for idx in remaining:
        candidate_path = sweeps[idx]["path"]

        # Forward: entry = path[0], exit = path[-1]
        _, fwd_dist = self._safe_ferry(current_end, candidate_path[0])
        fwd_cost = (fwd_dist
                    + self._ugv_bias * self._dist_to_ugv(candidate_path[-1])
                    + self._base_bias * self._dist_to_base(candidate_path[-1]))
        if fwd_cost < best_cost:
            best_cost = fwd_cost
            best_idx = idx
            best_reverse = False

        # Reversed: entry = path[-1], exit = path[0]
        _, rev_dist = self._safe_ferry(current_end, candidate_path[-1])
        rev_cost = (rev_dist
                    + self._ugv_bias * self._dist_to_ugv(candidate_path[0])
                    + self._base_bias * self._dist_to_base(candidate_path[0]))
        if rev_cost < best_cost:
            best_cost = rev_cost
            best_idx = idx
            best_reverse = True

    remaining.remove(best_idx)
    order.append((best_idx, best_reverse))
```

---

## Fase 2 — 2-opt Local Search

### Propósito

Mejora el orden producido por greedy NN invirtiendo subsecuencias para eliminar cruces.
Al invertir una subsecuencia también se voltea el flag `is_reversed` de cada elemento
(cambia el extremo de entrada del sweep).

### Complejidad

- O(n²) evaluaciones por pasada, cada una O(1) con distancia Euclidiana
- Ferries reales (obstacle-aware) solo se calculan al construir la ruta final

### Código

```python
def _two_opt_improve(self, sweeps, order, max_iter=5):
    n = len(order)
    if n < 3:
        return order

    def eucl(a, b):
        return math.hypot(b[0] - a[0], b[1] - a[1])

    def edge_eucl(order_, i):
        idx_i, rev_i = order_[i]
        idx_j, rev_j = order_[i + 1]
        a = self._sweep_endpoint(sweeps[idx_i]["path"], rev_i)
        b = self._sweep_startpoint(sweeps[idx_j]["path"], rev_j)
        return eucl(a, b)

    order = list(order)

    for _ in range(max_iter):
        improved = False
        for i in range(n - 2):
            for j in range(i + 2, n):
                cost_old = edge_eucl(order, i)
                if j < n - 1:
                    cost_old += edge_eucl(order, j)

                reversed_sub = [(idx, not rev) for idx, rev in reversed(order[i + 1: j + 1])]
                new_middle = reversed_sub

                idx_i, rev_i = order[i]
                idx_a, rev_a = new_middle[0]
                idx_z, rev_z = new_middle[-1]

                a_end = self._sweep_endpoint(sweeps[idx_i]["path"], rev_i)
                a_start = self._sweep_startpoint(sweeps[idx_a]["path"], rev_a)
                cost_new = eucl(a_end, a_start)

                if j < n - 1:
                    idx_next, rev_next = order[j + 1]
                    z_end = self._sweep_endpoint(sweeps[idx_z]["path"], rev_z)
                    next_start = self._sweep_startpoint(sweeps[idx_next]["path"], rev_next)
                    cost_new += eucl(z_end, next_start)

                if cost_new < cost_old - 1e-9:
                    order = order[:i + 1] + new_middle + order[j + 1:]
                    improved = True

        if not improved:
            break

    return order
```
