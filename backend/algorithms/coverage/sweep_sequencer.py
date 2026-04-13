"""Sweep ordering as a two-level chained TSP.

Sweeps come from the coverage decomposer grouped by cell: each convex
cell already has its own spatially coherent boustrophedon sequence. The
sequencer preserves that structure by treating each cell as an atomic
unit — it picks the order of cells and whether each cell should be
traversed forward or reversed, but never interleaves sweeps from
different cells.

This is the key to topologically coherent mission cycles. The downstream
MissionSegmenter slices the tour into cycles by energy/liquid budget;
if the tour stays within one cell at a time, each cycle falls inside a
contiguous region of the field instead of jumping between scattered
corners.

Two modes:
    'fast'  — euclidean distance; used in the GA hot path where each
              evaluation must be cheap.
    'full'  — geodesic distance via GeodesicSolver; used once on the
              winning solution to emit visually correct ferries.

If the input sweeps don't carry a 'cell_id' key the sequencer falls back
to flat NN+2opt over individual sweeps (legacy behavior).
"""

import math


_MAX_2OPT_ITERS = 20
_POINT_TOL = 1e-9


def _eu(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


class SweepSequencer:
    def __init__(self, solver, mode='full', base_point=None, cell_adjacency=None):
        if mode not in ('fast', 'full'):
            raise ValueError("mode must be 'fast' or 'full'")
        self._solver = solver
        self._mode = mode
        self._base = (
            (float(base_point[0]), float(base_point[1]))
            if base_point is not None else None
        )
        # Optional cell-id → set(cell-ids) adjacency graph. When supplied,
        # the cell-level NN walks this graph to guarantee that consecutive
        # cells in the tour share a physical boundary — producing mission
        # cycles that can never jump between disconnected regions.
        self._cell_adjacency = cell_adjacency

    # ------------------------------------------------------------------

    def _dist(self, a, b):
        if self._mode == 'fast':
            return _eu(a, b)
        _, d = self._solver.shortest_path(a, b)
        return d

    @staticmethod
    def _clone(sw):
        return {**sw, 'path': list(sw['path'])}

    @staticmethod
    def _flip_sweep(sw):
        """Return a new sweep dict with its path reversed."""
        return {**sw, 'path': list(reversed(sw['path']))}

    # ------------------------------------------------------------------

    def sequence(self, sweeps):
        """Return a flat list of sweep dicts, reordered and possibly flipped.

        If every sweep has a 'cell_id' key the sequencer operates at the
        cell level (cells are atomic, internal boustrophedon order is
        preserved). Otherwise it falls back to per-sweep NN+2opt.
        """
        if not sweeps:
            return []

        cloned = [self._clone(s) for s in sweeps
                  if s.get('path') and len(s['path']) >= 2]
        if len(cloned) <= 1:
            return cloned

        has_all_ids = all('cell_id' in s for s in cloned)
        if has_all_ids:
            cells, cell_ids = self._group_by_cell(cloned)
            if len(cells) >= 2:
                return self._sequence_cells(cells, cell_ids)

        # Fallback: flat per-sweep ordering (legacy behavior).
        ordered = self._nearest_neighbor_flat(cloned)
        ordered = self._two_opt_flat(ordered)
        return ordered

    # ------------------------------------------------------------------
    # Cell grouping
    # ------------------------------------------------------------------

    @staticmethod
    def _group_by_cell(cloned_sweeps):
        """Preserve input order within each cell while bucketing by cell_id.

        Returns (cells, cell_ids) where cells[i] is the list of sweeps
        belonging to cell with id cell_ids[i]. Cells are emitted in the
        insertion order encountered in the input.
        """
        buckets = {}
        order = []  # insertion order of cell_ids
        for sw in cloned_sweeps:
            cid = sw['cell_id']
            if cid not in buckets:
                buckets[cid] = []
                order.append(cid)
            buckets[cid].append(sw)
        cells = [buckets[cid] for cid in order]
        return cells, order

    # ------------------------------------------------------------------
    # Cell-level sequencing
    # ------------------------------------------------------------------

    def _cell_endpoints(self, cell, reversed_state):
        """Return the (start, end) points of a cell in a given state."""
        if reversed_state:
            # Traverse reversed: entry = last sweep's last point (which
            # becomes the first point after flipping), exit = first
            # sweep's first point (becomes its last).
            start = cell[-1]['path'][-1]
            end = cell[0]['path'][0]
        else:
            start = cell[0]['path'][0]
            end = cell[-1]['path'][-1]
        return start, end

    def _flatten_cell(self, cell, reversed_state):
        """Emit a cell's sweeps in the chosen state, flipping as needed."""
        if not reversed_state:
            return [self._clone(s) for s in cell]
        out = []
        for sw in reversed(cell):
            out.append(self._flip_sweep(sw))
        return out

    def _sequence_cells(self, cells, cell_ids):
        """Two-level sequencer: order cells, then emit flat sweeps.

        Phase 1: pick starting cell (nearest to base if available, else
                 the cell that was first in the input).
        Phase 2: greedy NN over remaining cells — at each step choose
                 the cell+state whose entry point is closest to the
                 current cursor. When a cell adjacency graph is provided,
                 restrict candidates to cells that share a boundary with
                 the current cell (topological constraint). Fall back to
                 global nearest only when all adjacent cells are visited.
        Phase 3: 2-opt over the cell sequence to refine.
        """
        n = len(cells)

        # Phase 1+2 — base-anchored NN over cells
        remaining = list(range(n))
        states = [False] * n  # reversed?
        order = []

        if self._base is not None:
            cursor = self._base
            cur_cell_id = None
        else:
            ordered_first = 0
            cursor = cells[ordered_first][-1]['path'][-1]
            order.append(ordered_first)
            states[ordered_first] = False
            remaining.remove(ordered_first)
            cur_cell_id = cell_ids[ordered_first]

        while remaining:
            # If we have adjacency info, restrict candidates to cells that
            # are physically adjacent to the current one. Otherwise, or if
            # no adjacent candidate remains, fall back to all remaining.
            candidates = remaining
            if (self._cell_adjacency is not None
                    and cur_cell_id is not None):
                neighbors = self._cell_adjacency.get(cur_cell_id, set())
                adj_candidates = [
                    idx for idx in remaining
                    if cell_ids[idx] in neighbors
                ]
                if adj_candidates:
                    candidates = adj_candidates

            best_i = candidates[0]
            best_d = math.inf
            best_rev = False
            for idx in candidates:
                cell = cells[idx]
                nat_start, _ = self._cell_endpoints(cell, False)
                rev_start, _ = self._cell_endpoints(cell, True)
                d_nat = self._dist(cursor, nat_start)
                d_rev = self._dist(cursor, rev_start)
                if d_nat < best_d:
                    best_d, best_i, best_rev = d_nat, idx, False
                if d_rev < best_d:
                    best_d, best_i, best_rev = d_rev, idx, True

            remaining.remove(best_i)
            states[best_i] = best_rev
            order.append(best_i)
            _, exit_pt = self._cell_endpoints(cells[best_i], best_rev)
            cursor = exit_pt
            cur_cell_id = cell_ids[best_i]

        # Phase 3 — 2-opt refinement over the cell sequence
        order, states = self._two_opt_cells(cells, order, states, cell_ids)

        # Emit flat sweep list
        flat = []
        for idx in order:
            flat.extend(self._flatten_cell(cells[idx], states[idx]))
        return flat

    def _count_non_adjacent(self, order, cell_ids):
        """Number of consecutive cell pairs in `order` that are not adjacent.

        Returns 0 when no adjacency graph is configured (unconstrained).
        """
        if self._cell_adjacency is None:
            return 0
        n = len(order)
        count = 0
        for k in range(n - 1):
            a = cell_ids[order[k]]
            b = cell_ids[order[k + 1]]
            if b not in self._cell_adjacency.get(a, set()):
                count += 1
        return count

    def _two_opt_cells(self, cells, order, states, cell_ids):
        """Reverse a sub-range [i..j] of the cell sequence if it lowers cost.

        Reversing a sub-range of cells:
          - reverses the order of cells
          - inverts the state of each cell in the sub-range (a cell
            previously traversed natural is now reversed, and vice versa)
        Tour cost includes base→first_cell entry and last_cell_exit→base
        when a base_point is provided.

        Adjacency-preserving: when a cell adjacency graph is provided, a
        swap is rejected if it introduces new non-adjacent transitions.
        This keeps the NN-produced topological walk intact.
        """
        n = len(order)
        if n < 3:
            return order, states

        baseline_non_adj = self._count_non_adjacent(order, cell_ids)

        # Pin position 0 for the same reason as _two_opt_flat.

        def entry(idx):
            return self._cell_endpoints(cells[order[idx]], states[order[idx]])[0]

        def exit_pt(idx):
            return self._cell_endpoints(cells[order[idx]], states[order[idx]])[1]

        def full_cost():
            total = 0.0
            if has_base:
                total += self._dist(self._base, entry(0))
            for k in range(n - 1):
                total += self._dist(exit_pt(k), entry(k + 1))
            if has_base:
                total += self._dist(exit_pt(n - 1), self._base)
            return total

        for _ in range(_MAX_2OPT_ITERS):
            improved = False
            for i in range(1, n - 1):
                for j in range(i + 1, n):
                    # Propose reversing order[i..j] and flipping each state.
                    new_order = order[:i] + list(reversed(order[i:j + 1])) + order[j + 1:]
                    new_states = list(states)
                    for k in range(i, j + 1):
                        new_states[new_order[k]] = not states[new_order[k]]

                    # Evaluate only the two edges that change: left (before i)
                    # and right (after j). All other edges are identical.
                    def _entry_of(pos, seq_order, seq_states):
                        cell = cells[seq_order[pos]]
                        return self._cell_endpoints(cell, seq_states[seq_order[pos]])[0]

                    def _exit_of(pos, seq_order, seq_states):
                        cell = cells[seq_order[pos]]
                        return self._cell_endpoints(cell, seq_states[seq_order[pos]])[1]

                    # Virtual base edges intentionally excluded — see rationale
                    # in _two_opt_flat. NN already anchors the tour at base.
                    # Position 0 is pinned (loop starts at i=1), so i-1 is valid.
                    left_anchor = _exit_of(i - 1, order, states)
                    right_anchor = _entry_of(j + 1, order, states) if j + 1 < n else None

                    left_old = (self._dist(left_anchor, _entry_of(i, order, states))
                                if left_anchor is not None else 0.0)
                    left_new = (self._dist(left_anchor, _entry_of(i, new_order, new_states))
                                if left_anchor is not None else 0.0)

                    right_old = (self._dist(_exit_of(j, order, states), right_anchor)
                                 if right_anchor is not None else 0.0)
                    right_new = (self._dist(_exit_of(j, new_order, new_states), right_anchor)
                                 if right_anchor is not None else 0.0)

                    delta = (left_new + right_new) - (left_old + right_old)
                    if delta < -_POINT_TOL:
                        # Adjacency guard: reject swaps that add jumps.
                        new_non_adj = self._count_non_adjacent(new_order, cell_ids)
                        if new_non_adj > baseline_non_adj:
                            continue
                        order = new_order
                        states = new_states
                        baseline_non_adj = new_non_adj
                        improved = True
                        break
                if improved:
                    break
            if not improved:
                break

        return order, states

    # ------------------------------------------------------------------
    # Legacy flat per-sweep NN+2opt (fallback when cell_id is missing)
    # ------------------------------------------------------------------

    def _nearest_neighbor_flat(self, sweeps):
        remaining = list(sweeps)

        if self._base is not None:
            best_i = 0
            best_d = math.inf
            best_flip = False
            for i, sw in enumerate(remaining):
                d0 = self._dist(self._base, sw['path'][0])
                d1 = self._dist(self._base, sw['path'][-1])
                if d0 < best_d:
                    best_d, best_i, best_flip = d0, i, False
                if d1 < best_d:
                    best_d, best_i, best_flip = d1, i, True
            first = remaining.pop(best_i)
            if best_flip:
                first['path'] = list(reversed(first['path']))
            ordered = [first]
        else:
            ordered = [remaining.pop(0)]

        while remaining:
            cur_end = ordered[-1]['path'][-1]
            best_i = 0
            best_d = math.inf
            best_flip = False
            for i, cand in enumerate(remaining):
                d_nat = self._dist(cur_end, cand['path'][0])
                d_flip = self._dist(cur_end, cand['path'][-1])
                if d_nat < best_d:
                    best_d, best_i, best_flip = d_nat, i, False
                if d_flip < best_d:
                    best_d, best_i, best_flip = d_flip, i, True
            nxt = remaining.pop(best_i)
            if best_flip:
                nxt['path'] = list(reversed(nxt['path']))
            ordered.append(nxt)

        return ordered

    def _two_opt_flat(self, ordered):
        """Minimize total inter-sweep ferry distance.

        Note: we deliberately DO NOT include the virtual base→first and
        last→base edges in the delta calculation. Including them biases
        2-opt toward "tucking" a base-close sweep at either end of the
        tour, creating huge internal jumps (e.g. 328m) to save a few
        metres of closing deadhead. NN's base-anchored start already
        places the closest sweep at position 0; 2-opt refines internal
        ordering without touching the virtual base edges.
        """
        n = len(ordered)
        if n < 3:
            return ordered

        # Pin position 0: the NN places the sweep closest to the base at
        # position 0 on purpose. 2-opt must not displace this anchor by
        # reversing any sub-range starting at 0, so the outer loop starts
        # at i=1.
        for _ in range(_MAX_2OPT_ITERS):
            improved = False
            for i in range(1, n - 1):
                for j in range(i + 1, n):
                    a_end = ordered[i - 1]['path'][-1]
                    d_start = ordered[j + 1]['path'][0] if j + 1 < n else None

                    b_start = ordered[i]['path'][0]
                    c_end = ordered[j]['path'][-1]

                    left_old = self._dist(a_end, b_start) if a_end is not None else 0.0
                    left_new = self._dist(a_end, c_end) if a_end is not None else 0.0
                    right_old = self._dist(c_end, d_start) if d_start is not None else 0.0
                    right_new = self._dist(b_start, d_start) if d_start is not None else 0.0

                    delta = (left_new + right_new) - (left_old + right_old)
                    if delta < -_POINT_TOL:
                        ordered[i:j + 1] = list(reversed(ordered[i:j + 1]))
                        for k in range(i, j + 1):
                            ordered[k]['path'] = list(reversed(ordered[k]['path']))
                        improved = True
                        break
                if improved:
                    break
            if not improved:
                break

        return ordered
