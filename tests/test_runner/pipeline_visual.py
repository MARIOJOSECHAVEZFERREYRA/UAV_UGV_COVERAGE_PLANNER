"""
Coverage pipeline visualizer — pipeline completo en un solo panel.

Capas acumulativas (toggle por botón):
  Geometría : Raw field | Safe bound | Obstacles | Cell fills | Cell edges | Cell labels
  Routing   : Sweep paths | Ferries
  Misión    : Cycles (color por ciclo) | Deadheads | Base point

Sliders    : Margin (m) | Heading (°) | Swath (m)
Interactivo: Click fuera del polígono → colocar base point

Usage:
    python3 pipeline_visual.py [field.json] [--angle DEG] [--swath M] [--margin M] [--base X,Y]
"""
import sys
import os
import argparse

import matplotlib.pyplot as plt
from matplotlib.widgets import Button, TextBox

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, '..', 'backend'))

from algorithms.coverage.decomposition import ConcaveDecomposer
from algorithms.coverage.path_planner import BoustrophedonPlanner
from algorithms.coverage.margin import MarginReducer
from algorithms.coverage.path_assembler import PathAssembler
from algorithms.energy.segmentation import MissionSegmenter
from algorithms.energy.energy_model import DroneEnergyModel
from shapely import affinity
from shapely.geometry import Point
from visual_base import (
    BG, PANEL_BG, CELLS, load_json, setup_dark_ax, make_slider,
    fix_axes_to_bounds, resolve_json_path,
)

# Dependency map — each layer declares which pipeline artifacts it needs.
# Mirrors the real MissionPlanner pipeline order:
#   safe_poly -> cells -> sweeps -> route_segments -> cycles/deadheads
LAYER_DEPS = {
    'raw':       set(),
    'safe':      {'safe_poly'},
    'obstacles': {'safe_poly'},
    'cell_fill': {'cells'},
    'cell_edge': {'cells'},
    'cell_lbl':  {'cells'},
    'paths':     {'route_segments'},
    'ferries':   {'route_segments'},
    'cycles':    {'route_segments', 'base_point'},
    'deadheads': {'route_segments', 'base_point'},
    'base_pt':   set(),
}

DEP_LABELS = {
    'safe_poly':      'margin valid',
    'cells':          'decomposition',
    'route_segments': 'routing',
    'base_point':     'base point',
}

# View chain — a layer also requires its upstream layer to be currently
# visible. Matches the conceptual pipeline order: a ferry has no meaning
# without the sweeps it connects, a cycle groups ferries + sweeps, and a
# deadhead closes a cycle.
VIEW_DEPS = {
    'ferries':   'paths',
    'cycles':    'ferries',
    'deadheads': 'cycles',
}

# Human-readable labels for view-chain messages.
VIEW_LABELS = {
    'paths':     'Sweeps',
    'ferries':   'Ferries',
    'cycles':    'Cycles',
    'deadheads': 'Deadheads',
}


def _get_drone(drone_name: str = "DJI Agras T30"):
    """Load drone directly from SQLite file — no SQLAlchemy needed."""
    import sqlite3
    from types import SimpleNamespace
    db_path = os.path.join(ROOT, '..', 'agriswarm.db')
    conn = sqlite3.connect(db_path)
    conn.row_factory = sqlite3.Row
    try:
        row = conn.execute("SELECT * FROM drones WHERE name = ?",
                           (drone_name,)).fetchone()
    finally:
        conn.close()
    if row is None:
        raise ValueError(f"Drone '{drone_name}' not found in agriswarm.db")
    return SimpleNamespace(**dict(row))

C_RAW      = '#4A90D9'
C_SAFE     = '#E74C3C'
C_FERRY    = '#ffffff'
C_DEADHEAD = '#E91E63'
C_BASE     = '#FFD700'

# Button state palette — uses background color, not alpha, so unavailable
# buttons don't look washed out.
BTN_BG_OFF         = '#2a2a2a'
BTN_BG_UNAVAILABLE = '#141414'
BTN_LBL_UNAVAILABLE = '#555555'
BTN_LBL_OFF_BRIGHT  = '#cfcfcf'

# Cycle colors (distinct from cell colors)
CYCLE_COLORS = ['#FF6B6B', '#4ECDC4', '#45B7D1', '#96CEB4',
                '#FFEAA7', '#DDA0DD', '#98FB98', '#FFA07A']


class PipelineVisualizer:

    TOGGLES = [
        # Row 1 — geometry
        ('raw',       'Raw field',   True,  C_RAW),
        ('safe',      'Safe bound',  True,  C_SAFE),
        ('obstacles', 'Obstacles',   True,  '#F39C12'),
        ('cell_fill', 'Cell fills',  True,  '#8E44AD'),
        # Row 2 — geometry cont.
        ('cell_edge', 'Cell edges',  True,  '#16A085'),
        ('cell_lbl',  'Cell labels', False, '#7f8c8d'),
        # Row 2/3 — routing + mission (labels prefixed "Show" to clarify
        # these are view filters on already-generated data, not pipeline steps).
        ('paths',     'Show sweeps',    True,  '#27AE60'),
        ('ferries',   'Show ferries',   True,  '#ffffff'),
        ('cycles',    'Show cycles',    False, '#FF6B6B'),
        ('deadheads', 'Show deadheads', False, C_DEADHEAD),
        ('base_pt',   'Base point',     True,  C_BASE),
    ]

    def __init__(self, polygon, name, drone, heading_deg=0.0, swath=5.0, margin=2.5,
                 base_point=None):
        self.raw_polygon = polygon
        self.drone = drone
        self.name = name
        self.show = {key: default for key, _, default, _ in self.TOGGLES}
        self.base_point = base_point  # (x, y) or None

        # Pipeline artifact availability, updated on each _draw().
        # Mirrors the dependency chain in MissionPlanner.run_mission_planning:
        #   safe_poly -> cells -> sweeps -> route_segments -> cycles/deadheads
        self._artifacts = {
            'safe_poly':      False,
            'cells':          False,
            'route_segments': False,
            'base_point':     base_point is not None,
        }
        self._status_msg = ''  # transient feedback line shown in info panel

        # ── Figure ────────────────────────────────────────────────────── #
        self.fig = plt.figure(figsize=(15, 9), facecolor=PANEL_BG)
        self.fig.suptitle(f'Coverage Pipeline  —  {name}',
                          color='white', fontsize=11, fontweight='bold')

        # Map axes
        self.ax = self.fig.add_axes([0.03, 0.30, 0.69, 0.66])
        setup_dark_ax(self.ax)
        self.ax.set_aspect('equal')

        # Info panel
        self.ax_info = self.fig.add_axes([0.74, 0.30, 0.24, 0.66])
        self.ax_info.set_facecolor(BG)
        self.ax_info.set_xticks([])
        self.ax_info.set_yticks([])

        # ── Sliders ───────────────────────────────────────────────────── #
        self.sl_margin = make_slider(
            self.fig, [0.06, 0.23, 0.60, 0.022],
            'Margin (m)', 0.0, 15.0, margin, 0.5, C_SAFE)
        # Heading slider is coarse (step 5°); the text box beside it lets
        # the user enter any integer angle for finer control.
        self.sl_angle = make_slider(
            self.fig, [0.06, 0.20, 0.53, 0.022],
            'Heading (°)', 0, 175, heading_deg, 5, C_RAW)
        self.sl_swath = make_slider(
            self.fig, [0.06, 0.17, 0.60, 0.022],
            'Swath (m)', 1.0, 20.0, swath, 0.5, '#27AE60')

        for sl in (self.sl_margin, self.sl_angle, self.sl_swath):
            sl.on_changed(self._on_param)

        # Heading text box — any integer 0..179. Kept in sync with slider.
        ax_tb = self.fig.add_axes([0.605, 0.20, 0.055, 0.022])
        ax_tb.set_facecolor('#2d2d44')
        self.tb_angle = TextBox(
            ax_tb, '', initial=str(int(heading_deg)),
            color='#2d2d44', hovercolor='#3d3d55',
        )
        self.tb_angle.text_disp.set_color('white')
        self.tb_angle.on_submit(self._on_angle_text)

        # ── Toggle buttons (3 rows × 4) ───────────────────────────────── #
        bw, bh, gx, gy = 0.155, 0.027, 0.006, 0.004
        x0 = 0.06
        y_rows = [0.128, 0.097, 0.066]

        self.btn_axes = {}
        self._btn_colors = {}  # key -> layer accent color
        self._btn_labels = {}  # key -> base label text (without state prefix)
        for idx, (key, label, default, color) in enumerate(self.TOGGLES):
            row, col = divmod(idx, 4)
            x = x0 + col * (bw + gx)
            y = y_rows[row]
            ax_b = self.fig.add_axes([x, y, bw, bh])
            btn = Button(ax_b, label, color=BTN_BG_OFF, hovercolor=color)
            btn.label.set_fontsize(7.5)
            btn.label.set_fontweight('bold')
            btn.on_clicked(lambda _, k=key: self._toggle(k))
            # Thin colored outline so the layer color is still identifiable
            # when the button is in the OFF state (dark bg).
            for spine in ax_b.spines.values():
                spine.set_edgecolor(color)
                spine.set_linewidth(1.2)
            self.btn_axes[key] = btn
            self._btn_colors[key] = color
            self._btn_labels[key] = label

        # Click on map → place base point
        self.fig.canvas.mpl_connect('button_press_event', self._on_click)

        self._draw()
        plt.show()

    # ------------------------------------------------------------------ #

    def _on_param(self, _=None):
        # Slider moved — sync the angle text box to match.
        if hasattr(self, 'tb_angle'):
            current = str(int(self.sl_angle.val))
            if self.tb_angle.text != current:
                # Suppress on_submit fire when we programmatically set text.
                self.tb_angle.eventson = False
                try:
                    self.tb_angle.set_val(current)
                finally:
                    self.tb_angle.eventson = True
        self._draw()

    def _on_angle_text(self, text):
        """Set heading to any integer 0..179 from the text box."""
        try:
            angle = int(float(text)) % 180
        except (ValueError, TypeError):
            return
        # Snap slider to nearest multiple of 5 (its valstep) without
        # triggering _on_param, then set the real angle and redraw.
        snap = (angle // 5) * 5
        self.sl_angle.eventson = False
        try:
            self.sl_angle.set_val(snap)
            # Override the slider value with the exact integer so _draw
            # uses the user-typed angle, not the snapped one.
            self.sl_angle.val = float(angle)
            self.sl_angle.valtext.set_text(f'{angle}')
        finally:
            self.sl_angle.eventson = True
        self._draw()

    def _missing_deps(self, key):
        """Return list of missing dependency labels for a layer.

        Combines two kinds of gating:
        - artifact deps: upstream pipeline data must exist
        - view deps:     upstream layer in the conceptual chain must be shown
        """
        missing = [DEP_LABELS[d] for d in LAYER_DEPS.get(key, set())
                   if not self._artifacts.get(d, False)]
        parent = VIEW_DEPS.get(key)
        if parent is not None and not self.show.get(parent, False):
            missing.append(f'{VIEW_LABELS[parent]} visible')
        return missing

    def _toggle(self, key):
        # Block activation when an upstream dependency is missing (artifact
        # or view-chain parent). Turning a layer OFF is always allowed and
        # cascades to any child layers that require it to be visible.
        if not self.show[key]:
            missing = self._missing_deps(key)
            if missing:
                self._status_msg = f"'{key}' needs: {', '.join(missing)}"
                self._draw()
                return
            self.show[key] = True
        else:
            self.show[key] = False
            # Transitive cascade OFF via the view chain: any layer whose
            # view-dep parent is now hidden must also hide.
            stack = [key]
            while stack:
                turned_off = stack.pop()
                for child, parent in VIEW_DEPS.items():
                    if parent == turned_off and self.show.get(child):
                        self.show[child] = False
                        stack.append(child)
        self._status_msg = ''
        self._draw()

    def _on_click(self, event):
        if event.inaxes is not self.ax:
            return
        if event.button != 1:
            return
        # Match MissionPlanner._validate_base_point: reject clicks inside
        # the safe spray area. Need a safe_poly snapshot to check.
        candidate = (event.xdata, event.ydata)
        try:
            safe = MarginReducer.shrink(self.raw_polygon, self.sl_margin.val)
            if not safe.is_empty and safe.contains(Point(candidate)):
                self._status_msg = 'base_point must be outside safe area'
                self._draw()
                return
        except Exception:
            pass
        self.base_point = candidate
        self._artifacts['base_point'] = True
        self._status_msg = ''
        self._draw()

    def _params(self):
        return self.sl_margin.val, self.sl_angle.val, self.sl_swath.val

    def _refresh_button_alphas(self):
        """Reflect layer state on buttons: active / off / unavailable.

        Uses background color + label color instead of alpha so disabled
        buttons stay legible. Unavailable means the upstream pipeline stage
        isn't ready — matches dependency chain in run_mission_planning.
        """
        for key, _, _, _ in self.TOGGLES:
            btn = self.btn_axes[key]
            color = self._btn_colors[key]
            label = self._btn_labels[key]
            missing = self._missing_deps(key)
            if missing:
                btn.ax.set_facecolor(BTN_BG_UNAVAILABLE)
                btn.color = BTN_BG_UNAVAILABLE
                btn.label.set_text(f'— {label}')
                btn.label.set_color(BTN_LBL_UNAVAILABLE)
                for spine in btn.ax.spines.values():
                    spine.set_edgecolor(BTN_LBL_UNAVAILABLE)
            elif self.show[key]:
                btn.ax.set_facecolor(color)
                btn.color = color
                btn.label.set_text(f'● {label}')
                btn.label.set_color('white')
                for spine in btn.ax.spines.values():
                    spine.set_edgecolor(color)
            else:
                btn.ax.set_facecolor(BTN_BG_OFF)
                btn.color = BTN_BG_OFF
                btn.label.set_text(f'○ {label}')
                btn.label.set_color(BTN_LBL_OFF_BRIGHT)
                for spine in btn.ax.spines.values():
                    spine.set_edgecolor(color)

    # ------------------------------------------------------------------ #

    def _draw(self):
        margin, angle, swath = self._params()
        ax = self.ax
        ax.cla()
        ax.set_aspect('equal')
        ax.set_facecolor(BG)
        self.ax_info.cla()
        self.ax_info.set_facecolor(BG)
        self.ax_info.set_xticks([])
        self.ax_info.set_yticks([])

        # Reset artifact availability — rebuilt as each stage succeeds.
        self._artifacts = {
            'safe_poly':      False,
            'cells':          False,
            'route_segments': False,
            'base_point':     self.base_point is not None,
        }

        # ── Raw field ────────────────────────────────────────────────── #
        if self.show['raw']:
            rx, ry = self.raw_polygon.exterior.xy
            ax.fill(rx, ry, fc=C_RAW, alpha=0.08)
            ax.plot(rx, ry, color=C_RAW, lw=2.0, alpha=0.7)

        # ── Margin ───────────────────────────────────────────────────── #
        safe_poly = None
        try:
            safe_poly = MarginReducer.shrink(self.raw_polygon, margin)
            if safe_poly.is_empty or safe_poly.area < 1:
                ax.set_title('Margin too large', color='red', fontsize=9)
                fix_axes_to_bounds(ax, self.raw_polygon)
                self._refresh_button_alphas()
                self.fig.canvas.draw_idle()
                return
        except Exception as e:
            ax.set_title(f'Margin error: {e}', color='red', fontsize=9)
            fix_axes_to_bounds(ax, self.raw_polygon)
            self._refresh_button_alphas()
            self.fig.canvas.draw_idle()
            return

        self._artifacts['safe_poly'] = True

        if self.show['safe']:
            sx, sy = safe_poly.exterior.xy
            ax.fill(sx, sy, fc=C_SAFE, alpha=0.09)
            ax.plot(sx, sy, color=C_SAFE, lw=2.0, alpha=0.85)

        if self.show['obstacles']:
            for interior in self.raw_polygon.interiors:
                ix, iy = interior.xy
                ax.fill(ix, iy, fc='#F39C12', alpha=0.30, ec='#F39C12', lw=1.5)
            for interior in safe_poly.interiors:
                ix, iy = interior.xy
                ax.fill(ix, iy, fc='#E67E22', alpha=0.18, ec='#E67E22', lw=1.2, ls='--')

        # ── Decomposition ────────────────────────────────────────────── #
        cells = []
        try:
            cells = ConcaveDecomposer.decompose(safe_poly, angle,
                                                channel_width=swath,
                                                min_swath=swath)
        except Exception as e:
            ax.set_title(f'Decomp error: {e}', color='red', fontsize=9)

        if cells:
            self._artifacts['cells'] = True

        for i, cell in enumerate(cells):
            c = CELLS[i % len(CELLS)]
            cx, cy = cell.exterior.xy
            if self.show['cell_fill']:
                ax.fill(cx, cy, fc=c, alpha=0.22)
            if self.show['cell_edge']:
                ax.plot(cx, cy, color=c, lw=1.4, alpha=0.65)
            if self.show['cell_lbl']:
                ctr = cell.centroid
                ax.text(ctr.x, ctr.y, str(i), color='white', fontsize=7,
                        ha='center', va='center', fontweight='bold',
                        bbox=dict(fc='#000000', alpha=0.45, pad=1.5, lw=0))

        # ── Sweep paths + ferries via PathAssembler ───────────────────── #
        route_segments = []
        ferry_dist = 0.0
        sweep_dist = 0.0

        assembler = None
        if cells:
            try:
                rotation_origin = safe_poly.centroid
                rotated_whole = affinity.rotate(safe_poly, -angle, origin=rotation_origin)
                global_y_origin = rotated_whole.bounds[1]

                planner = BoustrophedonPlanner(spray_width=swath)
                all_sweep_segs = []
                for cell_id, cell in enumerate(cells):
                    result = planner.generate_path(cell, angle,
                                                   global_y_origin=global_y_origin,
                                                   rotation_origin=rotation_origin)
                    segs = result.get('sweep_segments', [])
                    for s in segs:
                        s['cell_id'] = cell_id
                    all_sweep_segs.extend(segs)

                assembler = PathAssembler(
                    cells,
                    original_polygon=safe_poly,
                    base_point=self.base_point,
                )
                asm = assembler.assemble_connected(all_sweep_segs)
                route_segments = asm.get('route_segments', [])
                if route_segments:
                    self._artifacts['route_segments'] = True
                dists = asm.get('distances', {})
                sweep_dist = dists.get('sweep_m', 0.0)
                ferry_dist = dists.get('ferry_m', 0.0)

                for seg in route_segments:
                    stype = seg.get('segment_type', '')
                    pts = seg.get('path', [])
                    if len(pts) < 2:
                        continue
                    xs, ys = zip(*pts)
                    if stype == 'sweep' and self.show['paths']:
                        ax.plot(xs, ys, color='#27AE60', lw=1.3, alpha=0.9)
                    elif stype == 'ferry' and self.show['ferries']:
                        ax.plot(xs, ys, color=C_FERRY, lw=1.0, alpha=0.55,
                                ls='--')

            except Exception as e:
                ax.set_title(f'Assembly error: {e}', color='red', fontsize=9)

        # ── Mission cycles + deadheads ────────────────────────────────── #
        n_cycles = 0
        total_deadhead = 0.0
        has_base = self.base_point is not None

        if route_segments and has_base and assembler is not None \
                and (self.show['cycles'] or self.show['deadheads']):
            try:
                energy_model = DroneEnergyModel(self.drone)
                segmenter = MissionSegmenter(
                    self.drone,
                    target_rate_l_ha=self.drone.app_rate_default_l_ha,
                    work_speed_kmh=self.drone.speed_cruise_ms * 3.6,
                    swath_width=swath,
                    energy_model=energy_model,
                )

                # Reuse the routing assembler — rebuilding the adjacency graph
                # just for the segmenter's distance function is wasteful.
                def _dist_fn(a, b):
                    _, d = assembler.find_connection(a, b)
                    return d

                raw_cycles = segmenter.segment_path(
                    route_segments, self.base_point, dist_fn=_dist_fn)
                n_cycles = len(raw_cycles)

                for ci, cyc in enumerate(raw_cycles):
                    work_segs = cyc.get('segments', [])
                    if not work_segs:
                        continue
                    start_pt = cyc['start_pt']
                    end_pt   = cyc['end_pt']
                    cc = CYCLE_COLORS[ci % len(CYCLE_COLORS)]

                    # Cycle coloring — only sweep atoms, so the Ferries layer
                    # stays visible when both 'cycles' and 'ferries' are on.
                    # Reconstruct the sweep polyline per contiguous sweep run
                    # to avoid anti-aliasing seams between adjacent atoms.
                    if self.show['cycles']:
                        run = []
                        def _flush(run):
                            if len(run) >= 2:
                                xs, ys = zip(*run)
                                ax.plot(xs, ys, color=cc, lw=2.0, alpha=0.9,
                                        solid_capstyle='round',
                                        solid_joinstyle='round')
                        for seg in work_segs:
                            if seg.get('segment_type') != 'sweep':
                                _flush(run)
                                run = []
                                continue
                            if not run:
                                run.append(seg['p1'])
                            run.append(seg['p2'])
                        _flush(run)

                    # Deadheads: base → cycle start, cycle end → base
                    if self.show['deadheads']:
                        open_path, d_open = assembler.find_connection(self.base_point, start_pt)
                        close_path, d_close = assembler.find_connection(end_pt, self.base_point)
                        total_deadhead += d_open + d_close

                        for path, style in ((open_path, '--'), (close_path, '-.')):
                            if len(path) >= 2:
                                xs, ys = zip(*path)
                                ax.plot(xs, ys, color=C_DEADHEAD, lw=1.5,
                                        alpha=0.85, ls=style)

            except Exception as e:
                # Surface the failure so the user knows why cycles/deadheads
                # are missing instead of silently showing nothing.
                self._status_msg = f'Segmentation failed: {type(e).__name__}: {e}'[:80]

        # ── Base point ───────────────────────────────────────────────── #
        if self.show['base_pt']:
            if has_base:
                ax.plot(*self.base_point, 'o', color=C_BASE, ms=10, zorder=9)
                ax.plot(*self.base_point, '+', color='black', ms=8,
                        mew=2, zorder=10)
            else:
                ax.text(0.01, 0.01, 'Click map to place base point',
                        transform=ax.transAxes, color=C_BASE,
                        fontsize=8, alpha=0.8)

        fix_axes_to_bounds(ax, self.raw_polygon)

        area_loss = (1 - safe_poly.area / self.raw_polygon.area) * 100
        ax.set_title(
            f'Margin {margin:.1f}m  |  {angle:.0f}°  |  Swath {swath:.1f}m  |  '
            f'{len(cells)} cells  |  sweep {sweep_dist:.0f}m  ferry {ferry_dist:.0f}m'
            + (f'  |  {n_cycles} cycles' if n_cycles else ''),
            color='white', fontsize=8.5, pad=5)

        # ── Info panel ────────────────────────────────────────────────── #
        info = [
            self.name[:30],
            '',
            f'Margin   : {margin:.1f} m',
            f'Heading  : {angle:.0f} deg',
            f'Swath    : {swath:.1f} m',
            '',
            f'Raw area : {self.raw_polygon.area:.0f} m2',
            f'Safe area: {safe_poly.area:.0f} m2',
            f'Area loss: {area_loss:.1f}%',
            f'Obstacles: {len(list(self.raw_polygon.interiors))}',
            '',
            f'Cells    : {len(cells)}',
            f'Sweeps   : {sweep_dist:.0f} m',
            f'Ferries  : {ferry_dist:.0f} m',
            f'Total    : {sweep_dist + ferry_dist:.0f} m',
        ]
        if n_cycles:
            info += [
                '',
                f'Cycles   : {n_cycles}',
                f'Deadheads: {total_deadhead:.0f} m',
                f'Drone: {self.drone.name}',
            ]
        if not has_base:
            info += ['', 'Click map to place', 'base point for', 'cycles/deadheads']

        self.ax_info.text(0.05, 0.97, '\n'.join(info),
                          color='#bdc3c7', fontsize=8, va='top',
                          family='monospace', transform=self.ax_info.transAxes)

        if self._status_msg:
            self.ax_info.text(0.05, 0.02, self._status_msg,
                              color='#E74C3C', fontsize=8, va='bottom',
                              family='monospace', wrap=True,
                              transform=self.ax_info.transAxes)

        self._refresh_button_alphas()
        self.fig.canvas.draw_idle()


# ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('json',     nargs='?',       help='Path to field JSON')
    parser.add_argument('--angle',  type=float, default=0.0)
    parser.add_argument('--swath',  type=float, default=5.0)
    parser.add_argument('--margin', type=float, default=2.5)
    parser.add_argument('--base',   type=str,   default=None,
                        help='Base point as "x,y" (e.g. --base 100,50)')
    parser.add_argument('--drone',  type=str,   default='DJI Agras T30',
                        help='Drone name in DB (default: "DJI Agras T30")')
    args = parser.parse_args()

    json_path = resolve_json_path(args.json)
    poly, name = load_json(json_path)

    drone = _get_drone(args.drone)
    print(f'Drone: {drone.name}')

    base_point = None
    if args.base:
        try:
            bx, by = map(float, args.base.split(','))
            base_point = (bx, by)
        except ValueError:
            print(f'Invalid --base format: {args.base}  (expected "x,y")')

    print(f'Field: {name}  |  {poly.area:.0f} m2  |  {len(list(poly.interiors))} obstacles')
    if base_point:
        print(f'Base : {base_point}')
    else:
        print('Base : click on map to place')

    PipelineVisualizer(poly, name, drone,
                       heading_deg=args.angle,
                       swath=args.swath,
                       margin=args.margin,
                       base_point=base_point)


if __name__ == '__main__':
    main()
