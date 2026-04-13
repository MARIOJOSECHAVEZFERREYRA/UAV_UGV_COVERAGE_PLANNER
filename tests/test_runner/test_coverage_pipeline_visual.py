"""
Coverage pipeline visualizer — UN solo panel con todas las capas solapadas.

Capas (acumulativas, todas visibles al mismo tiempo):
  - Campo original (outline azul)
  - Polígono seguro post-margin (outline rojo)
  - Celdas de descomposición (fills coloreados transparentes)
  - Rutas boustrophedon (líneas sobre celdas)

Sliders: Margin / Heading / Swath — cualquier cambio redibuja todo.

Botones de visibilidad: toggle individual de cada capa.

Usage:
    python3 test_coverage_pipeline_visual.py [field.json] [--angle DEG] [--swath M] [--margin M]
"""
import sys
import os
import argparse

import matplotlib.pyplot as plt
from matplotlib.widgets import Button

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, '..', 'backend'))

from algorithms.coverage.decomposition import ConcaveDecomposer
from algorithms.coverage.path_planner import BoustrophedonPlanner
from algorithms.coverage.margin import MarginReducer
from visual_base import (
    BG, PANEL_BG, CELLS, load_json, setup_dark_ax, make_slider,
    fix_axes_to_bounds, resolve_json_path,
)

C_RAW  = '#4A90D9'
C_SAFE = '#E74C3C'


class CoveragePipelineVisualizer:
    def __init__(self, polygon, name, heading_deg=0.0, swath=5.0, margin=2.5):
        self.raw_polygon = polygon
        self.name = name

        # visibility toggles
        self.show = {'margin': True, 'decomp': True, 'paths': True}

        self.fig = plt.figure(figsize=(14, 9), facecolor=PANEL_BG)
        self.fig.suptitle(f'Coverage Pipeline  —  {name}',
                          color='white', fontsize=12, fontweight='bold')

        # Main axes (single panel)
        self.ax = self.fig.add_axes([0.03, 0.22, 0.72, 0.73])
        setup_dark_ax(self.ax)
        self.ax.set_aspect('equal')

        # Info panel
        self.ax_info = self.fig.add_axes([0.77, 0.22, 0.21, 0.73])
        self.ax_info.set_facecolor(BG)
        self.ax_info.set_xticks([])
        self.ax_info.set_yticks([])

        # Sliders
        self.sl_margin = make_slider(
            self.fig, [0.06, 0.14, 0.62, 0.025],
            'Margin', 0.0, 15.0, margin, 0.5, C_SAFE)
        self.sl_angle = make_slider(
            self.fig, [0.06, 0.10, 0.62, 0.025],
            'Heading', 0, 175, heading_deg, 5, C_RAW)
        self.sl_swath = make_slider(
            self.fig, [0.06, 0.06, 0.62, 0.025],
            'Swath', 1.0, 20.0, swath, 0.5, '#27AE60')

        for sl in (self.sl_margin, self.sl_angle, self.sl_swath):
            sl.on_changed(self._on_param)

        # Toggle buttons
        btn_specs = [
            ('Margin',  [0.77, 0.14, 0.07, 0.03], 'margin',  C_SAFE),
            ('Decomp',  [0.85, 0.14, 0.07, 0.03], 'decomp',  '#8E44AD'),
            ('Paths',   [0.77, 0.10, 0.07, 0.03], 'paths',   '#27AE60'),
        ]
        self.btns = {}
        for label, rect, key, color in btn_specs:
            ax_b = self.fig.add_axes(rect)
            btn = Button(ax_b, label, color=color, hovercolor='#ffffff')
            btn.label.set_color('white')
            btn.label.set_fontsize(8)
            btn.on_clicked(lambda _, k=key: self._toggle(k))
            self.btns[key] = btn

        self._draw()
        plt.show()

    def _on_param(self, _=None):
        self._draw()

    def _toggle(self, key):
        self.show[key] = not self.show[key]
        btn = self.btns[key]
        btn.ax.set_alpha(1.0 if self.show[key] else 0.3)
        self._draw()

    def _params(self):
        return self.sl_margin.val, self.sl_angle.val, self.sl_swath.val

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

        # ── Layer 0: raw field (always visible) ─────────────────────────
        rx, ry = self.raw_polygon.exterior.xy
        ax.fill(rx, ry, fc=C_RAW, alpha=0.08)
        ax.plot(rx, ry, color=C_RAW, lw=2.0, alpha=0.7, label='Raw field')
        for interior in self.raw_polygon.interiors:
            ix, iy = interior.xy
            ax.fill(ix, iy, fc='white', alpha=1.0, ec='#555', lw=1.5)

        # ── Layer 1: margin ──────────────────────────────────────────────
        safe_poly = None
        try:
            safe_poly = MarginReducer.shrink(self.raw_polygon, margin)
            if safe_poly.is_empty or safe_poly.area < 1:
                ax.set_title('Margin too large — polygon collapsed',
                             color='red', fontsize=9)
                fix_axes_to_bounds(ax, self.raw_polygon)
                self.fig.canvas.draw_idle()
                return
        except Exception as e:
            ax.set_title(f'Margin error: {e}', color='red', fontsize=9)
            fix_axes_to_bounds(ax, self.raw_polygon)
            self.fig.canvas.draw_idle()
            return

        if self.show['margin']:
            sx, sy = safe_poly.exterior.xy
            ax.fill(sx, sy, fc=C_SAFE, alpha=0.10)
            ax.plot(sx, sy, color=C_SAFE, lw=2.0, alpha=0.8, label='Safe polygon')
            for interior in safe_poly.interiors:
                ix, iy = interior.xy
                ax.fill(ix, iy, fc='#F39C12', alpha=0.25, ec='#F39C12', lw=1.5)

        # ── Layer 2: decomposition ────────────────────────────────────────
        cells = []
        try:
            cells = ConcaveDecomposer.decompose(safe_poly, angle, min_swath=swath)
        except Exception as e:
            ax.set_title(f'Decomp error: {e}', color='red', fontsize=9)

        if self.show['decomp']:
            for i, cell in enumerate(cells):
                c = CELLS[i % len(CELLS)]
                cx, cy = cell.exterior.xy
                ax.fill(cx, cy, fc=c, alpha=0.20)
                ax.plot(cx, cy, color=c, lw=1.2, alpha=0.6)

        # ── Layer 3: paths ────────────────────────────────────────────────
        planner = BoustrophedonPlanner(spray_width=swath)
        total_dist = 0.0
        total_turns = 0

        for i, cell in enumerate(cells):
            c = CELLS[i % len(CELLS)]
            result = planner.generate_path(cell, angle)
            segs = result.get('sweep_segments', [])
            metrics = result.get('metrics', {})
            total_dist  += metrics.get('spray_distance_m', 0.0)
            total_turns += metrics.get('turn_count', 0)

            if self.show['paths']:
                for seg in segs:
                    pts = seg.get('path', [])
                    if len(pts) >= 2:
                        xs, ys = zip(*pts)
                        ax.plot(xs, ys, color=c, lw=1.3, alpha=0.95)

        fix_axes_to_bounds(ax, self.raw_polygon)

        area_loss = (1 - safe_poly.area / self.raw_polygon.area) * 100
        ax.set_title(
            f'Margin {margin:.1f}m  |  {angle:.0f}°  |  Swath {swath:.1f}m  |  '
            f'{len(cells)} cells  |  {total_dist:.0f}m  |  {total_turns} turns',
            color='white', fontsize=9, pad=5)

        # Info panel
        info = [
            f'Field: {self.name}',
            '',
            f'Margin : {margin:.1f} m',
            f'Heading: {angle:.0f} deg',
            f'Swath  : {swath:.1f} m',
            '',
            f'Raw area  : {self.raw_polygon.area:.0f} m2',
            f'Safe area : {safe_poly.area:.0f} m2',
            f'Area loss : {area_loss:.1f}%',
            f'Obstacles : {len(list(self.raw_polygon.interiors))}',
            '',
            f'Cells     : {len(cells)}',
            f'Distance  : {total_dist:.0f} m',
            f'Turns     : {total_turns}',
            '',
            'Layers (toggle buttons):',
            f'  Margin : {"ON" if self.show["margin"] else "off"}',
            f'  Decomp : {"ON" if self.show["decomp"] else "off"}',
            f'  Paths  : {"ON" if self.show["paths"] else "off"}',
        ]
        self.ax_info.text(0.05, 0.97, '\n'.join(info),
                          color='#bdc3c7', fontsize=8.5, va='top',
                          family='monospace', transform=self.ax_info.transAxes)

        self.fig.canvas.draw_idle()


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('json',     nargs='?',       help='Path to field JSON')
    parser.add_argument('--angle',  type=float, default=0.0)
    parser.add_argument('--swath',  type=float, default=5.0)
    parser.add_argument('--margin', type=float, default=2.5)
    args = parser.parse_args()

    json_path = resolve_json_path(args.json)
    poly, name = load_json(json_path)
    print(f'Field: {name}  |  {poly.area:.0f} m2  |  {len(list(poly.interiors))} obstacles')

    CoveragePipelineVisualizer(poly, name,
                               heading_deg=args.angle,
                               swath=args.swath,
                               margin=args.margin)


if __name__ == '__main__':
    main()
