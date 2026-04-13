import sys
import os
import argparse

import matplotlib.pyplot as plt

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, '..', 'backend'))

from algorithms.coverage.margin import MarginReducer
from visual_base import (
    BG, PANEL_BG, load_json, setup_dark_ax, make_slider,
    fix_axes_to_bounds, resolve_json_path,
)

C_ORIGINAL = '#4A90D9'
C_SHRUNKEN = '#E74C3C'
C_OBSTACLE_ORIG = '#8E44AD'
C_OBSTACLE_EXPANDED = '#F39C12'


class MarginVisualizer:
    def __init__(self, polygon, name, margin=2.5):
        self.raw_polygon = polygon
        self.name = name
        self.margin = margin

        self.fig = plt.figure(figsize=(15, 8), facecolor=PANEL_BG)
        self.fig.suptitle(f'MarginReducer Visualizer  --  {name}',
                          color='white', fontsize=12, fontweight='bold')

        self.ax = self.fig.add_axes([0.04, 0.15, 0.72, 0.78])
        setup_dark_ax(self.ax)

        self.ax_info = self.fig.add_axes([0.78, 0.15, 0.20, 0.78])
        self.ax_info.set_facecolor(BG)
        self.ax_info.set_xticks([])
        self.ax_info.set_yticks([])

        self.sl_margin = make_slider(
            self.fig, [0.08, 0.06, 0.60, 0.03],
            'Margin (m)', 0.0, 15.0, margin, 0.5, '#E74C3C')
        self.sl_margin.on_changed(self._on_margin_change)

        self._draw()
        plt.show()

    def _on_margin_change(self, _):
        self.margin = self.sl_margin.val
        self._draw()

    def _draw(self):
        self.ax.cla()
        self.ax_info.cla()
        self.ax.set_aspect('equal')
        self.ax.set_facecolor(BG)
        self.ax_info.set_facecolor(BG)
        self.ax_info.set_xticks([])
        self.ax_info.set_yticks([])

        result = MarginReducer.shrink(self.raw_polygon, self.margin)

        # Original polygon
        rx, ry = self.raw_polygon.exterior.xy
        self.ax.fill(rx, ry, fc=C_ORIGINAL, alpha=0.12)
        self.ax.plot(rx, ry, color=C_ORIGINAL, lw=2.0, alpha=0.7, label='Original boundary')

        for interior in self.raw_polygon.interiors:
            ix, iy = interior.xy
            self.ax.fill(ix, iy, fc=C_OBSTACLE_ORIG, alpha=0.3, ec=C_OBSTACLE_ORIG, lw=1.5)

        # Result
        rx_res, ry_res = result.exterior.xy
        self.ax.fill(rx_res, ry_res, fc=C_SHRUNKEN, alpha=0.12)
        self.ax.plot(rx_res, ry_res, color=C_SHRUNKEN, lw=2.0, alpha=0.8, label='After margin')

        for interior in result.interiors:
            ix, iy = interior.xy
            self.ax.fill(ix, iy, fc=C_OBSTACLE_EXPANDED, alpha=0.3, ec=C_OBSTACLE_EXPANDED, lw=1.5)

        fix_axes_to_bounds(self.ax, self.raw_polygon)

        self.ax.set_title(
            f'Margin: {self.margin:.1f}m  |  '
            f'Valid: {"Y" if result.is_valid else "N"}',
            color='white' if result.is_valid else 'red',
            fontsize=10, pad=6)
        self.ax.legend(loc='upper left', fontsize=9, framealpha=0.9)

        # Info panel
        orig_area = self.raw_polygon.area
        result_area = result.area
        area_loss = ((orig_area - result_area) / orig_area * 100) if orig_area > 0 else 0

        info_lines = [
            f'Margin:           {self.margin:.1f} m',
            f'',
            f'Original:',
            f'  Area:           {orig_area:.0f} m2',
            f'  Perimeter:      {self.raw_polygon.exterior.length:.0f} m',
            f'  Vertices:       {len(list(self.raw_polygon.exterior.coords))-1}',
            f'  Obstacles:      {len(list(self.raw_polygon.interiors))}',
            f'',
            f'After Margin:',
            f'  Area:           {result_area:.0f} m2',
            f'  Perimeter:      {result.exterior.length:.0f} m',
            f'  Vertices:       {len(list(result.exterior.coords))-1}',
            f'  Obstacles:      {len(list(result.interiors))}',
            f'',
            f'Change:',
            f'  Area loss:      {area_loss:.1f}%',
            f'  Valid:          {"Y" if result.is_valid else "N"}',
        ]

        self.ax_info.text(0.05, 0.95, '\n'.join(info_lines), color='#bdc3c7',
                          fontsize=8.5, va='top', family='monospace',
                          transform=self.ax_info.transAxes)
        self.fig.canvas.draw_idle()


def main():
    parser = argparse.ArgumentParser(description='Visualize MarginReducer effects')
    parser.add_argument('json', nargs='?', help='Path to field JSON')
    parser.add_argument('--margin', type=float, default=2.5, help='Initial margin (m)')
    args = parser.parse_args()

    json_path = resolve_json_path(args.json)
    poly, name = load_json(json_path)
    print(f'Loaded: {name} ({len(list(poly.exterior.coords))-1} vertices, {poly.area:.0f} m2)')
    print(f'Obstacles: {len(list(poly.interiors))}')
    MarginVisualizer(poly, name, margin=args.margin)


if __name__ == '__main__':
    main()
