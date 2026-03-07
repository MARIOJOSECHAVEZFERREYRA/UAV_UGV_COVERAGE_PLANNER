import sys
import os
import json
import argparse
from shapely.geometry import Polygon, LineString
from shapely.geometry.polygon import orient

import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'src'))

from algorithms.margin import MarginReducer
from field_picker import pick_json

# Colors
BG = '#16213e'
PANEL_BG = '#1a1a2e'
C_ORIGINAL = '#4A90D9'
C_SHRUNKEN = '#E74C3C'
C_OBSTACLE_ORIG = '#8E44AD'
C_OBSTACLE_EXPANDED = '#F39C12'


def load_json(path: str) -> tuple:
    """Load polygon from JSON file. Returns (polygon, name)"""
    with open(path) as f:
        data = json.load(f)

    if 'boundary' in data:
        coords = [tuple(p) for p in data['boundary']]
        name = data.get('name', os.path.basename(path))
        obstacles = data.get('obstacles', [])
    elif 'polygon' in data:
        coords = [tuple(p[:2]) for p in data['polygon']]
        name = os.path.basename(path)
        obstacles = []
    else:
        raise ValueError(f'Unknown JSON format in {path}')

    poly = orient(Polygon(coords), sign=1.0)

    # Add obstacles as holes
    if obstacles:
        for obs in obstacles:
            poly = poly.difference(Polygon([tuple(p) for p in obs]))
        poly = orient(poly, sign=1.0)

    return poly, name


class MarginVisualizer:
    """Interactive visualizer for MarginReducer"""

    def __init__(self, polygon: Polygon, name: str, margin: float = 2.5):
        self.raw_polygon = polygon
        self.name = name
        self.margin = margin

        # Figure setup
        self.fig = plt.figure(figsize=(15, 8), facecolor=PANEL_BG)
        self.fig.suptitle(f'MarginReducer Visualizer  —  {name}',
                          color='white', fontsize=12, fontweight='bold')

        # Main plot
        self.ax = self.fig.add_axes([0.04, 0.15, 0.72, 0.78])
        self.ax.set_facecolor(BG)
        self.ax.tick_params(colors='#7f8c8d')
        for sp in self.ax.spines.values():
            sp.set_color('#2d2d44')

        # Info panel
        self.ax_info = self.fig.add_axes([0.78, 0.15, 0.20, 0.78])
        self.ax_info.set_facecolor(BG)
        self.ax_info.set_xticks([])
        self.ax_info.set_yticks([])

        # Margin slider
        ax_margin = self.fig.add_axes([0.08, 0.06, 0.60, 0.03])
        ax_margin.set_facecolor('#2d2d44')
        self.sl_margin = Slider(ax_margin, 'Margin (m)', 0.0, 15.0,
                               valinit=margin, valstep=0.5, color='#E74C3C')
        self.sl_margin.label.set_color('white')
        self.sl_margin.valtext.set_color('white')
        self.sl_margin.on_changed(self._on_margin_change)

        self._draw()
        plt.show()

    def _on_margin_change(self, val):
        """Handle margin slider change"""
        self.margin = self.sl_margin.val
        self._draw()

    def _draw(self):
        """Redraw visualization"""
        self.ax.cla()
        self.ax_info.cla()

        self.ax.set_aspect('equal')
        self.ax.set_facecolor(BG)
        self.ax_info.set_facecolor(BG)
        self.ax_info.set_xticks([])
        self.ax_info.set_yticks([])

        # Apply margin
        result = MarginReducer.shrink(self.raw_polygon, self.margin)

        # Draw original polygon
        rx, ry = self.raw_polygon.exterior.xy
        self.ax.fill(rx, ry, fc=C_ORIGINAL, alpha=0.12)
        self.ax.plot(rx, ry, color=C_ORIGINAL, lw=2.0, alpha=0.7, label='Original boundary')

        # Draw original obstacles
        for interior in self.raw_polygon.interiors:
            ix, iy = interior.xy
            self.ax.fill(ix, iy, fc=C_OBSTACLE_ORIG, alpha=0.3, ec=C_OBSTACLE_ORIG, lw=1.5)

        # Draw result (shrunken exterior)
        rx_res, ry_res = result.exterior.xy
        self.ax.fill(rx_res, ry_res, fc=C_SHRUNKEN, alpha=0.12)
        self.ax.plot(rx_res, ry_res, color=C_SHRUNKEN, lw=2.0, alpha=0.8, label='After margin')

        # Draw expanded obstacles
        for interior in result.interiors:
            ix, iy = interior.xy
            self.ax.fill(ix, iy, fc=C_OBSTACLE_EXPANDED, alpha=0.3, ec=C_OBSTACLE_EXPANDED, lw=1.5)

        # Fix axes to original bounds
        minx, miny, maxx, maxy = self.raw_polygon.bounds
        pw = (maxx - minx) * 0.1 or 10
        ph = (maxy - miny) * 0.1 or 10
        self.ax.set_xlim(minx - pw, maxx + pw)
        self.ax.set_ylim(miny - ph, maxy + ph)

        # Title
        self.ax.set_title(
            f'Margin: {self.margin:.1f}m  |  '
            f'Valid: {"✓" if result.is_valid else "✗"}',
            color='white' if result.is_valid else 'red',
            fontsize=10, pad=6)

        # Legend
        self.ax.legend(loc='upper left', fontsize=9, framealpha=0.9)

        # Info panel
        orig_area = self.raw_polygon.area
        result_area = result.area
        area_loss = ((orig_area - result_area) / orig_area * 100) if orig_area > 0 else 0

        orig_holes = len(list(self.raw_polygon.interiors))
        result_holes = len(list(result.interiors))

        orig_perimeter = self.raw_polygon.exterior.length
        result_perimeter = result.exterior.length

        info_lines = [
            f'Margin:           {self.margin:.1f} m',
            f'',
            f'Original:',
            f'  Area:           {orig_area:.0f} m²',
            f'  Perimeter:      {orig_perimeter:.0f} m',
            f'  Vertices:       {len(list(self.raw_polygon.exterior.coords))-1}',
            f'  Obstacles:      {orig_holes}',
            f'',
            f'After Margin:',
            f'  Area:           {result_area:.0f} m²',
            f'  Perimeter:      {result_perimeter:.0f} m',
            f'  Vertices:       {len(list(result.exterior.coords))-1}',
            f'  Obstacles:      {result_holes}',
            f'',
            f'Change:',
            f'  Area loss:      {area_loss:.1f}%',
            f'  Valid:          {"✓" if result.is_valid else "✗"}',
        ]

        text = '\n'.join(info_lines)
        self.ax_info.text(0.05, 0.95, text, color='#bdc3c7', fontsize=8.5,
                         va='top', family='monospace',
                         transform=self.ax_info.transAxes)

        self.fig.canvas.draw_idle()


def main():
    parser = argparse.ArgumentParser(description='Visualize MarginReducer effects')
    parser.add_argument('json', nargs='?', help='Path to field JSON')
    parser.add_argument('--margin', type=float, default=2.5, help='Initial margin (m)')
    args = parser.parse_args()

    json_path = args.json or pick_json(ROOT)
    if not os.path.isabs(json_path):
        json_path = os.path.join(ROOT, json_path)

    poly, name = load_json(json_path)
    print(f'Loaded: {name} ({len(list(poly.exterior.coords))-1} vertices, {poly.area:.0f} m²)')
    print(f'Obstacles: {len(list(poly.interiors))}')

    MarginVisualizer(poly, name, margin=args.margin)


if __name__ == '__main__':
    main()
