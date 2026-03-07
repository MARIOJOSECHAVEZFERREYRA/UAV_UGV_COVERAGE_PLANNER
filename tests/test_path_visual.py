import sys
import os
import json
import argparse
import glob
import numpy as np

import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
from shapely.geometry import Polygon, LineString
from shapely.geometry.polygon import orient

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'src'))

from algorithms.decomposition import ConcaveDecomposer
from algorithms.path_planner import BoustrophedonPlanner
from algorithms.margin import MarginReducer

BG = '#16213e'
PANEL_BG = '#1a1a2e'
CELLS = ['#8E44AD', '#16A085', '#E74C3C', '#2980B9', '#D35400', '#1ABC9C',
         '#F39C12', '#C0392B', '#27AE60', '#2C3E50']
DEFAULT_SWATH = 5.0
DEFAULT_MARGIN = 2.5


def load_json(path):
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
    if obstacles:
        for obs in obstacles:
            poly = poly.difference(Polygon([tuple(p) for p in obs]))
        poly = orient(poly, sign=1.0)
    return poly, name


def pick_json():
    data_dir = os.path.join(ROOT, 'data', 'test_fields')
    files = sorted(glob.glob(os.path.join(data_dir, '**', '*.json'), recursive=True))
    if not files:
        print(f'No JSON files found under {data_dir}')
        sys.exit(1)
    print('\nAvailable field files:')
    for i, f in enumerate(files):
        print(f'  [{i+1}] {os.path.relpath(f, ROOT)}')
    choice = input('\nEnter number (or full path): ').strip()
    if choice.isdigit():
        return files[int(choice) - 1]
    return choice


class PathVisualizer:
    def __init__(self, polygon, name, heading_deg, swath=DEFAULT_SWATH, margin=DEFAULT_MARGIN):
        self.raw_polygon = polygon  # original, before margin
        self.name = name
        self.heading_deg = heading_deg
        self.swath = swath
        self.margin = margin

        self.fig = plt.figure(figsize=(15, 8), facecolor=PANEL_BG)
        self.fig.suptitle(f'Path Planner Visualizer  --  {name}',
                          color='white', fontsize=12, fontweight='bold')

        self.ax = self.fig.add_axes([0.04, 0.22, 0.62, 0.72])
        self.ax_info = self.fig.add_axes([0.68, 0.22, 0.30, 0.72])
        for a in (self.ax, self.ax_info):
            a.set_facecolor(BG)
            a.tick_params(colors='#7f8c8d')
            for sp in a.spines.values():
                sp.set_color('#2d2d44')

        # Heading slider
        ax_heading = self.fig.add_axes([0.08, 0.13, 0.55, 0.03])
        ax_heading.set_facecolor('#2d2d44')
        self.sl_heading = Slider(ax_heading, 'Heading', 0, 175,
                                 valinit=heading_deg, valstep=5, color='#4A90D9')
        self.sl_heading.label.set_color('white')
        self.sl_heading.valtext.set_color('white')
        self.sl_heading.on_changed(self._on_param)

        # Swath slider
        ax_swath = self.fig.add_axes([0.08, 0.08, 0.55, 0.03])
        ax_swath.set_facecolor('#2d2d44')
        self.sl_swath = Slider(ax_swath, 'Swath (m)', 1.0, 20.0,
                               valinit=swath, valstep=0.5, color='#27AE60')
        self.sl_swath.label.set_color('white')
        self.sl_swath.valtext.set_color('white')
        self.sl_swath.on_changed(self._on_param)

        # Margin slider
        ax_margin = self.fig.add_axes([0.08, 0.03, 0.55, 0.03])
        ax_margin.set_facecolor('#2d2d44')
        self.sl_margin = Slider(ax_margin, 'Margin (m)', 0.0, 15.0,
                                valinit=margin, valstep=0.5, color='#E74C3C')
        self.sl_margin.label.set_color('white')
        self.sl_margin.valtext.set_color('white')
        self.sl_margin.on_changed(self._on_param)

        self._draw()
        plt.show()

    def _on_param(self, _=None):
        self.heading_deg = self.sl_heading.val
        self.swath = self.sl_swath.val
        self.margin = self.sl_margin.val
        self._draw()

    def _get_work_polygon(self):
        if self.margin > 0:
            return MarginReducer.shrink(self.raw_polygon, self.margin)
        return self.raw_polygon

    def _draw(self):
        self.ax.cla()
        self.ax_info.cla()
        self.ax.set_aspect('equal')
        self.ax.set_facecolor(BG)
        self.ax_info.set_facecolor(BG)
        self.ax_info.set_xticks([])
        self.ax_info.set_yticks([])

        angle = self.heading_deg
        work_poly = self._get_work_polygon()
        planner = BoustrophedonPlanner(spray_width=self.swath)

        if work_poly.is_empty or work_poly.area < 1:
            self.ax.set_title('Margin too large - polygon collapsed',
                              color='red', fontsize=10)
            self.fig.canvas.draw_idle()
            return

        cells = ConcaveDecomposer.decompose(work_poly, angle)

        # Draw original polygon (raw field boundary)
        rx, ry = self.raw_polygon.exterior.xy
        self.ax.fill(rx, ry, fc='#4A90D9', alpha=0.06)
        self.ax.plot(rx, ry, color='#4A90D9', lw=1, alpha=0.3, ls='--', label='Raw field')
        for interior in self.raw_polygon.interiors:
            ix, iy = interior.xy
            self.ax.fill(ix, iy, fc='white', alpha=0.5, ec='grey', lw=1, ls='--')

        # Draw work polygon (after margin)
        wx, wy = work_poly.exterior.xy
        self.ax.plot(wx, wy, color='#E74C3C', lw=1.5, alpha=0.6, label='After margin')
        for interior in work_poly.interiors:
            ix, iy = interior.xy
            self.ax.plot(ix, iy, color='#E74C3C', lw=1.2, alpha=0.5)
            self.ax.fill(ix, iy, fc='white', alpha=0.8, ec='#E74C3C', lw=1)

        total_dist = 0
        total_cov = 0
        total_turns = 0
        total_violations = 0
        cell_info = []

        for i, cell in enumerate(cells):
            c = CELLS[i % len(CELLS)]
            cx, cy = cell.exterior.xy
            self.ax.fill(cx, cy, fc=c, alpha=0.15)
            self.ax.plot(cx, cy, color=c, lw=1.5, alpha=0.7)
            for interior in cell.interiors:
                ix, iy = interior.xy
                self.ax.fill(ix, iy, fc=BG, alpha=0.9, ec='grey', lw=1)

            wps, dist, cov, turns = planner.generate_path(cell, angle)
            total_dist += dist
            total_cov += cov
            total_turns += turns

            violations = 0
            if len(wps) > 1:
                xs_path, ys_path = zip(*wps)
                self.ax.plot(xs_path, ys_path, color=c, lw=1.2, alpha=0.9)

                for j in range(len(wps) - 1):
                    seg = LineString([wps[j], wps[j + 1]])
                    if not work_poly.buffer(0.5).contains(seg):
                        violations += 1
                        self.ax.plot([wps[j][0], wps[j+1][0]],
                                     [wps[j][1], wps[j+1][1]],
                                     color='red', lw=2.5, alpha=0.9)

            total_violations += violations
            n_holes = len(list(cell.interiors))
            cell_info.append((i, len(wps), dist, turns, violations, n_holes))

        # Fix axes to raw polygon bounds
        minx, miny, maxx, maxy = self.raw_polygon.bounds
        pw = (maxx - minx) * 0.1 or 10
        ph = (maxy - miny) * 0.1 or 10
        self.ax.set_xlim(minx - pw, maxx + pw)
        self.ax.set_ylim(miny - ph, maxy + ph)

        status = 'OK' if total_violations == 0 else f'VIOLATIONS: {total_violations}'
        self.ax.set_title(
            f'Heading: {angle:.0f}  |  {len(cells)} cells  |  {status}',
            color='white' if total_violations == 0 else 'red',
            fontsize=10, pad=6)

        # Info panel
        raw_area = self.raw_polygon.area
        work_area = work_poly.area
        cov_pct = (total_cov / work_area * 100) if work_area > 0 else 0
        area_loss = ((raw_area - work_area) / raw_area * 100) if raw_area > 0 else 0
        info_lines = [
            f'Heading:     {angle:.0f}',
            f'Swath:       {self.swath:.1f} m',
            f'Margin:      {self.margin:.1f} m',
            f'',
            f'Raw area:    {raw_area:.0f} m2',
            f'Work area:   {work_area:.0f} m2',
            f'Area loss:   {area_loss:.1f}%',
            f'',
            f'Cells:       {len(cells)}',
            f'Distance:    {total_dist:.0f} m',
            f'Turns:       {total_turns}',
            f'Coverage:    {cov_pct:.1f}%',
            f'Violations:  {total_violations}',
            '',
            'Per cell:',
        ]
        for idx, nwps, d, t, v, h in cell_info:
            flag = ' !!!' if v > 0 else ''
            hole_str = f' H={h}' if h > 0 else ''
            info_lines.append(f'  [{idx}] {nwps:3d}wp {d:7.0f}m {t:2d}T{hole_str}{flag}')

        text = '\n'.join(info_lines)
        self.ax_info.text(0.05, 0.95, text, color='#bdc3c7', fontsize=9,
                          va='top', family='monospace',
                          transform=self.ax_info.transAxes)

        self.fig.canvas.draw_idle()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('json', nargs='?', help='Path to field JSON')
    parser.add_argument('--angle', type=float, default=0.0)
    parser.add_argument('--swath', type=float, default=DEFAULT_SWATH, help='Spray width (m)')
    parser.add_argument('--margin', type=float, default=DEFAULT_MARGIN, help='Safety margin (m)')
    args = parser.parse_args()

    json_path = args.json or pick_json()
    if not os.path.isabs(json_path):
        json_path = os.path.join(ROOT, json_path)

    poly, name = load_json(json_path)
    print(f'Loaded: {name} ({len(list(poly.exterior.coords))-1} vertices, {poly.area:.0f} m2)')
    PathVisualizer(poly, name, args.angle, swath=args.swath, margin=args.margin)


if __name__ == '__main__':
    main()
