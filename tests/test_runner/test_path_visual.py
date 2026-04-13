import sys
import os
import argparse

import matplotlib.pyplot as plt
from shapely.geometry import LineString

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, '..', 'backend'))

from algorithms.coverage.decomposition import ConcaveDecomposer
from algorithms.coverage.path_planner import BoustrophedonPlanner
from algorithms.coverage.margin import MarginReducer
from visual_base import (
    BG, PANEL_BG, CELLS, load_json, setup_dark_ax, make_slider,
    fix_axes_to_bounds, resolve_json_path,
)

DEFAULT_SWATH = 5.0
DEFAULT_MARGIN = 2.5


class PathVisualizer:
    def __init__(self, polygon, name, heading_deg, swath=DEFAULT_SWATH, margin=DEFAULT_MARGIN):
        self.raw_polygon = polygon
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
            setup_dark_ax(a)

        self.sl_heading = make_slider(
            self.fig, [0.08, 0.13, 0.55, 0.03],
            'Heading', 0, 175, heading_deg, 5, '#4A90D9')
        self.sl_heading.on_changed(self._on_param)

        self.sl_swath = make_slider(
            self.fig, [0.08, 0.08, 0.55, 0.03],
            'Swath (m)', 1.0, 20.0, swath, 0.5, '#27AE60')
        self.sl_swath.on_changed(self._on_param)

        self.sl_margin = make_slider(
            self.fig, [0.08, 0.03, 0.55, 0.03],
            'Margin (m)', 0.0, 15.0, margin, 0.5, '#E74C3C')
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

        # Raw field boundary
        rx, ry = self.raw_polygon.exterior.xy
        self.ax.fill(rx, ry, fc='#4A90D9', alpha=0.06)
        self.ax.plot(rx, ry, color='#4A90D9', lw=1, alpha=0.3, ls='--', label='Raw field')
        for interior in self.raw_polygon.interiors:
            ix, iy = interior.xy
            self.ax.fill(ix, iy, fc='white', alpha=0.5, ec='grey', lw=1, ls='--')

        # Work polygon (after margin)
        wx, wy = work_poly.exterior.xy
        self.ax.plot(wx, wy, color='#E74C3C', lw=1.5, alpha=0.6, label='After margin')
        for interior in work_poly.interiors:
            ix, iy = interior.xy
            self.ax.fill(ix, iy, fc='#F39C12', alpha=0.25, ec='#E74C3C', lw=1.5)
            self.ax.plot(ix, iy, color='#E74C3C', lw=1.2, alpha=0.7)

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

            result = planner.generate_path(cell, angle)
            segs = result.get('sweep_segments', [])
            metrics = result.get('metrics', {})
            dist  = metrics.get('spray_distance_m', 0.0)
            cov   = metrics.get('coverage_area_m2', 0.0)
            turns = metrics.get('turn_count', 0)
            total_dist += dist
            total_cov  += cov
            total_turns += turns

            violations = 0
            all_pts = [pt for seg in segs for pt in seg.get('path', [])]
            if len(all_pts) > 1:
                for seg in segs:
                    pts = seg.get('path', [])
                    if len(pts) >= 2:
                        xs_path, ys_path = zip(*pts)
                        self.ax.plot(xs_path, ys_path, color=c, lw=1.2, alpha=0.9)
                for j in range(len(all_pts) - 1):
                    line = LineString([all_pts[j], all_pts[j + 1]])
                    if not work_poly.buffer(0.5).contains(line):
                        violations += 1
                        self.ax.plot([all_pts[j][0], all_pts[j+1][0]],
                                     [all_pts[j][1], all_pts[j+1][1]],
                                     color='red', lw=2.5, alpha=0.9)

            total_violations += violations
            n_holes = len(list(cell.interiors))
            cell_info.append((i, len(all_pts), dist, turns, violations, n_holes))

        fix_axes_to_bounds(self.ax, self.raw_polygon)

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
            '',
            f'Raw area:    {raw_area:.0f} m2',
            f'Work area:   {work_area:.0f} m2',
            f'Area loss:   {area_loss:.1f}%',
            '',
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

        self.ax_info.text(0.05, 0.95, '\n'.join(info_lines), color='#bdc3c7',
                          fontsize=9, va='top', family='monospace',
                          transform=self.ax_info.transAxes)
        self.fig.canvas.draw_idle()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('json', nargs='?', help='Path to field JSON')
    parser.add_argument('--angle', type=float, default=0.0)
    parser.add_argument('--swath', type=float, default=DEFAULT_SWATH)
    parser.add_argument('--margin', type=float, default=DEFAULT_MARGIN)
    args = parser.parse_args()

    json_path = resolve_json_path(args.json)
    poly, name = load_json(json_path)
    print(f'Loaded: {name} ({len(list(poly.exterior.coords))-1} vertices, {poly.area:.0f} m2)')
    PathVisualizer(poly, name, args.angle, swath=args.swath, margin=args.margin)


if __name__ == '__main__':
    main()
