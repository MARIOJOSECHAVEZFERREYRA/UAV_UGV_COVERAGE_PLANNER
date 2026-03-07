import sys
import os
import json
import math
import argparse
import glob
import numpy as np

import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button
from shapely.geometry import Polygon, LineString, Point
from shapely.geometry.polygon import orient

# ── project imports ───────────────────────────────────────────────────────────
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'src'))

from algorithms.decomposition import ConcaveDecomposer
from algorithms.path_planner   import BoustrophedonPlanner
from data.field_io             import FieldIO

# ── colours ──────────────────────────────────────────────────────────────────
BG        = '#16213e'
PANEL_BG  = '#1a1a2e'
C_FIELD   = '#4A90D9'
C_CONC    = '#E74C3C'
C_T2      = '#F39C12'
C_T1      = '#7f8c8d'
C_SWEEP   = '#27AE60'
C_CUT     = '#E67E22'
C_PATH    = '#F1C40F'
CELLS     = ['#8E44AD','#16A085','#E74C3C','#2980B9','#D35400','#1ABC9C',
             '#F39C12','#8E44AD']
SWATH     = 5.0

# ── step collector ────────────────────────────────────────────────────────────
def build_steps(polygon: Polygon, heading_deg: float) -> list:
    heading_rad = np.radians(heading_deg)
    steps = []

    # Step 1: recursive decomposition (first step it inserts is 'scan')
    _collect(polygon, heading_rad, steps, depth=0)

    # Step 2: insert 'overview' right after the first 'scan' step
    coords = list(polygon.exterior.coords)
    if coords[0] == coords[-1]:
        coords = coords[:-1]

    all_t1, all_t2 = [], []
    for i in range(len(coords)):
        if ConcaveDecomposer._is_concave_topology_mapping(coords, i):
            v = coords[i]
            if ConcaveDecomposer._is_type_2(polygon, v, heading_rad):
                all_t2.append(v)
            else:
                all_t1.append(v)

    overview = {
        'type': 'overview',
        'polygon': polygon,
        'all_t1': all_t1,
        'all_t2': all_t2,
        'heading_rad': heading_rad,
        'depth': 0,
    }
    # Insert after the first 'scan' step (index 0)
    steps.insert(1, overview)
    return steps


def _collect(polygon: Polygon, heading_rad: float, steps: list, depth: int):
    if depth > 50 or polygon.is_empty:
        return
    coords = list(polygon.exterior.coords)
    if coords[0] == coords[-1]:
        coords = coords[:-1]
    n = len(coords)

    concave = [coords[i] for i in range(n)
               if ConcaveDecomposer._is_concave_topology_mapping(coords, i)]
    steps.append({'type': 'scan', 'polygon': polygon,
                  'concave': concave, 'heading_rad': heading_rad, 'depth': depth})

    for i in range(n):
        if not ConcaveDecomposer._is_concave_topology_mapping(coords, i):
            continue
        is_t2 = ConcaveDecomposer._is_type_2(polygon, coords[i], heading_rad)
        if not is_t2:
            continue
        steps.append({'type': 'type2', 'polygon': polygon, 'vertex': coords[i],
                      'is_t2': is_t2, 'heading_rad': heading_rad, 'depth': depth})
        subs = ConcaveDecomposer._split_polygon_at_vertex(
            polygon, coords[i], heading_rad)
        valid = [s for s in subs if s.area > 0.1 and s.area < 0.999 * polygon.area]
        if len(valid) < 2:
            continue
        steps.append({'type': 'cut', 'polygon': polygon, 'vertex': coords[i],
                      'heading_rad': heading_rad, 'pieces': subs, 'depth': depth})
        for sub in subs:
            # Re-orient to CCW: shapely split() doesn't guarantee winding order
            _collect(orient(sub, sign=1.0), heading_rad, steps, depth + 1)
        return

    # ── interior ring (obstacle) vertices ──
    for interior in polygon.interiors:
        hole_coords = list(interior.coords)
        if hole_coords[0] == hole_coords[-1]:
            hole_coords = hole_coords[:-1]

        for vertex in hole_coords:
            is_t2 = ConcaveDecomposer._is_type_2(polygon, vertex, heading_rad)
            if not is_t2:
                continue
            steps.append({'type': 'hole_t2', 'polygon': polygon, 'vertex': vertex,
                          'heading_rad': heading_rad, 'depth': depth})
            subs = ConcaveDecomposer._connect_hole_to_exterior(
                polygon, vertex, heading_rad)
            valid = [s for s in subs if s.area > 0.1]
            holes_before = len(list(polygon.interiors))
            holes_after = sum(len(list(s.interiors)) for s in valid)
            if not valid or (holes_after >= holes_before and len(valid) == 1):
                continue
            steps.append({'type': 'hole_cut', 'polygon': polygon, 'vertex': vertex,
                          'heading_rad': heading_rad, 'pieces': valid, 'depth': depth})
            for sub in valid:
                _collect(orient(sub, sign=1.0), heading_rad, steps, depth + 1)
            return

    steps.append({'type': 'final', 'polygon': polygon,
                  'heading_rad': heading_rad, 'depth': depth})


# ── drawing helpers ───────────────────────────────────────────────────────────
def draw_poly(ax, poly, fc, alpha=0.25, ec=None, lw=2):
    x, y = poly.exterior.xy
    ax.fill(x, y, fc=fc, alpha=alpha, ec='none')
    ax.plot(x, y, color=ec or fc, lw=lw)
    for interior in poly.interiors:
        ix, iy = interior.xy
        ax.fill(ix, iy, fc='white', alpha=1, ec='grey', lw=1)


def draw_sweep_probes(ax, polygon, vertex, heading_rad):
    eps = max(0.5, polygon.length * 0.005)
    ray = max(polygon.bounds[2]-polygon.bounds[0],
              polygon.bounds[3]-polygon.bounds[1]) * 3
    hv  = np.array([np.cos(heading_rad), np.sin(heading_rad)])
    adv = np.array([-np.sin(heading_rad), np.cos(heading_rad)])
    curr = np.array(vertex)

    for sign in (+1, -1):
        pt = curr + sign * eps * adv
        p0 = pt - ray * hv; p1 = pt + ray * hv
        ax.plot([p0[0], p1[0]], [p0[1], p1[1]],
                color=C_SWEEP, lw=1, ls='--', alpha=0.35)
        inter = LineString([(p0[0],p0[1]),(p1[0],p1[1])]).intersection(polygon)
        if inter.is_empty: continue
        segs = list(inter.geoms) if inter.geom_type=='MultiLineString' else [inter]
        for seg in segs:
            sx, sy = seg.xy
            ax.plot(sx, sy, color=C_SWEEP, lw=4, solid_capstyle='round', alpha=0.85)
        label = f'{len(segs)} seg{"s" if len(segs)>1 else ""}'
        mid = np.array([np.mean(segs[0].xy[0]), np.mean(segs[0].xy[1])])
        ax.annotate(label, mid, color=C_SWEEP, fontsize=7,
                    xytext=(4, 4), textcoords='offset points')


def _ray_hit_point(polygon, vertex, heading_rad):
    """Replicate the unidirectional ray logic from _split_polygon_at_vertex
    to find the hit point on the boundary, for visualization only."""
    vx, vy = vertex[0], vertex[1]
    hx = math.cos(heading_rad)
    hy = math.sin(heading_rad)
    eps = max(0.5, polygon.length * 0.002)
    ray_len = 1e6

    interior_sign = None
    for sign in (+1.0, -1.0):
        test_pt = Point(vx + sign * eps * hx, vy + sign * eps * hy)
        if polygon.contains(test_pt):
            interior_sign = sign
            break
    if interior_sign is None:
        return None

    dx = interior_sign * hx
    dy = interior_sign * hy
    ray_line = LineString([(vx, vy), (vx + ray_len * dx, vy + ray_len * dy)])

    boundary_lines = [polygon.exterior] + list(polygon.interiors)
    hit_pt = None
    min_dist = ray_len
    origin = Point(vx, vy)

    for ring in boundary_lines:
        inter = ray_line.intersection(ring)
        if inter.is_empty:
            continue
        if inter.geom_type == 'Point':
            candidates = [inter]
        elif inter.geom_type == 'MultiPoint':
            candidates = list(inter.geoms)
        else:
            raw = (list(inter.coords)
                   if hasattr(inter, 'coords')
                   else [c for g in getattr(inter, 'geoms', [inter])
                          for c in (g.coords if hasattr(g, 'coords') else [])])
            candidates = [Point(p) for p in raw]
        for pt in candidates:
            d = pt.distance(origin)
            if d > 1e-6 and d < min_dist:
                min_dist = d
                hit_pt = pt

    return hit_pt


def draw_cut_line(ax, vertex, heading_rad, polygon):
    """Draw the unidirectional ray from the T2 vertex to the first boundary hit."""
    vx, vy = vertex[0], vertex[1]
    hit_pt = _ray_hit_point(polygon, vertex, heading_rad)

    if hit_pt is not None:
        # Unidirectional ray: vertex → first wall hit
        ax.annotate('', xy=(hit_pt.x, hit_pt.y), xytext=(vx, vy),
                    arrowprops=dict(arrowstyle='->', color=C_CUT, lw=2,
                                   mutation_scale=16),
                    zorder=5)
        ax.plot([vx, hit_pt.x], [vy, hit_pt.y],
                color=C_CUT, lw=2, ls='--', zorder=5, label='Cut ray')
        ax.plot(hit_pt.x, hit_pt.y, 'D', color=C_CUT, ms=8, zorder=6,
                label='Ray hit point')
    else:
        # Fallback: draw full line if hit not found
        ray = max(polygon.bounds[2]-polygon.bounds[0],
                  polygon.bounds[3]-polygon.bounds[1]) * 2
        h = np.array([math.cos(heading_rad), math.sin(heading_rad)])
        ax.plot([vx - ray*h[0], vx + ray*h[0]],
                [vy - ray*h[1], vy + ray*h[1]],
                color=C_CUT, lw=2, ls='--', zorder=5, label='Cut line (fallback)')

    ax.plot(vx, vy, 's', color=C_CUT, ms=10, zorder=6)


def draw_path(ax, poly, heading_rad, color=C_PATH):
    try:
        planner = BoustrophedonPlanner(spray_width=SWATH)
        path, _, _, _ = planner.generate_path(poly, np.degrees(heading_rad))
        if len(path) > 1:
            xs, ys = zip(*path)
            ax.plot(xs, ys, color=color, lw=1.2, alpha=0.9)
    except Exception:
        pass


def heading_arrow(ax, poly, heading_rad):
    cx, cy = poly.centroid.x, poly.centroid.y
    span = max(poly.bounds[2]-poly.bounds[0], poly.bounds[3]-poly.bounds[1])
    L = span * 0.08
    hr = heading_rad
    ax.annotate('', xy=(cx+L*math.cos(hr), cy+L*math.sin(hr)),
                xytext=(cx, cy),
                arrowprops=dict(arrowstyle='->', color='white', lw=1.5,
                                mutation_scale=14))


# ── info text ─────────────────────────────────────────────────────────────────
STEP_DESC = {
    'overview': lambda s: (
        'OVERVIEW — All Concave Vertices',
        f"Type-1 (grey circles) : {len(s['all_t1'])}\n"
        f"Type-2 (orange stars) : {len(s['all_t2'])}\n\n"
        "T1 = sweep width changes,\n"
        "     no split needed.\n\n"
        "T2 = sweep splits in 2+ parts,\n"
        "     polygon must be cut.\n\n"
        "The algorithm cuts at the\n"
        "FIRST T2 found (in vertex order)\n"
        "then recurses on each piece."
    ),
    'scan': lambda s: (
        'SCANNING FOR CONCAVE VERTICES',
        f"Recursion depth : {s['depth']}\n"
        f"Polygon vertices: {len(list(s['polygon'].exterior.coords))-1}\n"
        f"Concave found   : {len(s['concave'])}\n\n"
        "RED dots = interior angle > 180°.\n"
        "Each will be tested for Type-2\n"
        "obstruction next."
    ),
    'type2': lambda s: (
        f"⚖️  TYPE-2 TEST — vertex ({s['vertex'][0]:.1f}, {s['vertex'][1]:.1f})",
        f"Two probe sweep lines drawn\n"
        f"BEFORE and AFTER this vertex.\n\n"
        f"If the segment COUNT changes\n"
        f"→ sweep topology changes here\n"
        f"→ drone must exit field → Type 2.\n\n"
        f"Result: {'TYPE 2 → will cut here' if s['is_t2'] else 'Type 1 → no cut needed'}"
    ),
    'cut': lambda s: (
        f"CUTTING POLYGON",
        f"Type-2 vertex at\n"
        f"  ({s['vertex'][0]:.1f}, {s['vertex'][1]:.1f})\n\n"
        f"Cut line parallel to heading\n"
        f"({np.degrees(s['heading_rad']):.0f}°).\n\n"
        f"Pieces produced: {len(s['pieces'])}\n\n"
        "Each piece will be processed\n"
        "recursively."
    ),
    'hole_t2': lambda s: (
        f"OBSTACLE TYPE-2 — vertex ({s['vertex'][0]:.1f}, {s['vertex'][1]:.1f})",
        f"Interior ring (obstacle) vertex\n"
        f"changes sweep-line topology.\n\n"
        f"A thin channel will connect\n"
        f"the obstacle to the exterior,\n"
        f"turning it into a concavity\n"
        f"that recursion will cut."
    ),
    'hole_cut': lambda s: (
        f"OBSTACLE CHANNEL CUT",
        f"Obstacle vertex at\n"
        f"  ({s['vertex'][0]:.1f}, {s['vertex'][1]:.1f})\n\n"
        f"Channel erased along heading\n"
        f"({np.degrees(s['heading_rad']):.0f}°).\n\n"
        f"Hole absorbed into exterior.\n"
        f"Pieces: {len(s['pieces'])}\n\n"
        "Recursion continues on\n"
        "each piece."
    ),
    'final': lambda s: (
        '✅  FINAL CELL — READY FOR SWEEP',
        f"Depth  : {s['depth']}\n"
        f"Area   : {s['polygon'].area:.1f} m²\n\n"
        "No Type-2 vertices found.\n"
        "Boustrophedon path planned\n"
        "(yellow lines).\n\n"
        f"Swath width: {SWATH} m"
    ),
}


# ── main visualizer ───────────────────────────────────────────────────────────
class Visualizer:
    def __init__(self, polygon: Polygon, name: str, heading_deg: float):
        self.base_poly   = polygon
        self.name        = name
        self.heading_deg = heading_deg
        self.steps       = build_steps(polygon, heading_deg)
        self.idx         = 0

        # ── figure ──
        self.fig = plt.figure(figsize=(14, 8), facecolor=PANEL_BG)
        self.fig.suptitle(f'Decomposition Visualizer  —  {name}',
                          color='white', fontsize=12, fontweight='bold')

        # axes
        self.ax      = self.fig.add_axes([0.04, 0.17, 0.64, 0.76])
        self.ax_info = self.fig.add_axes([0.70, 0.17, 0.28, 0.76])
        for a in (self.ax, self.ax_info):
            a.set_facecolor(BG)
            a.tick_params(colors='#7f8c8d')
            a.spines[:].set_color('#2d2d44')
        self.ax_info.set_xticks([]); self.ax_info.set_yticks([])
        self.ax_info.set_xlim(0,1);  self.ax_info.set_ylim(0,1)

        # slider
        ax_sl = self.fig.add_axes([0.04, 0.09, 0.64, 0.03])
        ax_sl.set_facecolor('#2d2d44')
        self.slider = Slider(ax_sl, 'Heading', 0, 175,
                             valinit=heading_deg, valstep=5, color=C_FIELD)
        self.slider.label.set_color('white')
        self.slider.valtext.set_color('white')
        self.slider.on_changed(self._on_angle)

        # buttons
        bspec = [
            ('[  PREV', 0.04,  self._prev),
            ('NEXT  ]', 0.22,  self._next),
            ('JUMP END', 0.40, self._end),
            ('RELOAD',   0.55, self._reload),
        ]
        self.btns = []
        for label, x, cb in bspec:
            ba = self.fig.add_axes([x, 0.02, 0.14, 0.05])
            b  = Button(ba, label, color='#2d3561', hovercolor='#4A90D9')
            b.label.set_color('white'); b.label.set_fontweight('bold')
            b.on_clicked(cb)
            self.btns.append(b)

        # keyboard
        self.fig.canvas.mpl_connect('key_press_event', self._on_key)

        self._draw(); plt.show()

    # ── navigation ──
    def _next(self, _=None):
        if self.idx < len(self.steps)-1: self.idx += 1
        self._draw()

    def _prev(self, _=None):
        if self.idx > 0: self.idx -= 1
        self._draw()

    def _end(self, _=None):
        self.idx = len(self.steps)-1; self._draw()

    def _reload(self, _=None):
        self.heading_deg = self.slider.val
        self.steps = build_steps(self.base_poly, self.heading_deg)
        self.idx   = 0; self._draw()

    def _on_angle(self, val):
        pass  # only apply on RELOAD to avoid lag

    def _on_key(self, ev):
        {'n': self._next, 'p': self._prev,
         'e': self._end,  'r': self._reload}.get(ev.key, lambda: None)()

    # ── rendering ──
    def _draw(self):
        self.ax.cla()
        self.ax.set_aspect('equal')
        self.ax.set_facecolor(BG)
        self.ax.tick_params(colors='#7f8c8d')
        self.ax.spines[:].set_color('#2d2d44')

        s    = self.steps[self.idx]
        stype = s['type']

        # always: faint original polygon in background
        draw_poly(self.ax, self.base_poly, C_FIELD, alpha=0.08, lw=1)

        # Cumulative: draw all 'final' cells seen so far (steps 0..idx)
        # with distinct colors; path uses the same color as its cell.
        final_steps = [st for st in self.steps[:self.idx + 1]
                       if st['type'] == 'final']
        for k, fst in enumerate(final_steps):
            c = CELLS[k % len(CELLS)]
            draw_poly(self.ax, fst['polygon'], c, alpha=0.28, lw=2)
            draw_path(self.ax, fst['polygon'], fst['heading_rad'], color=c)

        # active polygon (on top, only when not a final step — finals drawn above)
        if stype != 'final':
            draw_poly(self.ax, s['polygon'], C_FIELD, alpha=0.15, lw=2)

        if stype == 'overview':
            # All Type-1 (grey) and Type-2 (orange) at once
            for v in s['all_t1']:
                self.ax.plot(*v, 'o', color=C_T1, ms=10, zorder=6)
                self.ax.annotate('  T1', v, color=C_T1, fontsize=8, va='center')
            for v in s['all_t2']:
                self.ax.plot(*v, '*', color=C_T2, ms=14, zorder=7)
                self.ax.annotate('  T2', v, color=C_T2, fontsize=8,
                                 fontweight='bold', va='center')

        elif stype == 'scan':
            for v in s['concave']:
                self.ax.plot(*v, 'o', color=C_CONC, ms=11, zorder=6)
                self.ax.annotate(f'  concave', v, color=C_CONC, fontsize=7,
                                 va='center')

        elif stype == 'type2':
            draw_sweep_probes(self.ax, s['polygon'], s['vertex'], s['heading_rad'])
            c = C_T2 if s['is_t2'] else C_T1
            self.ax.plot(*s['vertex'], '*', color=c, ms=16, zorder=7)
            tag = 'TYPE 2' if s['is_t2'] else 'Type 1'
            self.ax.annotate(f'  {tag}', s['vertex'], color=c,
                             fontsize=9, fontweight='bold', va='center')

        elif stype == 'cut':
            draw_cut_line(self.ax, s['vertex'], s['heading_rad'], s['polygon'])
            for k, piece in enumerate(s['pieces']):
                draw_poly(self.ax, piece, CELLS[k % len(CELLS)], alpha=0.28, lw=1.5)

        elif stype == 'hole_t2':
            draw_sweep_probes(self.ax, s['polygon'], s['vertex'], s['heading_rad'])
            self.ax.plot(*s['vertex'], '*', color='#E91E63', ms=16, zorder=7)
            self.ax.annotate('  OBSTACLE T2', s['vertex'], color='#E91E63',
                             fontsize=9, fontweight='bold', va='center')

        elif stype == 'hole_cut':
            # Draw channel line from vertex to hit point
            draw_cut_line(self.ax, s['vertex'], s['heading_rad'], s['polygon'])
            for k, piece in enumerate(s['pieces']):
                draw_poly(self.ax, piece, CELLS[k % len(CELLS)], alpha=0.28, lw=1.5)

        elif stype == 'final':
            c = CELLS[s['depth'] % len(CELLS)]
            draw_poly(self.ax, s['polygon'], c, alpha=0.38, lw=2)
            draw_path(self.ax, s['polygon'], s['heading_rad'])

        # heading arrow
        heading_arrow(self.ax, s['polygon'], s['heading_rad'])

        # Pin axes to the base polygon bounds (+ 15% margin) so the long
        # sweep probe lines never cause the map to zoom out.
        minx, miny, maxx, maxy = self.base_poly.bounds
        pw = (maxx - minx) * 0.15 or 10
        ph = (maxy - miny) * 0.15 or 10
        self.ax.set_xlim(minx - pw, maxx + pw)
        self.ax.set_ylim(miny - ph, maxy + ph)
        self.ax.set_title(
            f"Step {self.idx+1} / {len(self.steps)}  —  "
            f"Heading: {np.degrees(s['heading_rad']):.0f}°  |  "
            f"[N] next  [P] prev  [E] end  [R] reload",
            color='white', fontsize=9, pad=6)

        # info panel
        title, body = STEP_DESC.get(stype, lambda s: ('?',''))(s)
        self.ax_info.cla()
        self.ax_info.set_facecolor(BG)
        self.ax_info.set_xticks([]); self.ax_info.set_yticks([])
        self.ax_info.set_xlim(0,1);  self.ax_info.set_ylim(0,1)
        self.ax_info.text(0.5, 0.97, f'Step {self.idx+1}/{len(self.steps)}',
                          color='#7f8c8d', fontsize=8, ha='center', va='top')
        self.ax_info.axhline(0.94, color='#2d2d44', lw=1)
        self.ax_info.text(0.05, 0.91, title,
                          color='#F39C12', fontsize=10, fontweight='bold', va='top')
        self.ax_info.axhline(0.86, color='#2d2d44', lw=1)
        self.ax_info.text(0.05, 0.83, body,
                          color='#bdc3c7', fontsize=9, va='top', family='monospace')

        # legend
        legend_items = [
            (C_FIELD, 'Field polygon'),
            (C_CONC,  'Concave vertex'),
            (C_T2,    'Type-2 (obstructive)'),
            (C_T1,    'Type-1 (ok)'),
            (C_SWEEP, 'Sweep probe lines'),
            (C_CUT,   'Cut line'),
            (C_PATH,  'Boustrophedon path'),
        ]
        for k, (c, lbl) in enumerate(legend_items):
            y = 0.38 - k*0.05
            self.ax_info.plot([0.05, 0.12], [y, y], color=c, lw=2)
            self.ax_info.text(0.15, y, lbl, color='#95a5a6', fontsize=8, va='center')

        self.fig.canvas.draw_idle()


# ── JSON loading ──────────────────────────────────────────────────────────────
def load_json(path: str) -> tuple:
    """Returns (polygon, name). Supports both FieldIO format and raw boundary list."""
    with open(path) as f:
        data = json.load(f)

    # raw boundary key (project format)
    if 'boundary' in data:
        coords = [tuple(p) for p in data['boundary']]
        name   = data.get('name', os.path.basename(path))
        obstacles = [Polygon([tuple(p) for p in obs])
                     for obs in data.get('obstacles', [])]
    # AgriSwarm session export
    elif 'polygon' in data:
        coords = [tuple(p[:2]) for p in data['polygon']]
        name   = os.path.basename(path)
        obstacles = []
    else:
        raise ValueError(f'Unknown JSON format in {path}')

    poly = orient(Polygon(coords), sign=1.0)  # ensure CCW

    # punch obstacles as holes
    if obstacles:
        for obs in obstacles:
            poly = poly.difference(obs)
        poly = orient(poly, sign=1.0)  # re-orient after difference()

    return poly, name


def pick_json() -> str:
    """List available JSONs and ask the user to pick one."""
    data_dir = os.path.join(ROOT, 'data', 'test_fields')
    files    = sorted(glob.glob(os.path.join(data_dir, '**', '*.json'),
                                recursive=True))
    if not files:
        print(f'No JSON files found under {data_dir}')
        sys.exit(1)

    print('\nAvailable field files:')
    for i, f in enumerate(files):
        rel = os.path.relpath(f, ROOT)
        print(f'  [{i+1}] {rel}')

    choice = input('\nEnter number (or full path): ').strip()
    if choice.isdigit():
        return files[int(choice)-1]
    return choice


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('json', nargs='?', help='Path to field JSON file')
    parser.add_argument('--angle', type=float, default=0.0,
                        help='Initial sweep heading in degrees (default 0)')
    args = parser.parse_args()

    json_path = args.json or pick_json()
    if not os.path.isabs(json_path):
        json_path = os.path.join(ROOT, json_path)

    poly, name = load_json(json_path)
    print(f'\nLoaded: {name}')
    print(f'Vertices: {len(list(poly.exterior.coords))-1}')
    print(f'Area    : {poly.area:.1f} m²')
    print(f'Heading : {args.angle:.0f}°')
    steps = build_steps(poly, args.angle)
    print(f'Steps   : {len(steps)}')

    Visualizer(poly, name, args.angle)


if __name__ == '__main__':
    main()
