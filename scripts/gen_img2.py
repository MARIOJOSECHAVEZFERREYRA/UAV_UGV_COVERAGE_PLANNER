"""
Generate a figure illustrating coverage path planning approaches
for concave polygon fields: standard boustrophedon vs exact cellular decomposition.

Uses the ACTUAL algorithms from the codebase:
- ConcaveDecomposer for recursive Type-2 vertex detection + ray cuts
- BoustrophedonPlanner for sweep path generation
"""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.lines import Line2D
from shapely.geometry import Polygon, LineString, Point
from shapely.prepared import prep
from shapely import affinity

from src.algorithms.decomposition import ConcaveDecomposer
from src.algorithms.path_planner import BoustrophedonPlanner

# ---------- common data ----------
FIELD_VERTS = np.array([
    [43.131, 6.428],
    [88.957, 30.067],
    [73.198, 92.068],
    [56.817, 83.981],
    [58.061, 63.037],
    [43.771, 55.191],
    [15.967, 77.138],
    [13.021, 58.170],
    [3.617, 54.977],
    [8.502, 40.643],
    [43.131, 6.428],
])

poly = Polygon(FIELD_VERTS[:-1])
HEADING_DEG = 0.0
SPRAY_WIDTH = 5.0

# Matplotlib style
plt.rcParams.update({
    "font.family": "serif",
    "font.size": 11,
    "axes.linewidth": 0.8,
})

fig, (ax_a, ax_b) = plt.subplots(1, 2, figsize=(12, 6))
bounds = poly.bounds
pad = 5

# =====================================================================
# Detect T2 vertices (shared by both panels)
# =====================================================================
heading_rad = np.radians(HEADING_DEG)
coords = list(poly.exterior.coords)
if coords[0] == coords[-1]:
    coords = coords[:-1]

concave_indices = ConcaveDecomposer._find_concave_indices(coords)
type2_indices = ConcaveDecomposer._classify_type2_batch(
    poly, coords, concave_indices, heading_rad
)

# Notch zone = area inside convex hull but outside polygon (the concave bites)
notch_zone = poly.convex_hull.difference(poly)

# =====================================================================
# (a) Standard boustrophedon — real connected path on whole polygon
# =====================================================================

ax_a.fill(FIELD_VERTS[:, 0], FIELD_VERTS[:, 1],
          facecolor="#d5f5d0", edgecolor="black", linewidth=1.5, zorder=2)

# Generate NAIVE boustrophedon: sweep segments + straight-line turns
# (no boundary-walk avoidance) to show the deadhead problem
planner = BoustrophedonPlanner(spray_width=SPRAY_WIDTH)
origin_pt = poly.centroid
rotated_whole = affinity.rotate(poly, -HEADING_DEG, origin=origin_pt)
global_y_origin = rotated_whole.bounds[1]

# Generate raw sweep segments by intersecting sweep lines with polygon
rotated_poly = affinity.rotate(poly, -HEADING_DEG, origin=origin_pt)
min_x, min_y, max_x, max_y = rotated_poly.bounds
sweep_ext = (max_x - min_x) + 1

first_y = global_y_origin + (SPRAY_WIDTH / 2)
if first_y < min_y:
    import math
    n = math.ceil((min_y - first_y) / SPRAY_WIDTH)
    first_y += n * SPRAY_WIDTH

# Collect ordered sweep segments in original coordinates
all_segments = []  # list of [(x1,y1), (x2,y2)] per segment
y_current = first_y
direction = True  # True = L->R

while y_current < max_y:
    sweepline = LineString([(min_x - sweep_ext, y_current),
                            (max_x + sweep_ext, y_current)])
    intersection = sweepline.intersection(rotated_poly)
    if not intersection.is_empty:
        if intersection.geom_type == "LineString":
            segs = [intersection]
        elif intersection.geom_type == "MultiLineString":
            segs = list(intersection.geoms)
        else:
            segs = [g for g in intersection.geoms
                    if g.geom_type == "LineString"]

        segs.sort(key=lambda s: s.coords[0][0])
        if not direction:
            segs.reverse()

        for seg in segs:
            seg_coords = list(seg.coords)
            if not direction:
                seg_coords.reverse()
            # Un-rotate back to original coordinates
            seg_line = LineString(seg_coords)
            restored = affinity.rotate(seg_line, HEADING_DEG, origin=origin_pt)
            all_segments.append(list(restored.coords))

    y_current += SPRAY_WIDTH
    direction = not direction

# Build the full naive path: connect segments with straight lines
prepared_poly_buf = prep(poly.buffer(0.1))
for i, seg_coords in enumerate(all_segments):
    # Draw sweep segment (always blue — it's inside the polygon)
    xs = [p[0] for p in seg_coords]
    ys = [p[1] for p in seg_coords]
    ax_a.plot(xs, ys, color="#2060c0", linewidth=1.5, zorder=3)

    # Connect to next segment with a straight line (naive turn)
    if i < len(all_segments) - 1:
        end_pt = seg_coords[-1]
        start_next = all_segments[i + 1][0]
        conn = LineString([end_pt, start_next])

        if prepared_poly_buf.contains(conn):
            # Turn stays inside — draw blue
            ax_a.plot([end_pt[0], start_next[0]], [end_pt[1], start_next[1]],
                      color="#2060c0", linewidth=1.5, zorder=3)
        else:
            # Check if it crosses the notch zone (deadhead)
            outside_part = conn.difference(poly)
            if not outside_part.is_empty and not outside_part.intersection(notch_zone).is_empty:
                ax_a.plot([end_pt[0], start_next[0]], [end_pt[1], start_next[1]],
                          color="#e03030", linewidth=1.8, zorder=3)
            else:
                ax_a.plot([end_pt[0], start_next[0]], [end_pt[1], start_next[1]],
                          color="#2060c0", linewidth=1.5, zorder=3)

ax_a.set_xlim(bounds[0] - pad, bounds[2] + pad)
ax_a.set_ylim(bounds[1] - pad, bounds[3] + pad)
ax_a.set_aspect("equal")
ax_a.axis("off")

# =====================================================================
# (b) Exact cellular decomposition using actual ConcaveDecomposer
# =====================================================================

cells = ConcaveDecomposer.decompose(poly, HEADING_DEG)

print(f"Decomposition produced {len(cells)} cells at heading {HEADING_DEG}")
for i, cell in enumerate(cells):
    print(f"  Cell {i+1}: area={cell.area:.1f}, vertices={len(cell.exterior.coords)-1}")

colors_fill = ["#cce5ff", "#ffe5cc", "#d5f5d0", "#f0d0ff", "#fffacd"]
colors_label = ["#1a4080", "#804020", "#1a6030", "#601080", "#806000"]

for idx, cell in enumerate(cells):
    c = colors_fill[idx % len(colors_fill)]
    xs, ys = cell.exterior.xy
    ax_b.fill(xs, ys, facecolor=c, edgecolor="black", linewidth=1.5, zorder=2)

    waypoints, dist, area, turns = planner.generate_path(
        cell, HEADING_DEG,
        global_y_origin=global_y_origin,
        rotation_origin=origin_pt,
    )

    if waypoints and len(waypoints) >= 2:
        wp = np.array(waypoints)
        ax_b.plot(wp[:, 0], wp[:, 1], color="#2060c0", linewidth=1.5, zorder=3)

    cx, cy = cell.centroid.x, cell.centroid.y
    lbl_color = colors_label[idx % len(colors_label)]
    ax_b.text(cx, cy, f"$R_{{{idx+1}}}$", ha="center", va="center",
              fontsize=14, fontweight="bold", color=lbl_color, zorder=5)

# Draw cut lines (cell edges not on original boundary)
original_boundary = poly.boundary
for cell in cells:
    cell_coords = list(cell.exterior.coords)
    for j in range(len(cell_coords) - 1):
        edge = LineString([cell_coords[j], cell_coords[j + 1]])
        mid = edge.interpolate(0.5, normalized=True)
        if original_boundary.distance(mid) > 0.3:
            xs_e, ys_e = edge.xy
            ax_b.plot(xs_e, ys_e, color="black", linewidth=2.2,
                      linestyle="--", zorder=4)

# Mark concave vertices on panel (b): Type-1 and Type-2
RAY_COLOR = "#8B008B"  # dark magenta — distinct from red deadhead and blue path
for idx in concave_indices:
    vx, vy = coords[idx]
    if idx in type2_indices:
        ax_b.plot(vx, vy, marker="*", color="#cc0000", markersize=14, zorder=6)

        # Ray arrow for T2 vertices
        hx, hy = np.cos(heading_rad), np.sin(heading_rad)
        interior_sign = ConcaveDecomposer._find_interior_direction(poly, vx, vy, hx, hy)
        if interior_sign is not None:
            dx = interior_sign * hx
            dy = interior_sign * hy
            hit_pt = ConcaveDecomposer._cast_ray_to_boundary(poly, vx, vy, dx, dy)
            if hit_pt is not None:
                ax_b.annotate("", xy=(hit_pt.x, hit_pt.y), xytext=(vx, vy),
                              arrowprops=dict(arrowstyle="-|>", color=RAY_COLOR,
                                              lw=2.0, linestyle="-.",
                                              mutation_scale=15),
                              zorder=5)
    else:
        ax_b.plot(vx, vy, marker="^", color="#cc8800", markersize=10, zorder=6)

ax_b.set_xlim(bounds[0] - pad, bounds[2] + pad)
ax_b.set_ylim(bounds[1] - pad, bounds[3] + pad)
ax_b.set_aspect("equal")
ax_b.axis("off")

# =====================================================================
# Shared legend below both panels
# =====================================================================

leg_working = mpatches.Patch(color="#2060c0", label="Рабочий маршрут")
leg_deadhead = mpatches.Patch(color="#e03030", label="Холостой перелёт")
leg_t1 = Line2D([], [], marker="^", color="#cc8800", linestyle="None",
                markersize=10, label="Вогнутая вершина (тип 1)")
leg_t2 = Line2D([], [], marker="*", color="#cc0000", linestyle="None",
                markersize=12, label="Вогнутая вершина (тип 2)")

leg_cells = [mpatches.Patch(facecolor=colors_fill[i % len(colors_fill)],
                            edgecolor="black",
                            label=f"Подобласть $R_{{{i+1}}}$")
             for i in range(len(cells))]
leg_cut = Line2D([], [], color="black", linewidth=2, linestyle="--",
                 label="Линия разреза")
leg_ray = Line2D([], [], color=RAY_COLOR, linestyle="-.", linewidth=2,
                 marker=">", markersize=6, label="Луч разреза")

all_handles = [leg_working, leg_deadhead, leg_t1, leg_t2] + leg_cells + [leg_cut, leg_ray]

fig.legend(handles=all_handles, loc="lower center",
           ncol=4, fontsize=10, framealpha=0.9,
           bbox_to_anchor=(0.5, -0.04),
           handlelength=2.5, handleheight=1.5,
           borderpad=1.0, labelspacing=0.8)

ax_a.set_title("(а) Бустрофедон на вогнутом полигоне", fontsize=11,
               pad=8, loc="center", y=-0.05)
ax_b.set_title("(б) Точная клеточная декомпозиция", fontsize=11,
               pad=8, loc="center", y=-0.05)

# =====================================================================
fig.tight_layout(pad=1.5)
fig.subplots_adjust(bottom=0.20)
out_path = "/home/mario/programacion/tesis/MISIS-thesis-private/docs/img/decomposition.png"
fig.savefig(out_path, dpi=300, bbox_inches="tight", facecolor="white")
plt.close(fig)
print(f"Saved: {out_path}")
