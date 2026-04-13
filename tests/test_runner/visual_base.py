import os
import json
import matplotlib
matplotlib.use('TkAgg')
from matplotlib.widgets import Slider
from shapely.geometry import Polygon
from shapely.geometry.polygon import orient

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FIELDS_DIR = os.path.join(ROOT, 'test_fields')

# -- Colors --
BG = '#16213e'
PANEL_BG = '#1a1a2e'
CELLS = ['#8E44AD', '#16A085', '#E74C3C', '#2980B9', '#D35400', '#1ABC9C',
         '#F39C12', '#C0392B', '#27AE60', '#2C3E50']


def load_json(path: str) -> tuple:
    """Load polygon from JSON file. Returns (polygon, name)."""
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


def draw_poly(ax, poly, fc, alpha=0.25, ec=None, lw=2):
    """Draw a polygon with optional holes."""
    x, y = poly.exterior.xy
    ax.fill(x, y, fc=fc, alpha=alpha, ec='none')
    ax.plot(x, y, color=ec or fc, lw=lw)
    for interior in poly.interiors:
        ix, iy = interior.xy
        ax.fill(ix, iy, fc='white', alpha=1, ec='grey', lw=1)


def setup_dark_ax(ax):
    """Apply dark theme to an axes."""
    ax.set_facecolor(BG)
    ax.tick_params(colors='#7f8c8d')
    for sp in ax.spines.values():
        sp.set_color('#2d2d44')


def make_slider(fig, rect, label, vmin, vmax, valinit, valstep, color):
    """Create a styled slider on the figure. Returns the Slider widget."""
    ax = fig.add_axes(rect)
    ax.set_facecolor('#2d2d44')
    sl = Slider(ax, label, vmin, vmax, valinit=valinit, valstep=valstep, color=color)
    sl.label.set_color('white')
    sl.valtext.set_color('white')
    return sl


def fix_axes_to_bounds(ax, polygon, pad_frac=0.1):
    """Set axes limits to polygon bounds with padding."""
    minx, miny, maxx, maxy = polygon.bounds
    pw = (maxx - minx) * pad_frac or 10
    ph = (maxy - miny) * pad_frac or 10
    ax.set_xlim(minx - pw, maxx + pw)
    ax.set_ylim(miny - ph, maxy + ph)


def resolve_json_path(json_arg):
    """Resolve a JSON path argument, falling back to field_picker if None."""
    from field_picker import pick_json
    path = json_arg or pick_json(ROOT)
    if not os.path.isabs(path):
        path = os.path.abspath(path)
    return path
