"""GA statistics: 3D convergence plot (Li Fig. 13 style) and JSON export."""

import json
import os
import numpy as np


_RESULTS_DIR = os.path.join(os.path.dirname(__file__), '../../data/test_results')


def save_gen_stats_json(gen_stats: list[dict], field_name: str, drone_name: str = "",
                        tag: str = "ga"):
    """Save per-generation GA stats. Each tag gets its own file, overwritten each run.

    tag="li_ga"  -> li_ga_stats.json
    tag="headless" -> headless_stats.json
    """
    os.makedirs(_RESULTS_DIR, exist_ok=True)
    out_path = os.path.join(_RESULTS_DIR, f'{tag}_stats.json')
    payload = {
        'field': field_name,
        'drone': drone_name,
        'generations': len(gen_stats),
        'stats': gen_stats,
    }
    with open(out_path, 'w') as f:
        json.dump(payload, f, indent=2)
    print(f"GA stats saved to {os.path.relpath(out_path)}")


def plot_3d_convergence(gen_stats: list[dict], title: str):
    """
    Li et al. (2023) Figure 13 style 3D surface:
    X = mean heading angle per generation
    Y = mean flight distance per generation
    Z = mean fitness per generation
    """
    import matplotlib
    matplotlib.use('TkAgg')
    import matplotlib.pyplot as plt
    from scipy.interpolate import griddata
    from matplotlib import cm

    mean_angle = np.array([s['mean_angle'] for s in gen_stats])
    mean_l = np.array([s['mean_l'] for s in gen_stats])
    mean_fitness = np.array([s['mean_fitness'] for s in gen_stats])

    grid_res = 40
    xi = np.linspace(mean_angle.min(), mean_angle.max(), grid_res)
    yi = np.linspace(mean_l.min(), mean_l.max(), grid_res)
    Xi, Yi = np.meshgrid(xi, yi)

    Zi = griddata((mean_angle, mean_l), mean_fitness, (Xi, Yi), method='cubic')
    mask = np.isnan(Zi)
    if mask.any():
        Zi_nearest = griddata((mean_angle, mean_l), mean_fitness, (Xi, Yi), method='nearest')
        Zi[mask] = Zi_nearest[mask]

    norm = plt.Normalize(vmin=mean_fitness.min(), vmax=mean_fitness.max())
    colors = cm.jet(norm(Zi))

    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111, projection='3d')

    ax.plot_surface(Xi, Yi, Zi, facecolors=colors, alpha=0.9,
                    rstride=1, cstride=1, linewidth=0.1,
                    edgecolor='gray', shade=True)

    mappable = cm.ScalarMappable(norm=norm, cmap='jet')
    mappable.set_array([])
    cb = fig.colorbar(mappable, ax=ax, shrink=0.6, pad=0.08)
    cb.set_label('Z-axis', fontsize=11)

    ax.set_xlabel('Mean value of solution\nper generation (°)', labelpad=12)
    ax.set_ylabel('Mean value of flight distance\nper generation', labelpad=12)
    ax.set_zlabel('Mean value of fitness function\nper generation', labelpad=12)
    ax.set_title(f'{title} — GA Convergence (Li Fig. 13 style)', fontsize=13, pad=10)
    ax.view_init(elev=30, azim=220)

    fig.canvas.mpl_connect('key_press_event', lambda e: plt.close('all') if e.key == 'q' else None)
    plt.tight_layout()
    plt.show()
