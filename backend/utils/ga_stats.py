"""GA statistics: convergence plot and JSON export."""
import json
import os
import numpy as np

_RESULTS_DIR = os.path.join(os.path.dirname(__file__), '../../data/test_results')


def save_gen_stats_json(gen_stats: list[dict], field_name: str, drone_name: str = "",
                        tag: str = "ga"):
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


def plot_2d_convergence(gen_stats: list[dict], title: str):
    """
    Convergence plot estilo Li Fig. 12:
      X = generación
      Y = mean_angle  (línea principal, color degradado)
      segunda línea   = mean_fitness (eje derecho)
    """
    import matplotlib
    matplotlib.use('TkAgg')
    import matplotlib.pyplot as plt
    from matplotlib.collections import LineCollection
    import matplotlib.cm as cm

    gens        = np.array([s['gen']          for s in gen_stats])
    mean_angle  = np.array([s['mean_angle']   for s in gen_stats])
    mean_fit    = np.array([s['mean_fitness'] for s in gen_stats])

    # --- degradado de color en la línea de ángulo ---
    points  = np.array([gens, mean_angle]).T.reshape(-1, 1, 2)
    segs    = np.concatenate([points[:-1], points[1:]], axis=1)
    norm    = plt.Normalize(mean_angle.min(), mean_angle.max())
    lc      = LineCollection(segs, cmap='jet', norm=norm, linewidth=1.8, zorder=3)
    lc.set_array(mean_angle[:-1])

    fig, ax1 = plt.subplots(figsize=(10, 5), facecolor='white')
    ax1.set_facecolor('white')

    ax1.add_collection(lc)
    ax1.autoscale()

    # puntos encima de la línea
    sc = ax1.scatter(gens, mean_angle, c=mean_angle, cmap='jet',
                     s=10, zorder=4, edgecolors='none')

    # eje derecho: fitness
    ax2 = ax1.twinx()
    ax2.plot(gens, mean_fit, color='gray', linewidth=1.2,
             linestyle='--', alpha=0.7, label='Mean fitness')
    ax2.set_ylabel('Mean fitness per generation', fontsize=10, color='gray')
    ax2.tick_params(axis='y', labelcolor='gray')

    # colorbar
    cb = fig.colorbar(sc, ax=ax1, pad=0.12, shrink=0.85)
    cb.set_label('Value of solution (°)', fontsize=10)

    ax1.set_xlabel('Iterations', fontsize=11)
    ax1.set_ylabel('Value of solution (°)', fontsize=11)
    ax1.set_title(f'{title} — GA Convergence', fontsize=12)
    ax1.grid(True, linestyle='--', linewidth=0.4, color='lightgray')

    # leyenda combinada
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax1.legend(lines2, labels2, loc='upper right', fontsize=9)

    fig.tight_layout()
    fig.canvas.mpl_connect(
        'key_press_event',
        lambda e: plt.close('all') if e.key == 'q' else None,
    )
    plt.show()


plot_3d_convergence = plot_2d_convergence