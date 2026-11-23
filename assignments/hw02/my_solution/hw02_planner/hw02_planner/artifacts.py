"""Save the received map, planning graph and figures without importing ROS."""

from pathlib import Path

import numpy as np


def save_planning_artifacts(result, output_dir):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.collections import LineCollection

    output = Path(output_dir)
    output.mkdir(parents=True, exist_ok=True)
    space = result.space
    edges = np.asarray(result.edges, dtype=np.int64).reshape(-1, 2)
    np.savez_compressed(
        output / 'planning.npz',
        raw_map=space.raw,
        cspace=space.blocked,
        resolution=space.resolution,
        origin=np.asarray(space.origin),
        yaw=space.yaw,
        planning_radius=space.radius,
        nodes=result.nodes,
        edges=edges,
        path=result.path,
    )
    height, width = space.blocked.shape
    columns, rows = np.meshgrid(np.arange(width + 1), np.arange(height + 1))
    corners = space.grid_to_world(np.stack((columns, rows), axis=-1))
    fig, axes = plt.subplots(1, 2, figsize=(13, 5.3), constrained_layout=True)
    for axis, grid, title in zip(
            axes, (space.raw != 0, space.blocked),
            ('Physical occupancy map', 'Configuration space, PRM and A*')):
        axis.pcolormesh(corners[..., 0], corners[..., 1], grid,
                        cmap='Greys', vmin=0, vmax=1, shading='flat', rasterized=True)
        axis.scatter(*result.nodes[0], c='#16814a', s=70, label='Start', zorder=5)
        axis.scatter(*result.nodes[1], c='#bd2026', marker='*', s=130, label='Goal', zorder=5)
        axis.set(title=title, xlabel='map X (m)', ylabel='map Y (m)', aspect='equal')
    if len(edges):
        axes[1].add_collection(LineCollection(result.nodes[edges], colors='#4596bb',
                                               linewidths=0.35, alpha=0.25))
    axes[1].scatter(result.nodes[2:, 0], result.nodes[2:, 1], s=3,
                    c='#2874a6', alpha=0.65, label='PRM samples')
    if len(result.path):
        axes[1].plot(result.path[:, 0], result.path[:, 1], color='#d62728',
                     linewidth=2.2, label='A* path', zorder=4)
    else:
        axes[1].text(0.5, 1.02, 'No roadmap path found within sampling budget',
                     transform=axes[1].transAxes, ha='center', fontsize=9)
    axes[1].legend(loc='upper right', fontsize=8)
    fig.savefig(output / 'planning.png', dpi=180)
    plt.close(fig)
