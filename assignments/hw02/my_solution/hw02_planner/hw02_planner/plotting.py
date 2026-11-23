"""Save report figures. Uses origin='lower' so the picture matches Gazebo/RViz."""
import matplotlib

matplotlib.use('Agg')                    # draw to files, no window needed
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402


def _extent(grid):
    return (*grid.x_range, *grid.y_range)


def _draw_map(axes, raw, cspace):
    # light grey = cells closed only by the robot radius; black = physical obstacles
    picture = np.zeros(raw.occupied.shape)
    picture[cspace.occupied] = 0.35
    picture[raw.occupied] = 1.0
    axes.imshow(picture, origin='lower', cmap='gray_r', vmin=0, vmax=1, extent=_extent(raw))
    axes.set_xlabel('map X (m)')
    axes.set_ylabel('map Y (m)')
    axes.set_aspect('equal')


def save_plan_figure(filename, raw, result, start, goal, title):
    figure, axes = plt.subplots(figsize=(10, 7.5))
    _draw_map(axes, raw, result.cspace)

    roadmap = result.roadmap
    if roadmap is not None:
        for a, b in roadmap.edges():
            (x1, y1), (x2, y2) = roadmap.nodes[a], roadmap.nodes[b]
            axes.plot([x1, x2], [y1, y2], color='tab:blue', linewidth=0.3, alpha=0.35)
        axes.scatter(roadmap.nodes[2:, 0], roadmap.nodes[2:, 1], s=3, color='tab:blue',
                     label=f'roadmap ({len(roadmap.nodes)} nodes, {len(roadmap.edges())} edges)')

    if result.found:
        path = np.array(result.waypoints)
        axes.plot(path[:, 0], path[:, 1], '-o', color='tab:orange', linewidth=2.5,
                  markersize=4, label=f'A* path ({result.length:.2f} m)')
    else:
        axes.text(0.5, 0.02, 'no path found: ' + result.problem, transform=axes.transAxes,
                  ha='center', color='tab:red', fontsize=11,
                  bbox=dict(facecolor='white', edgecolor='tab:red'))

    axes.plot(*start, 'o', color='tab:green', markersize=10, label='start')
    axes.plot(*goal, '*', color='tab:red', markersize=15, label='goal')
    axes.set_title(title)
    axes.legend(loc='upper center', bbox_to_anchor=(0.5, -0.08), ncol=4, fontsize=8)
    figure.tight_layout()
    figure.savefig(filename, dpi=150)
    plt.close(figure)


def save_cspace_comparison(filename, raw, cspace, title):
    figure, (left, right) = plt.subplots(1, 2, figsize=(14, 5.5))
    left.imshow(raw.occupied, origin='lower', cmap='gray_r', extent=_extent(raw))
    left.set_title('Physical map (robot centre is a point)')
    right.imshow(cspace.occupied, origin='lower', cmap='gray_r', extent=_extent(cspace))
    right.set_title('Configuration space (obstacles grown by 0.30 m)')
    for axes in (left, right):
        axes.set_xlabel('map X (m)')
        axes.set_ylabel('map Y (m)')
        axes.set_aspect('equal')
    figure.suptitle(title)
    figure.tight_layout()
    figure.savefig(filename, dpi=150)
    plt.close(figure)
