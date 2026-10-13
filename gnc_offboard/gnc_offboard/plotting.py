"""Matplotlib drawing helpers for the field map and flight paths.

Used by tools/plot_field.py and tools/plot_flight.py; import it in your own scripts
to draw a planner's output on the real field:

    import matplotlib.pyplot as plt
    from gnc_offboard.field_map import FieldMap
    from gnc_offboard.plotting import draw_field, draw_path, draw_violations

    fmap = FieldMap.load()
    fig, ax = plt.subplots(figsize=(8, 8))
    draw_field(ax, fmap, inflate=0.5)
    draw_path(ax, path, label='A*')
    draw_violations(ax, fmap.path_violations(path, inflate=0.5))
    ax.legend(); fig.savefig('path.png')
"""
import matplotlib.patches as mpatches
from matplotlib.lines import Line2D

# Categorical series, fixed order (validated palette, light mode). Max three per chart.
SERIES = ['#2a78d6', '#eb6834', '#1baf7a']
INK = '#0b0b0b'
INK_2 = '#52514e'
INK_3 = '#8a8984'
GRID = '#e4e3df'
SURFACE = '#fcfcfb'
OBSTACLE = '#6f6e69'
KEEPOUT = '#d8d7d2'
CRITICAL = '#c7302f'      # status: path violation (always with a marker + label)
DIE_COLORS = {'blue': '#1f4fd8', 'red': '#d81f1f', 'yellow': '#f2d21a', 'black': '#111111'}


def style_axes(ax, equal=False):
    ax.set_facecolor(SURFACE)
    ax.grid(True, color=GRID, linewidth=0.8)
    ax.set_axisbelow(True)
    for side in ('top', 'right'):
        ax.spines[side].set_visible(False)
    for side in ('left', 'bottom'):
        ax.spines[side].set_color(INK_3)
    ax.tick_params(colors=INK_2, labelsize=9)
    ax.xaxis.label.set_color(INK_2)
    ax.yaxis.label.set_color(INK_2)
    if equal:
        ax.set_aspect('equal', adjustable='box')


def draw_field(ax, fmap, inflate=0.5, label_obstacles=False, show_target=True):
    """Field boundary, start zone, obstacles with keep-out zones, dice. Field frame axes."""
    x0, y0, x1, y1 = fmap.bounds
    ax.add_patch(mpatches.Rectangle((x0, y0), x1 - x0, y1 - y0, fill=False,
                                    edgecolor=INK_2, linewidth=1.2, zorder=1))
    zx0, zy0, zx1, zy1 = fmap.start_zone
    ax.add_patch(mpatches.Rectangle((zx0, zy0), zx1 - zx0, zy1 - zy0, fill=False,
                                    edgecolor=INK_2, linewidth=1.0, linestyle='--', zorder=1))
    ax.annotate('start zone', (zx0 + (zx1 - zx0) / 2, zy1 if zy0 < 0 else zy0),
                xytext=(0, 4 if zy0 < 0 else -12), textcoords='offset points',
                ha='center', fontsize=8, color=INK_2)

    for ob in fmap.obstacles:
        if inflate > 0:
            ax.add_patch(mpatches.Polygon(ob.inflated_outline(inflate), closed=True,
                                          facecolor=KEEPOUT, edgecolor='none', alpha=0.7, zorder=2))
        if ob.shape == 'box':
            ax.add_patch(mpatches.Polygon(ob.corners(), closed=True, facecolor=OBSTACLE,
                                          edgecolor='none', zorder=3))
        else:
            ax.add_patch(mpatches.Circle((ob.x, ob.y), ob.r, facecolor=OBSTACLE,
                                         edgecolor='none', zorder=3))
        if label_obstacles:
            ax.annotate(ob.name.split('_', 2)[-1] if '_' in ob.name else ob.name,
                        (ob.x, ob.y), xytext=(0, -14), textcoords='offset points',
                        ha='center', fontsize=7, color=INK_3)

    target = fmap.scenario.get('initial_target') if show_target else None
    for t in fmap.targets:
        c = DIE_COLORS.get(t['color'], INK)
        ax.plot(t['x'], t['y'], marker='s', markersize=8, color=c,
                markeredgecolor=INK, markeredgewidth=0.6, linestyle='none', zorder=5)
        if t['color'] == target:
            ax.plot(t['x'], t['y'], marker='o', markersize=17, fillstyle='none',
                    color=INK, markeredgewidth=1.0, linestyle='none', zorder=5)
            ax.annotate(f"target ({t['color']})", (t['x'], t['y']), xytext=(10, 8),
                        textcoords='offset points', fontsize=8, color=INK)
    sx, sy = fmap.spawn
    ax.plot(sx, sy, marker='^', markersize=8, color=INK, linestyle='none', zorder=5)

    pad = 0.4
    ax.set_xlim(x0 - pad, x1 + pad)
    ax.set_ylim(y0 - pad, y1 + pad)
    ax.set_xlabel('x, east (m)')
    ax.set_ylabel('y, north (m)')
    style_axes(ax, equal=True)


def field_legend_handles(inflate):
    """Legend entries for the map symbols (append your path handles)."""
    h = [mpatches.Patch(facecolor=OBSTACLE, label='obstacle'),
         Line2D([], [], marker='s', color=INK_3, markeredgecolor=INK, linestyle='none',
                markersize=7, label='die'),
         Line2D([], [], marker='^', color=INK, linestyle='none', markersize=7, label='spawn')]
    if inflate > 0:
        h.insert(1, mpatches.Patch(facecolor=KEEPOUT, label=f'keep-out (+{inflate:.2f} m)'))
    return h


def draw_path(ax, pts, color=SERIES[0], label=None, linestyle='-', waypoints=True,
              linewidth=1.8, zorder=6):
    """Polyline [(x, y), ...]. With `waypoints`, vertices are dotted and numbered."""
    if len(pts) == 0:
        return
    xs, ys = [p[0] for p in pts], [p[1] for p in pts]
    ax.plot(xs, ys, color=color, linestyle=linestyle, linewidth=linewidth,
            label=label, zorder=zorder, solid_capstyle='round')
    if waypoints:
        ax.plot(xs, ys, marker='o', markersize=5, color=color, markeredgecolor=SURFACE,
                markeredgewidth=1.2, linestyle='none', zorder=zorder + 1)
        for i, (x, y) in enumerate(zip(xs, ys)):
            ax.annotate(str(i), (x, y), xytext=(5, 5), textcoords='offset points',
                        fontsize=8, color=INK_2, zorder=zorder + 1)


def draw_violations(ax, violations):
    """Mark the closest point of each violation with a red X and its clearance."""
    for v in violations:
        ax.plot(v['x'], v['y'], marker='X', markersize=10, color=CRITICAL,
                markeredgecolor=SURFACE, markeredgewidth=1.0, linestyle='none', zorder=9)
        ax.annotate(f"{v['clearance']:+.2f} m", (v['x'], v['y']), xytext=(7, -12),
                    textcoords='offset points', fontsize=8, color=CRITICAL, zorder=9)


def violation_handle():
    return Line2D([], [], marker='X', color=CRITICAL, linestyle='none', markersize=9,
                  label='too close (clearance)')
