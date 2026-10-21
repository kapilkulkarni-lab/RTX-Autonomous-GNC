"""Export searched ground as a north-up full-field map, independent of Gazebo UI."""
from pathlib import Path

import numpy as np


def coverage_summary(coverage, manifest):
    """Report clipped cell area; older exports can only estimate target coverage."""
    xmin, ymin, xmax, ymax = coverage['bounds']
    half = coverage['resolution_m']/2
    area = 0.0
    target = next((t for t in manifest.get('targets', [])
                   if t['color'] == manifest.get('scenario', {}).get('initial_target')), None)
    target_in_grid = False
    for x, y in coverage['searched_cell_centres']:
        left, right = max(xmin, x-half), min(xmax, x+half)
        bottom, top = max(ymin, y-half), min(ymax, y+half)
        area += max(0, right-left)*max(0, top-bottom)
        if target is not None:
            target_in_grid |= left <= target['x'] <= right and bottom <= target['y'] <= top
    arena = (xmax-xmin)*(ymax-ymin)
    if target is None:
        status = 'Target: not configured'
    elif 'target_found' not in coverage:
        status = f"Target ({target['color']}): {'inside' if target_in_grid else 'outside'} searched grid (estimate)"
    elif coverage['target_found'] is None:
        status = f"Target ({target['color']}): unknown — no valid observations"
    else:
        result = 'YES' if coverage['target_found'] else 'NO'
        status = f"Target found ({target['color']}): {result}"
    return area, arena, 100*area/arena, status


def save_coverage_png(coverage, manifest_path, output):
    # Use a standalone Agg canvas so export also works without a display.
    from matplotlib.figure import Figure
    from matplotlib.backends.backend_agg import FigureCanvasAgg
    from matplotlib.collections import PolyCollection
    from gnc_offboard.field_map import FieldMap
    from gnc_offboard.plotting import draw_field

    fmap = FieldMap(coverage['manifest']) if 'manifest' in coverage else FieldMap.load(manifest_path)
    if not np.allclose(fmap.bounds, coverage['bounds']):
        raise ValueError('Coverage bounds differ from the field manifest')
    fig = Figure(figsize=(9, 10), facecolor='white')
    FigureCanvasAgg(fig)
    ax = fig.add_subplot(111)
    draw_field(ax, fmap, inflate=0)
    half = coverage['resolution_m']/2
    xmin, ymin, xmax, ymax = coverage['bounds']
    cells = []
    for x, y in coverage['searched_cell_centres']:
        left, right = max(xmin, x-half), min(xmax, x+half)
        bottom, top = max(ymin, y-half), min(ymax, y+half)
        cells.append([(left, bottom), (right, bottom), (right, top), (left, top)])
    ax.add_collection(PolyCollection(cells, facecolors='#ffad42', edgecolors='none',
                                     alpha=.55, zorder=2))
    trajectory = coverage.get('trajectory_enu', [])
    if trajectory:
        xy = np.array([p[:2] if p is not None else [np.nan, np.nan] for p in trajectory])
        ax.plot(xy[:, 0], xy[:, 1], color='#1757a6', linestyle=':', linewidth=2,
                dash_capstyle='round', zorder=6)
    area, arena, percent, status = coverage_summary(coverage, fmap.manifest)
    fig.text(.11, .965, 'Camera coverage · bird’s-eye view', fontsize=17, weight='bold')
    fig.text(.11, .932, f'Arena covered: {percent:.1f}%  ·  approx. {area:.2f} / {arena:.2f} m²',
             fontsize=12)
    fig.text(.11, .902, status, fontsize=11)
    path_note = 'blue dotted = recorded drone path' if trajectory else 'drone path not recorded in this export'
    fig.text(.11, .875, f'Orange = searched ground; {path_note}.', fontsize=10, color='#52514e')
    fig.text(.5, .035, 'North is up · Flat-ground projection.',
             ha='center', fontsize=9, color='#52514e')
    fig.subplots_adjust(left=.11, right=.97, bottom=.10, top=.845)
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=160)
    fig.clear()
    return output
