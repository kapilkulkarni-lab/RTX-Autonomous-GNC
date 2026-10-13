#!/usr/bin/env python3
"""Draw the AVC field from a world manifest, optionally with a path checked for clearance.

    tools/plot_field.py                                   # current world
    tools/plot_field.py --waypoints "-4.5,-4.5, 0,0, 3,2" # field x,y pairs (metres)
    tools/plot_field.py --path my_path.json --inflate 0.6
    tools/plot_field.py --manifest worlds/other_manifest.json --show

--path takes JSON ([[x, y], ...] or {"path": [[x, y], ...]}) or CSV with x,y columns.
The path is checked against every obstacle grown by --inflate (drone radius plus
margin) and the field edge; problems are printed and marked with a red X.
Exit code is 1 if the path has violations, so scripts and CI can use it.
"""
import argparse
import csv
import json
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / 'gnc_offboard'))   # works with or without the ROS workspace

import matplotlib  # noqa: E402

from gnc_offboard.field_map import DEFAULT_INFLATE, FieldMap, default_manifest_path  # noqa: E402


def parse_waypoints(text):
    vals = [float(v) for v in text.replace(';', ',').split(',') if v.strip()]
    if len(vals) % 2:
        raise SystemExit('--waypoints needs x,y pairs')
    return list(zip(vals[0::2], vals[1::2]))


def load_path(path):
    path = Path(path)
    if path.suffix.lower() == '.json':
        data = json.loads(path.read_text())
        pts = data['path'] if isinstance(data, dict) else data
        return [(float(p[0]), float(p[1])) for p in pts]
    with path.open() as fh:
        rows = list(csv.reader(fh))
    if rows and not rows[0][0].lstrip('-').replace('.', '', 1).isdigit():
        rows = rows[1:]                                  # header
    return [(float(r[0]), float(r[1])) for r in rows if r]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--manifest', default=None, help=f'default: {default_manifest_path()}')
    ap.add_argument('--waypoints', help='field-frame x,y pairs, e.g. "-4.5,-4.5,0,0"')
    ap.add_argument('--path', help='JSON or CSV file of x,y points')
    ap.add_argument('--inflate', type=float, default=DEFAULT_INFLATE,
                    help=f'keep-out distance from obstacles, m (default {DEFAULT_INFLATE})')
    ap.add_argument('--labels', action='store_true', help='label obstacles by kind')
    ap.add_argument('-o', '--out', help='output PNG (default: plots/field_<world>.png)')
    ap.add_argument('--show', action='store_true', help='also open a window')
    args = ap.parse_args()

    if not args.show:
        matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from gnc_offboard.plotting import (INK, SERIES, draw_field, draw_path, draw_violations,
                                       field_legend_handles, violation_handle)

    fmap = FieldMap.load(args.manifest)
    path = parse_waypoints(args.waypoints) if args.waypoints else \
        load_path(args.path) if args.path else None

    fig, ax = plt.subplots(figsize=(8.5, 8.5))
    draw_field(ax, fmap, inflate=args.inflate, label_obstacles=args.labels)
    handles = field_legend_handles(args.inflate)

    violations = []
    if path:
        draw_path(ax, path, color=SERIES[0], label='path')
        violations = fmap.path_violations(path, inflate=args.inflate)
        draw_violations(ax, violations)
        handles.append(plt.Line2D([], [], color=SERIES[0], marker='o', label='path'))
        if violations:
            handles.append(violation_handle())

    ax.legend(handles=handles, loc='upper left', bbox_to_anchor=(1.01, 1.0),
              frameon=False, fontsize=9)
    status = ''
    if path:
        status = (f' — {len(violations)} violation(s)' if violations else ' — path clear')
    ax.set_title(f'{fmap.name}  ·  challenge {fmap.challenge}  ·  seed {fmap.seed}'
                 f'  ·  {len(fmap.obstacles)} obstacles{status}',
                 fontsize=11, color=INK, loc='left')
    fig.tight_layout()

    out = Path(args.out) if args.out else REPO / 'plots' / f'field_{fmap.name}.png'
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=140, facecolor='white', bbox_inches='tight')
    print(f'wrote {out}')

    if path:
        length = sum(((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2) ** 0.5 for a, b in zip(path, path[1:]))
        print(f'path: {len(path)} points, {length:.2f} m, keep-out {args.inflate:.2f} m')
        if violations:
            print('VIOLATIONS (segment i runs from point i to i+1):')
            for v in violations:
                print(f"  segment {v['segment']}: {v['obstacle']:<22} clearance {v['clearance']:+.2f} m"
                      f"  at ({v['x']:.2f}, {v['y']:.2f})")
        else:
            print('path clear')
    if args.show:
        plt.show()
    return 1 if violations else 0


if __name__ == '__main__':
    sys.exit(main())
