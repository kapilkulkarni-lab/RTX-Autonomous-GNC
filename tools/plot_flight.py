#!/usr/bin/env python3
"""Plot a PX4 flight log: path on the field map, altitude, speed and tracking error.

    tools/plot_flight.py                         # newest log from the PX4 SITL log folder
    tools/plot_flight.py path/to/log.ulg
    tools/plot_flight.py cubic.ulg carrot.ulg --labels cubic,carrot   # compare (max 3)
    tools/plot_flight.py --ned                   # no field map, PX4 local frame

The time window is the offboard period (what your node flew); --full plots the whole
log. Setpoint = what PX4's position controller tracked (vehicle_local_position_setpoint).
Tracking error = distance between that setpoint and the estimated position.

The map uses the current world manifest. PX4's local frame starts at the spawn
point, so the map only lines up if the world hasn't been regenerated since the
flight; the tool warns when the manifest is newer than the log.
"""
import argparse
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / 'gnc_offboard'))   # works with or without the ROS workspace

import matplotlib  # noqa: E402

from gnc_offboard.field_map import FieldMap, default_manifest_path  # noqa: E402
from gnc_offboard.flight_log import latest_ulog, load_ulog, log_dir  # noqa: E402

MAX_LOGS = 3
STYLES = ['-', '--', ':']   # second cue besides color when comparing


def print_metrics(flights, labels):
    cols = [('duration_s', 'time s', '{:.1f}'), ('path_length_m', 'path m', '{:.2f}'),
            ('rms_error_m', 'RMS err m', '{:.3f}'), ('max_error_m', 'max err m', '{:.3f}'),
            ('max_speed_mps', 'max v m/s', '{:.2f}'), ('max_accel_mps2', 'p99 a m/s²', '{:.2f}')]
    w = max(8, *(len(lb) for lb in labels))
    print(f"{'flight':<{w}}  window    " + '  '.join(f'{h:>10}' for _, h, _ in cols))
    for f, lb in zip(flights, labels):
        m = f.metrics()
        print(f'{lb:<{w}}  {f.mode:<8}  ' + '  '.join(f'{fmt.format(m[k]):>10}' for k, _, fmt in cols))


def make_figure(flights, labels, fmap=None, inflate=0.5):
    """Build the figure for 1 flight (actual vs setpoint) or 2-3 flights (overlaid)."""
    import matplotlib.pyplot as plt
    from gnc_offboard.plotting import (INK, INK_2, SERIES, draw_field, draw_path,
                                       field_legend_handles, style_axes)
    compare = len(flights) > 1

    def xy(arr):   # map axes: x east, y north in both frames
        return (arr[:, 0], arr[:, 1]) if fmap else (arr[:, 1], arr[:, 0])

    fig = plt.figure(figsize=(14, 8.2))
    gs = fig.add_gridspec(3, 2, width_ratios=[1.1, 1], hspace=0.38, wspace=0.18)
    ax_map = fig.add_subplot(gs[:, 0])
    ax_alt = fig.add_subplot(gs[0, 1])
    ax_spd = fig.add_subplot(gs[1, 1], sharex=ax_alt)
    ax_err = fig.add_subplot(gs[2, 1], sharex=ax_alt)

    # ---- map
    handles = []
    if fmap:
        draw_field(ax_map, fmap, inflate=inflate, show_target=not compare)
        handles = field_legend_handles(inflate)
    else:
        ax_map.set_xlabel('east (m)')
        ax_map.set_ylabel('north (m)')
        style_axes(ax_map, equal=True)
    for i, (f, lb) in enumerate(zip(flights, labels)):
        c = SERIES[i]
        if not compare and len(f.sp_t):
            sx, sy = xy(f.sp_pos)
            ax_map.plot(sx, sy, color=SERIES[1], linestyle='--', linewidth=1.4,
                        label='setpoint', zorder=6)
        x, y = xy(f.pos)
        draw_path(ax_map, list(zip(x, y)), color=c, waypoints=False,
                  label=lb if compare else 'actual', zorder=7 + i,
                  linestyle=STYLES[i] if compare else '-')
        ax_map.plot(x[0], y[0], marker='o', markersize=6, color=c, linestyle='none', zorder=8)
        ax_map.plot(x[-1], y[-1], marker='s', markersize=6, color=c, linestyle='none', zorder=8)
    if not fmap:
        ax_map.autoscale()
        ax_map.margins(0.08)
    line_handles, _ = ax_map.get_legend_handles_labels()
    ax_map.legend(handles=line_handles + handles, loc='upper center', bbox_to_anchor=(0.5, -0.09),
                  ncol=4, fontsize=8, frameon=False)

    # ---- time series
    for i, (f, lb) in enumerate(zip(flights, labels)):
        c = SERIES[i]
        name = lb if compare else 'actual'
        ls = STYLES[i] if compare else '-'
        ax_alt.plot(f.t, f.pos[:, 2], color=c, linestyle=ls, linewidth=1.6, label=name)
        ax_spd.plot(f.t, f.speed(), color=c, linestyle=ls, linewidth=1.6, label=name)
        ax_err.plot(f.t, f.tracking_error(), color=c, linestyle=ls, linewidth=1.6, label=name)
        if not compare and len(f.sp_t):
            ax_alt.plot(f.sp_t, f.sp_pos[:, 2], color=SERIES[1], linestyle='--',
                        linewidth=1.4, label='setpoint')
            ax_spd.plot(f.sp_t, f.sp_speed(), color=SERIES[1], linestyle='--',
                        linewidth=1.4, label='setpoint')
    ax_alt.set_ylabel('altitude (m)')
    ax_spd.set_ylabel('horizontal speed (m/s)')
    ax_err.set_ylabel('tracking error (m)')
    ax_err.set_xlabel(f'time (s), from start of {flights[0].mode} window')
    for ax in (ax_alt, ax_spd, ax_err):
        style_axes(ax)
        ax.set_ylim(bottom=0)
    ax_alt.legend(fontsize=8, frameon=False, loc='best')
    for ax in (ax_alt, ax_spd):
        plt.setp(ax.get_xticklabels(), visible=False)

    # ---- title with headline numbers
    def summary(f):
        m = f.metrics()
        return f"RMS err {m['rms_error_m']:.2f} m · max {m['max_error_m']:.2f} m · {m['duration_s']:.0f} s"
    if compare:
        title = '  vs  '.join(labels)
        sub = '   |   '.join(f'{lb}: {summary(f)}' for f, lb in zip(flights, labels))
    else:
        title = f'{labels[0]}  ({flights[0].mode} window)'
        sub = summary(flights[0])
    fig.suptitle(title, x=0.06, ha='left', fontsize=12, color=INK, y=0.985)
    fig.text(0.06, 0.945, sub, ha='left', fontsize=9.5, color=INK_2)
    return fig


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('logs', nargs='*', help=f'.ulg files (default: newest under {log_dir()})')
    ap.add_argument('--labels', help='comma-separated names for the logs')
    ap.add_argument('--manifest', default=None, help=f'default: {default_manifest_path()}')
    ap.add_argument('--ned', action='store_true', help='PX4 local frame, no field map')
    ap.add_argument('--inflate', type=float, default=0.5, help='keep-out shown on the map, m')
    ap.add_argument('--full', action='store_true', help='whole log, not just the offboard part')
    ap.add_argument('-o', '--out', help='output PNG (default: plots/flight_<log>.png)')
    ap.add_argument('--show', action='store_true', help='also open a window')
    args = ap.parse_args()

    if not args.show:
        matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    paths = [Path(p) for p in args.logs] or [latest_ulog()]
    if len(paths) > MAX_LOGS:
        raise SystemExit(f'compare at most {MAX_LOGS} logs at once')
    labels = args.labels.split(',') if args.labels else [p.stem for p in paths]
    if len(labels) != len(paths):
        raise SystemExit('--labels needs one name per log')

    fmap = None
    if not args.ned:
        mpath = Path(args.manifest) if args.manifest else default_manifest_path()
        if mpath.exists():
            fmap = FieldMap.load(mpath)
            if mpath.stat().st_mtime > max(p.stat().st_mtime for p in paths):
                print(f'warning: {mpath.name} is newer than the log; if the world was regenerated '
                      'since this flight, the map will not line up (use --ned)')
        else:
            print(f'no manifest at {mpath}; plotting in PX4 local frame')

    flights = [load_ulog(p, frame=fmap.frame if fmap else None, full=args.full) for p in paths]
    for f in flights:
        if len(f.t) == 0:
            raise SystemExit(
                f'{f.name}: no valid position estimate in the {f.mode} window '
                '(vehicle_local_position xy_valid/z_valid never true). PX4 had no '
                'horizontal position source: GPS off and no vision/flow fused yet.')

    fig = make_figure(flights, labels, fmap, args.inflate)
    compare = len(flights) > 1
    out = Path(args.out) if args.out else \
        REPO / 'plots' / ('compare_' + '_vs_'.join(labels) if compare else f'flight_{labels[0]}')
    out = out.with_suffix('.png')
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=130, facecolor='white', bbox_inches='tight')
    print(f'wrote {out}')
    print_metrics(flights, labels)
    if args.show:
        plt.show()
    return 0


if __name__ == '__main__':
    sys.exit(main())
