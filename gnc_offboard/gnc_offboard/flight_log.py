"""Read a PX4 ULog into plain arrays and compute tracking metrics. No ROS imports.

    from gnc_offboard.flight_log import load_ulog, latest_ulog
    f = load_ulog(latest_ulog())
    print(f.metrics())

Actual state comes from `vehicle_local_position`. The setpoint is what PX4's
position controller actually tracked (`vehicle_local_position_setpoint`), which
is fully populated in every mode; if a log lacks it, `trajectory_setpoint` (what
an offboard node sent) is used instead.

Times are seconds from the start of the analysis window: the offboard period if
there is one, otherwise the armed period, otherwise the whole log.
"""
import os
from pathlib import Path

import numpy as np

NAV_OFFBOARD = 14     # VehicleStatus.NAVIGATION_STATE_OFFBOARD
ARMED = 2             # VehicleStatus.ARMING_STATE_ARMED


def log_dir():
    px4 = os.environ.get('PX4_DIR', str(Path.home() / 'dev/PX4-Autopilot'))
    return Path(px4) / 'build' / 'px4_sitl_default' / 'rootfs' / 'log'


def latest_ulog(directory=None):
    logs = sorted(Path(directory or log_dir()).rglob('*.ulg'), key=lambda p: p.stat().st_mtime)
    if not logs:
        raise FileNotFoundError(f'no .ulg files under {directory or log_dir()}')
    return logs[-1]


def _windows(t, flag):
    """[(start, end), ...] time spans where boolean `flag` is true (t sorted)."""
    spans, start = [], None
    for ti, f in zip(t, flag):
        if f and start is None:
            start = ti
        elif not f and start is not None:
            spans.append((start, ti))
            start = None
    if start is not None:
        spans.append((start, t[-1]))
    return spans


class Flight:
    """Arrays in the field frame if a FieldFrame is given, else PX4 local NED.

    pos, sp_pos: (N, 3) positions [x, y, alt] (field: east, north, up) or [n, e, alt]
    vel, sp_vel: (N, 3) velocities in the same axes (alt rate positive up)
    t, sp_t: seconds from window start
    """

    def __init__(self, name, t, pos, vel, sp_t, sp_pos, sp_vel, window, mode,
                 offboard_spans=(), frame='ned', setpoint_topic=''):
        self.name, self.mode, self.frame, self.setpoint_topic = name, mode, frame, setpoint_topic
        self.window = window
        self.t, self.pos, self.vel = t, pos, vel
        self.sp_t, self.sp_pos, self.sp_vel = sp_t, sp_pos, sp_vel
        self.offboard_spans = list(offboard_spans)

    # ------------------------------------------------------------- derived series
    def speed(self):
        return np.hypot(self.vel[:, 0], self.vel[:, 1])

    def sp_speed(self):
        return np.hypot(self.sp_vel[:, 0], self.sp_vel[:, 1])

    def sp_at_t(self):
        """Setpoint position interpolated onto the actual-state timestamps (NaN where absent)."""
        out = np.full_like(self.pos, np.nan)
        if len(self.sp_t) < 2:
            return out
        for k in range(3):
            ok = np.isfinite(self.sp_pos[:, k])
            if ok.sum() >= 2:
                out[:, k] = np.interp(self.t, self.sp_t[ok], self.sp_pos[ok, k],
                                      left=np.nan, right=np.nan)
        return out

    def tracking_error(self):
        """3-D distance between actual position and setpoint, per actual sample."""
        return np.linalg.norm(self.pos - self.sp_at_t(), axis=1)

    def metrics(self):
        err = self.tracking_error()
        e = err[np.isfinite(err)]
        dt = np.diff(self.t)
        hacc = np.hypot(*(np.diff(self.vel[:, :2], axis=0) / np.maximum(dt, 1e-3)[:, None]).T) \
            if len(dt) else np.array([np.nan])
        return {
            'duration_s': float(self.t[-1] - self.t[0]) if len(self.t) else 0.0,
            'path_length_m': float(np.nansum(np.linalg.norm(np.diff(self.pos[:, :2], axis=0), axis=1))),
            'rms_error_m': float(np.sqrt(np.mean(e ** 2))) if len(e) else float('nan'),
            'max_error_m': float(e.max()) if len(e) else float('nan'),
            'max_speed_mps': float(np.nanmax(self.speed())) if len(self.t) else float('nan'),
            'max_accel_mps2': float(np.nanpercentile(hacc, 99)) if len(hacc) else float('nan'),
        }


def _ned_to_frame(ned, vel_ned, frame):
    """NED (n, e, d) arrays -> [x, y, alt]. With a FieldFrame, x/y are field east/north."""
    alt, valt = -ned[:, 2], -vel_ned[:, 2]
    if frame is None:
        return np.column_stack([ned[:, 0], ned[:, 1], alt]), \
            np.column_stack([vel_ned[:, 0], vel_ned[:, 1], valt])
    x, y = ned[:, 1] + frame.sx, ned[:, 0] + frame.sy
    return np.column_stack([x, y, alt]), np.column_stack([vel_ned[:, 1], vel_ned[:, 0], valt])


def load_ulog(path, frame=None, full=False):
    """Load a ULog. `frame` is a FieldFrame (field coordinates) or None (local NED)."""
    from pyulog import ULog   # imported here so the rest of the package works without it

    path = Path(path)
    topics = ['vehicle_local_position', 'vehicle_local_position_setpoint',
              'trajectory_setpoint', 'vehicle_status']
    ulog = ULog(str(path), topics)
    data = {d.name: d.data for d in ulog.data_list if d.multi_id == 0}
    if 'vehicle_local_position' not in data:
        raise ValueError(f'{path.name}: no vehicle_local_position in log')

    lp = data['vehicle_local_position']
    t_us = lp['timestamp'].astype(np.int64)
    ned = np.column_stack([lp['x'], lp['y'], lp['z']]).astype(float)
    vned = np.column_stack([lp['vx'], lp['vy'], lp['vz']]).astype(float)
    valid = (lp['xy_valid'] > 0) & (lp['z_valid'] > 0)

    if 'vehicle_local_position_setpoint' in data:
        sp = data['vehicle_local_position_setpoint']
        sp_topic = 'vehicle_local_position_setpoint'
        sp_ned = np.column_stack([sp['x'], sp['y'], sp['z']]).astype(float)
        sp_vned = np.column_stack([sp['vx'], sp['vy'], sp['vz']]).astype(float)
    elif 'trajectory_setpoint' in data:
        sp = data['trajectory_setpoint']
        sp_topic = 'trajectory_setpoint'
        sp_ned = np.column_stack([sp[f'position[{k}]'] for k in range(3)]).astype(float)
        sp_vned = np.column_stack([sp[f'velocity[{k}]'] for k in range(3)]).astype(float)
    else:
        sp, sp_topic = {'timestamp': np.array([], dtype=np.int64)}, 'none'
        sp_ned = sp_vned = np.zeros((0, 3))
    sp_t_us = sp['timestamp'].astype(np.int64)

    # Analysis window: offboard > armed > whole log.
    off_spans, mode = [], 'whole log'
    window = (int(t_us[0]), int(t_us[-1]))
    if 'vehicle_status' in data and not full:
        st = data['vehicle_status']
        st_t = st['timestamp'].astype(np.int64)
        armed = st['arming_state'] == ARMED
        off_spans = _windows(st_t, armed & (st['nav_state'] == NAV_OFFBOARD))
        arm_spans = _windows(st_t, armed)
        if off_spans:
            window, mode = (off_spans[0][0], off_spans[-1][1]), 'offboard'
        elif arm_spans:
            window, mode = (arm_spans[0][0], arm_spans[-1][1]), 'armed'

    t0 = window[0]
    keep = valid & (t_us >= window[0]) & (t_us <= window[1])
    sp_keep = (sp_t_us >= window[0]) & (sp_t_us <= window[1])
    pos, vel = _ned_to_frame(ned[keep], vned[keep], frame)
    sp_pos, sp_vel = _ned_to_frame(sp_ned[sp_keep], sp_vned[sp_keep], frame)
    return Flight(
        name=path.stem, mode=mode, frame='field' if frame is not None else 'ned',
        setpoint_topic=sp_topic, window=((window[0] - t0) * 1e-6, (window[1] - t0) * 1e-6),
        t=(t_us[keep] - t0) * 1e-6, pos=pos, vel=vel,
        sp_t=(sp_t_us[sp_keep] - t0) * 1e-6, sp_pos=sp_pos, sp_vel=sp_vel,
        offboard_spans=[((a - t0) * 1e-6, (b - t0) * 1e-6) for a, b in off_spans])
