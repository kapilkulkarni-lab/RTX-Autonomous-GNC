"""The drone's cameras, from config/cameras.yaml: the one camera config that mission_manager
and camera_fov (sim) both read. Each camera is a camera_model.CameraModel.

Also the projection helpers the mission needs: footprints of every camera from a PX4 pose,
and the down camera's footprint width at the search altitude (for row spacing).
"""
import dataclasses
import math
import os
from pathlib import Path
from typing import Dict, Optional, Sequence, Union

import numpy as np
import yaml

from gnc_offboard.camera_model import CameraModel
from gnc_offboard.frames import FieldFrame

CAMERA_NAMES = ('d455', 'down')
DOWN = 'down'
_FIELDS = tuple(f.name for f in dataclasses.fields(CameraModel))


def default_cameras_path() -> str:
    """The installed config/cameras.yaml (falls back to the source tree)."""
    try:
        from ament_index_python.packages import get_package_share_directory
        installed = os.path.join(get_package_share_directory('gnc_offboard'), 'config',
                                 'cameras.yaml')
        if os.path.exists(installed):
            return installed
    except Exception:
        pass
    return str(Path(__file__).resolve().parents[2] / 'config' / 'cameras.yaml')


def load_cameras(path: Optional[Union[str, Path]] = None) -> Dict[str, CameraModel]:
    """{'d455': CameraModel, 'down': CameraModel}. Every CameraModel field must be given."""
    data = yaml.safe_load(Path(path or default_cameras_path()).read_text())
    cameras = data.get('cameras', {})
    if set(cameras) != set(CAMERA_NAMES):
        raise ValueError(f'cameras file must define exactly {CAMERA_NAMES}, got {sorted(cameras)}')
    out = {}
    for name in CAMERA_NAMES:
        values = cameras[name]
        missing, extra = set(_FIELDS) - set(values), set(values) - set(_FIELDS)
        if missing or extra:
            raise ValueError(f'camera {name}: missing {sorted(missing)}, unknown {sorted(extra)}')
        out[name] = CameraModel(**{k: float(values[k]) for k in _FIELDS})
    return out


def footprints(cameras: Dict[str, CameraModel], position_ned: Sequence[float],
               q_wxyz: Sequence[float], frame: FieldFrame, bounds) -> Dict[str, np.ndarray]:
    """Ground footprint polygon (field ENU, Nx2) of every camera; empty if it sees nothing."""
    out = {}
    for name, camera in cameras.items():
        try:
            _, polygon = camera.project(position_ned, q_wxyz, frame, bounds)
        except ValueError:
            polygon = np.empty((0, 2))
        out[name] = polygon
    return out


def yaw_quaternion(yaw_ned: float):
    """Level attitude with heading `yaw_ned` (PX4 NED, radians) as (w, x, y, z)."""
    return (math.cos(yaw_ned / 2.0), 0.0, 0.0, math.sin(yaw_ned / 2.0))


def down_footprint_extent(cameras: Dict[str, CameraModel], altitude_m: float,
                          yaw_ned: float):
    """(width along field x, width along field y) of the down camera's footprint when level at
    `altitude_m` with heading `yaw_ned` (range clipping included)."""
    frame = FieldFrame(0.0, 0.0)
    big = 10.0 * max(1.0, altitude_m) + 100.0
    _, polygon = cameras[DOWN].project((0.0, 0.0, -altitude_m), yaw_quaternion(yaw_ned),
                                       frame, (-big, -big, big, big))
    if len(polygon) < 3:
        return 0.0, 0.0
    return float(np.ptp(polygon[:, 0])), float(np.ptp(polygon[:, 1]))
