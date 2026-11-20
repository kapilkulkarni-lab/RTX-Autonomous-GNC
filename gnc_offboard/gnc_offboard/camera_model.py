"""Flat-ground pinhole projection. PX4 inputs are NED position and FRD attitude.

Mount translation is body FRD (forward, right, down), metres. Mount angles
are degrees: pitch down, yaw right, roll clockwise looking forward.
Output polygons use field ENU, clipped to bounds and horizontal max range.
"""
from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class CameraModel:
    hfov_deg: float = 70.0
    vfov_deg: float = 50.0
    forward_m: float = 0.2
    right_m: float = 0.0
    down_m: float = 0.0
    pitch_down_deg: float = 45.0
    yaw_right_deg: float = 0.0
    roll_deg: float = 0.0
    max_range_m: float = 30.0

    def __post_init__(self):
        if not all(math.isfinite(v) for v in vars(self).values()):
            raise ValueError('camera parameters must be finite')
        if not (0 < self.hfov_deg < 179 and 0 < self.vfov_deg < 179):
            raise ValueError('FOV angles must be between 0 and 179 degrees')
        if self.max_range_m <= 0:
            raise ValueError('max_range_m must be positive')

    def mount_rotation(self):
        y, p, r = map(math.radians,
                      (self.yaw_right_deg, self.pitch_down_deg, self.roll_deg))
        cy, sy, cp, sp, cr, sr = (math.cos(y), math.sin(y), math.cos(p),
                                  math.sin(p), math.cos(r), math.sin(r))
        return (np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
                @ np.array([[cp, 0, -sp], [0, 1, 0], [sp, 0, cp]])
                @ np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]]))

    def project(self, position_ned, quaternion_wxyz, frame, bounds, ground_z=0.0):
        """Return (camera origin ENU, visible ground polygon Nx2).

        Intersect the ground with the frustum half-planes, including horizon
        cases; never replace an upward ray with an artificial ground hit.
        Range uses a conservative inscribed 64-sided horizontal disk.
        """
        q = np.asarray(quaternion_wxyz, dtype=float)
        pos = np.asarray(position_ned, dtype=float)
        if (q.shape != (4,) or pos.shape != (3,) or not np.isfinite(q).all()
                or not np.isfinite(pos).all() or np.linalg.norm(q) < 1e-9):
            raise ValueError('invalid position or quaternion')
        w, x, y, z = q / np.linalg.norm(q)
        body_to_ned = np.array([
            [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
            [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
            [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
        ned_to_enu = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
        body_to_enu = ned_to_enu @ body_to_ned
        origin = (np.array(frame.to_field(*pos)) + body_to_enu
                  @ np.array([self.forward_m, self.right_m, self.down_m]))
        if origin[2] <= ground_z:
            return origin, np.empty((0, 2))
        rotation = body_to_enu @ self.mount_rotation()
        xmin, ymin, xmax, ymax = bounds
        polygon = np.array([[xmin, ymin], [xmax, ymin], [xmax, ymax], [xmin, ymax]])
        h, v = math.tan(math.radians(self.hfov_deg/2)), math.tan(math.radians(self.vfov_deg/2))
        # Camera coordinates: +x optical axis, +y image right, +z image down.
        for normal in ([1, 0, 0], [h, 1, 0], [h, -1, 0], [v, 0, 1], [v, 0, -1]):
            n = rotation @ normal
            polygon = clip(polygon, n[:2], n[2]*(ground_z-origin[2]) - n[:2] @ origin[:2])
        radius = self.max_range_m * math.cos(math.pi/64)
        normals = -np.stack([np.cos(_RANGE_ANGLES), np.sin(_RANGE_ANGLES)], axis=1)
        offsets = radius - normals @ origin[:2]
        if len(polygon):
            # Clipping only shrinks a convex polygon, so a range plane that no vertex violates
            # now can never cut it later: clip by the violated planes only (same result).
            violated = np.any(polygon @ normals.T + offsets < 0, axis=0)
            for n, offset in zip(normals[violated], offsets[violated]):
                polygon = clip(polygon, n, offset)
        return origin, polygon


_RANGE_ANGLES = np.arange(64)*2*math.pi/64


def clip(polygon, normal, offset):
    """Convex polygon clipped by dot(normal, point) + offset >= 0."""
    out = []
    for a, b in zip(polygon, np.roll(polygon, -1, axis=0)):
        da, db = normal @ a + offset, normal @ b + offset
        if da >= 0:
            out.append(a)
        if (da >= 0) != (db >= 0):
            out.append(a + (b-a)*da/(da-db))
    return np.asarray(out, dtype=float).reshape(-1, 2)


def contains_point(polygon, point):
    """Whether a point is inside a nondegenerate CCW footprint, including edges."""
    if len(polygon) < 3:
        return False
    polygon = np.asarray(polygon)
    following = np.roll(polygon, -1, axis=0)
    area2 = np.sum(polygon[:, 0]*following[:, 1]-polygon[:, 1]*following[:, 0])
    if area2 <= 1e-12:
        return False
    delta = following-polygon
    relative = np.asarray(point)-polygon
    return bool(np.all(delta[:, 0]*relative[:, 1]-delta[:, 1]*relative[:, 0] >= -1e-9))


class CoverageGrid:
    """Persistent cell-centre coverage, bounded in memory by the field extent."""
    def __init__(self, bounds, resolution=0.25):
        if not math.isfinite(resolution) or resolution <= 0:
            raise ValueError('resolution must be positive and finite')
        self.bounds = tuple(bounds)
        self.resolution = resolution
        xmin, ymin, xmax, ymax = bounds
        self.width = math.ceil((xmax-xmin)/resolution)
        self.height = math.ceil((ymax-ymin)/resolution)
        if self.width*self.height > 1_000_000:
            raise ValueError('coverage grid exceeds one million cells')
        xx, yy = np.meshgrid(xmin+(np.arange(self.width)+.5)*resolution,
                             ymin+(np.arange(self.height)+.5)*resolution)
        self.centres = np.stack([xx, yy], axis=-1)
        self.searched = np.zeros((self.height, self.width), dtype=bool)
        self.valid = (xx <= xmax) & (yy <= ymax)
        self.exposure_s = np.zeros_like(xx)
        self.in_view = np.zeros_like(self.searched)

    def inside(self, polygon):
        """Cells whose centres lie in a CCW polygon (empty polygon: none)."""
        if len(polygon) < 3:
            return np.zeros_like(self.searched)
        polygon = np.asarray(polygon, dtype=float)
        inside = self.valid.copy()
        for a, b in zip(polygon, np.roll(polygon, -1, axis=0)):
            d = b-a
            inside &= (d[0]*(self.centres[..., 1]-a[1])
                       - d[1]*(self.centres[..., 0]-a[0])) >= -1e-9
        return inside

    def update(self, polygon, elapsed_s=0.0):
        """Credit elapsed real time to the footprint held since the previous sample."""
        self.update_many([polygon], elapsed_s)

    def update_many(self, polygons, elapsed_s=0.0):
        """update() for several cameras at once: the view is the union of their footprints."""
        self.exposure_s[self.in_view] += elapsed_s
        view = np.zeros_like(self.searched)
        for polygon in polygons:
            view |= self.inside(polygon)
        self.in_view = view
        self.searched |= view

    def mark(self, polygon):
        """Mark a footprint as searched (no exposure or current-view bookkeeping)."""
        self.searched |= self.inside(polygon)

    def is_seen(self, x, y):
        i = math.floor((x-self.bounds[0])/self.resolution)
        j = math.floor((y-self.bounds[1])/self.resolution)
        return 0 <= i < self.width and 0 <= j < self.height and bool(self.searched[j, i])

    def clear(self):
        self.searched.fill(False)
        self.exposure_s.fill(0.0)
        self.in_view.fill(False)

    @property
    def count(self):
        return int(np.count_nonzero(self.searched))

    @property
    def area_m2(self):
        xmin, ymin, xmax, ymax = self.bounds
        half = self.resolution/2
        widths = np.maximum(0, np.minimum(self.centres[..., 0]+half, xmax)
                            - np.maximum(self.centres[..., 0]-half, xmin))
        heights = np.maximum(0, np.minimum(self.centres[..., 1]+half, ymax)
                             - np.maximum(self.centres[..., 1]-half, ymin))
        return float(np.sum((widths*heights)[self.searched]))
