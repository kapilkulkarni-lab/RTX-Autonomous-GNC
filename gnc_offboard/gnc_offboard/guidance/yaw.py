"""Yaw setpoints: face the field center (the D455 looks forward), with a deadband near the
center and a rate limit. Pure (no ROS). Yaw is PX4 NED (0 = north, clockwise positive).
"""
import math
from typing import Sequence

from gnc_offboard.frames import FieldFrame, wrap_pi


def face_point_yaw(drone_xy: Sequence[float], point_xy: Sequence[float], last_yaw: float,
                   deadband_m: float) -> float:
    """NED yaw that points the nose from `drone_xy` at `point_xy` (field ENU). Within
    `deadband_m` of the point the bearing is unstable, so keep `last_yaw`."""
    dx, dy = point_xy[0] - drone_xy[0], point_xy[1] - drone_xy[1]
    if math.hypot(dx, dy) < deadband_m:
        return last_yaw
    return FieldFrame.yaw_to_ned(math.atan2(dy, dx))


def rate_limited(current: float, target: float, max_rate_rad_s: float, dt: float) -> float:
    """Step from `current` toward `target` by at most max_rate * dt, the short way round."""
    error = wrap_pi(target - current)
    step = max_rate_rad_s * max(0.0, dt)
    if abs(error) <= step:
        return wrap_pi(target)
    return wrap_pi(current + math.copysign(step, error))


class YawCommand:
    """The commanded yaw, carried across states so the nose never jumps on a transition."""

    def __init__(self, yaw: float, now: float):
        self.yaw = wrap_pi(yaw)
        self.t = now

    def hold(self, now: float) -> float:
        """Keep the current yaw (precision phases: over the die, DESCENDING, on the die)."""
        self.t = now
        return self.yaw

    def face(self, now: float, drone_xy: Sequence[float], point_xy: Sequence[float],
             deadband_m: float, max_rate_rad_s: float) -> float:
        target = face_point_yaw(drone_xy, point_xy, self.yaw, deadband_m)
        self.yaw = rate_limited(self.yaw, target, max_rate_rad_s, now - self.t)
        self.t = now
        return self.yaw


def field_center(bounds: Sequence[float]) -> tuple:
    x0, y0, x1, y1 = bounds
    return ((x0 + x1) / 2.0, (y0 + y1) / 2.0)
