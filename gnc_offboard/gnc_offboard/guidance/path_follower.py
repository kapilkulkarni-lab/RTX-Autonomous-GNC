"""PathFollower: turns a waypoint list into a smooth position setpoint, one tick at a time.
Pure (no ROS). Works in PX4 local NED. Each leg is a rest-to-rest CubicSegment (the same
guidance the search-pattern executables fly); a waypoint counts as reached when the segment
has finished and the vehicle is within `acceptance_m` of it.
"""
import math
from typing import List, Optional, Sequence, Tuple

from gnc_offboard.trajectory import CubicSegment

Vec3 = Tuple[float, float, float]


class PathFollower:
    def __init__(self, waypoints: Sequence[Sequence[float]], start: Sequence[float], yaw: float,
                 speed: float, accel: float, acceptance_m: float):
        if speed <= 0 or accel <= 0 or acceptance_m <= 0:
            raise ValueError('speed, accel and acceptance_m must be positive')
        self.waypoints: List[Vec3] = [tuple(float(v) for v in w) for w in waypoints]
        self.setpoint: Vec3 = tuple(float(v) for v in start)
        self.yaw = yaw
        self.speed, self.accel, self.acceptance = speed, accel, acceptance_m
        self.index = 0
        self.segment: Optional[CubicSegment] = None

    @property
    def done(self) -> bool:
        return self.index >= len(self.waypoints)

    @property
    def target(self) -> Optional[Vec3]:
        return None if self.done else self.waypoints[self.index]

    def update(self, now: float, position: Optional[Sequence[float]]) -> Tuple[Vec3, float]:
        """Advance and return (setpoint NED, yaw). Holds the last setpoint once done."""
        if self.done:
            return self.setpoint, self.yaw
        target = self.waypoints[self.index]
        if self.segment is None:
            self.segment = CubicSegment(self.setpoint, target, now, self.speed, self.accel,
                                        yaw0=self.yaw, yaw1=self.yaw)
        if self.segment.done(now):
            self.setpoint = target
            if position is not None and math.dist(position, target) <= self.acceptance:
                self.index += 1
                self.segment = None
        else:
            pos = self.segment.sample(now)[0]
            self.setpoint = tuple(float(v) for v in pos)
        return self.setpoint, self.yaw
