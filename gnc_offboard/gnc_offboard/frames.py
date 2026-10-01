"""Field frame <-> PX4 local NED conversions. No ROS imports, so planners and tests can use it.

Field frame (Gazebo world, generator manifest): ENU, origin at field centre,
    +x = east, +y = north, +z = up.
PX4 local frame: NED, origin where the EKF initialised (the spawn point),
    +x = north, +y = east, +z = down.
"""
import math


def wrap_pi(angle):
    """Wrap an angle to [-pi, pi)."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


class FieldFrame:
    def __init__(self, spawn_x, spawn_y):
        self.sx = float(spawn_x)
        self.sy = float(spawn_y)

    @classmethod
    def from_manifest(cls, manifest):
        x, y = (float(v) for v in manifest['px4_spawn_pose'].split(',')[:2])
        return cls(x, y)

    def to_ned(self, x, y, z=0.0):
        """Field ENU point -> PX4 local NED (north, east, down)."""
        return (y - self.sy, x - self.sx, -z)

    def to_field(self, n, e, d=0.0):
        """PX4 local NED point -> field ENU (x, y, z)."""
        return (e + self.sx, n + self.sy, -d)

    @staticmethod
    def yaw_to_ned(yaw_enu):
        """ENU yaw (0 = east, CCW+) -> NED yaw (0 = north, CW+)."""
        return wrap_pi(math.pi / 2.0 - yaw_enu)

    @staticmethod
    def yaw_to_enu(yaw_ned):
        return wrap_pi(math.pi / 2.0 - yaw_ned)
