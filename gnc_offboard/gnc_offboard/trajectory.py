"""Time-defined cubic guidance between waypoints (no ROS dependencies).

Same blend as guidanceMultiWaypoint.m in GNC Notes sec. 5.5. For one segment from
p0 to p1 lasting T seconds:

    tau  = (t - t_start) / T              in [0, 1]
    s    = 3 tau^2 - 2 tau^3              smoothstep: s(0)=0, s(1)=1, s'(0)=s'(1)=0
    p(t) = p0 + s (p1 - p0)
    v(t) = (6 tau - 6 tau^2) / T   * (p1 - p0)
    a(t) = (6 - 12 tau)      / T^2 * (p1 - p0)

The vehicle starts and ends each segment at rest, so it can hover at every waypoint.
Peak values along a segment of length L:  |v|max = 1.5 L / T  (at tau = 0.5),
|a|max = 6 L / T^2  (at tau = 0 and 1). The duration is the shortest T that keeps
both under the limits you give it.

Yaw uses the same blend, taking the short way round.
"""
import math


def segment_duration(length, v_max, a_max, t_min=0.5):
    """Shortest T keeping peak speed <= v_max and peak acceleration <= a_max."""
    if length <= 0.0:
        return t_min
    return max(1.5 * length / v_max, math.sqrt(6.0 * length / a_max), t_min)


def wrap_pi(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class CubicSegment:
    """One rest-to-rest segment. Works in any frame; mission_demo feeds it PX4 NED."""

    def __init__(self, p0, p1, t_start, v_max, a_max, yaw0=None, yaw1=None):
        self.p0 = [float(v) for v in p0]
        self.d = [float(b) - a for a, b in zip(self.p0, p1)]
        self.length = math.sqrt(sum(v * v for v in self.d))
        self.t_start = t_start
        self.T = segment_duration(self.length, v_max, a_max)
        self.yaw0 = yaw0
        self.dyaw = wrap_pi(yaw1 - yaw0) if yaw0 is not None and yaw1 is not None else 0.0

    def done(self, t):
        return t - self.t_start >= self.T

    def sample(self, t):
        """Return (position, velocity, acceleration, yaw, yaw_rate) at time t."""
        tau = min(max((t - self.t_start) / self.T, 0.0), 1.0)
        s = 3 * tau ** 2 - 2 * tau ** 3
        ds = (6 * tau - 6 * tau ** 2) / self.T
        dds = (6 - 12 * tau) / self.T ** 2
        if tau >= 1.0:                       # hold exactly at rest at the end
            ds = dds = 0.0
        pos = [a + s * d for a, d in zip(self.p0, self.d)]
        vel = [ds * d for d in self.d]
        acc = [dds * d for d in self.d]
        if self.yaw0 is None:
            return pos, vel, acc, float('nan'), float('nan')
        return pos, vel, acc, wrap_pi(self.yaw0 + s * self.dyaw), ds * self.dyaw
