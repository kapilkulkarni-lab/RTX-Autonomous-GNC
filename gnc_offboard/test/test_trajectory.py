"""Checks for the cubic guidance. Run: python3 -m pytest gnc_offboard/test -q"""
import math

from gnc_offboard.trajectory import CubicSegment, segment_duration

V, A = 1.0, 0.5


def norm(v):
    return math.sqrt(sum(x * x for x in v))


def test_rest_to_rest_endpoints():
    seg = CubicSegment([0, 0, -1.5], [6, 8, -1.5], 10.0, V, A)
    p, v, a, _, _ = seg.sample(10.0)
    assert p == [0, 0, -1.5] and norm(v) == 0
    p, v, a, _, _ = seg.sample(10.0 + seg.T)
    assert [round(x, 9) for x in p] == [6, 8, -1.5] and norm(v) == 0 and norm(a) == 0


def test_limits_respected():
    for L in (0.2, 1.0, 5.0, 14.0):
        seg = CubicSegment([0, 0, 0], [L, 0, 0], 0.0, V, A)
        n = 2000
        vs = [norm(seg.sample(seg.T * i / n)[1]) for i in range(n + 1)]
        accs = [norm(seg.sample(seg.T * i / n)[2]) for i in range(1, n)]
        assert max(vs) <= V + 1e-9 and max(accs) <= A + 1e-6


def test_velocity_is_derivative_of_position():
    seg = CubicSegment([1, 2, 3], [4, -2, 1], 0.0, V, A)
    h = 1e-5
    for t in (0.3 * seg.T, 0.5 * seg.T, 0.8 * seg.T):
        p1, p2 = seg.sample(t - h)[0], seg.sample(t + h)[0]
        fd = [(b - a) / (2 * h) for a, b in zip(p1, p2)]
        assert all(abs(f - v) < 1e-5 for f, v in zip(fd, seg.sample(t)[1]))


def test_yaw_takes_short_way():
    seg = CubicSegment([0, 0, 0], [5, 0, 0], 0.0, V, A, yaw0=math.radians(170),
                       yaw1=math.radians(-170))
    yaw_mid = seg.sample(seg.T / 2)[3]
    assert abs(abs(yaw_mid) - math.pi) < 1e-9   # passes through 180, not through 0


def test_duration_formula():
    assert segment_duration(14.0, 1.0, 0.5) == 21.0      # speed-limited
    assert abs(segment_duration(0.5, 1.0, 0.5) - math.sqrt(6.0)) < 1e-12  # accel-limited
