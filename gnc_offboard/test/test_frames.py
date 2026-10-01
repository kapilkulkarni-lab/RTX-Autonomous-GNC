import math

from gnc_offboard.frames import FieldFrame


def test_field_centre_from_sw_corner():
    f = FieldFrame(-6.096, -6.096)
    n, e, d = f.to_ned(0.0, 0.0, 1.5)
    assert math.isclose(n, 6.096) and math.isclose(e, 6.096) and d == -1.5


def test_round_trip():
    f = FieldFrame(4.2, -3.1)
    for p in [(0, 0, 0), (1.5, -2.0, 1.0), (-6, 6, 2.5)]:
        back = f.to_field(*f.to_ned(*p))
        assert all(math.isclose(a, b, abs_tol=1e-12) for a, b in zip(p, back))


def test_yaw():
    assert math.isclose(FieldFrame.yaw_to_ned(0.0), math.pi / 2)        # east
    assert math.isclose(FieldFrame.yaw_to_ned(math.pi / 2), 0.0, abs_tol=1e-12)  # north
    for y in [-3.0, -1.0, 0.3, 2.9]:
        assert math.isclose(FieldFrame.yaw_to_enu(FieldFrame.yaw_to_ned(y)), y)
