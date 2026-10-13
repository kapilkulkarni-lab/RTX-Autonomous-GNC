"""Checks for field geometry and flight metrics. Run: python3 -m pytest gnc_offboard/test -q"""
import math

import numpy as np

from gnc_offboard.field_map import FieldMap, Obstacle
from gnc_offboard.flight_log import Flight, _windows

MANIFEST = {
    'world_name': 'test', 'seed': 1, 'challenge': 2,
    'field': {'bounds': [-5.0, -5.0, 5.0, 5.0]},
    'start_zone': {'bounds': [-5.0, -5.0, -3.5, -3.5], 'center': [-4.25, -4.25]},
    'px4_spawn_pose': '-4.25,-4.25,0.2,0,0,0.785',
    'obstacles': [
        {'name': 'box', 'kind': 'box_large', 'x': 0.0, 'y': 0.0, 'yaw': 0.0,
         'footprint_radius': 0.375, 'footprint': {'shape': 'box', 'w': 0.6, 'd': 0.45}},
        {'name': 'old_style', 'kind': 'bucket', 'x': 3.0, 'y': 3.0, 'yaw': 0.0,
         'footprint_radius': 0.15},   # pre-footprint manifest entry
    ],
    'targets': [{'color': 'red', 'x': 2.0, 'y': -2.0}],
    'scenario': {'initial_target': 'red'},
}


def box(yaw):
    return Obstacle({'name': 'b', 'kind': 'k', 'x': 1.0, 'y': 2.0, 'yaw': yaw,
                     'footprint_radius': 0.5, 'footprint': {'shape': 'box', 'w': 0.6, 'd': 0.4}})


def test_box_clearance_axis_aligned():
    b = box(0.0)
    assert math.isclose(b.clearance(1.0 + 0.3 + 0.5, 2.0), 0.5)       # 0.5 m off the +x face
    assert math.isclose(b.clearance(1.0, 2.0 + 0.2 + 0.25), 0.25)      # off the +y face
    assert math.isclose(b.clearance(1.0 + 0.3 + 0.3, 2.0 + 0.2 + 0.4), 0.5)  # off a corner
    assert b.clearance(1.0, 2.0) < 0                                    # inside


def test_box_clearance_rotated():
    b = box(math.pi / 2)          # long side now along y
    assert math.isclose(b.clearance(1.0, 2.0 + 0.3 + 0.5), 0.5)
    assert math.isclose(b.clearance(1.0 + 0.2 + 0.5, 2.0), 0.5)


def test_inflated_outline_sits_at_inflate_distance():
    for yaw in (0.0, 0.7, -2.0):
        b = box(yaw)
        for x, y in b.inflated_outline(0.5):
            assert math.isclose(b.clearance(x, y), 0.5, abs_tol=1e-9)


def test_old_manifest_falls_back_to_circle():
    fmap = FieldMap(MANIFEST)
    ob = fmap.obstacles[1]
    assert ob.shape == 'circle' and math.isclose(ob.clearance(3.0, 3.5), 0.35)


def test_path_violations():
    fmap = FieldMap(MANIFEST)
    clear = [(-4.0, -4.0), (-4.0, 2.0), (2.0, 2.0)]
    assert fmap.path_violations(clear, inflate=0.5) == []
    through_box = [(-3.0, 0.0), (3.0, 0.0)]
    v = fmap.path_violations(through_box, inflate=0.5)
    assert len(v) == 1 and v[0]['obstacle'] == 'box' and v[0]['clearance'] < 0
    near_edge = [(-4.0, -4.0), (4.8, -4.0)]
    assert [x['kind'] for x in fmap.path_violations(near_edge, inflate=0.5)] == ['edge']


def test_target_and_frame():
    fmap = FieldMap(MANIFEST)
    assert fmap.target()['color'] == 'red'
    n, e, d = fmap.frame.to_ned(-4.25, -4.25, 1.0)
    assert (round(n, 9), round(e, 9), d) == (0.0, 0.0, -1.0)


def test_windows():
    t = np.arange(6)
    assert _windows(t, [0, 1, 1, 0, 1, 1]) == [(1, 3), (4, 5)]


def test_flight_metrics():
    t = np.linspace(0, 10, 101)
    pos = np.column_stack([t * 0.5, np.zeros_like(t), np.full_like(t, 1.5)])
    vel = np.column_stack([np.full_like(t, 0.5), np.zeros_like(t), np.zeros_like(t)])
    sp = pos + np.array([0.1, 0.0, 0.0])           # constant 0.1 m lead
    f = Flight('x', t, pos, vel, t, sp, vel, (0, 10), 'offboard')
    m = f.metrics()
    assert math.isclose(m['rms_error_m'], 0.1) and math.isclose(m['max_error_m'], 0.1)
    assert math.isclose(m['path_length_m'], 5.0) and math.isclose(m['max_speed_mps'], 0.5)
