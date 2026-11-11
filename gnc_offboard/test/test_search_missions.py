"""Verify fixed search paths and flight I/O without arming a vehicle."""
import importlib
import math
from unittest.mock import patch

import pytest

pytest.importorskip('rclpy')
pytest.importorskip('px4_msgs.msg')
import rclpy
from gnc_offboard import search_planners as sp
from gnc_offboard import search_regions as sr
from gnc_offboard.mission_demo import run_mission

MISSIONS = [('lawnmower', 'LawnmowerMission'), ('spiral_in', 'SpiralInMission'),
            ('spiral_out', 'SpiralOutMission'), ('frontier', 'FrontierMission'),
            ('zones', 'ZoneMission'), ('hybrid', 'HybridMission')]


@pytest.mark.parametrize('name,cls', MISSIONS)
def test_mission_builds_fixed_path_and_flight_io(name, cls):
    rclpy.init()
    node = None
    try:
        node = getattr(importlib.import_module('gnc_offboard.mission_' + name), cls)()
        original = list(node.waypoints)
        assert len(original) > 2
        assert all(math.isfinite(v) for p in original for v in p)
        for waypoint in original[:-1]:
            x, y, z = node.frame.to_field(*waypoint)
            assert node.area.contains((x, y))
            assert z == node.alt
        assert original[-1] == node.frame.to_ned(*node.start_xy, node.alt)
        node.setup_io()
        assert len(list(node.timers)) == 1
        assert len(list(node.subscriptions)) == 2
        assert all('camera' not in sub.topic_name for sub in node.subscriptions)
        assert node.waypoints == original
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


def test_shared_runner_sets_up_io_before_spinning():
    with patch('gnc_offboard.mission_demo.rclpy') as ros:
        with patch('gnc_offboard.mission_demo.MissionDemo') as factory:
            node = factory.return_value
            node.get_topic_names_and_types.return_value = [
                ('/fmu/out/vehicle_local_position_v1', [])]
            ros.spin.side_effect = KeyboardInterrupt
            run_mission(factory, args=['--ros-args'])
            node.setup_io.assert_called_once()
            ros.spin.assert_called_once_with(node)
            node.destroy_node.assert_called_once()


@pytest.mark.parametrize('start', [(-6, -6), (6, -6), (-6, 6), (6, 6)])
def test_patterns_support_every_start_corner(start):
    area = sp.Area.square(13.716, 1)
    spacing = sp.row_spacing(1.5, 70, .2)
    for plan in [sp.lawnmower, sp.expanding_spiral, sp.contracting_spiral,
                 sp.zone_sweep, sp.hybrid_sweep]:
        path = plan(area, start, spacing)
        assert path and all(area.contains(p) for p in path)


@pytest.mark.parametrize('overlap', [-.1, 1, float('nan')])
def test_invalid_overlap_rejected(overlap):
    with pytest.raises(ValueError):
        sp.row_spacing(1.5, 70, overlap)

@pytest.mark.parametrize('name,cls', MISSIONS)
def test_missions_parse_regions_and_route_around_keepouts(name, cls):
    rclpy.init(args=['--ros-args', '-p', "region_names:=['north half']",
                     '-p', 'regions:=[-1.0, 0.0, 1.0, 3.0]',
                     '-p', 'keepouts:=[-1.0, -2.5, 1.0, 2.5]',
                     '-p', 'obstacles:=[3.0, 4.0, 0.3]'])
    node = None
    try:
        node = getattr(importlib.import_module('gnc_offboard.mission_' + name), cls)()
        path = [node.start_xy] + [node.frame.to_field(*wp)[:2] for wp in node.waypoints]
        assert any(y > 3.0 for _, y in path[1:-1])
        assert path[-1] == pytest.approx(node.start_xy)
        for start, end in zip(path, path[1:]):
            assert not any(sr._seg_hits_rect(start, end, rect) for rect in node.exclusions)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


def test_keepout_routes_home_or_rejects_blocked_route():
    area = sp.Area(-5, 5, -5, 5)
    keepout = sp.Area(-1, 1, -2, 2)
    start, far = (-4, 0), (4, 0)
    path = [start, *sr.route_path(start, [far, start], [keepout], area)]
    assert len(path) > 3
    assert path[-1] == start
    assert all(not sr._seg_hits_rect(a, b, keepout) for a, b in zip(path, path[1:]))
    with pytest.raises(ValueError, match='no route'):
        sr.route_path(start, [far], [sp.Area(-1, 1, -6, 6)], area)
    with pytest.raises(ValueError, match='start position'):
        sr.route_path((0, 0), [far], [keepout], area)
