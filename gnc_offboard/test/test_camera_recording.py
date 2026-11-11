"""Exercise recording and export without arming or sending flight commands."""
import json
import time
from unittest.mock import patch
import numpy as np

import pytest

rclpy = pytest.importorskip('rclpy')
pytest.importorskip('px4_msgs.msg')
from px4_msgs.msg import VehicleOdometry
from avc_interfaces.msg import TargetDetection
from gnc_offboard.camera_fov import CameraFov, DETECTION_QOS


def test_records_vehicle_path_target_and_exports_map(tmp_path):
    manifest = tmp_path/'manifest.json'
    manifest.write_text(json.dumps({
        'px4_spawn_pose': '1,-1,0,0,0,0',
        'field': {'bounds': [-5, -5, 5, 5]},
        'start_zone': {'bounds': [-1, -1, 1, 1], 'center': [0, 0]},
        'scenario': {'initial_target': 'yellow'},
        'targets': [{'color': 'yellow', 'x': 0, 'y': 2},
                    {'color': 'red', 'x': 0, 'y': -2},
                    {'color': 'red', 'x': 1, 'y': -2}],
    }))
    output = tmp_path/'coverage.json'
    rclpy.init(args=['--ros-args', '-p', f'manifest:={manifest}',
                    '-p', f'output:={output}', '-p', 'gazebo_markers:=false'])
    node = None
    listener = None
    try:
        node = CameraFov()
        listener = rclpy.create_node('detection_test_listener')
        received = []
        subscription = listener.create_subscription(
            TargetDetection, '/camera/target_detection', received.append, DETECTION_QOS)

        def wait_for(predicate):
            deadline = time.monotonic() + 5
            while not predicate() and time.monotonic() < deadline:
                rclpy.spin_once(listener, timeout_sec=.05)
            assert predicate()

        node.publish_visuals()
        wait_for(lambda: len(received) > 0)
        assert received[-1].found is False
        assert received[-1].header.frame_id == 'map'
        outside = VehicleOdometry()
        outside.timestamp = 99
        outside.pose_frame = VehicleOdometry.POSE_FRAME_NED
        outside.position = [1.0, -1.0, -2.0]
        outside.q = [0.0, 0.0, 0.0, 1.0]
        sample_time = time.monotonic()
        with patch('gnc_offboard.camera_fov.time.monotonic', return_value=sample_time):
            node.on_odometry(outside)
        assert node.target_found is False
        assert list(node.observed_dice.values()) == [
            {'color': 'red', 'position_enu': [0, -2, 0]},
            {'color': 'red', 'position_enu': [1, -2, 0]},
        ]
        node.trajectory.clear()
        for stamp, north in [(1, 0.0), (2, .1)]:
            msg = VehicleOdometry()
            msg.timestamp = stamp
            msg.pose_frame = VehicleOdometry.POSE_FRAME_NED
            msg.position = [north + 1.0, -1.0, -2.0]
            msg.q = [1.0, 0.0, 0.0, 0.0]
            with patch('gnc_offboard.camera_fov.time.monotonic',
                       return_value=sample_time + stamp * .25):
                node.on_odometry(msg)
        assert node.target_found
        # Moving out of view after a telemetry gap must not add exposure or clear detections.
        msg.timestamp = 3
        msg.position = [1.0, -1.0, -2.0]
        msg.q = [0.0, 0.0, 0.0, 1.0]
        with patch('gnc_offboard.camera_fov.time.monotonic', return_value=sample_time + 2.0):
            node.on_odometry(msg)
        assert node.target_found
        listener.destroy_subscription(subscription)
        node.publish_visuals()
        received.clear()
        # A late subscriber gets the retained result without another publish.
        subscription = listener.create_subscription(
            TargetDetection, '/camera/target_detection', received.append, DETECTION_QOS)
        wait_for(lambda: len(received) > 0)
        result = received[-1]
        assert result.found is True
        assert (result.position.x, result.position.y, result.position.z) == (0, 2, 0)
        assert result.header.frame_id == 'map'
        assert result.header.stamp.sec > 0
        node.last_receive = time.monotonic() - 2
        assert node.target_found  # Stale odometry cannot erase history.
        # Also tolerate samples retained in NumPy form by an older recorder.
        node.trajectory.append({'time_s': 3, 'position': [np.float32(0),
                                                         np.float32(.2), np.float32(2)]})
        node.trajectory.append({'time_s': 4, 'position': None})
        node.save()
        data = json.loads(output.read_text())
        assert len(data['trajectory_enu']) == 6
        assert data['trajectory_enu'][-1] is None
        assert data['trajectory_enu'][0] == [0, 0, 2]
        assert data['observed_dice'] == [
            {'color': 'red', 'position_enu': [0, -2, 0]},
            {'color': 'red', 'position_enu': [1, -2, 0]},
            {'color': 'yellow', 'position_enu': [0, 2, 0]},
        ]
        assert data['target_found'] is True
        assert data['arena_area_m2'] == 100
        exposure = dict(zip(map(tuple, data['searched_cell_centres']),
                            data['searched_cell_exposure_s']))
        assert exposure[(.125, -2.125)] == pytest.approx(.25)
        assert exposure[(.125, 2.125)] == pytest.approx(.25)
        assert output.with_suffix('.png').read_bytes().startswith(b'\x89PNG')
        node.destroy_node()
        node = None
        node = CameraFov()
        assert node.target_found is False
        assert node.target_position is None
        assert node.observed_dice == {}
    finally:
        if listener is not None:
            listener.destroy_node()
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
