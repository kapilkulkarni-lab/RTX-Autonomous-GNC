"""Exercise recording and export without arming or sending flight commands."""
import json
import time
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
        'px4_spawn_pose': '0,0,0,0,0,0',
        'field': {'bounds': [-5, -5, 5, 5]},
        'start_zone': {'bounds': [-1, -1, 1, 1], 'center': [0, 0]},
        'scenario': {'initial_target': 'yellow'},
        'targets': [{'color': 'yellow', 'x': 0, 'y': 2}],
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
        outside.position = [0.0, 0.0, -2.0]
        outside.q = [0.0, 0.0, 0.0, 1.0]
        node.on_odometry(outside)
        assert node.target_found is False
        node.trajectory.clear()
        for stamp, north in [(1, 0.0), (2, .1)]:
            msg = VehicleOdometry()
            msg.timestamp = stamp
            msg.pose_frame = VehicleOdometry.POSE_FRAME_NED
            msg.position = [north, 0.0, -2.0]
            msg.q = [1.0, 0.0, 0.0, 0.0]
            node.on_odometry(msg)
        assert node.target_found
        # Moving out of view must not clear a previous detection.
        msg.timestamp = 3
        msg.position = [0.0, 0.0, -2.0]
        msg.q = [0.0, 0.0, 0.0, 1.0]
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
        assert len(data['trajectory_enu']) == 5
        assert data['trajectory_enu'][-1] is None
        assert data['trajectory_enu'][0] == [0, 0, 2]
        assert data['target_found'] is True
        assert data['arena_area_m2'] == 100
        assert output.with_suffix('.png').read_bytes().startswith(b'\x89PNG')
        node.destroy_node()
        node = None
        node = CameraFov()
        assert node.target_found is False
        assert node.target_position is None
    finally:
        if listener is not None:
            listener.destroy_node()
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
