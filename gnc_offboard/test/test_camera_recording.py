"""Exercise recording and export without arming or sending flight commands."""
import json
import numpy as np

import pytest

rclpy = pytest.importorskip('rclpy')
pytest.importorskip('px4_msgs.msg')
from px4_msgs.msg import VehicleOdometry
from gnc_offboard.camera_fov import CameraFov


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
    try:
        node = CameraFov()
        for stamp, north in [(1, 0.0), (2, .1)]:
            msg = VehicleOdometry()
            msg.timestamp = stamp
            msg.pose_frame = VehicleOdometry.POSE_FRAME_NED
            msg.position = [north, 0.0, -2.0]
            msg.q = [1.0, 0.0, 0.0, 0.0]
            node.on_odometry(msg)
        assert node.target_found
        # Also tolerate samples retained in NumPy form by an older recorder.
        node.trajectory.append({'time_s': 3, 'position': [np.float32(0),
                                                         np.float32(.2), np.float32(2)]})
        node.trajectory.append({'time_s': 4, 'position': None})
        node.save()
        data = json.loads(output.read_text())
        assert len(data['trajectory_enu']) == 4
        assert data['trajectory_enu'][-1] is None
        assert data['trajectory_enu'][0] == [0, 0, 2]
        assert data['target_found'] is True
        assert data['arena_area_m2'] == 100
        assert output.with_suffix('.png').read_bytes().startswith(b'\x89PNG')
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
