"""camera_fov as the one sim node: both cameras from config/cameras.yaml (the same file and
models as mission_manager), per-camera footprints, union coverage, the Perception stand-in
(dice from either camera, target_relative and color from the down camera) and its switches,
and the coverage JSON still read by tools/compare_search_runs.py and coverage_plot.py."""
import json
import subprocess
import sys
from pathlib import Path

import pytest

from conftest import CAMERAS_YAML, CONFIG_YAML, TEST_MANIFEST

rclpy = pytest.importorskip('rclpy')
REPO = Path(__file__).resolve().parents[2]


@pytest.fixture
def node(tmp_path):
    from gnc_offboard.camera_fov import CameraFov
    rclpy.init(args=['--ros-args', '-p', f'manifest:={TEST_MANIFEST}',
                     '-p', f'cameras_file:={CAMERAS_YAML}', '-p', 'gazebo_markers:=false',
                     '-p', f'output:={tmp_path / "coverage.json"}',
                     '-p', 'target_relative_noise_m:=0.0'])
    try:
        n = CameraFov()
    except Exception:
        rclpy.shutdown()
        raise
    n.sent = {}
    pubs = {'map_pub', 'rel_pub', 'color_pub', 'obstacle_pub', 'hb_pub', 'footprint_pub'}
    for name in pubs:
        getattr(n, name).publish = lambda m, name=name: n.sent.setdefault(name, []).append(m)
    for cam, pub in n.camera_footprint_pubs.items():
        pub.publish = lambda m, cam=cam: n.sent.setdefault(f'footprint/{cam}', []).append(m)
    yield n
    n.destroy_node()
    rclpy.shutdown()


def fly(node, x, y, height, yaw=0.0, stamp=[0]):
    import math
    from px4_msgs.msg import VehicleOdometry
    n, e, _ = node.frame.to_ned(x, y)
    m = VehicleOdometry()
    stamp[0] += 1
    m.timestamp = stamp[0]
    m.pose_frame = VehicleOdometry.POSE_FRAME_NED
    m.position = [float(n), float(e), float(-height)]
    m.q = [math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)]
    node.on_odometry(m)


# ---- one camera config --------------------------------------------------------------------------

def test_sim_and_mission_share_the_camera_config(node):
    from gnc_offboard.mission.cameras import load_cameras
    from gnc_offboard.mission.config import MissionConfig, load_yaml_params
    assert node.models == load_cameras(CAMERAS_YAML)
    cfg = MissionConfig.from_dict(load_yaml_params(CONFIG_YAML))
    assert cfg.cameras_file == ''                       # both default to the installed copy
    assert node.get_parameter('down.hfov_deg').value == 120.0
    assert node.get_parameter('d455.pitch_down_deg').value == 45.0


def test_mission_node_loads_the_same_cameras():
    from gnc_offboard.mission.cameras import load_cameras
    from gnc_offboard.ros.mission_manager_node import MissionManagerNode
    rclpy.init(args=['--ros-args', '-p', f'field_file:={TEST_MANIFEST}',
                     '-p', f'cameras_file:={CAMERAS_YAML}'])
    try:
        mm = MissionManagerNode(config_path=str(CONFIG_YAML))
        assert mm.ctx.cameras == load_cameras(CAMERAS_YAML)
        mm.destroy_node()
    finally:
        rclpy.shutdown()


# ---- footprints and coverage --------------------------------------------------------------------

def test_both_footprints_published_and_down_kept_on_the_old_topic(node):
    fly(node, -2.0, -2.0, 2.5)
    node.publish_visuals()
    down = node.sent['footprint/down'][-1].polygon.points
    d455 = node.sent['footprint/d455'][-1].polygon.points
    assert len(down) >= 4 and len(d455) >= 3
    assert node.sent['footprint_pub'][-1].polygon.points == down
    assert min(p.y for p in d455) > -2.0                 # D455 looks ahead (north at yaw 0)


def test_coverage_is_the_union_of_both_cameras(node):
    fly(node, -2.0, -2.0, 2.5)
    from gnc_offboard.camera_model import CoverageGrid
    down_only = CoverageGrid(node.grid.bounds, node.grid.resolution)
    down_only.mark(node.polygons['down'])
    assert node.grid.count > down_only.count
    assert node.grid.is_seen(-2.0, 2.5)                  # only the D455 reaches 4.5 m ahead


def test_live_geometry_change_per_camera(node):
    from rclpy.parameter import Parameter
    result = node.set_parameters([Parameter('down.hfov_deg', value=60.0)])[0]
    assert result.successful and node.models['down'].hfov_deg == 60.0
    assert node.models['d455'].hfov_deg == 90.0
    refused = node.set_parameters([Parameter('resolution_m', value=0.5)])[0]
    assert not refused.successful


# ---- Perception stand-in ------------------------------------------------------------------------

def test_dice_map_from_either_camera(node):
    fly(node, -2.0, -2.0, 2.5)          # red (2, 2) out of the down view, but in the D455's?
    node.detect()
    node.publish_map()
    seen = {d.color for d in node.sent['map_pub'][-1].dice}
    in_any = {d.color for d in node.perception.dice if node.in_any_view(d.x, d.y)}
    assert seen == in_any and seen
    assert 'color_pub' not in node.sent                  # too high to confirm


def test_color_and_target_relative_only_from_the_down_camera(node):
    from avc_interfaces.msg import MissionTarget
    t = MissionTarget()
    t.known, t.centroid.x, t.centroid.y = True, 2.0, 2.0
    node.on_target(t)
    fly(node, 2.1, 2.0, 1.0)            # red die below, hover height
    node.detect()
    assert node.sent['color_pub'][-1].color_guess == 'red'
    rel = node.sent['rel_pub'][-1]
    assert rel.valid and (rel.offset.x, rel.offset.y) == pytest.approx((-0.1, 0.0), abs=1e-4)
    # Same die seen only by the D455 (looking at it from 3 m south): no target_relative.
    fly(node, 2.0, -1.5, 1.0)
    assert node.in_any_view(2.0, 2.0) and not node.in_down_view(2.0, 2.0)
    node.detect()
    assert node.sent['rel_pub'][-1].valid is False


def test_switches(node):
    from rclpy.parameter import Parameter
    fly(node, 2.0, 2.0, 1.0)
    assert node.set_parameters([Parameter('drop_detections', value=True),
                                Parameter('heartbeat', value=False)])[0].successful
    node.detect()
    node.publish_heartbeat()
    assert 'color_pub' not in node.sent and 'hb_pub' not in node.sent
    node.publish_obstacles()
    assert node.sent['obstacle_pub'][-1].obstacles == []     # test world has none


def test_wrong_color_first(tmp_path):
    from gnc_offboard.camera_fov import CameraFov
    rclpy.init(args=['--ros-args', '-p', f'manifest:={TEST_MANIFEST}',
                     '-p', f'cameras_file:={CAMERAS_YAML}', '-p', 'gazebo_markers:=false',
                     '-p', 'wrong_color_first:=red'])
    try:
        n = CameraFov()
        fly(n, -4.5, -1.0, 2.5)          # black die first, under the drone
        n.detect()
        reported = dict((c, p) for c, p in n.perceived.dice)
        assert 'red' in reported and reported['red'][:2] == (-4.5, -1.0)
        n.destroy_node()
    finally:
        rclpy.shutdown()


# ---- coverage JSON consumers --------------------------------------------------------------------

def test_coverage_json_still_works_with_compare_and_plot(node, tmp_path):
    for x in (-4.0, -1.0, 2.0):
        fly(node, x, -1.0, 2.5)
    node.save()
    path = tmp_path / 'coverage.json'
    data = json.loads(path.read_text())
    for key in ('bounds', 'resolution_m', 'searched_cell_centres', 'searched_area_m2',
                'arena_area_m2', 'trajectory_enu', 'observed_dice', 'target_found', 'manifest'):
        assert key in data
    assert path.with_suffix('.png').read_bytes().startswith(b'\x89PNG')
    out = subprocess.run([sys.executable, str(REPO / 'tools' / 'compare_search_runs.py'),
                          str(path), str(path)], capture_output=True, text=True, timeout=60)
    assert out.returncode == 0, out.stderr
    assert 'coverage' in out.stdout.lower()


# ---- Milestone 6: blind height, noise, rangefinder stand-in -------------------------------------

def test_no_target_relative_or_color_below_the_blind_height(node):
    from avc_interfaces.msg import MissionTarget
    from gnc_offboard.mission.cameras import load_blind_height
    assert node.blind_height_m == load_blind_height(CAMERAS_YAML) == 0.20
    t = MissionTarget()
    t.known, t.centroid.x, t.centroid.y = True, 2.0, 2.0
    node.on_target(t)
    fly(node, 2.0, 2.0, 0.3)
    node.detect()
    assert node.sent['rel_pub'][-1].valid and node.sent['color_pub'][-1].color_guess == 'red'
    colors = len(node.sent['color_pub'])
    fly(node, 2.0, 2.0, 0.15)            # below the blind height
    node.detect()
    assert node.sent['rel_pub'][-1].valid is False and len(node.sent['color_pub']) == colors


def test_target_relative_noise(tmp_path):
    from avc_interfaces.msg import MissionTarget
    from gnc_offboard.camera_fov import CameraFov
    rclpy.init(args=['--ros-args', '-p', f'manifest:={TEST_MANIFEST}',
                     '-p', f'cameras_file:={CAMERAS_YAML}', '-p', 'gazebo_markers:=false',
                     '-p', 'noise_seed:=3'])
    try:
        n = CameraFov()
        assert n.get_parameter('target_relative_noise_m').value == 0.02
        sent = []
        n.rel_pub.publish = sent.append
        t = MissionTarget()
        t.known, t.centroid.x, t.centroid.y = True, 2.0, 2.0
        n.on_target(t)
        fly(n, 2.0, 2.0, 1.0)
        for _ in range(200):
            n.detect()
        xs = [m.offset.x for m in sent]
        mean = sum(xs) / len(xs)
        std = (sum((x - mean) ** 2 for x in xs) / len(xs)) ** 0.5
        assert abs(mean) < 0.01 and std == pytest.approx(0.02, abs=0.005)
        n.destroy_node()
    finally:
        rclpy.shutdown()


def test_rangefinder_stand_in(node):
    from px4_msgs.msg import DistanceSensor
    from gnc_offboard.mission import qos
    sent = []
    node.range_pub.publish = sent.append
    fly(node, 0.0, 0.0, 1.5)
    node.publish_range()
    m = sent[-1]
    assert m.current_distance == pytest.approx(1.5, abs=0.03)
    assert m.orientation == DistanceSensor.ROTATION_DOWNWARD_FACING and m.signal_quality == 100
    assert node.range_pub.topic_name == '/fmu/out/distance_sensor'
    assert node.range_pub.qos_profile == qos.PX4      # same QoS mission_manager subscribes with


def test_landing_accuracy_tool_records_touchdowns():
    import importlib.util
    from avc_msgs.msg import MissionState
    from px4_msgs.msg import VehicleOdometry
    spec = importlib.util.spec_from_file_location('landing_accuracy',
                                                  REPO / 'tools' / 'landing_accuracy.py')
    tool = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(tool)
    rclpy.init()
    try:
        node = tool.LandingAccuracy(TEST_MANIFEST, 2, '/fmu/out/vehicle_odometry')
        for offset in (0.01, 0.03):
            odom = VehicleOdometry()
            n, e, _ = node.frame.to_ned(2.0 + offset, 2.0)
            odom.position = [float(n), float(e), 0.0]
            node.on_odom(odom)
            for state in ('DESCENDING', 'ON_DIE_TEST', 'LIFTOFF'):
                m = MissionState()
                m.state, m.active_target = state, 'red'
                node.on_state(m)
        assert node.done and node.offsets == pytest.approx([0.01, 0.03], abs=1e-5)
        assert 'mean offset 2.0 cm, worst 3.0 cm' in node.summary()
        node.destroy_node()
    finally:
        rclpy.shutdown()
