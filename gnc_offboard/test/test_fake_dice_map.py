"""fake_dice_map node: camera projection + PerceptionModel + publishers, without a sim."""
import pytest

from conftest import TEST_MANIFEST

rclpy = pytest.importorskip('rclpy')


@pytest.fixture
def node():
    from gnc_offboard.sim.fake_dice_map import FakeDiceMap
    rclpy.init(args=['--ros-args', '-p', f'manifest:={TEST_MANIFEST}'])
    n = FakeDiceMap()
    n.sent = {}
    for name in ('map_pub', 'rel_pub', 'color_pub', 'obstacle_pub', 'hb_pub'):
        getattr(n, name).publish = lambda m, name=name: n.sent.setdefault(name, []).append(m)
    yield n
    n.destroy_node()
    rclpy.shutdown()


def odom(n, e, d):
    from px4_msgs.msg import VehicleOdometry
    m = VehicleOdometry()
    m.position = [float(n), float(e), float(d)]
    m.q = [1.0, 0.0, 0.0, 0.0]                  # level, facing north
    return m


def over(node, x, y, height):
    n, e, _ = node.frame.to_ned(x, y)
    node.on_odom(odom(n, e, -height))


def test_sees_and_confirms_the_die_below(node):
    over(node, 2.0, 2.0, 1.0)                   # red die at (2, 2), hover height
    node.detect()
    node.publish_map()
    dice = node.sent['map_pub'][-1].dice
    assert [(d.color, d.centroid.x, d.centroid.y) for d in dice] == [('red', 2.0, 2.0)]
    assert node.sent['color_pub'][-1].color_guess == 'red'


def test_nothing_before_odometry_and_far_away(node):
    node.detect()
    assert 'rel_pub' not in node.sent
    over(node, -6.0, -6.0, 1.5)                 # start corner: no die in view
    node.detect()
    node.publish_map()
    assert node.sent['map_pub'][-1].dice == [] and 'color_pub' not in node.sent
    assert node.sent['rel_pub'][-1].valid is False


def test_target_relative_follows_mission_target(node):
    from avc_interfaces.msg import MissionTarget
    t = MissionTarget()
    t.known, t.centroid.x, t.centroid.y = True, 2.0, 2.0
    node.on_target(t)
    over(node, 1.8, 1.9, 1.0)
    node.detect()
    rel = node.sent['rel_pub'][-1]
    assert rel.valid and (rel.offset.x, rel.offset.y) == pytest.approx((0.2, 0.1), abs=1e-5)


def test_switches(node):
    over(node, 2.0, 2.0, 1.0)
    node.set_parameters([rclpy.parameter.Parameter('drop_detections', value=True),
                         rclpy.parameter.Parameter('heartbeat', value=False)])
    node.detect()
    node.publish_heartbeat()
    assert 'color_pub' not in node.sent and 'hb_pub' not in node.sent
    node.publish_obstacles()
    assert node.sent['obstacle_pub'][-1].obstacles == []      # test world has none
