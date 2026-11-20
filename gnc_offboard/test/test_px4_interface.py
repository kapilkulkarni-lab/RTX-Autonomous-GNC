"""guidance/px4_interface.py: the PX4 code lifted from mission_demo.py."""
import math

import pytest
from px4_msgs.msg import VehicleCommand

from conftest import AUTO_LAND, OFFBOARD, POSCTL, battery, land_detected, position, status
from gnc_offboard.guidance import px4_interface as px4
from gnc_offboard.mission import qos


@pytest.mark.parametrize('names, expected', [
    ([], '/fmu/out/vehicle_status'),
    (['/fmu/out/vehicle_status'], '/fmu/out/vehicle_status'),
    (['/fmu/out/vehicle_status_v1', '/other'], '/fmu/out/vehicle_status_v1'),
    (['/fmu/out/vehicle_status', '/fmu/out/vehicle_status_v1'], '/fmu/out/vehicle_status_v1'),
    (['/fmu/out/vehicle_status_v2', '/fmu/out/vehicle_status_v10'],
     '/fmu/out/vehicle_status_v10'),
    (['/fmu/out/vehicle_status_extra'], '/fmu/out/vehicle_status'),
])
def test_resolve_topic(names, expected):
    assert px4.resolve_topic('/fmu/out/vehicle_status', names) == expected


def test_offboard_control_mode_is_position():
    m = px4.offboard_control_mode(123)
    assert m.timestamp == 123 and m.position
    assert not (m.velocity or m.acceleration or m.attitude or m.body_rate)


def test_trajectory_setpoint_nan_means_not_given():
    m = px4.trajectory_setpoint((1.0, 2.0, -1.5), 0.3, 99)
    assert list(m.position) == [1.0, 2.0, -1.5]
    assert all(math.isnan(v) for v in m.velocity) and all(math.isnan(v) for v in m.acceleration)
    assert m.yaw == pytest.approx(0.3) and math.isnan(m.yawspeed)
    m = px4.trajectory_setpoint((0, 0, 0), 0.0, 1, vel=(1, 0, 0), acc=(0, 1, 0), yawspeed=0.2)
    assert list(m.velocity) == [1.0, 0.0, 0.0] and list(m.acceleration) == [0.0, 1.0, 0.0]


@pytest.mark.parametrize('builder, command, p1, p2', [
    (px4.arm_command, VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 0.0),
    (px4.disarm_command, VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0, 0.0),
    (px4.offboard_command, VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0),
    (px4.land_command, VehicleCommand.VEHICLE_CMD_NAV_LAND, 0.0, 0.0),
])
def test_vehicle_commands(builder, command, p1, p2):
    m = builder(42)
    assert (m.command, m.param1, m.param2, m.timestamp) == (command, p1, p2, 42)
    assert m.target_system == m.target_component == 1
    assert m.source_system == m.source_component == 1
    assert m.from_external


def test_disarm_is_not_forced():
    """A forced disarm (param2 21196) would cut motors in flight; we never send it."""
    assert px4.disarm_command(0).param2 == 0.0


def test_status_predicates():
    assert px4.armed_offboard(status(True, OFFBOARD))
    assert not px4.armed_offboard(status(True, POSCTL))
    assert not px4.armed_offboard(status(False, OFFBOARD))
    assert px4.is_auto_land(status(True, AUTO_LAND))
    assert px4.is_disarmed(status(False)) and not px4.is_disarmed(None)
    assert not px4.is_armed(None) and not px4.is_offboard(None)
    assert px4.in_failsafe(status(failsafe=True)) and not px4.in_failsafe(None)


def test_position_land_battery():
    assert px4.position_valid(position()) and not px4.position_valid(position(valid=False))
    assert not px4.position_valid(None)
    p = position()
    p.z_valid = False
    assert not px4.position_valid(p)
    assert px4.is_landed(land_detected(True)) and not px4.is_landed(land_detected(False))
    assert px4.battery_percent(battery(0.42)) == pytest.approx(42.0)
    assert px4.battery_percent(battery(-1.0)) is None
    assert px4.battery_percent(battery(float('nan'))) is None
    assert px4.battery_percent(None) is None


def test_px4_qos_is_the_one_profile():
    assert px4.PX4_QOS is qos.PX4


def test_px4_interface_publishes_on_resolved_topics():
    rclpy = pytest.importorskip('rclpy')
    from rclpy.node import Node
    rclpy.init()
    try:
        node = Node('px4_interface_test')
        iface = px4.Px4Interface(node)
        pubs = {p.topic_name for p in node.publishers}
        assert set(px4.PX4_IN_TOPICS) <= pubs
        iface.stream((0.0, 0.0, -1.0), 0.0)
        iface.arm(); iface.set_offboard(); iface.land(); iface.disarm()
        node.destroy_node()
    finally:
        rclpy.shutdown()
