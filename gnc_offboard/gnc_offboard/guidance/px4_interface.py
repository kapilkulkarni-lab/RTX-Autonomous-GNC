"""PX4 offboard interface: arming, disarming, the offboard handshake and setpoints.

Lifted from the retired mission_demo.py. The top half is pure (message builders, topic name
resolution, status predicates) and is unit-tested without a ROS graph. `Px4Interface` is the
thin ROS part: it owns the /fmu/in publishers on a node.

Offboard handshake (PX4 rule): stream OffboardControlMode + TrajectorySetpoint continuously,
for about a second before requesting offboard, and keep streaming for as long as offboard
should stay engaged. Stop streaming and PX4 leaves offboard (its failsafe).
"""
import math
import re
from typing import Iterable, Optional, Sequence

from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint, VehicleCommand,
                          VehicleStatus)

from gnc_offboard.mission.qos import PX4 as PX4_QOS

NAN3 = [float('nan')] * 3

# PX4 topics this package uses (PX4 may add a version suffix such as _v1).
PX4_OUT_TOPICS = ('/fmu/out/vehicle_status', '/fmu/out/vehicle_local_position',
                  '/fmu/out/vehicle_land_detected', '/fmu/out/battery_status')
PX4_IN_TOPICS = ('/fmu/in/offboard_control_mode', '/fmu/in/trajectory_setpoint',
                 '/fmu/in/vehicle_command')
DISCOVERY_TOPIC = '/fmu/out/vehicle_local_position'

# VEHICLE_CMD_DO_SET_MODE params: base mode "custom" (1), PX4 custom main mode OFFBOARD (6).
_MODE_CUSTOM = 1.0
_PX4_MAIN_MODE_OFFBOARD = 6.0


# --- Topic names ---------------------------------------------------------------------------------

def resolve_topic(base: str, names: Iterable[str]) -> str:
    """The name PX4 actually uses for `base`: base itself or base_vN (highest N wins)."""
    pat = re.compile(re.escape(base) + r'(_v\d+)?')
    found = sorted((n for n in names if pat.fullmatch(n)), key=_version)
    return found[-1] if found else base


def _version(name: str) -> int:
    m = re.search(r'_v(\d+)$', name)
    return int(m.group(1)) if m else 0


# --- Message builders (timestamps in microseconds) -----------------------------------------------

def offboard_control_mode(timestamp_us: int) -> OffboardControlMode:
    """Offboard heartbeat: position is primary; velocity/acceleration are feedforward only."""
    m = OffboardControlMode()
    m.timestamp = int(timestamp_us)
    m.position = True
    return m


def trajectory_setpoint(ned: Sequence[float], yaw: float, timestamp_us: int,
                        vel: Optional[Sequence[float]] = None,
                        acc: Optional[Sequence[float]] = None,
                        yawspeed: float = float('nan')) -> TrajectorySetpoint:
    """Position setpoint in PX4 local NED. NaN fields tell PX4 'not given'."""
    m = TrajectorySetpoint()
    m.timestamp = int(timestamp_us)
    m.position = [float(v) for v in ned]
    m.velocity = [float(v) for v in vel] if vel is not None else list(NAN3)
    m.acceleration = [float(v) for v in acc] if acc is not None else list(NAN3)
    m.yaw = float(yaw)
    m.yawspeed = float(yawspeed)
    return m


def vehicle_command(command: int, timestamp_us: int, param1: float = 0.0,
                    param2: float = 0.0) -> VehicleCommand:
    m = VehicleCommand()
    m.timestamp = int(timestamp_us)
    m.command = int(command)
    m.param1, m.param2 = float(param1), float(param2)
    m.target_system = m.target_component = 1
    m.source_system = m.source_component = 1
    m.from_external = True
    return m


def arm_command(timestamp_us: int) -> VehicleCommand:
    return vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, timestamp_us, 1.0)


def disarm_command(timestamp_us: int) -> VehicleCommand:
    """Normal disarm: PX4 refuses it in flight (no force flag), which is what we want."""
    return vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, timestamp_us, 0.0)


def offboard_command(timestamp_us: int) -> VehicleCommand:
    return vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, timestamp_us,
                           _MODE_CUSTOM, _PX4_MAIN_MODE_OFFBOARD)


def land_command(timestamp_us: int) -> VehicleCommand:
    return vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND, timestamp_us)


# --- Status predicates (None = not received yet) -------------------------------------------------

def is_armed(status) -> bool:
    return status is not None and status.arming_state == VehicleStatus.ARMING_STATE_ARMED


def is_disarmed(status) -> bool:
    return status is not None and status.arming_state == VehicleStatus.ARMING_STATE_DISARMED


def is_offboard(status) -> bool:
    return status is not None and status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD


def is_auto_land(status) -> bool:
    return status is not None and status.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LAND


def armed_offboard(status) -> bool:
    return is_armed(status) and is_offboard(status)


def in_failsafe(status) -> bool:
    return status is not None and bool(status.failsafe)


def position_valid(local_position) -> bool:
    return (local_position is not None and bool(local_position.xy_valid)
            and bool(local_position.z_valid))


def is_landed(land_detected) -> bool:
    return land_detected is not None and bool(land_detected.landed)


def battery_percent(battery) -> Optional[float]:
    """Remaining battery in percent, or None if unknown (no message, -1, NaN)."""
    if battery is None:
        return None
    r = float(battery.remaining)
    if math.isnan(r) or r < 0.0:
        return None
    return 100.0 * r


# --- ROS part ------------------------------------------------------------------------------------

def px4_topics_visible(node) -> bool:
    pat = re.compile(re.escape(DISCOVERY_TOPIC) + r'(_v\d+)?')
    return any(pat.fullmatch(n) for n, _ in node.get_topic_names_and_types())


def wait_for_px4(node, timeout_s: float) -> bool:
    """Spin until DDS discovery sees PX4, so versioned topic names resolve. False on timeout."""
    import rclpy
    deadline = node.get_clock().now().nanoseconds * 1e-9 + timeout_s
    while not px4_topics_visible(node):
        if node.get_clock().now().nanoseconds * 1e-9 > deadline:
            node.get_logger().error(
                f'No PX4 topics after {timeout_s:.0f} s. Is the sim + XRCE agent running?')
            return False
        node.get_logger().info('Waiting for PX4 topics...', throttle_duration_sec=2.0)
        rclpy.spin_once(node, timeout_sec=0.2)
    return True


class Px4Interface:
    """Publishers for /fmu/in on `node`, with PX4's resolved (possibly versioned) topic names."""

    def __init__(self, node):
        self.node = node
        names = [n for n, _ in node.get_topic_names_and_types()]
        self.topics = {b: resolve_topic(b, names) for b in PX4_OUT_TOPICS + PX4_IN_TOPICS}
        for base in PX4_IN_TOPICS + PX4_OUT_TOPICS:
            node.get_logger().info(f'topic {self.topics[base]}')
        self._mode_pub = node.create_publisher(
            OffboardControlMode, self.topics['/fmu/in/offboard_control_mode'], PX4_QOS)
        self._sp_pub = node.create_publisher(
            TrajectorySetpoint, self.topics['/fmu/in/trajectory_setpoint'], PX4_QOS)
        self._cmd_pub = node.create_publisher(
            VehicleCommand, self.topics['/fmu/in/vehicle_command'], PX4_QOS)

    def timestamp_us(self) -> int:
        return int(self.node.get_clock().now().nanoseconds / 1000)

    def publish_offboard_mode(self) -> None:
        self._mode_pub.publish(offboard_control_mode(self.timestamp_us()))

    def publish_setpoint(self, ned, yaw, vel=None, acc=None, yawspeed=float('nan')) -> None:
        self._sp_pub.publish(trajectory_setpoint(ned, yaw, self.timestamp_us(), vel, acc, yawspeed))

    def stream(self, ned, yaw, vel=None, acc=None, yawspeed=float('nan')) -> None:
        """One tick of the offboard stream: heartbeat + setpoint."""
        self.publish_offboard_mode()
        self.publish_setpoint(ned, yaw, vel, acc, yawspeed)

    def arm(self) -> None:
        self._cmd_pub.publish(arm_command(self.timestamp_us()))

    def disarm(self) -> None:
        self._cmd_pub.publish(disarm_command(self.timestamp_us()))

    def set_offboard(self) -> None:
        self._cmd_pub.publish(offboard_command(self.timestamp_us()))

    def land(self) -> None:
        self._cmd_pub.publish(land_command(self.timestamp_us()))
