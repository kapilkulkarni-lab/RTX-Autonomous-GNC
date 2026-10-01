"""Bird Dog demo mission: take off, fly to the target die, hover, return to the start zone, land.

Waypoints are straight lines at a fixed altitude. This node is the *execution* layer:
the path planner's job is to replace `self.waypoints` with a real, obstacle-aware path.

Run (sim + agent already up, drone landed and disarmed):
    ros2 run gnc_offboard mission_demo
    ros2 run gnc_offboard mission_demo --ros-args -p altitude:=2.0 -p speed:=0.8
    ros2 run gnc_offboard mission_demo --ros-args -p waypoints:="[0.0, 0.0, 3.0, -2.0]"
"""
import json
import math
import os
import re
from pathlib import Path

import rclpy
from rclpy.exceptions import ParameterUninitializedException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint, VehicleCommand,
                          VehicleLocalPosition, VehicleStatus)

from gnc_offboard.frames import FieldFrame

# PX4's uXRCE-DDS topics require this QoS, or subscriptions silently receive nothing.
PX4_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     history=HistoryPolicy.KEEP_LAST, depth=1)

RATE_HZ = 20.0
NAN3 = [float('nan')] * 3


class MissionDemo(Node):
    def __init__(self):
        super().__init__('mission_demo')
        sim_dir = os.environ.get('AVC_SIM_DIR', str(Path.home() / 'dev/avc_sim'))
        default_manifest = str(Path(sim_dir) / 'worlds/avc_field_manifest.json')
        self.declare_parameter('manifest', default_manifest)
        self.declare_parameter('altitude', 1.5)          # m above spawn
        self.declare_parameter('speed', 1.0)             # m/s, carrot speed (indoors!)
        self.declare_parameter('acceptance_radius', 0.25)
        self.declare_parameter('hover_time', 3.0)        # s at each waypoint
        self.declare_parameter('waypoints', Parameter.Type.DOUBLE_ARRAY)  # field x,y pairs

        p = lambda name: self.get_parameter(name).value  # noqa: E731
        self.alt, self.speed = p('altitude'), p('speed')
        self.acc_r, self.hover_time = p('acceptance_radius'), p('hover_time')

        manifest = json.loads(Path(p('manifest')).read_text())
        self.frame = FieldFrame.from_manifest(manifest)
        try:
            field_wps = p('waypoints')
        except ParameterUninitializedException:
            field_wps = None  # not given on the command line: use the demo mission
        if field_wps:
            if len(field_wps) % 2:
                raise ValueError('waypoints must be x,y pairs')
            pts = list(zip(field_wps[0::2], field_wps[1::2]))
            self.get_logger().info(f'Flying {len(pts)} custom waypoints')
        else:
            color = manifest['scenario']['initial_target']
            die = next(t for t in manifest['targets'] if t['color'] == color)
            zone = manifest['start_zone']['center']
            pts = [(die['x'], die['y']), (zone[0], zone[1])]
            self.get_logger().info(
                f"Target: {color} die at field ({die['x']:.2f}, {die['y']:.2f}); "
                'will return to start zone and land')
        # Mission waypoints in PX4 local NED; the takeoff point is prepended at arming.
        self.waypoints = [self.frame.to_ned(x, y, self.alt) for x, y in pts]

        self.pos = None          # latest VehicleLocalPosition
        self.status = None       # latest VehicleStatus
        self.state = 'WAIT_POS'
        self.stream_count = 0
        self.last_cmd_time = -1.0
        self.carrot = None       # moving setpoint [n, e, d]
        self.wp_idx = 0
        self.hover_start = None
        self.yaw = float('nan')

    # ---------------------------------------------------------------- setup
    def resolve(self, base):
        """PX4 may version topic names (e.g. vehicle_status_v1). Use whatever exists."""
        names = [n for n, _ in self.get_topic_names_and_types()]
        pat = re.compile(re.escape(base) + r'(_v\d+)?')
        found = sorted(n for n in names if pat.fullmatch(n))
        return found[-1] if found else base

    def setup_io(self):
        t = {b: self.resolve(b) for b in (
            '/fmu/out/vehicle_local_position', '/fmu/out/vehicle_status',
            '/fmu/in/offboard_control_mode', '/fmu/in/trajectory_setpoint',
            '/fmu/in/vehicle_command')}
        for b, r in t.items():
            self.get_logger().info(f'topic {r}')
        self.create_subscription(VehicleLocalPosition, t['/fmu/out/vehicle_local_position'],
                                 self.on_pos, PX4_QOS)
        self.create_subscription(VehicleStatus, t['/fmu/out/vehicle_status'],
                                 self.on_status, PX4_QOS)
        self.mode_pub = self.create_publisher(OffboardControlMode,
                                              t['/fmu/in/offboard_control_mode'], PX4_QOS)
        self.sp_pub = self.create_publisher(TrajectorySetpoint,
                                            t['/fmu/in/trajectory_setpoint'], PX4_QOS)
        self.cmd_pub = self.create_publisher(VehicleCommand,
                                             t['/fmu/in/vehicle_command'], PX4_QOS)
        self.create_timer(1.0 / RATE_HZ, self.tick)

    def on_pos(self, msg):
        if msg.xy_valid and msg.z_valid:
            self.pos = msg

    def on_status(self, msg):
        self.status = msg

    # ---------------------------------------------------------------- helpers
    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def stamp(self):
        return int(self.get_clock().now().nanoseconds / 1000)

    def send_cmd(self, command, p1=0.0, p2=0.0):
        m = VehicleCommand()
        m.timestamp = self.stamp()
        m.command = command
        m.param1, m.param2 = float(p1), float(p2)
        m.target_system = m.target_component = 1
        m.source_system = m.source_component = 1
        m.from_external = True
        self.cmd_pub.publish(m)

    def publish_mode(self):
        m = OffboardControlMode()
        m.timestamp = self.stamp()
        m.position = True
        self.mode_pub.publish(m)

    def publish_setpoint(self, ned, yaw):
        m = TrajectorySetpoint()
        m.timestamp = self.stamp()
        m.position = [float(v) for v in ned]
        m.velocity = NAN3
        m.acceleration = NAN3
        m.yaw = float(yaw)
        self.sp_pub.publish(m)

    def vehicle_ned(self):
        return [self.pos.x, self.pos.y, self.pos.z]

    def armed_offboard(self):
        s = self.status
        return (s is not None
                and s.arming_state == VehicleStatus.ARMING_STATE_ARMED
                and s.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD)

    def advance_carrot(self, target):
        """Move the setpoint toward `target` at `speed`, so PX4 never sees a big jump."""
        step = self.speed / RATE_HZ
        d = [t - c for t, c in zip(target, self.carrot)]
        dist = math.sqrt(sum(v * v for v in d))
        if dist <= step:
            self.carrot = list(target)
        else:
            self.carrot = [c + v * step / dist for c, v in zip(self.carrot, d)]
        if math.hypot(d[0], d[1]) > 0.3:  # face direction of horizontal travel
            self.yaw = math.atan2(d[1], d[0])

    # ---------------------------------------------------------------- state machine
    def tick(self):
        if self.state == 'WAIT_POS':
            if self.pos is None:
                self.get_logger().info('Waiting for a valid position estimate...',
                                       throttle_duration_sec=2.0)
                return
            self.carrot = self.vehicle_ned()
            self.yaw = self.pos.heading
            self.state = 'STREAM'

        if self.state in ('DONE', 'LANDING', 'ABORTED'):
            if self.state == 'LANDING' and self.status is not None \
                    and self.status.arming_state == VehicleStatus.ARMING_STATE_DISARMED:
                self.get_logger().info('Landed and disarmed. Mission complete.')
                self.state = 'DONE'
                raise SystemExit
            return

        # Offboard requires a continuous setpoint stream, before AND during the mode.
        self.publish_mode()

        if self.state == 'STREAM':
            self.publish_setpoint(self.carrot, self.yaw)
            self.stream_count += 1
            if self.stream_count >= int(RATE_HZ):  # ~1 s of setpoints first
                self.state = 'ARMING'
            return

        if self.state == 'ARMING':
            self.publish_setpoint(self.carrot, self.yaw)
            if self.armed_offboard():
                n, e, _ = self.vehicle_ned()
                self.waypoints.insert(0, (n, e, -self.alt))  # climb in place first
                self.get_logger().info('Armed in offboard. Taking off.')
                self.state = 'FLYING'
            elif self.now_s() - self.last_cmd_time > 1.0:
                self.send_cmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)  # offboard
                self.send_cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
                self.last_cmd_time = self.now_s()
            return

        # FLYING
        if not self.armed_offboard():
            self.get_logger().warn('Left offboard mode (pilot/failsafe). Stopping commands.')
            self.state = 'ABORTED'
            return

        target = self.waypoints[self.wp_idx]
        self.advance_carrot(target)
        self.publish_setpoint(self.carrot, self.yaw)

        err = math.dist(self.vehicle_ned(), target)
        if self.carrot == list(target) and err < self.acc_r:
            if self.hover_start is None:
                self.hover_start = self.now_s()
                x, y, z = self.frame.to_field(*target)
                self.get_logger().info(
                    f'Reached waypoint {self.wp_idx} (field {x:.2f}, {y:.2f}, alt {z:.2f})')
            elif self.now_s() - self.hover_start >= self.hover_time:
                self.hover_start = None
                self.wp_idx += 1
                if self.wp_idx >= len(self.waypoints):
                    self.get_logger().info('Path complete. Landing.')
                    self.send_cmd(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                    self.state = 'LANDING'


def main():
    rclpy.init()
    node = MissionDemo()
    # Wait for DDS discovery to see PX4, so versioned topic names resolve correctly.
    pat = re.compile(r'/fmu/out/vehicle_local_position(_v\d+)?')
    deadline = node.now_s() + 20.0
    while not any(pat.fullmatch(n) for n, _ in node.get_topic_names_and_types()):
        if node.now_s() > deadline:
            node.get_logger().error('No PX4 topics after 20 s. Is the sim + XRCE agent running?')
            node.destroy_node()
            rclpy.shutdown()
            return
        node.get_logger().info('Waiting for PX4 topics...', throttle_duration_sec=2.0)
        rclpy.spin_once(node, timeout_sec=0.2)
    node.setup_io()
    try:
        rclpy.spin(node)
    except (SystemExit, KeyboardInterrupt):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
