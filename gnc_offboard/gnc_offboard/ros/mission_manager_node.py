"""mission_manager: the one node that flies the AVC mission.

Thin ROS wrapper around the ROS-free mission core (gnc_offboard/mission):
  - subscription callbacks only store the latest message + receive time in MissionContext
  - a 20 Hz timer streams the current setpoint (and the offboard heartbeat) to PX4, whatever
    the state machine is doing
  - /mission/state at state_pub_hz, and at once on every state change or handled intent
  - the SMACH state machine runs in a worker thread (its execute() blocks); shutdown preempts it

Parameters come from config/mission.yaml (installed copy) as defaults; override with
`--ros-args -p name:=value` or `--params-file`.

Run (sim + XRCE agent up, drone landed and disarmed):
    ros2 run gnc_offboard mission_manager
"""
import os
import threading
import time
from typing import Optional

import rclpy
from rclpy.node import Node

from avc_interfaces.msg import (ColorCheck, DiceMap, MissionTarget, ObstacleArray,
                                PerceptionHeartbeat, TargetRelative)
from avc_msgs.msg import Intent, LlmHeartbeat, MissionState
from px4_msgs.msg import (BatteryStatus, VehicleLandDetected, VehicleLocalPosition,
                          VehicleOdometry, VehicleStatus)

from gnc_offboard.field_map import default_manifest_path
from gnc_offboard.guidance import px4_interface as px4
from gnc_offboard.mission import qos
from gnc_offboard.mission.cameras import default_cameras_path, load_cameras
from gnc_offboard.mission.clock import MonotonicClock
from gnc_offboard.mission.config import MissionConfig, load_yaml_params
from gnc_offboard.mission.context import MissionContext
from gnc_offboard.mission.field import FieldConfig
from gnc_offboard.mission.io import MissionIO
from gnc_offboard.mission.mission_manager_smach import build_state_machine, run_mission
from gnc_offboard.mission.state_msg import MissionStateBuilder

NODE_NAME = 'mission_manager'


def default_config_path() -> str:
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory('gnc_offboard'), 'config', 'mission.yaml')


class RosMissionIO(MissionIO):
    """MissionIO on the node. Setpoints go to the context for the stream timer; PX4 commands
    go straight out. Hardware topics are wired in Milestone 7."""

    def __init__(self, node: 'MissionManagerNode'):
        self.node = node

    def set_setpoint(self, x, y, z, yaw):
        with self.node.ctx.lock:
            self.node.ctx.setpoint = (float(x), float(y), float(z), float(yaw))

    def clear_setpoint(self):
        with self.node.ctx.lock:
            self.node.ctx.setpoint = None

    def arm(self):
        self.node.px4.arm()

    def disarm(self):
        self.node.px4.disarm()

    def set_offboard(self):
        self.node.px4.set_offboard()

    def land(self):
        self.node.px4.land()

    def nfc_command(self, command):
        self.log('warn', f'nfc_command({command!r}) not wired until Milestone 7')

    def gripper_command(self, command):
        self.log('warn', f'gripper_command({command!r}) not wired until Milestone 7')

    def publish_nfc_result(self, msg):
        self.log('warn', 'publish_nfc_result not wired until Milestone 7')

    def request_state_publish(self):
        self.node.publish_state()

    def log(self, level, text):
        logger = self.node.get_logger()
        {'info': logger.info, 'warn': logger.warning, 'error': logger.error}.get(
            level, logger.info)(text)


class MissionManagerNode(Node):

    def __init__(self, config_path: Optional[str] = None):
        super().__init__(NODE_NAME)
        defaults = load_yaml_params(config_path or default_config_path())
        for name in MissionConfig.names():
            self.declare_parameter(name, defaults[name])
        self.cfg = MissionConfig.from_dict(
            {name: self.get_parameter(name).value for name in MissionConfig.names()})
        if self.cfg.require_pilot_arm:
            self.get_logger().info('require_pilot_arm: true. The pilot arms; GNC never sends arm.')
        if not self.cfg.require_perception:
            self.get_logger().warning(
                'require_perception: false. The Perception heartbeat check is OFF '
                '(lab flights without Perception only).')

        self.ctx = MissionContext()
        field_file = self.cfg.field_file or str(default_manifest_path())
        self.ctx.field_config = FieldConfig.load(field_file)
        self.get_logger().info(f'Field frame from {field_file}')
        cameras_file = self.cfg.cameras_file or default_cameras_path()
        self.ctx.cameras = load_cameras(cameras_file)
        self.get_logger().info(f'Cameras {sorted(self.ctx.cameras)} from {cameras_file}')
        self._last_target = None
        self.io = RosMissionIO(self)
        self.clock = MonotonicClock()
        self.builder = MissionStateBuilder()
        self._pub_lock = threading.Lock()
        self.px4: Optional[px4.Px4Interface] = None
        self.sm = None
        self.worker: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self.sm_outcome = None

        self.state_pub = self.create_publisher(MissionState, '/mission/state', qos.MISSION_STATE)
        self.create_subscription(Intent, '/mission/intent', self._store('intent'), qos.INTENT)
        self.create_subscription(LlmHeartbeat, '/llm/heartbeat', self._store('llm_heartbeat'),
                                 qos.LLM_HEARTBEAT)
        for msg_type, topic, name in (
                (DiceMap, '/perception/dice_map', 'dice_map'),
                (TargetRelative, '/perception/target_relative', 'target_relative'),
                (ColorCheck, '/perception/color_check', 'color_check'),
                (ObstacleArray, '/perception/obstacle_detections', 'obstacles'),
                (PerceptionHeartbeat, '/perception/heartbeat', 'perception_heartbeat')):
            self.create_subscription(msg_type, topic, self._store(name), qos.BY_TOPIC[topic])
        self.target_pub = self.create_publisher(MissionTarget, '/mission/target',
                                                qos.MISSION_TARGET)
        self.create_timer(1.0 / self.cfg.state_pub_hz, self.publish_state)
        self.create_timer(1.0 / self.cfg.state_pub_hz, self.publish_target)

    # ---- callbacks: store only ------------------------------------------------------------------
    def _store(self, name):
        def callback(msg):
            self.ctx.update_input(name, msg, time.monotonic())
        return callback

    # ---- PX4 ------------------------------------------------------------------------------------
    def setup_px4(self) -> None:
        """Create PX4 I/O once discovery has resolved the (possibly versioned) topic names."""
        self.px4 = px4.Px4Interface(self)
        t = self.px4.topics
        for msg_type, base, name in (
                (VehicleStatus, '/fmu/out/vehicle_status', 'vehicle_status'),
                (VehicleLocalPosition, '/fmu/out/vehicle_local_position', 'local_position'),
                (VehicleLandDetected, '/fmu/out/vehicle_land_detected', 'land_detected'),
                (BatteryStatus, '/fmu/out/battery_status', 'battery_status'),
                (VehicleOdometry, '/fmu/out/vehicle_odometry', 'odometry')):
            self.create_subscription(msg_type, t[base], self._store(name), qos.PX4)
        self.create_timer(1.0 / self.cfg.setpoint_hz, self.stream_setpoint)

    def stream_setpoint(self) -> None:
        """Offboard heartbeat + setpoint at setpoint_hz while the mission holds a setpoint."""
        with self.ctx.lock:
            sp = self.ctx.setpoint
        if sp is not None and self.px4 is not None:
            x, y, z, yaw = sp
            self.px4.stream((x, y, z), yaw)

    # ---- /mission/state -------------------------------------------------------------------------
    def publish_state(self) -> None:
        now = time.monotonic()
        with self.ctx.lock:
            if not self.ctx.state:
                return
            fields = dict(state=self.ctx.state, previous_state=self.ctx.previous_state,
                          paused_from=self.ctx.paused_from, reason=self.ctx.reason,
                          active_target=self.ctx.active_target,
                          last_intent_id=self.ctx.last_intent_id,
                          last_intent_result=self.ctx.last_intent_result)
        fields['mission_time_s'] = self.ctx.mission_time_s(now)
        with self._pub_lock:
            msg = self.builder.build(stamp_ns=self.get_clock().now().nanoseconds, **fields)
            self.state_pub.publish(msg)

    def publish_target(self) -> None:
        """/mission/target whenever the active color or its known centroid changes."""
        with self.ctx.lock:
            current = (self.ctx.active_target, self.ctx.target_centroid)
        if current == self._last_target or not current[0]:
            return
        self._last_target = current
        msg = MissionTarget()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.color = current[0]
        msg.known = current[1] is not None
        if msg.known:
            msg.centroid.x, msg.centroid.y, msg.centroid.z = (float(v) for v in current[1])
        self.target_pub.publish(msg)

    # ---- state machine thread -------------------------------------------------------------------
    def start_mission(self) -> None:
        self.sm = build_state_machine(self.ctx, self.io, self.cfg, self.clock)
        self.worker = threading.Thread(target=self._run_sm, name='mission_sm', daemon=True)
        self.worker.start()

    def _run_sm(self) -> None:
        try:
            # After the machine ends, run_mission keeps answering intents until shutdown.
            self.sm_outcome = run_mission(self.sm, self.ctx, self.io, self.cfg, self.clock,
                                          should_stop=self._stop.is_set)
            self.get_logger().info(f'Mission worker stopped: {self.sm_outcome}')
        except Exception as exc:     # never die silently: stop streaming, report, keep node up
            self.io.clear_setpoint()
            self.get_logger().error(f'State machine crashed: {exc!r}')

    def stop_mission(self, timeout_s: float = 2.0) -> None:
        self._stop.set()
        if self.sm is not None and self.worker is not None and self.worker.is_alive():
            self.sm.request_preempt()
            self.worker.join(timeout_s)


def _route_smach_logs(node: Node) -> None:
    """SMACH logs through its own functions; send them to the node's logger."""
    import smach
    logger = node.get_logger()
    smach.set_loggers(lambda m: logger.debug(m), lambda m: logger.warning(m),
                      lambda m: logger.debug(m), lambda m: logger.error(m))


def main(args=None):
    rclpy.init(args=args)
    node = MissionManagerNode()
    _route_smach_logs(node)
    try:
        if not px4.wait_for_px4(node, node.cfg.px4_discovery_timeout_s):
            return
        node.setup_px4()
        node.start_mission()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_mission()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
