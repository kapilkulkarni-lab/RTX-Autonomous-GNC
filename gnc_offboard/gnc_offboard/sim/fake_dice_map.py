"""fake_dice_map: SIM ONLY stand-in for Perception, from the world manifest and the camera
model. Publishes only what the downward camera has actually seen:

  /perception/dice_map             avc_interfaces/DiceMap (placeholder)         map_rate_hz
  /perception/target_relative      avc_interfaces/TargetRelative (placeholder)  rate_hz
  /perception/color_check          avc_interfaces/ColorCheck (placeholder)      rate_hz, only when
                                                                                a color is confirmed
  /perception/obstacle_detections  avc_interfaces/ObstacleArray (placeholder)   1 Hz, from the manifest
  /perception/heartbeat            avc_interfaces/PerceptionHeartbeat           heartbeat_hz

Test switches (also settable while running with `ros2 param set /fake_dice_map ...`):
  wrong_color_first:=red   the first die seen goes into the map as red until a close look
                           confirms its true color (startup only)
  drop_detections:=true    no map updates, target_relative or color_check
  heartbeat:=false         stop the heartbeat (Perception dropout)

Run (sim up): ros2 run gnc_offboard fake_dice_map
"""
import json
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node

from avc_interfaces.msg import (ColorCheck, DiceMap, DiceMapEntry, MissionTarget, Obstacle,
                                ObstacleArray, PerceptionHeartbeat, TargetRelative)
from px4_msgs.msg import VehicleOdometry

from gnc_offboard.camera_model import CameraModel, contains_point
from gnc_offboard.field_map import FieldMap, default_manifest_path
from gnc_offboard.frames import FieldFrame
from gnc_offboard.mission import qos
from gnc_offboard.sim.perception_model import PerceptionModel

PUB_DEPTH = 10      # publishers: reliable, volatile (mission_manager subscribes best effort)


class FakeDiceMap(Node):
    def __init__(self):
        super().__init__('fake_dice_map')
        p = self.declare_parameter
        p('manifest', str(default_manifest_path()))
        p('odometry_topic', '/fmu/out/vehicle_odometry')
        p('detect_fov_deg', 70.0)          # keep equal to mission_manager's detect_fov_deg
        p('pitch_down_deg', 90.0)          # one camera pointing straight down
        p('confirm_radius_m', 0.3)
        p('confirm_max_alt_m', 1.3)
        p('target_match_m', 1.0)
        p('wrong_color_first', '')
        p('drop_detections', False)
        p('heartbeat', True)
        p('rate_hz', 10.0)
        p('map_rate_hz', 2.0)
        p('heartbeat_hz', 5.0)
        g = lambda name: self.get_parameter(name).value  # noqa: E731

        manifest = json.loads(Path(g('manifest')).read_text())
        self.frame = FieldFrame.from_manifest(manifest)
        self.bounds = manifest['field']['bounds']
        self.field_map = FieldMap(manifest)
        fov = float(g('detect_fov_deg'))
        self.camera = CameraModel(hfov_deg=fov, vfov_deg=fov, forward_m=0.0,
                                  pitch_down_deg=float(g('pitch_down_deg')))
        self.model = PerceptionModel.from_manifest(
            manifest, confirm_radius_m=g('confirm_radius_m'),
            confirm_max_alt_m=g('confirm_max_alt_m'), target_match_m=g('target_match_m'),
            wrong_color_first=g('wrong_color_first'))
        self.odom = None
        self.target_xy = None
        self.latest = None

        self.map_pub = self.create_publisher(DiceMap, '/perception/dice_map', PUB_DEPTH)
        self.rel_pub = self.create_publisher(TargetRelative, '/perception/target_relative',
                                             PUB_DEPTH)
        self.color_pub = self.create_publisher(ColorCheck, '/perception/color_check', PUB_DEPTH)
        self.obstacle_pub = self.create_publisher(ObstacleArray,
                                                  '/perception/obstacle_detections', PUB_DEPTH)
        self.hb_pub = self.create_publisher(PerceptionHeartbeat, '/perception/heartbeat',
                                            PUB_DEPTH)
        self.create_subscription(VehicleOdometry, g('odometry_topic'), self.on_odom, qos.PX4)
        self.create_subscription(MissionTarget, '/mission/target', self.on_target,
                                 qos.MISSION_TARGET)
        self.create_timer(1.0 / g('rate_hz'), self.detect)
        self.create_timer(1.0 / g('map_rate_hz'), self.publish_map)
        self.create_timer(1.0 / g('heartbeat_hz'), self.publish_heartbeat)
        self.create_timer(1.0, self.publish_obstacles)
        self.get_logger().warning('SIM ONLY: fake Perception from the world manifest')

    def on_odom(self, msg):
        self.odom = msg

    def on_target(self, msg):
        self.target_xy = (msg.centroid.x, msg.centroid.y) if msg.known else None

    def stamp(self, msg):
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        return msg

    def detect(self):
        if self.odom is None:
            return
        pos = [float(v) for v in self.odom.position]
        q = [float(v) for v in self.odom.q]
        if not all(np.isfinite(pos + q)):
            return
        _, polygon = self.camera.project(pos, q, self.frame, self.bounds)
        drone = self.frame.to_field(*pos)
        out = self.model.update(drone, lambda x, y: contains_point(polygon, (x, y)),
                                self.target_xy, drop=self.get_parameter('drop_detections').value)
        self.latest = out
        rel = self.stamp(TargetRelative())
        rel.valid = out.target_relative is not None
        if rel.valid:
            rel.offset.x, rel.offset.y, rel.offset.z = out.target_relative
        self.rel_pub.publish(rel)
        if out.color:
            cc = self.stamp(ColorCheck())
            cc.color_guess, cc.confidence = out.color, 1.0
            self.color_pub.publish(cc)

    def publish_map(self):
        if self.latest is None:
            return
        msg = self.stamp(DiceMap())
        for color, (x, y, z) in self.latest.dice:
            entry = DiceMapEntry()
            entry.color = color
            entry.centroid.x, entry.centroid.y, entry.centroid.z = x, y, z
            msg.dice.append(entry)
        self.map_pub.publish(msg)

    def publish_obstacles(self):
        msg = self.stamp(ObstacleArray())
        for ob in self.field_map.obstacles:
            o = Obstacle()
            o.x, o.y, o.radius = ob.x, ob.y, ob.radius
            msg.obstacles.append(o)
        self.obstacle_pub.publish(msg)

    def publish_heartbeat(self):
        if not self.get_parameter('heartbeat').value:
            return
        hb = self.stamp(PerceptionHeartbeat())
        hb.vio_ok = hb.detector_ok = True
        self.hb_pub.publish(hb)


def main(args=None):
    rclpy.init(args=args)
    node = FakeDiceMap()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
