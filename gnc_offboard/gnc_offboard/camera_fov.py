"""Observe PX4 odometry; publish ground footprint, coverage, and Gazebo markers."""
import json
import math
import os
from pathlib import Path
import time
from dataclasses import fields

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, HistoryPolicy
from avc_interfaces.msg import TargetDetection
from rcl_interfaces.msg import SetParametersResult
from geometry_msgs.msg import Point32, PolygonStamped
from nav_msgs.msg import OccupancyGrid
from px4_msgs.msg import VehicleOdometry

from gnc_offboard.camera_model import CameraModel, CoverageGrid, contains_point
from gnc_offboard.frames import FieldFrame
from gnc_offboard.mission_demo import PX4_QOS


DETECTION_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST, depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL)


class CameraFov(Node):
    def __init__(self):
        super().__init__('camera_fov')
        repo = os.environ.get('AVC_SIM_DIR', str(Path.home()/'dev/avc-gnc'))
        self.declare_parameter('manifest', str(Path(repo)/'worlds/avc_field_manifest.json'))
        self.declare_parameter('resolution_m', 0.25)
        self.declare_parameter('ground_z_m', 0.0)
        self.declare_parameter('gazebo_markers', True)
        self.declare_parameter('output', str(Path(repo)/'plots/camera_coverage.json'))
        self.declare_parameter('odometry_topic', '/fmu/out/vehicle_odometry')
        for field in fields(CameraModel):
            self.declare_parameter(field.name, getattr(CameraModel(), field.name))
        self.model = self.read_model()
        manifest = json.loads(Path(self.get_parameter('manifest').value).read_text())
        self.manifest = manifest
        self.frame = FieldFrame.from_manifest(manifest)
        self.grid = CoverageGrid(manifest['field']['bounds'], self.get_parameter('resolution_m').value)
        self.ground = float(self.get_parameter('ground_z_m').value)
        if not math.isfinite(self.ground):
            raise ValueError('ground_z_m must be finite')
        self.add_on_set_parameters_callback(self.parameters_changed)
        self.footprint_pub = self.create_publisher(PolygonStamped, '/camera/footprint', 1)
        self.coverage_pub = self.create_publisher(OccupancyGrid, '/camera/searched', 1)
        self.detection_pub = self.create_publisher(
            TargetDetection, '/camera/target_detection', DETECTION_QOS)
        self.create_subscription(VehicleOdometry, self.get_parameter('odometry_topic').value,
                                 self.on_odometry, PX4_QOS)
        self.last_receive = None
        self.last_timestamp = None
        self.origin = None
        self.polygon = []
        self.trajectory = []
        self.target = next((t for t in manifest['targets']
                            if t['color'] == manifest['scenario']['initial_target']), None)
        self.target_found = False
        self.target_position = None
        self.observations = 0
        self.gz = None
        self.next_gz_attempt = 0.0
        if self.get_parameter('gazebo_markers').value:
            try:
                from gz.transport13 import Node as GzNode
                self.gz = GzNode()
            except ImportError:
                self.get_logger().warning('Gazebo Python bindings unavailable; ROS coverage remains enabled.')
        self.control_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(.5, self.publish_visuals, clock=self.control_clock)
        self.get_logger().info('Camera coverage active: cyan=current view, orange=searched ground.')

    def read_model(self, changes=None):
        values = {f.name: self.get_parameter(f.name).value for f in fields(CameraModel)}
        values.update(changes or {})
        return CameraModel(**values)

    def parameters_changed(self, parameters):
        names = {f.name for f in fields(CameraModel)}
        if any(p.name not in names for p in parameters):
            return SetParametersResult(successful=False, reason='Only camera geometry can change live; restart for grid/output settings.')
        try:
            self.model = self.read_model({p.name: p.value for p in parameters})
        except (ValueError, TypeError) as exc:
            return SetParametersResult(successful=False, reason=str(exc))
        return SetParametersResult(successful=True)

    def on_odometry(self, msg):
        if (msg.pose_frame != VehicleOdometry.POSE_FRAME_NED
                or msg.timestamp == self.last_timestamp):
            return
        try:
            origin, polygon = self.model.project(msg.position, msg.q, self.frame,
                                                  self.grid.bounds, self.ground)
        except ValueError:
            return
        self.last_timestamp = msg.timestamp
        self.last_receive = time.monotonic()
        self.origin, self.polygon = origin, polygon
        self.grid.update(polygon)
        self.observations += 1
        # Record vehicle position, not the translated camera origin. Preserve
        # sample gaps as breaks rather than inventing a flight path across them.
        position = [float(value) for value in self.frame.to_field(*msg.position)]
        if self.trajectory and self.last_receive-self.trajectory[-1]['time_s'] > 1.0:
            self.trajectory.append({'time_s': self.last_receive, 'position': None})
        self.trajectory.append({'time_s': self.last_receive, 'position': position})
        if self.target is not None:
            if not self.target_found and contains_point(polygon, (self.target['x'], self.target['y'])):
                self.target_found = True
                self.target_position = (float(self.target['x']), float(self.target['y']), self.ground)

    def publish_visuals(self):
        detection = TargetDetection()
        detection.header.frame_id = 'map'
        detection.header.stamp = self.get_clock().now().to_msg()
        detection.found = self.target_found
        if self.target_position is not None:
            detection.position.x, detection.position.y, detection.position.z = self.target_position
        self.detection_pub.publish(detection)
        fresh = self.last_receive is not None and time.monotonic()-self.last_receive < 1.0
        polygon = self.polygon if fresh else []
        footprint = PolygonStamped()
        footprint.header.frame_id = 'map'
        footprint.header.stamp = self.get_clock().now().to_msg()
        footprint.polygon.points = [Point32(x=float(x), y=float(y), z=self.ground) for x, y in polygon]
        self.footprint_pub.publish(footprint)
        grid = OccupancyGrid()
        grid.header = footprint.header
        grid.info.resolution = self.grid.resolution
        grid.info.width, grid.info.height = self.grid.width, self.grid.height
        grid.info.origin.position.x, grid.info.origin.position.y = map(float, self.grid.bounds[:2])
        grid.info.origin.position.z = self.ground
        grid.info.origin.orientation.w = 1.0
        # 100 = searched; 0 = not yet searched, -1 = outside field.
        grid.data = [100 if seen else (0 if valid else -1) for seen, valid
                     in zip(self.grid.searched.flat, self.grid.valid.flat)]
        self.coverage_pub.publish(grid)
        if self.gz is not None and time.monotonic() >= self.next_gz_attempt:
            self.draw_gazebo(polygon)

    def draw_gazebo(self, polygon):
        from gz.msgs10.marker_pb2 import Marker
        from gz.msgs10.empty_pb2 import Empty
        from gz.msgs10.marker_v_pb2 import Marker_V

        # /marker is a one-way Gazebo service: its Empty response is not sent.
        # Probe /marker/list for availability instead of treating a reply timeout
        # from /marker as failure (the marker may already have been rendered).
        ok, _ = self.gz.request('/marker/list', Empty(), Empty, Marker_V, 50)
        if not ok:
            self.next_gz_attempt = time.monotonic()+5
            self.get_logger().info('Waiting for Gazebo GUI /marker service (open with gz sim -g).',
                                   throttle_duration_sec=30)
            return

        def marker(ident, points, color, height):
            m = Marker(ns='avc_camera', id=ident, action=Marker.ADD_MODIFY,
                       type=Marker.TRIANGLE_LIST)
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 1.0
            # Current view expires if the observer is stopped; searched area persists.
            if ident == 1:
                m.lifetime.sec = 2
            for attr in ('ambient', 'diffuse'):
                c = getattr(m.material, attr)
                c.r, c.g, c.b, c.a = color
            for x, y in points:
                p = m.point.add()
                p.x, p.y, p.z = float(x), float(y), self.ground+height
            return m

        triangles = []
        for i in range(1, len(polygon)-1):
            triangles.extend([polygon[0], polygon[i], polygon[i+1]])
        current = marker(1, triangles, (0.0, .85, 1.0, .65), .035)
        if not triangles:
            current.action = Marker.DELETE_MARKER
        self.gz.request('/marker', current, Marker, Empty, 5)
        cells = []
        half = self.grid.resolution/2
        xmin, ymin, xmax, ymax = self.grid.bounds
        for x, y in self.grid.centres[self.grid.searched]:
            a, b = max(xmin, x-half), min(xmax, x+half)
            c, d = max(ymin, y-half), min(ymax, y+half)
            cells.extend([(a, c), (b, c), (b, d), (a, c), (b, d), (a, d)])
        coverage = marker(2, cells, (1.0, .55, .05, .4), .02)
        if not cells:
            coverage.action = Marker.DELETE_MARKER
        self.gz.request('/marker', coverage, Marker, Empty, 5)

    def save(self):
        output = self.get_parameter('output').value
        if output:
            path = Path(output).expanduser()
            path.parent.mkdir(parents=True, exist_ok=True)
            coverage = {
                'frame': 'field ENU metres', 'ground_z_m': self.ground,
                'manifest': self.manifest,
                'bounds': self.grid.bounds, 'resolution_m': self.grid.resolution,
                'searched_area_m2': self.grid.area_m2,
                'searched_cell_centres': self.grid.centres[self.grid.searched].tolist(),
                'arena_area_m2': ((self.grid.bounds[2]-self.grid.bounds[0])
                                  * (self.grid.bounds[3]-self.grid.bounds[1])),
                'trajectory_enu': [None if sample['position'] is None else
                                   [float(value) for value in sample['position']]
                                   for sample in self.trajectory],
                'target_found': (self.target_found if self.target is not None
                                 and self.observations else None),
                'target_found_method': 'target centre inside an observed ground footprint',
            }
            path.write_text(json.dumps(coverage, indent=2))
            message = f'Saved {self.grid.area_m2:.2f} m² searched ground to {path}'
            try:
                from gnc_offboard.coverage_plot import save_coverage_png
                png = save_coverage_png(coverage, self.get_parameter('manifest').value,
                                        path.with_suffix('.png'))
                message += f' and {png}'
            except Exception as exc:
                # Retain the recorded coverage even if an optional image export fails.
                message += f'; PNG export failed: {exc}'
            if rclpy.ok():
                self.get_logger().info(message)
            else:
                print(message, flush=True)


def main(args=None):
    rclpy.init(args=args)
    node = CameraFov()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.save()
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
