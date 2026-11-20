"""SIM ONLY camera node: observe PX4 odometry, project BOTH cameras (config/cameras.yaml, the
same file mission_manager reads), publish footprints, coverage and Gazebo markers, and stand
in for Perception from the world manifest.

Camera outputs
  /camera/footprint/d455, /camera/footprint/down   geometry_msgs/PolygonStamped, field ENU
  /camera/footprint                                the down camera (kept for existing RViz configs)
  /camera/searched                                 nav_msgs/OccupancyGrid: union of both cameras
  /camera/target_detection                         avc_interfaces/TargetDetection (either camera)
  Gazebo markers: current view of each camera, searched ground shaded by sensing time
  Coverage JSON on exit (`output`), read by tools/compare_search_runs.py and coverage_plot.py

Perception stand-in (only what the cameras have actually seen)
  /perception/dice_map             dice inside either camera's footprint          map_rate_hz
  /perception/target_relative      the target die, down camera only              rate_hz
  /perception/color_check          a confirmed color, down camera only, when close  rate_hz
  /perception/obstacle_detections  the manifest's obstacles                      1 Hz
  /perception/heartbeat            PerceptionHeartbeat                           heartbeat_hz
  Switches: wrong_color_first:=<color> (startup), drop_detections, heartbeat (live:
  `ros2 param set /camera_fov drop_detections true`).

Camera geometry can be changed live per camera (`ros2 param set /camera_fov down.pitch_down_deg
85.0`); that affects this node only. mission_manager reads cameras.yaml at startup.
"""
import json
import math
from pathlib import Path
import time
from dataclasses import asdict, fields

import numpy as np
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, HistoryPolicy
from avc_interfaces.msg import (ColorCheck, DiceMap, DiceMapEntry, MissionTarget, Obstacle,
                                ObstacleArray, PerceptionHeartbeat, TargetDetection,
                                TargetRelative)
from rcl_interfaces.msg import SetParametersResult
from geometry_msgs.msg import Point32, PolygonStamped
from nav_msgs.msg import OccupancyGrid
from px4_msgs.msg import VehicleOdometry

from gnc_offboard.camera_model import CameraModel, CoverageGrid, contains_point
from gnc_offboard.coverage_colors import DARKEST_AT_S, sensing_color
from gnc_offboard.field_map import FieldMap, default_manifest_path
from gnc_offboard.frames import FieldFrame
from gnc_offboard.mission import qos
from gnc_offboard.mission.cameras import CAMERA_NAMES, DOWN, default_cameras_path, load_cameras
from gnc_offboard.mission.qos import PX4 as PX4_QOS
from gnc_offboard.sim.perception_model import PerceptionModel


ODOMETRY_FRESH_S = 1.0
PUB_DEPTH = 10      # Perception publishers: reliable, volatile (mission_manager subscribes best effort)
LIVE_SWITCHES = ('drop_detections', 'heartbeat')
GEOMETRY = tuple(f.name for f in fields(CameraModel))
# Gazebo marker ids: coverage shades use 2 .. 2 + DARKEST_AT_S.
VIEW_MARKERS = {DOWN: (1, (0.0, .85, 1.0, .65)), 'd455': (100, (1.0, .3, .9, .55))}

DETECTION_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST, depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL)


class CameraFov(Node):
    def __init__(self):
        super().__init__('camera_fov')
        p = self.declare_parameter
        p('manifest', str(default_manifest_path()))
        p('cameras_file', '')
        p('resolution_m', 0.25)
        p('ground_z_m', 0.0)
        p('gazebo_markers', True)
        p('output', str(default_manifest_path().parents[1] / 'plots/camera_coverage.json'))
        p('odometry_topic', '/fmu/out/vehicle_odometry')
        # Perception stand-in
        p('confirm_radius_m', 0.3)
        p('confirm_max_alt_m', 1.3)
        p('target_match_m', 1.0)
        p('wrong_color_first', '')
        p('drop_detections', False)
        p('heartbeat', True)
        p('rate_hz', 10.0)
        p('map_rate_hz', 2.0)
        p('heartbeat_hz', 5.0)
        # Camera geometry, per camera, seeded from the one camera config.
        self.cameras_file = self.get_parameter('cameras_file').value or default_cameras_path()
        for name, model in load_cameras(self.cameras_file).items():
            for field in GEOMETRY:
                self.declare_parameter(f'{name}.{field}', float(getattr(model, field)))
        self.models = self.read_models()

        manifest = json.loads(Path(self.get_parameter('manifest').value).read_text())
        self.manifest = manifest
        self.frame = FieldFrame.from_manifest(manifest)
        self.field_map = FieldMap(manifest)
        self.grid = CoverageGrid(manifest['field']['bounds'], self.get_parameter('resolution_m').value)
        self.ground = float(self.get_parameter('ground_z_m').value)
        if not math.isfinite(self.ground):
            raise ValueError('ground_z_m must be finite')
        g = lambda name: self.get_parameter(name).value  # noqa: E731
        self.perception = PerceptionModel.from_manifest(
            manifest, confirm_radius_m=g('confirm_radius_m'),
            confirm_max_alt_m=g('confirm_max_alt_m'), target_match_m=g('target_match_m'),
            wrong_color_first=g('wrong_color_first'))
        self.add_on_set_parameters_callback(self.parameters_changed)

        self.footprint_pub = self.create_publisher(PolygonStamped, '/camera/footprint', 1)
        self.camera_footprint_pubs = {
            name: self.create_publisher(PolygonStamped, f'/camera/footprint/{name}', 1)
            for name in CAMERA_NAMES}
        self.coverage_pub = self.create_publisher(OccupancyGrid, '/camera/searched', 1)
        self.detection_pub = self.create_publisher(
            TargetDetection, '/camera/target_detection', DETECTION_QOS)
        self.map_pub = self.create_publisher(DiceMap, '/perception/dice_map', PUB_DEPTH)
        self.rel_pub = self.create_publisher(TargetRelative, '/perception/target_relative',
                                             PUB_DEPTH)
        self.color_pub = self.create_publisher(ColorCheck, '/perception/color_check', PUB_DEPTH)
        self.obstacle_pub = self.create_publisher(ObstacleArray,
                                                  '/perception/obstacle_detections', PUB_DEPTH)
        self.hb_pub = self.create_publisher(PerceptionHeartbeat, '/perception/heartbeat',
                                            PUB_DEPTH)
        self.create_subscription(VehicleOdometry, self.get_parameter('odometry_topic').value,
                                 self.on_odometry, PX4_QOS)
        self.create_subscription(MissionTarget, '/mission/target', self.on_target,
                                 qos.MISSION_TARGET)
        self.last_receive = None
        self.last_timestamp = None
        self.origin = None
        self.polygons = {name: np.empty((0, 2)) for name in CAMERA_NAMES}
        self.drone = None
        self.target_xy = None
        self.perceived = None
        self.trajectory = []
        self.target = next((t for t in manifest['targets']
                            if t['color'] == manifest['scenario']['initial_target']), None)
        # Manifest index identifies a die even when multiple dice share a color.
        self.observed_dice = {}
        self.target_found = False
        self.target_position = None
        self.observations = 0
        self.gz = None
        self.next_gz_attempt = 0.0
        self.gz_coverage_shades = None
        if self.get_parameter('gazebo_markers').value:
            try:
                from gz.transport13 import Node as GzNode
                self.gz = GzNode()
            except ImportError:
                self.get_logger().warning('Gazebo Python bindings unavailable; ROS coverage remains enabled.')
        self.control_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(.5, self.publish_visuals, clock=self.control_clock)
        self.create_timer(1.0 / g('rate_hz'), self.detect)
        self.create_timer(1.0 / g('map_rate_hz'), self.publish_map)
        self.create_timer(1.0 / g('heartbeat_hz'), self.publish_heartbeat)
        self.create_timer(1.0, self.publish_obstacles)
        self.get_logger().info(
            f'Cameras {", ".join(CAMERA_NAMES)} from {self.cameras_file}. Coverage active: '
            'cyan=down camera, magenta=D455, darker orange=more sensing time.')
        self.get_logger().warning('SIM ONLY: also standing in for Perception from the world manifest')

    # ---- parameters -----------------------------------------------------------------------------
    @property
    def polygon(self):
        """The down camera's current footprint (the single-camera name, kept for callers)."""
        return self.polygons[DOWN]

    def read_models(self, changes=None):
        values = {f'{n}.{f}': self.get_parameter(f'{n}.{f}').value
                  for n in CAMERA_NAMES for f in GEOMETRY}
        values.update(changes or {})
        return {name: CameraModel(**{f: float(values[f'{name}.{f}']) for f in GEOMETRY})
                for name in CAMERA_NAMES}

    def parameters_changed(self, parameters):
        geometry = {f'{n}.{f}' for n in CAMERA_NAMES for f in GEOMETRY}
        names = geometry | set(LIVE_SWITCHES)
        if any(p.name not in names for p in parameters):
            return SetParametersResult(
                successful=False, reason='Only camera geometry (<camera>.<field>), '
                'drop_detections and heartbeat can change live; restart for other settings.')
        changes = {p.name: p.value for p in parameters if p.name in geometry}
        if changes:
            try:
                self.models = self.read_models(changes)
            except (ValueError, TypeError) as exc:
                return SetParametersResult(successful=False, reason=str(exc))
        return SetParametersResult(successful=True)

    # ---- odometry: footprints, coverage, observed dice ------------------------------------------
    def on_odometry(self, msg):
        if (msg.pose_frame != VehicleOdometry.POSE_FRAME_NED
                or msg.timestamp == self.last_timestamp):
            return
        polygons = {}
        try:
            for name, model in self.models.items():
                origin, polygons[name] = model.project(msg.position, msg.q, self.frame,
                                                       self.grid.bounds, self.ground)
                if name == DOWN:
                    self.origin = origin
        except ValueError:
            return
        now = time.monotonic()
        elapsed = now - self.last_receive if self.last_receive is not None else 0.0
        self.last_timestamp = msg.timestamp
        self.last_receive = now
        self.polygons = polygons
        self.grid.update_many(list(polygons.values()), elapsed if elapsed < ODOMETRY_FRESH_S else 0.0)
        self.observations += 1
        # Record vehicle position, not the translated camera origin. Preserve
        # sample gaps as breaks rather than inventing a flight path across them.
        position = [float(value) for value in self.frame.to_field(*msg.position)]
        self.drone = position
        if self.trajectory and self.last_receive-self.trajectory[-1]['time_s'] >= ODOMETRY_FRESH_S:
            self.trajectory.append({'time_s': self.last_receive, 'position': None})
        self.trajectory.append({'time_s': self.last_receive, 'position': position})
        for index, die in enumerate(self.manifest['targets']):
            if index in self.observed_dice:
                continue
            if not self.in_any_view(die['x'], die['y']):
                continue
            position = (float(die['x']), float(die['y']), self.ground)
            self.observed_dice[index] = {
                'color': die['color'], 'position_enu': list(position),
            }
            east, north, up = position
            self.get_logger().info(
                f"Observed {die['color']} die at field ENU "
                f"(E={east:.4f}, N={north:.4f}, U={up:.4f}) m")
            if die is self.target:
                self.target_found = True
                self.target_position = position

    def in_any_view(self, x, y):
        return any(contains_point(p, (x, y)) for p in self.polygons.values())

    def in_down_view(self, x, y):
        return contains_point(self.polygons[DOWN], (x, y))

    # ---- Perception stand-in --------------------------------------------------------------------
    def on_target(self, msg):
        self.target_xy = (msg.centroid.x, msg.centroid.y) if msg.known else None

    def stamp(self, msg):
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        return msg

    def fresh(self):
        return self.last_receive is not None and time.monotonic()-self.last_receive < ODOMETRY_FRESH_S

    def detect(self):
        if self.drone is None:
            return
        out = self.perception.update(self.drone, self.in_any_view, self.target_xy,
                                     drop=self.get_parameter('drop_detections').value,
                                     close_view=self.in_down_view)
        self.perceived = out
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
        if self.perceived is None:
            return
        msg = self.stamp(DiceMap())
        for color, (x, y, z) in self.perceived.dice:
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

    # ---- camera visuals -------------------------------------------------------------------------
    def publish_visuals(self):
        detection = TargetDetection()
        detection.header.frame_id = 'map'
        detection.header.stamp = self.get_clock().now().to_msg()
        detection.found = self.target_found
        if self.target_position is not None:
            detection.position.x, detection.position.y, detection.position.z = self.target_position
        self.detection_pub.publish(detection)
        fresh = self.fresh()
        polygons = {name: (poly if fresh else []) for name, poly in self.polygons.items()}
        stamp = self.get_clock().now().to_msg()
        for name, polygon in polygons.items():
            footprint = self.footprint_msg(polygon, stamp)
            self.camera_footprint_pubs[name].publish(footprint)
            if name == DOWN:
                self.footprint_pub.publish(footprint)
        grid = OccupancyGrid()
        grid.header.frame_id = 'map'
        grid.header.stamp = stamp
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
            self.draw_gazebo(polygons)

    def footprint_msg(self, polygon, stamp):
        footprint = PolygonStamped()
        footprint.header.frame_id = 'map'
        footprint.header.stamp = stamp
        footprint.polygon.points = [Point32(x=float(x), y=float(y), z=self.ground)
                                    for x, y in polygon]
        return footprint

    def draw_gazebo(self, polygons):
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

        view_ids = {ident for ident, _ in VIEW_MARKERS.values()}

        def marker(ident, points, color, height):
            m = Marker(ns='avc_camera', id=ident, action=Marker.ADD_MODIFY,
                       type=Marker.TRIANGLE_LIST)
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 1.0
            # Current views expire if the observer is stopped; searched area persists.
            if ident in view_ids:
                m.lifetime.sec = 2
            for attr in ('ambient', 'diffuse'):
                c = getattr(m.material, attr)
                c.r, c.g, c.b, c.a = color
            for x, y in points:
                p = m.point.add()
                p.x, p.y, p.z = float(x), float(y), self.ground+height
            return m

        for name, polygon in polygons.items():
            ident, color = VIEW_MARKERS[name]
            triangles = []
            for i in range(1, len(polygon)-1):
                triangles.extend([polygon[0], polygon[i], polygon[i+1]])
            current = marker(ident, triangles, color, .035 if name == DOWN else .03)
            if not triangles:
                current.action = Marker.DELETE_MARKER
            self.gz.request('/marker', current, Marker, Empty, 5)
        shades = np.minimum(self.grid.exposure_s, DARKEST_AT_S).astype(int)
        shades[~self.grid.searched] = -1
        previous = self.gz_coverage_shades
        if previous is None:
            # Clear any persistent coverage markers from an earlier recorder.
            changed = range(DARKEST_AT_S + 1)
        else:
            moved = previous != shades
            changed = np.union1d(previous[moved], shades[moved])
        half = self.grid.resolution/2
        xmin, ymin, xmax, ymax = self.grid.bounds
        for shade in changed:
            if shade < 0:
                continue
            cells = []
            for x, y in self.grid.centres[shades == shade]:
                a, b = max(xmin, x-half), min(xmax, x+half)
                c, d = max(ymin, y-half), min(ymax, y+half)
                cells.extend([(a, c), (b, c), (b, d), (a, c), (b, d), (a, d)])
            coverage = marker(2 + int(shade), cells, (*sensing_color(shade), .75), .02)
            if not cells:
                coverage.action = Marker.DELETE_MARKER
            self.gz.request('/marker', coverage, Marker, Empty, 5)
        self.gz_coverage_shades = shades

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
                'searched_cell_exposure_s': self.grid.exposure_s[self.grid.searched].tolist(),
                'arena_area_m2': ((self.grid.bounds[2]-self.grid.bounds[0])
                                  * (self.grid.bounds[3]-self.grid.bounds[1])),
                'trajectory_enu': [None if sample['position'] is None else
                                   [float(value) for value in sample['position']]
                                   for sample in self.trajectory],
                'target_found': (self.target_found if self.target is not None
                                 and self.observations else None),
                'observed_dice_frame': 'field ENU metres',
                'observed_dice': list(self.observed_dice.values()),
                'target_found_method': 'target centre inside either camera footprint',
                'cameras': {name: asdict(model) for name, model in self.models.items()},
                'cameras_file': str(self.cameras_file),
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
