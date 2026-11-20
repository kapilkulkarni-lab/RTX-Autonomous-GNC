"""Shared base for the search-pattern missions (pattern-comparison tools, not the mission).

WaypointMission flies a fixed list of waypoints: arm, offboard, climb in place, fly each
leg with cubic or carrot guidance, hover, land. It is the waypoint node that used to live in
mission_demo.py (retired in Milestone 3; mission_manager flies the real mission), and it uses
guidance/px4_interface.py for arming, the offboard handshake and setpoints.

SearchMission is a WaypointMission whose waypoint list is a search pattern.

Each pattern file (mission_lawnmower.py, ...) only supplies `plan()`.

Common parameters (all optional):
    margin             m, how far to stay in from the walls   (default 1.0)
    fov_deg            deg, camera field of view (PLACEHOLDER: set to the real camera)
    overlap            0..1, shared fraction between rows     (default 0.2)
    search_hover_time  s, pause at each corner                (default 0.0)
    return_home        fly back to the start zone at the end  (default true)
    clearance          m, keep this far from obstacle edges   (default 0.5)
    obstacles          [x, y, radius, x, y, radius, ...] known obstacles, field frame
    regions, keepouts  [x0, y0, x1, y1, ...] rectangles, field frame
    region_names, keepout_names  named field halves, quadrants, thirds, center, all

Use `search_hover_time`, not `hover_time`: the inherited hover_time pauses at EVERY
waypoint, which is far too slow for a search path.

WaypointMission parameters: manifest, altitude (m above spawn), speed (m/s), max_accel
(m/s^2), guidance ('cubic' or 'carrot'), acceptance_radius (m), hover_time (s).
"""
import json
import math
from pathlib import Path

import rclpy
from rclpy.exceptions import ParameterUninitializedException
from rclpy.node import Node
from rclpy.parameter import Parameter

from px4_msgs.msg import VehicleLocalPosition, VehicleStatus

from gnc_offboard import search_planners as sp
from gnc_offboard import search_regions as sr
from gnc_offboard.field_map import default_manifest_path
from gnc_offboard.frames import FieldFrame
from gnc_offboard.guidance import px4_interface as px4
from gnc_offboard.trajectory import CubicSegment

RATE_HZ = 20.0


class WaypointMission(Node):
    """Fly `self.waypoints` (PX4 local NED) in order, then land.

    Per-leg speed: `self.wp_speeds[i]` is the speed INTO waypoint i (None = `speed`).
    Subclasses set `self.waypoints` (and optionally `self.wp_speeds`) in __init__.
    """
    NODE_NAME = 'waypoint_mission'

    def __init__(self):
        super().__init__(self.NODE_NAME)
        self.declare_parameter('manifest', str(default_manifest_path()))
        self.declare_parameter('altitude', 1.5)          # m above spawn
        self.declare_parameter('speed', 1.0)             # m/s: carrot speed, or cubic peak speed
        self.declare_parameter('max_accel', 0.5)         # m/s^2, cubic peak acceleration
        self.declare_parameter('guidance', 'cubic')      # 'cubic' or 'carrot'
        self.declare_parameter('acceptance_radius', 0.25)
        self.declare_parameter('hover_time', 3.0)        # s at each waypoint

        p = lambda name: self.get_parameter(name).value  # noqa: E731
        self.alt, self.speed, self.max_accel = p('altitude'), p('speed'), p('max_accel')
        self.acc_r, self.hover_time = p('acceptance_radius'), p('hover_time')
        self.guidance = p('guidance')
        if self.guidance not in ('cubic', 'carrot'):
            raise ValueError("guidance must be 'cubic' or 'carrot'")
        self.get_logger().info(f'Guidance: {self.guidance}')
        self.frame = FieldFrame.from_manifest(json.loads(Path(p('manifest')).read_text()))

        self.waypoints = []      # PX4 local NED; the climb-in-place point is prepended at arming
        self.wp_speeds = []      # speed into each waypoint, None = `speed`
        self.pos = None          # latest valid VehicleLocalPosition
        self.status = None       # latest VehicleStatus
        self.state = 'WAIT_POS'
        self.stream_count = 0
        self.last_cmd_time = -1.0
        self.carrot = None       # current position setpoint [n, e, d] (either guidance mode)
        self.segment = None      # active CubicSegment (cubic guidance only)
        self.wp_idx = 0
        self.hover_start = None
        self.yaw = float('nan')
        self.px4 = None

    # ---------------------------------------------------------------- setup
    def setup_io(self):
        self.px4 = px4.Px4Interface(self)
        t = self.px4.topics
        self.create_subscription(VehicleLocalPosition, t['/fmu/out/vehicle_local_position'],
                                 self.on_pos, px4.PX4_QOS)
        self.create_subscription(VehicleStatus, t['/fmu/out/vehicle_status'],
                                 self.on_status, px4.PX4_QOS)
        self.create_timer(1.0 / RATE_HZ, self.tick)

    def on_pos(self, msg):
        if px4.position_valid(msg):
            self.pos = msg

    def on_status(self, msg):
        self.status = msg

    # ---------------------------------------------------------------- helpers
    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def vehicle_ned(self):
        return [self.pos.x, self.pos.y, self.pos.z]

    # ---------------------------------------------------------------- guidance
    def leg_speed(self, idx):
        """Speed for the leg into waypoint `idx`, falling back to the `speed` parameter."""
        s = self.wp_speeds[idx] if idx < len(self.wp_speeds) else None
        return s if s else self.speed

    def advance_carrot(self, target, speed):
        """Carrot guidance: move the setpoint toward `target` at `speed`, position only."""
        step = speed / RATE_HZ
        d = [t - c for t, c in zip(target, self.carrot)]
        dist = math.sqrt(sum(v * v for v in d))
        if dist <= step:
            self.carrot = list(target)
        else:
            self.carrot = [c + v * step / dist for c, v in zip(self.carrot, d)]
        if math.hypot(d[0], d[1]) > 0.3:  # face direction of horizontal travel
            self.yaw = math.atan2(d[1], d[0])
        self.px4.publish_setpoint(self.carrot, self.yaw)

    def advance_cubic(self, target, speed):
        """Cubic guidance: sample the time-defined segment and publish the full state."""
        now = self.now_s()
        if self.segment is None:
            # New segment starts from the last setpoint (where the vehicle is holding).
            dn, de = target[0] - self.carrot[0], target[1] - self.carrot[1]
            yaw1 = math.atan2(de, dn) if math.hypot(dn, de) > 0.3 else self.yaw
            self.segment = CubicSegment(self.carrot, target, now, speed,
                                        self.max_accel, yaw0=self.yaw, yaw1=yaw1)
            self.get_logger().info(
                f'Segment {self.wp_idx}: {self.segment.length:.2f} m in {self.segment.T:.1f} s '
                f'(peak speed {speed:.2f} m/s)')
        pos, vel, acc, yaw, yawspeed = self.segment.sample(now)
        if self.segment.done(now):
            pos = list(target)   # land exactly on the waypoint, no float drift
        self.carrot, self.yaw = pos, yaw
        self.px4.publish_setpoint(pos, yaw, vel, acc, yawspeed)

    def prepare_takeoff(self):
        """Allow missions to validate their route against the live launch position."""

    # ---------------------------------------------------------------- state machine
    def tick(self):
        if self.state == 'WAIT_POS':
            if self.pos is None:
                self.get_logger().info('Waiting for a valid position estimate...',
                                       throttle_duration_sec=2.0)
                return
            self.prepare_takeoff()
            self.carrot = self.vehicle_ned()
            self.yaw = self.pos.heading
            self.state = 'STREAM'

        if self.state in ('DONE', 'LANDING', 'ABORTED'):
            if self.state == 'LANDING' and px4.is_disarmed(self.status):
                self.get_logger().info('Landed and disarmed. Mission complete.')
                self.state = 'DONE'
                raise SystemExit
            return

        # Offboard requires a continuous setpoint stream, before AND during the mode.
        self.px4.publish_offboard_mode()

        if self.state == 'STREAM':
            self.px4.publish_setpoint(self.carrot, self.yaw)
            self.stream_count += 1
            if self.stream_count >= int(RATE_HZ):  # ~1 s of setpoints first
                self.state = 'ARMING'
            return

        if self.state == 'ARMING':
            self.px4.publish_setpoint(self.carrot, self.yaw)
            if px4.armed_offboard(self.status):
                n, e, _ = self.vehicle_ned()
                # Keep speeds aligned with waypoints even if a planner replaced only
                # self.waypoints; missing entries mean "default speed".
                k = len(self.waypoints)
                self.wp_speeds = (list(self.wp_speeds) + [None] * k)[:k]
                self.waypoints.insert(0, (n, e, -self.alt))  # climb in place first
                self.wp_speeds.insert(0, None)               # climb at default speed
                self.get_logger().info('Armed in offboard. Taking off.')
                self.state = 'FLYING'
            elif self.now_s() - self.last_cmd_time > 1.0:
                self.px4.set_offboard()
                self.px4.arm()
                self.last_cmd_time = self.now_s()
            return

        # FLYING
        if not px4.armed_offboard(self.status):
            self.get_logger().warn('Left offboard mode (pilot/failsafe). Stopping commands.')
            self.state = 'ABORTED'
            return

        target = self.waypoints[self.wp_idx]
        speed = self.leg_speed(self.wp_idx)
        if self.guidance == 'cubic':
            self.advance_cubic(target, speed)
        else:
            self.advance_carrot(target, speed)

        err = math.dist(self.vehicle_ned(), target)
        if self.carrot == list(target) and err < self.acc_r:
            if self.hover_start is None:
                self.hover_start = self.now_s()
                x, y, z = self.frame.to_field(*target)
                self.get_logger().info(
                    f'Reached waypoint {self.wp_idx} (field {x:.2f}, {y:.2f}, alt {z:.2f})')
            elif self.now_s() - self.hover_start >= self.hover_time:
                self.hover_start = None
                self.segment = None   # next waypoint gets a fresh cubic
                self.wp_idx += 1
                if self.wp_idx >= len(self.waypoints):
                    self.get_logger().info('Path complete. Landing.')
                    self.px4.land()
                    self.state = 'LANDING'


class SearchMission(WaypointMission):
    PATTERN = 'search'
    # extra scalar parameters for a pattern: {name: default}. Declared automatically.
    PARAMS = {}

    @property
    def NODE_NAME(self):  # noqa: N802 - one node name per pattern
        return 'mission_' + self.PATTERN

    def __init__(self):
        super().__init__()                       # waypoint setup, incl. self.frame/self.alt

        self.declare_parameter('margin', 1.0)
        self.declare_parameter('fov_deg', 70.0)
        self.declare_parameter('overlap', 0.2)
        self.declare_parameter('search_hover_time', 0.0)
        self.declare_parameter('return_home', True)
        self.declare_parameter('clearance', 0.5)
        self.declare_parameter('obstacles', Parameter.Type.DOUBLE_ARRAY)
        for name in ('regions', 'keepouts'):
            self.declare_parameter(name, Parameter.Type.DOUBLE_ARRAY)
        for name in ('region_names', 'keepout_names'):
            self.declare_parameter(name, Parameter.Type.STRING_ARRAY)
        for name, default in self.PARAMS.items():
            self.declare_parameter(name, default)
        self.declare_extra_parameters()

        g = lambda name: self.get_parameter(name).value  # noqa: E731
        manifest = json.loads(Path(g('manifest')).read_text())
        xmin, ymin, xmax, ymax = manifest['field']['bounds']
        margin = g('margin')
        if not math.isfinite(margin) or margin < 0:
            raise ValueError('margin must be finite and nonnegative')
        # Use the same arena as the camera recorder, including rectangular fields.
        self.area = sp.Area(xmin + margin, xmax - margin,
                            ymin + margin, ymax - margin)
        if self.area.width <= 0 or self.area.height <= 0:
            raise ValueError('margin leaves no flight area')
        self.margin = g('margin')
        self.fp = sp.footprint(self.alt, g('fov_deg'))
        self.spacing = sp.row_spacing(self.alt, g('fov_deg'), g('overlap'))
        self.pp = {name: g(name) for name in self.PARAMS}
        self.hover_time = g('search_hover_time')

        zone = manifest['start_zone']['center']
        self.start_xy = (float(zone[0]), float(zone[1]))

        flat = self.optional_array('obstacles') or []
        if len(flat) % 3:
            raise ValueError('obstacles must be x, y, radius triples')
        self.obstacles = [tuple(flat[i:i + 3]) for i in range(0, len(flat), 3)]

        field = sp.Area(xmin, xmax, ymin, ymax)
        regions = self.rectangles('regions', 'region_names', field)
        keepouts = self.rectangles('keepouts', 'keepout_names', field)
        clearance = g('clearance')
        if not math.isfinite(clearance) or clearance < 0:
            raise ValueError('clearance must be finite and nonnegative')
        if any(not all(math.isfinite(value) for value in obstacle) or obstacle[2] <= 0
               for obstacle in self.obstacles):
            raise ValueError('obstacles must have finite coordinates and positive radii')

        self.exclusions = []
        if regions or keepouts:
            self.exclusions = [rect.expand(clearance) for rect in keepouts]
            # Use a conservative box around each circular obstacle so all detours
            # are solved together and cannot re-enter a different exclusion.
            for x, y, radius in self.obstacles:
                buffer = radius + clearance
                self.exclusions.append(sp.Area(x - buffer, x + buffer, y - buffer, y + buffer))
            pts = sr.restricted_search(regions or [field], self.exclusions, self.start_xy,
                                       self.area, self.plan_region)
        else:
            pts = self.plan(self.area, self.start_xy)
        if g('return_home'):
            pts = pts + [self.start_xy]
        if self.exclusions:
            pts = sr.route_path(self.start_xy, pts, self.exclusions, self.area)
        elif self.obstacles:
            pts, failed = sp.avoid_obstacles(pts, self.start_xy, self.obstacles,
                                             g('clearance'), self.area)
            if failed:
                raise ValueError(f'{failed} leg(s) could not be routed around obstacles')

        self.get_logger().info(
            f'{self.PATTERN}: {len(pts)} waypoints, {sp.path_length(self.start_xy, pts):.1f} m, '
            f'~{sp.estimate_time(self.start_xy, pts, self.speed, self.max_accel, self.hover_time):.0f} s '
            f'(footprint {self.fp:.2f} m, row spacing {self.spacing:.2f} m)')

        # Field (x, y) -> PX4 NED at the flight altitude.
        self.waypoints = [self.frame.to_ned(x, y, self.alt) for x, y in pts]

    # ---- hooks for the pattern files ---------------------------------------------------
    def declare_extra_parameters(self):
        """Declare array parameters a pattern needs (scalars go in PARAMS)."""

    def plan(self, area, start):
        """Return the search path as a list of field-frame (x, y) points."""
        raise NotImplementedError

    def plan_region(self, area, start):
        """Plan within one flyable rectangle."""
        return self.plan(area, start)

    def prepare_takeoff(self):
        if self.exclusions:
            x, y, _ = self.frame.to_field(*self.vehicle_ned())
            if (x, y) != self.start_xy:
                # The main route was planned from the manifest start zone.
                entry = sr.route_path((x, y), [self.start_xy], self.exclusions, self.area)
                self.waypoints[:0] = [self.frame.to_ned(px, py, self.alt) for px, py in entry]

    def rectangles(self, coords, names, field):
        return (sr.rects_from_flat(self.optional_array(coords) or []) +
                [sr.named_region(name, field) for name in (self.optional_array(names) or [])])

    # ---- helper ------------------------------------------------------------------------
    def optional_array(self, name):
        """Value of an array parameter that may not have been given, else None."""
        try:
            value = self.get_parameter(name).value
        except ParameterUninitializedException:
            return None
        return list(value) if value is not None and len(value) else None


def run(node_class, args=None):
    """Discover PX4, set up flight I/O, and spin a waypoint mission until it lands."""
    rclpy.init(args=args)
    node = node_class()
    if not px4.wait_for_px4(node, timeout_s=20.0):
        node.destroy_node()
        rclpy.shutdown()
        return
    node.setup_io()
    try:
        rclpy.spin(node)
    except (SystemExit, KeyboardInterrupt):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
