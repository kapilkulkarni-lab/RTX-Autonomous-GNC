"""Shared base for the search-pattern missions.

SearchMission is a MissionDemo whose waypoint list is replaced by a search pattern.
mission_demo.py already says this is the intended hook: "the path planner's job is to
replace `self.waypoints`". Everything else (arming, offboard streaming, cubic or carrot
guidance, hover, landing) is inherited unchanged.

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
"""
import json
import math
from pathlib import Path

from rclpy.exceptions import ParameterUninitializedException
from rclpy.parameter import Parameter

from gnc_offboard.mission_demo import MissionDemo, run_mission
from gnc_offboard import search_planners as sp
from gnc_offboard import search_regions as sr


class SearchMission(MissionDemo):
    PATTERN = 'search'
    # extra scalar parameters for a pattern: {name: default}. Declared automatically.
    PARAMS = {}

    def __init__(self):
        super().__init__()                       # normal demo setup, incl. self.frame/self.alt

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

        # Same call mission_demo uses: field (x, y) -> PX4 NED at the flight altitude.
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
    """Use the demo's PX4 discovery, flight I/O, and shutdown lifecycle."""
    run_mission(node_class, args=args)
