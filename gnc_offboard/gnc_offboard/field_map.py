"""The AVC field as geometry: obstacles, dice, start zone, and path clearance checks.

No ROS or plotting imports, so planners, tests and the plotting tools all share it.
Everything is in the field frame (ENU, origin at field centre, metres); use
`fmap.frame` (a FieldFrame) to convert to and from PX4 local NED.

    from gnc_offboard.field_map import FieldMap
    fmap = FieldMap.load()                      # $AVC_SIM_DIR/worlds/avc_field_manifest.json
    fmap.clearance(1.0, 2.0)                    # metres from the nearest obstacle edge
    fmap.path_violations([(x0, y0), (x1, y1)], inflate=0.5)

`inflate` is the drone's radius plus a safety margin: a path is safe when its
centreline stays at least `inflate` from every obstacle footprint and inside the field.
"""
import json
import math
import os
from pathlib import Path

from gnc_offboard.frames import FieldFrame

DEFAULT_INFLATE = 0.5   # m: x500 is ~0.35 m radius to the prop tips, plus 0.15 m margin


def default_manifest_path():
    sim_dir = os.environ.get('AVC_SIM_DIR', str(Path.home() / 'dev/avc-gnc'))
    return Path(sim_dir) / 'worlds' / 'avc_field_manifest.json'


class Obstacle:
    def __init__(self, entry):
        self.name = entry['name']
        self.kind = entry['kind']
        self.x, self.y, self.yaw = float(entry['x']), float(entry['y']), float(entry['yaw'])
        self.height = float(entry.get('height', 0.0))
        # Older manifests only carry the bounding circle; treat it as the footprint.
        fp = entry.get('footprint') or {'shape': 'circle', 'r': entry['footprint_radius']}
        self.shape = fp['shape']
        if self.shape == 'box':
            self.w, self.d = float(fp['w']), float(fp['d'])
            self.radius = 0.5 * math.hypot(self.w, self.d)
        else:
            self.r = float(fp['r'])
            self.radius = self.r

    def clearance(self, px, py):
        """Distance from a point to the footprint edge (negative inside)."""
        dx, dy = px - self.x, py - self.y
        if self.shape != 'box':
            return math.hypot(dx, dy) - self.r
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        lx, ly = abs(c * dx + s * dy) - self.w / 2, abs(-s * dx + c * dy) - self.d / 2
        if lx <= 0 and ly <= 0:
            return max(lx, ly)
        return math.hypot(max(lx, 0.0), max(ly, 0.0))

    def corners(self):
        """Box corners in the field frame (None for circular footprints)."""
        if self.shape != 'box':
            return None
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        hw, hd = self.w / 2, self.d / 2
        return [(self.x + c * u - s * v, self.y + s * u + c * v)
                for u, v in ((hw, hd), (-hw, hd), (-hw, -hd), (hw, -hd))]

    def inflated_outline(self, inflate, arc_pts=8):
        """Outline of the keep-out zone: the footprint grown by `inflate` (rounded corners)."""
        if self.shape != 'box':
            r = self.r + inflate
            n = 4 * arc_pts
            return [(self.x + r * math.cos(2 * math.pi * k / n),
                     self.y + r * math.sin(2 * math.pi * k / n)) for k in range(n)]
        pts = []
        base = [0.0, math.pi / 2, math.pi, 3 * math.pi / 2]   # corner arcs start angles (local)
        for (cx, cy), a0 in zip(self.corners(), base):
            for k in range(arc_pts + 1):
                a = self.yaw + a0 + (math.pi / 2) * k / arc_pts
                pts.append((cx + inflate * math.cos(a), cy + inflate * math.sin(a)))
        return pts


class FieldMap:
    def __init__(self, manifest):
        self.manifest = manifest
        self.name = manifest.get('world_name', 'avc_field')
        self.seed = manifest.get('seed')
        self.challenge = manifest.get('challenge')
        self.bounds = tuple(manifest['field']['bounds'])               # x0, y0, x1, y1
        self.start_zone = tuple(manifest['start_zone']['bounds'])
        self.start_center = tuple(manifest['start_zone']['center'])
        self.frame = FieldFrame.from_manifest(manifest)
        self.spawn = (self.frame.sx, self.frame.sy)
        self.obstacles = [Obstacle(o) for o in manifest.get('obstacles', [])]
        self.targets = manifest.get('targets', [])
        self.scenario = manifest.get('scenario', {})

    @classmethod
    def load(cls, path=None):
        path = Path(path) if path else default_manifest_path()
        return cls(json.loads(path.read_text()))

    def target(self, color=None):
        """The die of `color`, or the scenario's initial target."""
        color = color or self.scenario.get('initial_target')
        return next(t for t in self.targets if t['color'] == color)

    # ------------------------------------------------------------ clearance checks
    def boundary_margin(self, x, y):
        """Distance inside the field edge (negative when outside)."""
        x0, y0, x1, y1 = self.bounds
        return min(x - x0, x1 - x, y - y0, y1 - y)

    def clearance(self, x, y):
        """(distance to the nearest obstacle edge, that obstacle). inf if no obstacles."""
        best, who = math.inf, None
        for ob in self.obstacles:
            c = ob.clearance(x, y)
            if c < best:
                best, who = c, ob
        return best, who

    def path_violations(self, points, inflate=DEFAULT_INFLATE, step=0.05, edge_margin=None):
        """Check a polyline [(x, y), ...] sampled every `step` metres.

        Returns one dict per (segment, obstacle) that comes closer than `inflate`,
        at its closest point, plus one per segment that leaves the field (closer than
        `edge_margin` to the edge, default `inflate`). An empty list means the path is
        clear. Sampling means the reported minimum can be off by up to step/2.
        """
        edge_margin = inflate if edge_margin is None else edge_margin
        worst = {}
        for i, (p, q) in enumerate(zip(points[:-1], points[1:])):
            n = max(1, int(math.ceil(math.dist(p, q) / step)))
            for k in range(n + 1):
                x = p[0] + (q[0] - p[0]) * k / n
                y = p[1] + (q[1] - p[1]) * k / n
                for ob in self.obstacles:
                    if math.hypot(x - ob.x, y - ob.y) - ob.radius >= inflate:  # cheap reject
                        continue
                    c = ob.clearance(x, y)
                    key = (i, ob.name)
                    if c < inflate and (key not in worst or c < worst[key]['clearance']):
                        worst[key] = {'segment': i, 'x': x, 'y': y, 'clearance': c,
                                      'obstacle': ob.name, 'kind': 'obstacle'}
                m = self.boundary_margin(x, y)
                key = (i, 'field_edge')
                if m < edge_margin and (key not in worst or m < worst[key]['clearance']):
                    worst[key] = {'segment': i, 'x': x, 'y': y, 'clearance': m,
                                  'obstacle': 'field edge', 'kind': 'edge'}
        return sorted(worst.values(), key=lambda v: (v['segment'], v['clearance']))
