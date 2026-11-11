"""Search-pattern planners for the Bird Dog mission.

Pure Python (standard library only): no ROS, no numpy. That means these functions can be
unit-tested and previewed on any machine, and the ROS nodes just call them.

Coordinates are FIELD frame (ENU): origin at field center, +x east, +y north, metres.
Convert to PX4 NED only through gnc_offboard/frames.py (done in search_base.py).

Every planner returns a list of (x, y) waypoints that does NOT include the start point.
Waypoints are kept sparse on purpose: only corners where the direction changes. The cubic
guidance in mission_demo slows to a stop at each waypoint, so extra points cost time.

Patterns (numbers match Search_Pattern_Options.pdf):
    1 lawnmower          parallel rows across the field
    2 expanding_spiral   expanding L-shaped rings out from the start corner
    3 contracting_spiral rectangular spiral from the outer edge inward
    4 frontier           greedy "go to the nearest unseen area" on a coverage grid
    5 zone_sweep         split field into zones, visit by priority, sweep each zone
    6 hybrid             coarse (every other row) sweep, then fill pass; dives on a candidate
"""
from __future__ import annotations

import math
from dataclasses import dataclass

YD = 0.9144   # metres per yard
FT = 0.3048   # metres per foot


# ------------------------------------------------------------------ small geometry helpers
def _dist(a, b):
    return math.hypot(b[0] - a[0], b[1] - a[1])


def _lerp(a, b, t):
    return (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t)


@dataclass(frozen=True)
class Area:
    """Axis-aligned rectangle the UAV is allowed to fly in (field frame, metres)."""
    xmin: float
    xmax: float
    ymin: float
    ymax: float

    @classmethod
    def square(cls, field_size, margin=0.0):
        """Centered square. margin = how far to stay in from the walls."""
        h = field_size / 2.0 - margin
        if h <= 0:
            raise ValueError('margin is larger than half the field')
        return cls(-h, h, -h, h)

    @property
    def width(self):
        return self.xmax - self.xmin

    @property
    def height(self):
        return self.ymax - self.ymin

    @property
    def center(self):
        return ((self.xmin + self.xmax) / 2.0, (self.ymin + self.ymax) / 2.0)

    def expand(self, m):
        return Area(self.xmin - m, self.xmax + m, self.ymin - m, self.ymax + m)

    def contains(self, p, tol=1e-9):
        return (self.xmin - tol <= p[0] <= self.xmax + tol and
                self.ymin - tol <= p[1] <= self.ymax + tol)

    def clamp(self, p):
        return (min(max(p[0], self.xmin), self.xmax), min(max(p[1], self.ymin), self.ymax))


def footprint(altitude, fov_deg):
    """Width of ground seen by a downward camera: 2 * altitude * tan(FOV / 2)."""
    if not math.isfinite(altitude) or altitude <= 0:
        raise ValueError('altitude must be positive and finite')
    if not math.isfinite(fov_deg) or not 0 < fov_deg < 179:
        raise ValueError('fov_deg must be between 0 and 179')
    return 2.0 * altitude * math.tan(math.radians(fov_deg) / 2.0)


def row_spacing(altitude, fov_deg, overlap=0.2):
    """Distance between search rows. overlap = fraction of the footprint shared by rows."""
    if not math.isfinite(overlap) or not 0 <= overlap < 1:
        raise ValueError('overlap must be in [0, 1)')
    return footprint(altitude, fov_deg) * (1.0 - overlap)


class CornerFrame:
    """Local (u, v) coordinates with the origin at the area corner nearest `start`.

    u and v both point INTO the area. With swap=True, u runs along field y instead of x.
    Writing each pattern in (u, v) means it works from whichever corner the judges pick.
    """

    def __init__(self, area, start, swap=False):
        self.sx = 1 if abs(start[0] - area.xmin) <= abs(start[0] - area.xmax) else -1
        self.sy = 1 if abs(start[1] - area.ymin) <= abs(start[1] - area.ymax) else -1
        self.cx = area.xmin if self.sx == 1 else area.xmax
        self.cy = area.ymin if self.sy == 1 else area.ymax
        self.swap = swap
        self.U = area.height if swap else area.width
        self.V = area.width if swap else area.height

    def to_field(self, u, v):
        if self.swap:
            u, v = v, u
        return (self.cx + self.sx * u, self.cy + self.sy * v)


def simplify(path, tol=1e-6):
    """Drop repeated points and middle points on a straight line (same direction)."""
    out = []
    for p in path:
        if out and _dist(out[-1], p) < tol:
            continue
        out.append(p)
    if len(out) <= 2:
        return out
    res = [out[0]]
    for k in range(1, len(out) - 1):
        a, b, c = res[-1], out[k], out[k + 1]
        cross = (b[0] - a[0]) * (c[1] - b[1]) - (b[1] - a[1]) * (c[0] - b[0])
        dot = (b[0] - a[0]) * (c[0] - b[0]) + (b[1] - a[1]) * (c[1] - b[1])
        if abs(cross) < 1e-6 and dot > 0:
            continue
        res.append(b)
    res.append(out[-1])
    return res


# ------------------------------------------------------------------ path metrics
def path_length(start, pts):
    total, prev = 0.0, start
    for p in pts:
        total += _dist(prev, p)
        prev = p
    return total


def turn_count(start, pts, min_angle_deg=20.0):
    """Number of heading changes bigger than min_angle_deg."""
    seq = [start] + list(pts)
    n, prev_h = 0, None
    for a, b in zip(seq, seq[1:]):
        if _dist(a, b) < 1e-9:
            continue
        h = math.atan2(b[1] - a[1], b[0] - a[0])
        if prev_h is not None:
            d = abs((h - prev_h + math.pi) % (2 * math.pi) - math.pi)
            if math.degrees(d) > min_angle_deg:
                n += 1
        prev_h = h
    return n


def leg_time(d, speed, accel):
    """Time for one smoothstep-cubic leg of length d.

    For x(t) = d*(3s^2 - 2s^3), s = t/T:  peak speed = 1.5*d/T, peak accel = 6*d/T^2.
    So T is the larger of the two limits. This is an ESTIMATE; trajectory.py is the truth.
    """
    if d <= 1e-9:
        return 0.0
    return max(1.5 * d / speed, math.sqrt(6.0 * d / accel))


def estimate_time(start, pts, speed, accel, hover=0.0):
    seq = [start] + list(pts)
    t = sum(leg_time(_dist(a, b), speed, accel) for a, b in zip(seq, seq[1:]))
    return t + hover * len(pts)


# ------------------------------------------------------------------ 1. lawnmower
def _row_positions(V, spacing):
    """Row offsets across a span V, evenly spaced, first and last rows on the edges."""
    if V <= 1e-9:
        return [0.0]
    n = max(2, math.ceil(V / spacing) + 1)
    return [i * V / (n - 1) for i in range(n)]


def _snake(cf, vs, start_at_u0=True):
    """Alternating rows. Returns (points, next_row_starts_at_u0)."""
    pts, at0 = [], start_at_u0
    for v in vs:
        a, b = (0.0, cf.U) if at0 else (cf.U, 0.0)
        pts += [cf.to_field(a, v), cf.to_field(b, v)]
        at0 = not at0
    return pts, at0


def lawnmower(area, start, spacing, rows_along='x'):
    """Parallel rows, starting at the corner nearest `start`, stepping away from it."""
    cf = CornerFrame(area, start, swap=(rows_along == 'y'))
    pts, _ = _snake(cf, _row_positions(cf.V, spacing))
    return simplify(pts)


# ------------------------------------------------------------------ 2. expanding spiral
def expanding_spiral(area, start, spacing):
    """Expanding square spiral from the start corner.

    The start zone is in a corner, so each "ring" around it is an L-shape clipped to the
    field. Rings sit at radius 0.5*spacing, 1.5*spacing, ... and are flown back and forth.
    """
    cf = CornerFrame(area, start)
    U, V = cf.U, cf.V
    K = max(1, math.ceil(max(U, V) / spacing))
    radii = [(k - 0.5) * spacing for k in range(1, K + 1)]
    if radii[-1] < max(U, V) - 1e-9:
        radii.append(max(U, V))          # last ring on the far edge so the camera reaches the wall
    pts = []
    for k, r in enumerate(radii, start=1):
        ru, rv = min(r, U), min(r, V)
        leg = [(ru, 0.0), (ru, rv), (0.0, rv)]
        if k % 2 == 0:
            leg.reverse()
        pts += [cf.to_field(u, v) for u, v in leg]
    return simplify(pts)


# ------------------------------------------------------------------ 3. contracting spiral
def contracting_spiral(area, start, spacing):
    """Rectangular spiral: perimeter first, then each lap inward, ending in the middle.

    The pitch is shrunk slightly (never widened) so the laps tile exactly into the center
    and leave no uncovered sliver there.
    """
    cf = CornerFrame(area, start)
    half = min(cf.U, cf.V) / 2.0
    pitch = half / max(1, math.ceil(half / spacing)) if half > 1e-9 else spacing
    x0, x1, y0, y1 = 0.0, cf.U, 0.0, cf.V
    pts, step, eps = [], 0, 1e-9
    for _ in range(500):  # hard stop; the box collapses long before this
        if step % 4 == 0:      # go right (+u) along the bottom
            pos = (x1, y0)
            y0 += pitch
        elif step % 4 == 1:    # go up (+v)
            pos = (x1, y1)
            x1 -= pitch
        elif step % 4 == 2:    # go left (-u)
            pos = (x0, y1)
            y1 -= pitch
        else:                  # go down (-v)
            pos = (x0, y0)
            x0 += pitch
        pts.append(pos)
        step += 1
        if x1 < x0 - eps or y1 < y0 - eps:
            break
    return simplify([cf.to_field(u, v) for u, v in pts])


# ------------------------------------------------------------------ 5. zone / sector
def zone_grid(area, nx, ny):
    """Zones numbered row by row from the bottom-left: index = j*nx + i."""
    if nx <= 0 or ny <= 0:
        raise ValueError('zone counts must be positive')
    w, h = area.width / nx, area.height / ny
    return [Area(area.xmin + i * w, area.xmin + (i + 1) * w,
                 area.ymin + j * h, area.ymin + (j + 1) * h)
            for j in range(ny) for i in range(nx)]


def _zone_rows(zone, cur, spacing, area, pad):
    """Mini lawnmower inside one zone, entering from whichever side is closest to `cur`.

    Zones on the field boundary are stretched outward by `pad` (the wall margin) so the
    camera still sees the strip between the flight boundary and the wall.
    """
    lo = zone.ymin - (pad if zone.ymin <= area.ymin + 1e-9 else 0.0)
    hi = zone.ymax + (pad if zone.ymax >= area.ymax - 1e-9 else 0.0)
    n = max(1, math.ceil((hi - lo) / spacing))
    ys = [min(max(lo + (i + 0.5) * (hi - lo) / n, zone.ymin), zone.ymax) for i in range(n)]
    best, best_d = None, float('inf')
    for flip in (False, True):
        for left_first in (True, False):
            rows = ys[::-1] if flip else ys
            pts, left = [], left_first
            for y in rows:
                a, b = (zone.xmin, zone.xmax) if left else (zone.xmax, zone.xmin)
                pts += [(a, y), (b, y)]
                left = not left
            d = _dist(cur, pts[0])
            if d < best_d:
                best, best_d = pts, d
    return best


def zone_sweep(area, start, spacing, nx=3, ny=3, weights=None, cleared=(), pad=0.0):
    """Visit zones by priority, sweeping each one.

    priority = weight / (1 + distance from current position to zone center)
    With all weights equal this is "nearest zone next". Raise a weight to search a zone
    sooner (e.g. where a detection was seen). `cleared` lists zones to skip, so the same
    function replans after a target change: pass the current position as `start`.
    `pad` is the wall margin (field edge minus flight-area edge).
    """
    zones = zone_grid(area, nx, ny)
    if weights is not None and len(weights) != len(zones):
        raise ValueError(f'weights needs {len(zones)} values (nx*ny)')
    remaining = [i for i in range(len(zones)) if i not in set(cleared)]
    cur, path = start, []
    while remaining:
        def score(i):
            w = 1.0 if weights is None else weights[i]
            return w / (1.0 + _dist(cur, zones[i].center))
        best = max(remaining, key=score)
        sweep = _zone_rows(zones[best], cur, spacing, area, pad)
        path += sweep
        cur = sweep[-1]
        remaining.remove(best)
    return simplify(path)


# ------------------------------------------------------------------ 6. hybrid
def hybrid_sweep(area, start, spacing, rows_along='x'):
    """Coarse pass over every other row, then a fill pass over the rows in between."""
    cf = CornerFrame(area, start, swap=(rows_along == 'y'))
    vs = _row_positions(cf.V, spacing)
    pass1, at0 = _snake(cf, vs[0::2], True)
    pass2, _ = _snake(cf, vs[1::2][::-1], at0)
    return simplify(pass1 + pass2)


def _circle_hit(a, b, c, r):
    """Smallest t in [0, 1] where a + t*(b-a) is within r of c, or None."""
    dx, dy = b[0] - a[0], b[1] - a[1]
    fx, fy = a[0] - c[0], a[1] - c[1]
    C = fx * fx + fy * fy - r * r
    if C <= 0:
        return 0.0
    A = dx * dx + dy * dy
    if A < 1e-12:
        return None
    B = 2.0 * (dx * fx + dy * fy)
    disc = B * B - 4.0 * A * C
    if disc < 0:
        return None
    t = (-B - math.sqrt(disc)) / (2.0 * A)
    return t if 0.0 <= t <= 1.0 else None


def first_view(points, start, target, radius):
    """First place along start -> points[0] -> ... within `radius` of target.

    Returns (i, point): the point lies on the leg ending at points[i]; None if never seen.
    """
    prev = start
    for i, p in enumerate(points):
        t = _circle_hit(prev, p, target, radius)
        if t is not None:
            return i, _lerp(prev, p, t)
        prev = p
    return None


def hybrid(area, start, spacing, detect_radius, candidate=None,
           candidate_is_target=True, rows_along='x'):
    """Hybrid search. With `candidate` (x, y) it simulates a detection.

    The sweep runs until the candidate comes within detect_radius. Then the UAV flies to
    it. If it is the real target the path ends there (the node then returns home). If it
    is a false alarm the UAV goes back to where it left the sweep and resumes: this is
    the "drop the candidate and keep sweeping" behaviour Challenge 3 needs.
    """
    sweep = hybrid_sweep(area, start, spacing, rows_along)
    if candidate is None:
        return sweep
    hit = first_view(sweep, start, candidate, detect_radius)
    if hit is None:
        return sweep
    i, p = hit
    head = sweep[:i] + [p, tuple(candidate)]
    if candidate_is_target:
        return simplify(head)
    return simplify(head + [p] + sweep[i:])


# ------------------------------------------------------------------ obstacles (2-D, no fly-over)
# An obstacle is (x, y, radius). Offline stand-in for a real planner: at the competition
# obstacles are NOT known ahead of time, so perception has to supply them.
def _seg_point_dist(a, b, c):
    dx, dy = b[0] - a[0], b[1] - a[1]
    L2 = dx * dx + dy * dy
    t = 0.0 if L2 < 1e-12 else max(0.0, min(1.0, ((c[0] - a[0]) * dx + (c[1] - a[1]) * dy) / L2))
    return _dist((a[0] + t * dx, a[1] + t * dy), c), t


def _blocked_point(p, obstacles, clearance):
    return any(_dist(p, (ox, oy)) < r + clearance for ox, oy, r in obstacles)


def _route(a, b, obstacles, clearance, area, depth=0):
    """Waypoints from a to b (ending at b) that stay clear of obstacles, or None."""
    blockers = []
    for ob in obstacles:
        d, t = _seg_point_dist(a, b, (ob[0], ob[1]))
        if d < ob[2] + clearance:
            blockers.append((t, ob))
    if not blockers:
        return [b]
    if depth > 4:
        return None
    _, (ox, oy, r) = min(blockers, key=lambda x: x[0])
    R = r + clearance + 0.25
    L = _dist(a, b)
    nx_, ny_ = -(b[1] - a[1]) / L, (b[0] - a[0]) / L
    options = []
    for side in (1, -1):
        w = area.clamp((ox + side * nx_ * R, oy + side * ny_ * R))
        if _blocked_point(w, obstacles, clearance):
            continue
        options.append((_dist(a, w) + _dist(w, b), w))
    for _, w in sorted(options):
        r1 = _route(a, w, obstacles, clearance, area, depth + 1)
        r2 = _route(w, b, obstacles, clearance, area, depth + 1) if r1 else None
        if r1 and r2:
            return r1 + r2
    return None


def _push_out(p, obstacles, clearance, area):
    """Move a waypoint just outside any obstacle clearance bubble it sits in."""
    for _ in range(6):
        moved = False
        for ox, oy, r in obstacles:
            d = _dist(p, (ox, oy))
            need = r + clearance + 0.05
            if d < need:
                ux, uy = ((p[0] - ox) / d, (p[1] - oy) / d) if d > 1e-9 else (1.0, 0.0)
                p = area.clamp((ox + ux * need, oy + uy * need))
                moved = True
        if not moved:
            break
    return p


def avoid_obstacles(path, start, obstacles, clearance, area):
    """Insert detour waypoints so every leg clears every obstacle.

    Waypoints that sit inside an obstacle's clearance bubble are nudged just outside it
    first. Returns (new_path, number_of_legs_that_could_not_be_routed). Those legs are
    left straight, so check the count.
    """
    if not obstacles:
        return list(path), 0
    out, failed, prev = [], 0, start
    for p in [_push_out(q, obstacles, clearance, area) for q in path]:
        r = _route(prev, p, obstacles, clearance, area)
        if r is None:
            failed += 1
            r = [p]
        out += r
        prev = p
    return simplify(out), failed


# ------------------------------------------------------------------ 4. frontier
def frontier(area, start, fp, pad=0.0, cell=0.5, obstacles=(), clearance=0.5,
             goal=0.995, max_legs=400):
    """Greedy frontier search on a coverage grid.

    The field is cut into `cell`-sized squares. A square is "seen" once the camera
    footprint (a square of side fp) has covered it. Each step: pick the nearest unseen
    square (with a small penalty for sharp turns), fly toward it, and keep going straight
    while the footprint's leading edge is still over unseen squares. Stop at `goal`
    coverage. `pad` extends the grid beyond the flight area (use the wall margin).

    Offline version: it assumes the map fills in exactly as predicted. On the real UAV the
    same idea runs online, updating "seen" from the camera.
    """
    if not math.isfinite(cell) or cell <= 0:
        raise ValueError('cell must be positive and finite')
    if not math.isfinite(goal) or not 0 < goal <= 1:
        raise ValueError('goal must be in (0, 1]')
    cover = area.expand(pad)
    nx = max(1, math.ceil(cover.width / cell))
    ny = max(1, math.ceil(cover.height / cell))
    cw, ch = cover.width / nx, cover.height / ny
    half = fp / 2.0
    seen = [[False] * nx for _ in range(ny)]
    state = {'unseen': nx * ny}

    def center(i, j):
        return (cover.xmin + (i + 0.5) * cw, cover.ymin + (j + 0.5) * ch)

    def set_seen(i, j):
        if not seen[j][i]:
            seen[j][i] = True
            state['unseen'] -= 1

    def cell_of(p):
        i, j = int((p[0] - cover.xmin) / cw), int((p[1] - cover.ymin) / ch)
        return (i, j) if 0 <= i < nx and 0 <= j < ny else None

    for j in range(ny):               # squares under an obstacle are not searched
        for i in range(nx):
            c = center(i, j)
            if any(_dist(c, (ox, oy)) <= r for ox, oy, r in obstacles):
                set_seen(i, j)
    free_total = state['unseen']

    def mark(p):
        i0 = max(0, int((p[0] - half - cover.xmin) / cw))
        i1 = min(nx - 1, int((p[0] + half - cover.xmin) / cw))
        j0 = max(0, int((p[1] - half - cover.ymin) / ch))
        j1 = min(ny - 1, int((p[1] + half - cover.ymin) / ch))
        for j in range(j0, j1 + 1):
            for i in range(i0, i1 + 1):
                c = center(i, j)
                if abs(c[0] - p[0]) <= half and abs(c[1] - p[1]) <= half:
                    set_seen(i, j)

    def mark_leg(a, b):
        n = max(1, math.ceil(_dist(a, b) / (0.5 * min(cw, ch))))
        for k in range(n + 1):
            mark(_lerp(a, b, k / n))

    cur, heading, path = start, None, []
    mark(cur)
    step = min(cw, ch)
    for _ in range(max_legs):
        if state['unseen'] <= (1.0 - goal) * free_total:
            break
        best = None
        for j in range(ny):
            for i in range(nx):
                if seen[j][i]:
                    continue
                v = area.clamp(center(i, j))
                if _blocked_point(v, obstacles, clearance):
                    continue
                d = _dist(cur, v)
                if d < 1e-9:
                    continue
                cost = d
                if heading is not None:
                    cosang = ((v[0] - cur[0]) * heading[0] + (v[1] - cur[1]) * heading[1]) / d
                    cost *= 1.0 + 0.4 * (1.0 - cosang) / 2.0
                if best is None or cost < best[0]:
                    best = (cost, (i, j), v)
        if best is None:
            break
        _, (ci, cj), v = best
        d = _dist(cur, v)
        direction = ((v[0] - cur[0]) / d, (v[1] - cur[1]) / d)
        p = v
        while True:                    # run on while the leading edge still sees new squares
            q = (p[0] + direction[0] * step, p[1] + direction[1] * step)
            if not area.contains(q) or _blocked_point(q, obstacles, clearance):
                break
            lead = cell_of((q[0] + direction[0] * half, q[1] + direction[1] * half))
            if lead is not None and seen[lead[1]][lead[0]]:
                break
            p = q
        route = _route(cur, p, obstacles, clearance, area)
        if route is None:
            set_seen(ci, cj)           # cannot reach it: give up on this square
            continue
        for w in route:
            mark_leg(cur, w)
            heading = ((w[0] - cur[0]) / max(_dist(cur, w), 1e-9),
                       (w[1] - cur[1]) / max(_dist(cur, w), 1e-9))
            path.append(w)
            cur = w
        set_seen(ci, cj)               # guarantees progress at the wall margin
    return simplify([start] + path)[1:]
