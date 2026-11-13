"""Rectangular search areas and visibility routing in field coordinates."""
import math

from gnc_offboard.search_planners import Area, _dist, simplify


def rect_subtract(r, k):
    """Disjoint rectangles that make up r minus k (r itself if they do not overlap)."""
    ix0, ix1 = max(r.xmin, k.xmin), min(r.xmax, k.xmax)
    iy0, iy1 = max(r.ymin, k.ymin), min(r.ymax, k.ymax)
    if ix0 >= ix1 or iy0 >= iy1:
        return [r]
    out = []
    if r.ymin < iy0:
        out.append(Area(r.xmin, r.xmax, r.ymin, iy0))
    if iy1 < r.ymax:
        out.append(Area(r.xmin, r.xmax, iy1, r.ymax))
    if r.xmin < ix0:
        out.append(Area(r.xmin, ix0, iy0, iy1))
    if ix1 < r.xmax:
        out.append(Area(ix1, r.xmax, iy0, iy1))
    return out


def rects_from_flat(flat):
    """[x0, y0, x1, y1, x0, y0, x1, y1, ...] -> list of Area (corners in any order)."""
    if len(flat) % 4 or not all(math.isfinite(value) for value in flat):
        raise ValueError('rectangles must be x0, y0, x1, y1 groups of four finite numbers')
    rectangles = [Area(min(flat[i], flat[i + 2]), max(flat[i], flat[i + 2]),
                 min(flat[i + 1], flat[i + 3]), max(flat[i + 1], flat[i + 3]))
            for i in range(0, len(flat), 4)]
    if any(rect.width <= 0 or rect.height <= 0 for rect in rectangles):
        raise ValueError('rectangles must have positive width and height')
    return rectangles


# fractions of the field: (xmin, xmax, ymin, ymax). x = east, y = north.
_NAMED = {'all': (0, 1, 0, 1), 'north': (0, 1, .5, 1), 'south': (0, 1, 0, .5),
          'east': (.5, 1, 0, 1), 'west': (0, .5, 0, 1),
          'ne': (.5, 1, .5, 1), 'nw': (0, .5, .5, 1), 'se': (.5, 1, 0, .5), 'sw': (0, .5, 0, .5),
          'center': (1 / 3, 2 / 3, 1 / 3, 2 / 3),
          'north third': (0, 1, 2 / 3, 1), 'south third': (0, 1, 0, 1 / 3),
          'east third': (2 / 3, 1, 0, 1), 'west third': (0, 1 / 3, 0, 1)}
_ALIASES = {'northeast': 'ne', 'northwest': 'nw', 'southeast': 'se', 'southwest': 'sw',
            'middle': 'center', 'centre': 'center', 'whole': 'all', 'everything': 'all'}


def named_region(name, field):
    """'north half', 'ne quadrant', 'center', 'east third', 'all' -> Area inside `field`."""
    key = name.lower().replace('_', ' ').replace('-', ' ')
    for w in ('half', 'quadrant', 'side', 'part', 'of the field'):
        key = key.replace(w, '')
    key = ' '.join(key.split())
    key = _ALIASES.get(key, key)
    if key not in _NAMED:
        raise ValueError(f'unknown region name {name!r}; try one of {sorted(_NAMED)}')
    fx0, fx1, fy0, fy1 = _NAMED[key]
    return Area(field.xmin + fx0 * field.width, field.xmin + fx1 * field.width,
                field.ymin + fy0 * field.height, field.ymin + fy1 * field.height)


def _in_open(p, k, eps=1e-9):
    return k.xmin + eps < p[0] < k.xmax - eps and k.ymin + eps < p[1] < k.ymax - eps


def _seg_hits_rect(a, b, r, eps=1e-9):
    """True if segment a-b passes through the OPEN interior of rectangle r (Liang-Barsky)."""
    x0, x1, y0, y1 = r.xmin + eps, r.xmax - eps, r.ymin + eps, r.ymax - eps
    if x0 >= x1 or y0 >= y1:
        return False
    dx, dy = b[0] - a[0], b[1] - a[1]
    t0, t1 = 0.0, 1.0
    for p, q in ((-dx, a[0] - x0), (dx, x1 - a[0]), (-dy, a[1] - y0), (dy, y1 - a[1])):
        if abs(p) < 1e-12:
            if q <= 0:
                return False
        else:
            t = q / p
            if p < 0:
                if t > t1:
                    return False
                t0 = max(t0, t)
            else:
                if t < t0:
                    return False
                t1 = min(t1, t)
    return t0 < t1 - 1e-9


def route_rects(a, b, keepouts, area):
    """Shortest path a -> b around keep-out rectangles (already inflated by clearance).

    Visibility graph over the rectangle corners plus Dijkstra. Returns the waypoints
    AFTER a, ending at b, or None if a or b is inside a keep-out or no route exists.
    """
    if any(_in_open(a, k) or _in_open(b, k) for k in keepouts):
        return None

    def free(p, q):
        return not any(_seg_hits_rect(p, q, k) for k in keepouts)

    if free(a, b):
        return [b]
    nodes = [a, b]
    for k in keepouts:
        for c in ((k.xmin, k.ymin), (k.xmin, k.ymax), (k.xmax, k.ymin), (k.xmax, k.ymax)):
            c = area.clamp(c)
            if not any(_in_open(c, k2) for k2 in keepouts):
                nodes.append(c)
    n = len(nodes)
    dist = [float('inf')] * n
    prev = [-1] * n
    done = [False] * n
    dist[0] = 0.0
    for _ in range(n):
        u = min((i for i in range(n) if not done[i]), key=lambda i: dist[i], default=None)
        if u is None or dist[u] == float('inf'):
            break
        done[u] = True
        if u == 1:
            break
        for v in range(n):
            if not done[v] and free(nodes[u], nodes[v]):
                d = dist[u] + _dist(nodes[u], nodes[v])
                if d < dist[v]:
                    dist[v], prev[v] = d, u
    if dist[1] == float('inf'):
        return None
    out, i = [], 1
    while i != 0:
        out.append(nodes[i])
        i = prev[i]
    return out[::-1]


def restricted_search(regions, keepouts, start, area, plan):
    """Sweep the disjoint flyable pieces of the selected regions."""
    pieces = []
    for region in regions:
        clipped = Area(max(region.xmin, area.xmin), min(region.xmax, area.xmax),
                       max(region.ymin, area.ymin), min(region.ymax, area.ymax))
        if clipped.width <= 0 or clipped.height <= 0:
            continue
        remaining = [clipped]
        for exclusion in [*keepouts, *pieces]:
            remaining = [part for piece in remaining for part in rect_subtract(piece, exclusion)]
        pieces.extend(remaining)

    path, current = [], start
    while pieces:
        piece = min(pieces, key=lambda rect: _dist(current, rect.clamp(current)))
        pieces.remove(piece)
        corners = [(x, y) for x in (piece.xmin, piece.xmax) for y in (piece.ymin, piece.ymax)]
        entry = min(corners, key=lambda point: _dist(current, point))
        sweep = plan(piece, entry)
        if sweep is None:  # e.g. every zone in this piece was cleared
            continue
        path.extend([entry, *sweep])
        current = path[-1]
    if not path:
        raise ValueError('no searchable area remains inside the flight bounds')
    return simplify(path)


def route_path(start, waypoints, keepouts, area):
    """Route every leg around the same exclusions, or reject the mission."""
    if any(_in_open(start, rect) for rect in keepouts):
        raise ValueError('start position is inside a keepout or its clearance buffer')
    path, current = [], start
    for target in waypoints:
        leg = route_rects(current, target, keepouts, area)
        if leg is None:
            raise ValueError(f'no route around keepouts from {current} to {target}')
        path.extend(leg)
        current = target
    return path
