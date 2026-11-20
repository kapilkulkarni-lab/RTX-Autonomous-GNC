"""SEARCHING's path, from the repo's search planners (search_planners.py), chosen by the
`search_pattern` parameter. Field frame (ENU, meters). The coverage memory is a
camera_model.CoverageGrid marked with both cameras' projected footprints.

  lawnmower   (default) parallel rows (`search_rows_along`), row spacing from the down
              camera's footprint width across the rows; rows already covered are skipped,
              the rest are flown in order from the corner nearest the drone, entering each
              row at its nearer end
  frontier    nearest unseen ground first, starting from the coverage memory
  zone_sweep  zones by distance from the current position (for comparison; it does not skip
              covered ground except to stop once the area is covered)
"""
import math
from typing import List, Sequence, Tuple

import numpy as np

from gnc_offboard import search_planners as sp
from gnc_offboard.camera_model import CoverageGrid

PATTERNS = ('lawnmower', 'frontier', 'zone_sweep')
ROWS_ALONG = ('x', 'y')
COVERED_GOAL = 0.995      # "covered" means 99.5% of the cells (the goal frontier uses)

Row = Tuple[Tuple[float, float], Tuple[float, float]]


def _cells_in(coverage: CoverageGrid, x0, x1, y0, y1):
    c = coverage.centres
    return (coverage.valid & (c[..., 0] >= x0) & (c[..., 0] <= x1)
            & (c[..., 1] >= y0) & (c[..., 1] <= y1))


def _covered(coverage: CoverageGrid, mask) -> bool:
    total = int(np.count_nonzero(mask))
    return total == 0 or np.count_nonzero(coverage.searched & mask) >= COVERED_GOAL * total


def area_covered(coverage: CoverageGrid, area: sp.Area) -> bool:
    return _covered(coverage, _cells_in(coverage, area.xmin, area.xmax, area.ymin, area.ymax))


def lawnmower_rows(area: sp.Area, start: Sequence[float], spacing_m: float,
                   rows_along: str = 'x') -> List[Row]:
    """The lawnmower's rows, in order from the corner nearest `start` (same rows as
    search_planners.lawnmower)."""
    if rows_along not in ROWS_ALONG:
        raise ValueError(f'search_rows_along must be x or y, got {rows_along!r}')
    cf = sp.CornerFrame(area, start, swap=(rows_along == 'y'))
    return [(cf.to_field(0.0, v), cf.to_field(cf.U, v))
            for v in sp._row_positions(cf.V, spacing_m)]


def row_covered(coverage: CoverageGrid, row: Row, swath_m: float) -> bool:
    """A row is covered when its swath (the row line +- swath_m / 2) is covered."""
    (ax, ay), (bx, by) = row
    half = swath_m / 2.0
    if abs(ay - by) < 1e-9:          # row along x
        mask = _cells_in(coverage, min(ax, bx), max(ax, bx), ay - half, ay + half)
    else:                            # row along y
        mask = _cells_in(coverage, ax - half, ax + half, min(ay, by), max(ay, by))
    return _covered(coverage, mask)


def lawnmower_skipping(area: sp.Area, start: Sequence[float], coverage: CoverageGrid,
                       spacing_m: float, rows_along: str) -> List[Tuple[float, float]]:
    """Lawnmower without the rows already covered; each row entered at its nearer end."""
    path, cur = [], tuple(start)
    for row in lawnmower_rows(area, start, spacing_m, rows_along):
        if row_covered(coverage, row, spacing_m):
            continue
        a, b = row
        if math.dist(cur, b) < math.dist(cur, a):
            a, b = b, a
        path += [a, b]
        cur = b
    return sp.simplify(path)


def plan_search(pattern: str, area: sp.Area, start_xy: Sequence[float], coverage: CoverageGrid,
                footprint_m: float, spacing_m: float, wall_pad_m: float,
                obstacles: List[Tuple[float, float, float]], clearance_m: float,
                rows_along: str = 'x') -> List[Tuple[float, float]]:
    """Waypoints (x, y) from `start_xy` (not included). [] means the area is covered.

    footprint_m: the down camera's footprint width (frontier's square footprint);
    spacing_m: row spacing (lawnmower, zone_sweep)."""
    start = area.clamp(tuple(start_xy))
    if pattern == 'frontier':
        return sp.frontier(area, start, footprint_m, pad=wall_pad_m, cell=coverage.resolution,
                           obstacles=obstacles, clearance=clearance_m,
                           goal=COVERED_GOAL, seen=coverage.is_seen)
    if area_covered(coverage, area):
        return []
    if pattern == 'lawnmower':
        path = lawnmower_skipping(area, start, coverage, spacing_m, rows_along)
        if not path:
            return []
    elif pattern == 'zone_sweep':
        path = sp.zone_sweep(area, start, spacing_m, pad=wall_pad_m)
    else:
        raise ValueError(f'search_pattern must be one of {PATTERNS}, got {pattern!r}')
    path, _failed = sp.avoid_obstacles(path, start, obstacles, clearance_m, area)
    return path
