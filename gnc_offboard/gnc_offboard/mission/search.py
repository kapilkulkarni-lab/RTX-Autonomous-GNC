"""SEARCHING's path: one of the repo's search planners (search_planners.py), chosen by the
`search_pattern` parameter. Pure; field frame (ENU, meters).

  frontier    (default) nearest unseen ground first, starting from the coverage memory, so a
              replanned search never re-flies covered ground; routes around obstacles itself
  lawnmower   fixed rows from the current position (for comparison; ignores coverage memory
              except to stop once the area is covered)
  zone_sweep  zones by distance from the current position (same caveat)
"""
from typing import List, Sequence, Tuple

from gnc_offboard import search_planners as sp
from gnc_offboard.mission.coverage import CoverageMap

PATTERNS = ('frontier', 'lawnmower', 'zone_sweep')
COVERED_GOAL = 0.995      # same goal frontier uses: "covered" means 99.5% of the area


def area_covered(coverage: CoverageMap, area: sp.Area, goal: float = COVERED_GOAL) -> bool:
    total = seen = 0
    for j in range(coverage.ny):
        for i in range(coverage.nx):
            c = coverage.center(i, j)
            if area.contains(c):
                total += 1
                seen += coverage.is_seen(*c)
    return total == 0 or seen >= goal * total


def plan_search(pattern: str, area: sp.Area, start_xy: Sequence[float], coverage: CoverageMap,
                footprint_m: float, spacing_m: float, wall_pad_m: float,
                obstacles: List[Tuple[float, float, float]], clearance_m: float
                ) -> List[Tuple[float, float]]:
    """Waypoints (x, y) from `start_xy` (not included). [] means the area is covered."""
    start = area.clamp(tuple(start_xy))
    if pattern == 'frontier':
        return sp.frontier(area, start, footprint_m, pad=wall_pad_m, cell=coverage.cw,
                           obstacles=obstacles, clearance=clearance_m,
                           goal=COVERED_GOAL, seen=coverage.is_seen)
    if area_covered(coverage, area):
        return []
    if pattern == 'lawnmower':
        path = sp.lawnmower(area, start, spacing_m)
    elif pattern == 'zone_sweep':
        path = sp.zone_sweep(area, start, spacing_m, pad=wall_pad_m)
    else:
        raise ValueError(f'search_pattern must be one of {PATTERNS}, got {pattern!r}')
    path, _failed = sp.avoid_obstacles(path, start, obstacles, clearance_m, area)
    return path
