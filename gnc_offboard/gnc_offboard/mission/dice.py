"""Reading Perception's dice map (avc_interfaces/DiceMap, placeholder) and obstacle list.
Pure: messages are read through duck typing. GNC keeps no die memory of its own; the only
local state is which dice the mission gave up on, so it does not fly straight back to them:
a die whose wrong color Perception confirmed is excluded at once; a die that was lost or
never confirmed is excluded only after max_die_attempts such failures (count_failure).
"""
import math
from typing import Iterable, List, Optional, Sequence, Tuple

Point = Tuple[float, float, float]


def dice_of(dice_map_msg) -> List[Tuple[str, Point]]:
    """[(color, (x, y, z))] from a DiceMap message (None -> [])."""
    if dice_map_msg is None:
        return []
    out = []
    for die in getattr(dice_map_msg, 'dice', []) or []:
        c = die.centroid
        out.append(((die.color or '').strip().lower(), (float(c.x), float(c.y), float(c.z))))
    return out


def find_die(dice_map_msg, color: str, near: Optional[Sequence[float]] = None,
             exclude: Iterable[Sequence[float]] = (), exclude_radius_m: float = 0.0,
             fits=lambda point: True) -> Optional[Point]:
    """The centroid of a die of `color` in the dice map, nearest to `near` (if given).

    Dice within `exclude_radius_m` of an `exclude` point are skipped. `fits(point)` filters by
    the current intent: TEMPORARY until Milestone 9, every position fits.
    """
    color = (color or '').strip().lower()
    if not color:
        return None
    exclude = list(exclude)
    best, best_d = None, math.inf
    for c, point in dice_of(dice_map_msg):
        if c != color or not fits(point):
            continue
        if any(math.hypot(point[0] - e[0], point[1] - e[1]) <= exclude_radius_m
               for e in exclude):
            continue
        d = 0.0 if near is None else math.hypot(point[0] - near[0], point[1] - near[1])
        if d < best_d:
            best, best_d = point, d
    return best


def count_failure(failures: List[list], point: Sequence[float], radius_m: float) -> int:
    """Record one failed approach (lost / never confirmed) at `point` and return that die's
    failure count. `failures` is a list of [point, count]; points within `radius_m` are the
    same die (the first match keeps its original point)."""
    for entry in failures:
        p = entry[0]
        if math.hypot(p[0] - point[0], p[1] - point[1]) <= radius_m:
            entry[1] += 1
            return entry[1]
    failures.append([tuple(point), 1])
    return 1


def obstacles_of(obstacle_msg) -> List[Tuple[float, float, float]]:
    """[(x, y, radius)] from an ObstacleArray message (None -> [])."""
    if obstacle_msg is None:
        return []
    return [(float(o.x), float(o.y), float(o.radius))
            for o in getattr(obstacle_msg, 'obstacles', []) or []]
