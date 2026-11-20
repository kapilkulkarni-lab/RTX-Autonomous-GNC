"""What a stand-in Perception reports, given what the camera can see. Pure (no ROS), so the
fake_dice_map node and the tests share it. Field frame (ENU, meters); ground at z = 0.

  dice map         every die that has been inside the detection footprint, with the color
                   Perception believes (true color, or `wrong_color_first` for the first die
                   seen, until a close look confirms the true color and corrects the map)
  target_relative  the in-view die nearest the mission's target centroid, relative to the drone
  color check      a confirmed color only for the die nearest straight below, when it is within
                   confirm_radius_m horizontally and the drone is no higher than
                   confirm_max_alt_m
  drop             no detections at all (no map updates, target_relative or color)
"""
import math
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Sequence, Tuple


@dataclass(frozen=True)
class Die:
    color: str
    x: float
    y: float
    z: float = 0.0


@dataclass
class PerceptionOutput:
    dice: List[Tuple[str, Tuple[float, float, float]]] = field(default_factory=list)
    target_relative: Optional[Tuple[float, float, float]] = None
    color: Optional[str] = None


class PerceptionModel:
    def __init__(self, dice: Sequence[Die], confirm_radius_m: float = 0.3,
                 confirm_max_alt_m: float = 1.3, target_match_m: float = 1.0,
                 wrong_color_first: str = ''):
        self.dice = list(dice)
        self.confirm_radius_m = confirm_radius_m
        self.confirm_max_alt_m = confirm_max_alt_m
        self.target_match_m = target_match_m
        self.wrong_color_first = (wrong_color_first or '').strip().lower()
        self.reported: Dict[int, str] = {}       # die index -> color in Perception's map

    @classmethod
    def from_manifest(cls, manifest, **kwargs) -> 'PerceptionModel':
        dice = [Die(t['color'], float(t['x']), float(t['y']), float(t.get('z', 0.0)))
                for t in manifest.get('targets', [])]
        return cls(dice, **kwargs)

    def update(self, drone: Sequence[float], in_view: Callable[[float, float], bool],
               target_xy: Optional[Sequence[float]] = None, drop: bool = False
               ) -> PerceptionOutput:
        out = PerceptionOutput()
        if not drop:
            visible = [i for i, d in enumerate(self.dice) if in_view(d.x, d.y)]
            for i in visible:
                if i not in self.reported:
                    first = not self.reported
                    self.reported[i] = (self.wrong_color_first
                                        if first and self.wrong_color_first
                                        else self.dice[i].color)
            if target_xy is not None and visible:
                i = min(visible, key=lambda k: _dist(self.dice[k], target_xy))
                if _dist(self.dice[i], target_xy) <= self.target_match_m:
                    d = self.dice[i]
                    out.target_relative = (d.x - drone[0], d.y - drone[1], d.z - drone[2])
            if visible and drone[2] <= self.confirm_max_alt_m:
                i = min(visible, key=lambda k: _dist(self.dice[k], drone))
                if _dist(self.dice[i], drone) <= self.confirm_radius_m:
                    out.color = self.dice[i].color
                    self.reported[i] = out.color      # a close look corrects the map
        out.dice = [(color, (self.dice[i].x, self.dice[i].y, self.dice[i].z))
                    for i, color in sorted(self.reported.items())]
        return out


def _dist(die: Die, xy: Sequence[float]) -> float:
    return math.hypot(die.x - xy[0], die.y - xy[1])


def square_view(drone: Sequence[float], side_m: float) -> Callable[[float, float], bool]:
    """Nadir square footprint of `side_m` under the drone (tests; the node uses CameraModel)."""
    half = side_m / 2.0
    return lambda x, y: abs(x - drone[0]) <= half and abs(y - drone[1]) <= half
