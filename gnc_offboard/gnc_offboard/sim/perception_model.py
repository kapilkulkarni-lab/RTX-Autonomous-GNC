"""What a stand-in Perception reports, given what the cameras can see. Pure (no ROS), so the
camera_fov sim node and the tests share it. Field frame (ENU, meters); ground at z = 0.

  dice map         every die that has been inside either camera's footprint, with the color
                   Perception believes (true color, or `wrong_color_first` for the first die
                   seen, until a close look confirms the true color and corrects the map)
  target_relative  the die in the down camera's view nearest the mission's target centroid,
                   relative to the drone
  color check      a confirmed color (down camera only) for the die nearest straight below,
                   when it is within confirm_radius_m horizontally and the drone is no higher
                   than confirm_max_alt_m
  drop             no detections at all (no map updates, target_relative or color)
  blind_height_m   below this height the down camera sees nothing (no target_relative, color)
  noise_m          Gaussian noise (std, per axis) on target_relative, for realistic landings
"""
import math
import random
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
                 wrong_color_first: str = '', blind_height_m: float = 0.0,
                 noise_m: float = 0.0, seed: Optional[int] = None):
        self.dice = list(dice)
        self.confirm_radius_m = confirm_radius_m
        self.confirm_max_alt_m = confirm_max_alt_m
        self.target_match_m = target_match_m
        self.wrong_color_first = (wrong_color_first or '').strip().lower()
        self.reported: Dict[int, str] = {}       # die index -> color in Perception's map
        self.blind_height_m = blind_height_m
        self.noise_m = noise_m
        self.rng = random.Random(seed)

    @classmethod
    def from_manifest(cls, manifest, **kwargs) -> 'PerceptionModel':
        dice = [Die(t['color'], float(t['x']), float(t['y']), float(t.get('z', 0.0)))
                for t in manifest.get('targets', [])]
        return cls(dice, **kwargs)

    def update(self, drone: Sequence[float], in_view: Callable[[float, float], bool],
               target_xy: Optional[Sequence[float]] = None, drop: bool = False,
               close_view: Optional[Callable[[float, float], bool]] = None
               ) -> PerceptionOutput:
        """`in_view`: either camera (dice map). `close_view`: the down camera, used for
        target_relative and the color check (default: `in_view`)."""
        out = PerceptionOutput()
        close_view = close_view or in_view
        if not drop:
            seen = [i for i, d in enumerate(self.dice) if in_view(d.x, d.y)]
            blind = drone[2] < self.blind_height_m
            visible = [] if blind else [i for i, d in enumerate(self.dice)
                                        if close_view(d.x, d.y)]
            for i in seen:
                if i not in self.reported:
                    first = not self.reported
                    self.reported[i] = (self.wrong_color_first
                                        if first and self.wrong_color_first
                                        else self.dice[i].color)
            if target_xy is not None and visible:
                i = min(visible, key=lambda k: _dist(self.dice[k], target_xy))
                if _dist(self.dice[i], target_xy) <= self.target_match_m:
                    d = self.dice[i]
                    nx = self.rng.gauss(0.0, self.noise_m) if self.noise_m > 0 else 0.0
                    ny = self.rng.gauss(0.0, self.noise_m) if self.noise_m > 0 else 0.0
                    out.target_relative = (d.x - drone[0] + nx, d.y - drone[1] + ny,
                                           d.z - drone[2])
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
