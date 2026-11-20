"""Coverage memory: the ground the downward camera has already covered, so the search never
re-flies it. Pure Python; field frame (ENU, meters).

A cell counts as covered once its center has been inside the detection footprint: a square
of side footprint(altitude, detect_fov_deg) centered under the drone (nadir camera).
"""
import math
from typing import Tuple

Bounds = Tuple[float, float, float, float]


class CoverageMap:
    def __init__(self, bounds: Bounds, cell_m: float):
        if not math.isfinite(cell_m) or cell_m <= 0:
            raise ValueError('cell_m must be positive')
        self.x0, self.y0, self.x1, self.y1 = bounds
        self.nx = max(1, math.ceil((self.x1 - self.x0) / cell_m))
        self.ny = max(1, math.ceil((self.y1 - self.y0) / cell_m))
        self.cw = (self.x1 - self.x0) / self.nx
        self.ch = (self.y1 - self.y0) / self.ny
        self.clear()

    def clear(self) -> None:
        self._seen = [[False] * self.nx for _ in range(self.ny)]
        self.count = 0

    def _cell(self, x, y):
        i, j = int((x - self.x0) / self.cw), int((y - self.y0) / self.ch)
        if 0 <= i < self.nx and 0 <= j < self.ny:
            return i, j
        return None

    def center(self, i, j):
        return (self.x0 + (i + 0.5) * self.cw, self.y0 + (j + 0.5) * self.ch)

    def mark(self, x: float, y: float, side_m: float) -> int:
        """Mark cells whose centers lie in the square of `side_m` centered at (x, y).
        Returns how many cells were newly covered."""
        if not (math.isfinite(x) and math.isfinite(y)) or side_m <= 0:
            return 0
        half = side_m / 2.0
        i0 = max(0, int((x - half - self.x0) / self.cw))
        i1 = min(self.nx - 1, int((x + half - self.x0) / self.cw))
        j0 = max(0, int((y - half - self.y0) / self.ch))
        j1 = min(self.ny - 1, int((y + half - self.y0) / self.ch))
        new = 0
        for j in range(j0, j1 + 1):
            for i in range(i0, i1 + 1):
                cx, cy = self.center(i, j)
                if abs(cx - x) <= half and abs(cy - y) <= half and not self._seen[j][i]:
                    self._seen[j][i] = True
                    new += 1
        self.count += new
        return new

    def is_seen(self, x: float, y: float) -> bool:
        cell = self._cell(x, y)
        return cell is not None and self._seen[cell[1]][cell[0]]

    @property
    def fraction(self) -> float:
        return self.count / (self.nx * self.ny)
