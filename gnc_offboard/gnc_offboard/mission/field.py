"""FieldConfig: field geometry and the PX4 <-> field frame link, from one of two sources.

  - sim:      the world manifest JSON written by generate_avc_world.py (follows start_sim.sh -k)
  - hardware: config/field.yaml (field bounds, start zone, and where PX4's local NED origin sits
              in the field frame: the spawn point in sim, the mocap origin under mocap)

Both load into the same FieldConfig. Field frame: ENU, origin at the field center, +x east,
+y north, meters. The PX4 <-> field transform is a translation only (PX4 north = field +y);
field north is an open question in the outline.
"""
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Tuple, Union

import yaml

from gnc_offboard.frames import FieldFrame
from gnc_offboard.search_planners import Area

Bounds = Tuple[float, float, float, float]      # x0, y0, x1, y1


@dataclass(frozen=True)
class FieldConfig:
    bounds: Bounds
    start_zone: Bounds
    start_center: Tuple[float, float]
    px4_origin: Tuple[float, float]             # PX4 local NED origin, in the field frame
    source: str = ''

    @property
    def frame(self) -> FieldFrame:
        return FieldFrame(*self.px4_origin)

    def area(self, margin: float = 0.0) -> Area:
        x0, y0, x1, y1 = self.bounds
        area = Area(x0 + margin, x1 - margin, y0 + margin, y1 - margin)
        if area.width <= 0 or area.height <= 0:
            raise ValueError('margin leaves no flight area')
        return area

    def contains(self, x: float, y: float) -> bool:
        x0, y0, x1, y1 = self.bounds
        return x0 <= x <= x1 and y0 <= y <= y1

    @classmethod
    def load(cls, path: Union[str, Path]) -> 'FieldConfig':
        path = Path(path)
        if path.suffix == '.json':
            return cls.from_manifest(json.loads(path.read_text()), str(path))
        return cls.from_yaml(yaml.safe_load(path.read_text()), str(path))

    @classmethod
    def from_manifest(cls, manifest, source: str = '') -> 'FieldConfig':
        frame = FieldFrame.from_manifest(manifest)
        return cls(bounds=_bounds(manifest['field']['bounds']),
                   start_zone=_bounds(manifest['start_zone']['bounds']),
                   start_center=_pair(manifest['start_zone']['center']),
                   px4_origin=(frame.sx, frame.sy), source=source)

    @classmethod
    def from_yaml(cls, data, source: str = '') -> 'FieldConfig':
        return cls(bounds=_bounds(data['field']['bounds']),
                   start_zone=_bounds(data['start_zone']['bounds']),
                   start_center=_pair(data['start_zone']['center']),
                   px4_origin=_pair(data['px4_origin']), source=source)


def _bounds(values) -> Bounds:
    x0, y0, x1, y1 = (float(v) for v in values)
    if not (x0 < x1 and y0 < y1):
        raise ValueError(f'bounds must be x0 < x1, y0 < y1, got {values}')
    return (x0, y0, x1, y1)


def _pair(values) -> Tuple[float, float]:
    x, y = (float(v) for v in values)
    return (x, y)
