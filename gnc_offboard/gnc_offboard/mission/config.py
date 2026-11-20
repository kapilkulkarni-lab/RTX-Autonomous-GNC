"""MissionConfig: typed view of config/mission.yaml. No defaults here; the yaml is the source."""
import dataclasses
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Union

import yaml

NODE_KEY = 'mission_manager'


@dataclass(frozen=True)
class MissionConfig:
    loop_hz: float
    setpoint_hz: float
    state_pub_hz: float
    ekf_stable_s: float
    arm_timeout_s: float
    offboard_warmup_s: float
    cmd_retry_s: float
    require_pilot_arm: bool
    search_altitude_m: float
    altitude_tol_m: float
    hover_test_s: float               # TEMPORARY (Milestone 3)
    landed_disarm_grace_s: float
    pose_invalid_s: float
    px4_status_stale_s: float
    battery_return_pct: float
    force_return_at_s: float
    land_after_hold_s: float
    px4_discovery_timeout_s: float

    @classmethod
    def names(cls):
        return tuple(f.name for f in dataclasses.fields(cls))

    @classmethod
    def from_dict(cls, values: Dict[str, Any]) -> 'MissionConfig':
        names = set(cls.names())
        missing, extra = names - set(values), set(values) - names
        if missing or extra:
            raise ValueError(f'mission config: missing {sorted(missing)}, unknown {sorted(extra)}')
        typed = {}
        for f in dataclasses.fields(cls):
            v = values[f.name]
            if f.type is bool:
                if not isinstance(v, bool):
                    raise ValueError(f'{f.name} must be true or false, got {v!r}')
                typed[f.name] = v
            else:
                typed[f.name] = float(v)
        return cls(**typed)


def load_yaml_params(path: Union[str, Path]) -> Dict[str, Any]:
    """The ros__parameters dict for mission_manager from a ROS params yaml file."""
    data = yaml.safe_load(Path(path).read_text())
    return dict(data[NODE_KEY]['ros__parameters'])
