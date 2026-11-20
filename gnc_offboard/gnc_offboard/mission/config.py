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
    landed_disarm_grace_s: float
    pose_invalid_s: float
    perception_stale_s: float
    require_perception: bool
    px4_status_stale_s: float
    battery_return_pct: float
    force_return_at_s: float
    land_after_hold_s: float
    px4_discovery_timeout_s: float
    field_file: str
    search_pattern: str
    detect_fov_deg: float
    search_overlap: float
    coverage_cell_m: float
    wall_margin_m: float
    obstacle_clearance_m: float
    cruise_speed_mps: float
    max_accel_mps2: float
    acceptance_radius_m: float
    hover_height_m: float
    target_lost_s: float
    color_confirm_timeout_s: float
    approach_center_tol_m: float
    die_match_radius_m: float
    max_die_attempts: int
    target_hover_s: float             # TEMPORARY (Milestone 5; DESCENDING in Milestone 6)

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
            elif f.type is str:
                typed[f.name] = str(v)
            elif f.type is int:
                if isinstance(v, bool) or int(v) != v or int(v) < 1:
                    raise ValueError(f'{f.name} must be a whole number >= 1, got {v!r}')
                typed[f.name] = int(v)
            else:
                typed[f.name] = float(v)
        from gnc_offboard.mission.search import PATTERNS
        if typed['search_pattern'] not in PATTERNS:
            raise ValueError(f'search_pattern must be one of {PATTERNS}, '
                             f'got {typed["search_pattern"]!r}')
        return cls(**typed)


def load_yaml_params(path: Union[str, Path]) -> Dict[str, Any]:
    """The ros__parameters dict for mission_manager from a ROS params yaml file."""
    data = yaml.safe_load(Path(path).read_text())
    return dict(data[NODE_KEY]['ros__parameters'])
