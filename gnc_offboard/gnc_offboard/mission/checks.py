"""Pure decision functions for the mission state machine.

No rclpy, no clock and no parameters inside: every time is passed in as `now` (seconds, monotonic)
and every threshold is passed in from config/mission.yaml by the caller. Messages are read
through duck typing, so tests can pass simple stand-ins.

Region and until-rule checks are deferred to Milestone 9 (region encoding is still open).
"""
import enum
import math
from typing import Optional, Sequence

from .context import Stamped


# --- NFC read completion -------------------------------------------------------------------------

def read_complete(read_id: int, published_at: float, heartbeat: Optional[Stamped]) -> bool:
    """True when comms has acknowledged NFC read `read_id` (option B, decided Nov 19).

    avc_msgs/LlmHeartbeat has no verdict field. A read is complete when the newest heartbeat
    echoes this read's id in last_nfc_read_id. The read's success or failure then comes from
    the hardware result (NfcResult.success), not from comms.

    The heartbeat must have been received after the result was first published (`published_at`):
    read_id restarts at 1 when GNC restarts, and comms may still be echoing the previous run's
    read 1, so an older heartbeat must not complete the new read. An echo of a different
    read_id does not complete it. There is deliberately no timeout.

    If comms later adds a verdict field, this is the only function that changes.
    """
    if heartbeat is None or read_id <= 0:
        return False
    if heartbeat.rx_time <= published_at:
        return False
    return heartbeat.msg.last_nfc_read_id == read_id


# --- Color check ---------------------------------------------------------------------------------

class ColorResult(enum.Enum):
    RIGHT = 'right'
    WRONG = 'wrong'
    PENDING = 'pending'   # no confirmed color yet: keep waiting; never causes a transition


def check_color(confirmed_color: Optional[str], target: str) -> ColorResult:
    """Compare Perception's confirmed die color with the active target.

    Perception owns the color check end to end (BrainChip, windowing, confidence threshold) and
    publishes a confirmed color. GNC reads only the color (ColorCheck.color_guess), never the
    confidence. A missing or empty color is PENDING. A die that is never confirmed is handled by
    the target-lost timeout, not here.
    """
    color = (confirmed_color or '').strip().lower()
    if not color:
        return ColorResult.PENDING
    return ColorResult.RIGHT if color == target.strip().lower() else ColorResult.WRONG


def confirmed_color(color_check: Optional[Stamped]) -> Optional[str]:
    """The color field of the latest stored ColorCheck, or None if none has arrived."""
    return None if color_check is None else color_check.msg.color_guess


# --- Staleness -----------------------------------------------------------------------------------

def is_stale(stamped: Optional[Stamped], now: float, max_age_s: float) -> bool:
    """True if an input never arrived or its last receive time is older than `max_age_s`.

    Inputs are judged by age, never by a "received" flag.
    """
    if stamped is None:
        return True
    return (now - stamped.rx_time) > max_age_s


def invalid_for(since: Optional[float], now: float, limit_s: float) -> bool:
    """True once a condition that started at `since` (None = not active) has lasted over `limit_s`.

    Used for "position estimate invalid for N s".
    """
    return since is not None and (now - since) > limit_s


# --- Centering -----------------------------------------------------------------------------------

def centering_error(dx_m: float, dy_m: float) -> float:
    """Horizontal (sideways) distance between the drone and the target, in meters."""
    return math.hypot(dx_m, dy_m)


def stage_tolerance(height_m: float, descent_stages_m: Sequence[float],
                    stage_lateral_tol_m: Sequence[float]) -> float:
    """Sideways tolerance for the descent stage at `height_m`.

    `descent_stages_m` are the stage heights, highest first (e.g. [1.0, 0.5, 0.25]), and
    `stage_lateral_tol_m` the matching tolerances. Above the first stage height the first
    tolerance applies; at or below a stage height that stage's tolerance applies.
    """
    if len(descent_stages_m) != len(stage_lateral_tol_m) or not descent_stages_m:
        raise ValueError('descent_stages_m and stage_lateral_tol_m must be the same, non-zero length')
    if list(descent_stages_m) != sorted(descent_stages_m, reverse=True):
        raise ValueError('descent_stages_m must be ordered highest first')
    tol = stage_lateral_tol_m[0]
    for stage_h, stage_tol in zip(descent_stages_m, stage_lateral_tol_m):
        if height_m <= stage_h:
            tol = stage_tol
    return tol


def is_centered(dx_m: float, dy_m: float, tolerance_m: float) -> bool:
    """True while the sideways error is within `tolerance_m`."""
    return centering_error(dx_m, dy_m) <= tolerance_m


# --- Touchdown -----------------------------------------------------------------------------------

def update_below_since(below_since: Optional[float], range_m: Optional[float],
                       touchdown_range_m: float, now: float) -> Optional[float]:
    """Track when the rangefinder first read at or under `touchdown_range_m`.

    Returns the new `below_since`: kept while the reading stays low, set to `now` when it first
    goes low, and None when it is high or missing (NaN or None).
    """
    if range_m is None or math.isnan(range_m) or range_m > touchdown_range_m:
        return None
    return now if below_since is None else below_since


def touchdown_confirmed(below_since: Optional[float], now: float, touchdown_hold_s: float) -> bool:
    """Touchdown is confirmed from the rangefinder: low continuously for `touchdown_hold_s`.

    PX4's land detector alone is not enough (outline, DESCENDING).
    """
    return below_since is not None and (now - below_since) >= touchdown_hold_s
