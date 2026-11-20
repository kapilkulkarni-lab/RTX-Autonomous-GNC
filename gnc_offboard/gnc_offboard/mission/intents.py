"""Intent vocabulary and the stored region fields (outline: Intent handling, Regions).

Pure: no rclpy. Messages are read through duck typing (avc_msgs/Intent fields).
"""
from dataclasses import dataclass, replace
from typing import Optional

# Action names: avc_comms/avc_comms/schema.py ACTIONS at avc-comms-ws 366e2ba. avc_msgs/Intent
# carries the name as a string; there are no constants for it in the message.
UPDATE_ONLY = 'update_only'
RETRIEVE = 'retrieve'
READ_CHIP = 'read_chip'
ABORT = 'abort'
PAUSE = 'pause'
RETURN_TO_START = 'return_to_start'
RESUME = 'resume'
ACTIONS = (UPDATE_ONLY, RETRIEVE, READ_CHIP, ABORT, PAUSE, RETURN_TO_START, RESUME)

# While PAUSED these are stored and count as a new intent (any action except pause and resume
# that does not act at once: abort and return_to_start act at once, read_chip is rejected).
STORED_WHILE_PAUSED = frozenset({RETRIEVE, UPDATE_ONLY})

# The only three rejections (reason text exactly as in the outline).
REJECT_READ_CHIP = 'read_chip rejected: not on the die'
REJECT_RESUME = 'resume ignored: not paused'
REJECT_MISSION_OVER = 'mission over: intent ignored'


def action_of(msg) -> str:
    """The intent's action; empty means update_only."""
    return (getattr(msg, 'action', '') or '').strip().lower() or UPDATE_ONLY


def color_of(msg) -> str:
    """The intent's target color, '' if not set."""
    return (getattr(msg, 'target_color', '') or '').strip().lower()


@dataclass(frozen=True)
class RegionFields:
    """Region fields from intents, stored as received (quadrant masks, 0 = not set).

    Decoding and behavior (avoid, stay, until, target quadrant) come in Milestone 9.
    `quadrant_hint_color` is the color an update_only named with its target_quadrant: a
    location hint for that die, never a retarget.
    """
    avoid_mask: int = 0
    stay_mask: int = 0
    until_mask: int = 0
    target_quadrant: int = 0
    quadrant_hint_color: str = ''


_MASKS = ('avoid_mask', 'stay_mask', 'until_mask', 'target_quadrant')


def merge_regions(current: Optional[RegionFields], msg, hint: bool = False) -> RegionFields:
    """Apply an intent's region fields: set fields replace, empty (0) fields mean no change.

    With `hint` (update_only), a color sent together with a target_quadrant is stored as the
    location hint for that quadrant.
    """
    merged = current or RegionFields()
    changes = {name: int(getattr(msg, name, 0) or 0) for name in _MASKS}
    merged = replace(merged, **{k: v for k, v in changes.items() if v})
    if hint and changes['target_quadrant'] and color_of(msg):
        merged = replace(merged, quadrant_hint_color=color_of(msg))
    return merged
