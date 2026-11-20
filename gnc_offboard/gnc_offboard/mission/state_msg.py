"""The one builder for every avc_msgs/MissionState that mission_manager publishes."""
from typing import Optional

from avc_msgs.msg import MissionState

from . import state_names

REASON_MAX_CHARS = 79          # comms: "under 80 characters"
_UINT32_WRAP = 2 ** 32
_RESULTS = ('', MissionState.RESULT_APPLIED, MissionState.RESULT_REJECTED)


def clean_reason(reason: Optional[str]) -> str:
    """Plain one-line text, no '|' (the radio frame separator), cut to under 80 characters."""
    text = (reason or '').replace('|', '')
    text = ' '.join(text.split())          # newlines and runs of whitespace -> single spaces
    return text[:REASON_MAX_CHARS].rstrip()


class MissionStateBuilder:
    """Builds MissionState messages and owns `seq` (+1 on every build; wraps to 1, never 0)."""

    def __init__(self) -> None:
        self.seq = 0

    def build(self, *, state: str, previous_state: str = '', paused_from: str = '',
              reason: str = '', active_target: str = '', mission_time_s: float = 0.0,
              last_intent_id: int = 0, last_intent_result: str = '',
              stamp_ns: int = 0) -> MissionState:
        if state not in state_names.PUBLISHABLE:
            raise ValueError(f'unknown state {state!r}')
        if state == state_names.PAUSED:
            # While paused, previous_state is the state the pause came from.
            if paused_from not in state_names.PUBLISHABLE or paused_from == state_names.PAUSED:
                raise ValueError(f'PAUSED needs a valid paused_from, got {paused_from!r}')
            previous_state = paused_from
        elif previous_state and previous_state not in state_names.PUBLISHABLE:
            raise ValueError(f'unknown previous_state {previous_state!r}')
        if last_intent_result not in _RESULTS:
            raise ValueError(f'last_intent_result must be one of {_RESULTS}, got {last_intent_result!r}')

        # Wrap at 2**32 but skip 0: comms' last_state_seq uses 0 for "none received".
        self.seq = (self.seq + 1) % _UINT32_WRAP or 1

        msg = MissionState()
        msg.header.stamp.sec = int(stamp_ns // 1_000_000_000)
        msg.header.stamp.nanosec = int(stamp_ns % 1_000_000_000)
        msg.state = state
        msg.previous_state = previous_state
        msg.reason = clean_reason(reason)
        msg.active_target = active_target
        msg.mission_time_s = float(mission_time_s)
        msg.seq = self.seq
        msg.last_intent_id = int(last_intent_id)
        msg.last_intent_result = last_intent_result
        return msg
