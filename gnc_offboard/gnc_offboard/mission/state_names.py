"""The one place mission state names live.

Every name that comms defines is read from avc_msgs/MissionState's STATE_* constants, never
retyped. Nothing else in the package may hard-code a state name (test_state_names.py checks).
"""
from avc_msgs.msg import MissionState as _MissionState

STANDBY = _MissionState.STATE_STANDBY
PREFLIGHT = _MissionState.STATE_PREFLIGHT
TAKEOFF = _MissionState.STATE_TAKEOFF
SEARCHING = _MissionState.STATE_SEARCHING
APPROACHING = _MissionState.STATE_APPROACHING
NFC_READY = _MissionState.STATE_NFC_READY
READING = _MissionState.STATE_READING
RETRIEVING = _MissionState.STATE_RETRIEVING
RETURNING = _MissionState.STATE_RETURNING
LANDING = _MissionState.STATE_LANDING
RETURN_CONFIRMED = _MissionState.STATE_RETURN_CONFIRMED
ABORTED = _MissionState.STATE_ABORTED
PAUSED = _MissionState.STATE_PAUSED

# TODO(comms): switch to _MissionState.STATE_DESCENDING / STATE_LIFTOFF once Abhi adds them to
# avc_msgs/MissionState (open question in the outline). Local placeholders until then.
DESCENDING = 'DESCENDING'
LIFTOFF = 'LIFTOFF'

# All fifteen states, in the outline's table order.
ALL = (
    STANDBY, PREFLIGHT, TAKEOFF, SEARCHING, APPROACHING, DESCENDING, NFC_READY, READING,
    RETRIEVING, LIFTOFF, RETURNING, LANDING, RETURN_CONFIRMED, ABORTED, PAUSED,
)

# TEMPORARY (Milestone 6 only; replaced by NFC_READY / READING in Milestone 7): on the die after
# touchdown for on_die_test_s, then LIFTOFF -> RETURNING. Published honestly; kept out of ALL.
ON_DIE_TEST = 'ON_DIE_TEST'
TEMPORARY = (ON_DIE_TEST,)

# "On the die" means NFC_READY, READING or RETRIEVING (and ON_DIE_TEST while it exists).
ON_DIE = frozenset({NFC_READY, READING, RETRIEVING, ON_DIE_TEST})

# Terminal states: no pause, and every intent except abort is rejected after them.
END_STATES = frozenset({RETURN_CONFIRMED, ABORTED})

# Every name the state machine may publish right now.
PUBLISHABLE = ALL + TEMPORARY

# Flying (not on the ground or on the die).
AIRBORNE = frozenset({TAKEOFF, SEARCHING, APPROACHING, DESCENDING, LIFTOFF, RETURNING, LANDING})
