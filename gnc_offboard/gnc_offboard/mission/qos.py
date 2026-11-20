"""Every QoS profile mission_manager uses, in one place (outline: Interfaces).

Profiles with a QoS in the outline's Interfaces tables match them exactly. The rest are marked
"# TBD: not in Interfaces table" and follow the placeholder rule below until the owning team
confirms them.

Compatibility rule (DDS request/offer): a subscription connects to a publisher only if the
publisher offers at least what the subscription requests.
  - Reliability: a RELIABLE subscription needs a RELIABLE publisher; a BEST_EFFORT subscription
    connects to either.
  - Durability: a TRANSIENT_LOCAL subscription needs a TRANSIENT_LOCAL publisher; a VOLATILE
    subscription connects to either (and gets no history).
A mismatch is silent: no error, just no messages (`ros2 topic info -v` shows both sides).
So, for placeholders:
  - Subscriptions whose publisher's QoS we don't know request the least, BEST_EFFORT and
    VOLATILE, so they connect to any publisher.
  - Our own publishers offer RELIABLE; /mission/target also offers TRANSIENT_LOCAL so a late
    subscriber still gets the latest target.

Only this module imports rclpy.qos; it builds profiles and never creates a node.
"""
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

_RELIABLE = ReliabilityPolicy.RELIABLE
_BEST_EFFORT = ReliabilityPolicy.BEST_EFFORT
_VOLATILE = DurabilityPolicy.VOLATILE
_TRANSIENT_LOCAL = DurabilityPolicy.TRANSIENT_LOCAL


def _profile(reliability, durability, depth):
    return QoSProfile(reliability=reliability, durability=durability,
                      history=HistoryPolicy.KEEP_LAST, depth=depth)


# --- From the Interfaces tables ------------------------------------------------------------------

# /mission/intent (sub): reliable, volatile, keep last 10.
INTENT = _profile(_RELIABLE, _VOLATILE, 10)

# /llm/heartbeat (sub): reliable, volatile, keep last 10 (Abhi's proposal). Must stay volatile:
# comms publishes volatile, and a transient local subscription would get nothing.
LLM_HEARTBEAT = _profile(_RELIABLE, _VOLATILE, 10)

# /mission/state (pub): reliable, transient local, keep last 1.
MISSION_STATE = _profile(_RELIABLE, _TRANSIENT_LOCAL, 1)

# /mission/nfc_result (pub): reliable, transient local, keep last 10.
NFC_RESULT = _profile(_RELIABLE, _TRANSIENT_LOCAL, 10)

# /fmu/in/* and /fmu/out/* (PX4 uXRCE-DDS): best effort. Durability and depth are not in the
# table; transient local, keep last 1 is what PX4's uXRCE-DDS bridge needs (also used by
# guidance/px4_interface.py, search_base.py and camera_fov.py).
PX4 = _profile(_BEST_EFFORT, _TRANSIENT_LOCAL, 1)


# --- Placeholders --------------------------------------------------------------------------------

# Subscriptions with an unknown publisher: connect to any publisher.
PERCEPTION_DICE_MAP = _profile(_BEST_EFFORT, _VOLATILE, 10)         # TBD: not in Interfaces table
PERCEPTION_TARGET_RELATIVE = _profile(_BEST_EFFORT, _VOLATILE, 10)  # TBD: not in Interfaces table
PERCEPTION_COLOR_CHECK = _profile(_BEST_EFFORT, _VOLATILE, 10)      # TBD: not in Interfaces table
PERCEPTION_OBSTACLES = _profile(_BEST_EFFORT, _VOLATILE, 10)        # TBD: not in Interfaces table
PERCEPTION_HEARTBEAT = _profile(_BEST_EFFORT, _VOLATILE, 10)        # TBD: not in Interfaces table
HW_NFC_STATUS = _profile(_BEST_EFFORT, _VOLATILE, 10)               # TBD: not in Interfaces table
HW_NFC_RESULT = _profile(_BEST_EFFORT, _VOLATILE, 10)               # TBD: not in Interfaces table
HW_GRIPPER_STATUS = _profile(_BEST_EFFORT, _VOLATILE, 10)           # TBD: not in Interfaces table

# Our commands to hardware. The table says reliable; durability and depth are placeholders.
HW_NFC_COMMAND = _profile(_RELIABLE, _VOLATILE, 10)                 # TBD: not in Interfaces table
HW_GRIPPER_COMMAND = _profile(_RELIABLE, _VOLATILE, 10)             # TBD: not in Interfaces table

# /mission/target (pub). The table says reliable, transient local; the depth is a placeholder.
MISSION_TARGET = _profile(_RELIABLE, _TRANSIENT_LOCAL, 1)           # TBD: not in Interfaces table


# Topic -> profile, for the node and for test_qos.py.
BY_TOPIC = {
    '/mission/intent': INTENT,
    '/llm/heartbeat': LLM_HEARTBEAT,
    '/perception/dice_map': PERCEPTION_DICE_MAP,
    '/perception/target_relative': PERCEPTION_TARGET_RELATIVE,
    '/perception/color_check': PERCEPTION_COLOR_CHECK,
    '/perception/obstacle_detections': PERCEPTION_OBSTACLES,
    '/perception/heartbeat': PERCEPTION_HEARTBEAT,
    '/hw/nfc/status': HW_NFC_STATUS,
    '/hw/nfc/result': HW_NFC_RESULT,
    '/hw/gripper/status': HW_GRIPPER_STATUS,
    '/mission/state': MISSION_STATE,
    '/mission/nfc_result': NFC_RESULT,
    '/mission/target': MISSION_TARGET,
    '/hw/nfc/command': HW_NFC_COMMAND,
    '/hw/gripper/command': HW_GRIPPER_COMMAND,
}

# Topics whose profile (or part of it) is a placeholder, to revisit when the owner confirms.
TBD_TOPICS = frozenset({
    '/perception/dice_map', '/perception/target_relative', '/perception/color_check',
    '/perception/obstacle_detections', '/perception/heartbeat',
    '/hw/nfc/status', '/hw/nfc/result', '/hw/gripper/status',
    '/hw/nfc/command', '/hw/gripper/command', '/mission/target',
})
