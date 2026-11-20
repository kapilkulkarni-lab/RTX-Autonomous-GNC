"""MissionContext: the state shared by the ROS node and the SMACH worker thread.

The node's subscription callbacks only call `update_input()` (store the whole latest message and
its receive time; no decisions). States read a consistent `Inputs` snapshot and change mission
fields under the lock. Nothing here imports rclpy.
"""
import collections
import dataclasses
import threading
from dataclasses import dataclass, field
from typing import Any, Deque, Dict, List, Optional, Tuple

# Field names of Inputs, one per subscribed topic.
INPUT_NAMES = (
    'intent',              # /mission/intent              avc_msgs/Intent
    'llm_heartbeat',       # /llm/heartbeat               avc_msgs/LlmHeartbeat
    'dice_map',            # /perception/dice_map         avc_interfaces/DiceMap (placeholder)
    'target_relative',     # /perception/target_relative  type TBD (Perception)
    'color_check',         # /perception/color_check      avc_interfaces/ColorCheck (placeholder)
    'obstacles',           # /perception/obstacle_detections  type TBD
    'perception_heartbeat',  # /perception/heartbeat      type TBD
    'nfc_status',          # /hw/nfc/status
    'nfc_result',          # /hw/nfc/result
    'gripper_status',      # /hw/gripper/status
    'vehicle_status',      # /fmu/out/vehicle_status
    'battery_status',      # /fmu/out/battery_status
    'local_position',      # /fmu/out/vehicle_local_position
    'land_detected',       # /fmu/out/vehicle_land_detected
    'distance_sensor',     # /fmu/out/distance_sensor
)


@dataclass(frozen=True)
class Stamped:
    """A whole message plus the time it was received (seconds, monotonic clock)."""
    msg: Any
    rx_time: float


@dataclass(frozen=True)
class Inputs:
    """Snapshot of the latest message on every input; None until the first one arrives."""
    intent: Optional[Stamped] = None
    llm_heartbeat: Optional[Stamped] = None
    dice_map: Optional[Stamped] = None
    target_relative: Optional[Stamped] = None
    color_check: Optional[Stamped] = None
    obstacles: Optional[Stamped] = None
    perception_heartbeat: Optional[Stamped] = None
    nfc_status: Optional[Stamped] = None
    nfc_result: Optional[Stamped] = None
    gripper_status: Optional[Stamped] = None
    vehicle_status: Optional[Stamped] = None
    battery_status: Optional[Stamped] = None
    local_position: Optional[Stamped] = None
    land_detected: Optional[Stamped] = None
    distance_sensor: Optional[Stamped] = None


assert tuple(f.name for f in dataclasses.fields(Inputs)) == INPUT_NAMES


@dataclass
class PendingNfcResult:
    """An NfcResult published to comms and republished each heartbeat until echoed."""
    msg: Any                 # avc_msgs/NfcResult
    published_at: float      # first publish time; read_complete() ignores older heartbeats


@dataclass
class MissionContext:
    """Shared mission state. Hold `lock` for any read-modify-write of the mission fields."""

    # Inputs (written only by update_input).
    _inputs: Dict[str, Optional[Stamped]] = field(
        default_factory=lambda: {name: None for name in INPUT_NAMES})

    # Mission.
    active_target: str = ''                  # blue | red | yellow | black, '' if none
    regions: Any = None                      # intents.RegionFields as received; behavior: Milestone 9
    mission_start_time: Optional[float] = None
    start_position: Optional[Any] = None     # recorded in PREFLIGHT: PX4 NED x, y, z, yaw
    start_field: Optional[Any] = None        # the same point in the field frame: x, y, z

    # Field and search (Milestone 5).
    field_config: Any = None                 # field.FieldConfig
    coverage: Any = None                     # coverage.CoverageMap: ground already searched
    target_centroid: Optional[Any] = None    # field x, y, z of the target die once known
    given_up_dice: List[Any] = field(default_factory=list)  # field points excluded from the search
    die_failures: List[Any] = field(default_factory=list)   # [point, count]: lost / never confirmed

    # State bookkeeping (state names come from state_names.py).
    state: str = ''
    previous_state: str = ''
    reason: str = ''
    pending_reason: str = ''                 # set by a state with its outcome; used on transition
    abort_cause: str = ''                    # an AbortCause value, set when ABORTED is chosen
    abort_from: str = ''                     # the state ABORTED was entered from
    offboard_engaged: bool = False           # PX4 reached armed + offboard at least once
    pose_bad_since: Optional[float] = None   # position invalid or stale since (global check)
    paused_from: str = ''
    stored_intents: List[Stamped] = field(default_factory=list)  # new intents received while PAUSED
    after_liftoff: str = ''                  # where LIFTOFF goes next

    # Retries and on-die progress.
    descent_retries: int = 0
    motor_stage: int = 1                     # 1 = armed at idle, 2 = motors off
    nfc_tries: Dict[int, int] = field(default_factory=lambda: {1: 0, 2: 0})
    redescended: bool = False                # both motor stages failed once already

    # Commands.
    setpoint: Optional[Tuple[float, float, float, float]] = None  # PX4 NED x, y, z, yaw; None = no stream

    # Intents. Every message on /mission/intent is a new intent, so they queue (never dropped,
    # never ordered by intent_id) as well as being stored as the latest input.
    intent_queue: Deque[Stamped] = field(default_factory=collections.deque)
    last_intent_id: int = 0
    last_intent_result: str = ''

    # NFC reads.
    read_id: int = 0                         # last read_id handed out
    pending_nfc: Dict[int, PendingNfcResult] = field(default_factory=dict)

    lock: threading.RLock = field(default_factory=threading.RLock, repr=False, compare=False)

    def update_input(self, name: str, msg: Any, now: float) -> None:
        """Store the latest message on input `name` with its receive time. Callbacks call only this."""
        if name not in self._inputs:
            raise KeyError(f'unknown input {name!r}')
        stamped = Stamped(msg, now)
        with self.lock:
            self._inputs[name] = stamped
            if name == 'intent':
                self.intent_queue.append(stamped)

    def take_intents(self) -> List[Stamped]:
        """Remove and return every intent received since the last call, oldest first."""
        with self.lock:
            intents = list(self.intent_queue)
            self.intent_queue.clear()
            return intents

    def snapshot(self) -> Inputs:
        """A consistent copy of the latest inputs, safe to use outside the lock."""
        with self.lock:
            return Inputs(**self._inputs)

    def mission_time_s(self, now: float) -> float:
        """Seconds since the mission started; 0 before it starts. Keeps running while paused."""
        with self.lock:
            return 0.0 if self.mission_start_time is None else now - self.mission_start_time

    def next_read_id(self) -> int:
        """Hand out a new read_id (+1 per read attempt)."""
        with self.lock:
            self.read_id += 1
            return self.read_id
