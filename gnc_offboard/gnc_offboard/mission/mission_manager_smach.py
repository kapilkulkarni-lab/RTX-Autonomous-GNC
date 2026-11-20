"""mission_manager state machine (SMACH). No rclpy here: states act only through MissionIO and
MissionContext, and get time from an injected clock, so every state runs under pytest.

Built so far (Milestones 3-6):
    STANDBY -> PREFLIGHT -> TAKEOFF -> SEARCHING -> APPROACHING -> DESCENDING
    -> ON_DIE_TEST (TEMPORARY) -> LIFTOFF -> RETURNING -> LANDING -> RETURN_CONFIRMED,
    plus ABORTED, PAUSED, the global checks, full intent handling and retarget.

TEMPORARY until later milestones (search for "TEMPORARY"; outline: Temporary stand-ins):
    - ON_DIE_TEST stands in for NFC_READY / READING (Milestone 7): stay on the die, then
      LIFTOFF -> RETURNING (or back to DESCENDING for landing-accuracy runs).
    - RETURNING is the simple version: battery and time thresholds come in Milestone 8.
    - Every dice-map position fits the intent, and region fields are stored only (Milestone 9).
    - read_chip is always rejected (Milestone 7).
"""
import enum
import math
from typing import Callable, List, Optional

import smach
from avc_msgs.msg import MissionState

from gnc_offboard import search_planners as sp
from gnc_offboard.guidance import px4_interface as px4
from gnc_offboard.guidance.path_follower import PathFollower
from gnc_offboard.mission import intents as I
from gnc_offboard.mission import state_names as S
from gnc_offboard.mission.checks import (ColorResult, check_color, confirmed_color, invalid_for,
                                         is_centered, is_stale)
from gnc_offboard.camera_model import CoverageGrid
from gnc_offboard.mission.cameras import (down_footprint_extent, footprints, load_blind_height,
                                          load_cameras)
from gnc_offboard.mission.checks import touchdown_confirmed, update_below_since
from gnc_offboard.mission.dice import count_failure, find_die, obstacles_of
from gnc_offboard.mission.search import plan_search
from gnc_offboard.mission.config import MissionConfig
from gnc_offboard.mission.context import MissionContext, Stamped
from gnc_offboard.mission.io import MissionIO

# Global outcomes and SMACH's preempt outcome.
ABORTED_OUT = 'aborted'
RETURN_OUT = 'return'
PAUSE_OUT = 'pause'
RETARGET_SEEN = 'retarget_seen'
_COS_HALF_5_DEG = math.cos(math.radians(5.0) / 2.0)   # quaternion |dot| for a 5 deg turn
RETARGET_UNSEEN = 'retarget_unseen'
PREEMPTED = 'preempted'
RESUME_NEW_INTENT = 'resume_new_intent'
RESUME_NEW_INTENT_LIFTOFF = 'resume_new_intent_liftoff'   # paused on the die: LIFTOFF first
RETURN_GROUND = 'return_ground'                             # paused on the ground: LANDING
RETURN_LIFTOFF = 'return_liftoff'                           # paused on the die: LIFTOFF first
LIFTOFF_TO = {S.DESCENDING: 'to_descending', S.SEARCHING: 'to_searching',
              S.APPROACHING: 'to_approaching', S.RETURNING: 'to_returning'}

# Container outcomes.
MISSION_COMPLETE = 'mission_complete'
MISSION_ABORTED = 'mission_aborted'

GROUND_STATES = frozenset({S.STANDBY, S.PREFLIGHT})

# Where the global `return` checks apply (judged by paused_from while PAUSED). Battery: airborne
# and on-die states. Mission clock: all states before RETURNING. Neither applies while already
# landing or returning.
BATTERY_STATES = (S.AIRBORNE | S.ON_DIE) - {S.LANDING, S.RETURNING}
CLOCK_STATES = frozenset(S.PUBLISHABLE) - {S.STANDBY, S.RETURNING, S.LANDING, S.PAUSED} \
    - S.END_STATES
# Where a return_to_start intent returns: airborne, on the die, and PREFLIGHT (stop, disarm,
# RETURN_CONFIRMED). LANDING is already landing in place, and STANDBY has nothing to do.
RETURN_INTENT_STATES = ((S.AIRBORNE | S.ON_DIE) - {S.LANDING}) | {S.PREFLIGHT}
# Where a retrieve with a new color retargets ("TAKEOFF through RETURNING"; the on-die states,
# on the die via LIFTOFF first). Elsewhere a new color only updates the target.
RETARGET_STATES = frozenset({S.TAKEOFF, S.SEARCHING, S.APPROACHING, S.DESCENDING, S.LIFTOFF,
                             S.RETURNING}) | S.ON_DIE
FLYING_GLOBALS = (ABORTED_OUT, RETURN_OUT, PAUSE_OUT, RETARGET_SEEN, RETARGET_UNSEEN)


def resume_outcome(state: str) -> str:
    """PAUSED's outcome for going back to `state` (one per pausable state)."""
    return 'resume_' + state.lower()


class AbortCause(str, enum.Enum):
    PX4_CONTROL = 'px4_control'          # failsafe, offboard lost, pilot takeover, PX4 link lost
    INTENT = 'intent'                    # comms `abort`
    POSE = 'pose'                        # position estimate invalid
    PERCEPTION = 'perception'            # Perception heartbeat stale
    LIFTOFF_FAILED = 'liftoff_failed'    # re-arm / offboard refused on the die
    PREFLIGHT_FAILED = 'preflight_failed'


def _msg(stamped: Optional[Stamped]):
    return None if stamped is None else stamped.msg


class MissionStateBase(smach.State):
    """Common loop for every state: global checks and intents first, then the state's own
    checks. Intent handling for every row of the outline's Intent handling table lives here,
    so no state can forget a row."""

    NAME = ''
    OUTCOMES = ()     # the state's own outcomes
    GLOBALS = ()      # global outcomes that apply to this state

    def __init__(self, ctx: MissionContext, io: MissionIO, cfg: MissionConfig, clock):
        super().__init__(outcomes=list(self.OUTCOMES) + list(self.GLOBALS) + [PREEMPTED])
        self.ctx, self.io, self.cfg, self.clock = ctx, io, cfg, clock
        self.entered_at = 0.0
        self._disarmed_unlanded_since = None

    def mode(self) -> str:
        """The state the global checks judge by: paused_from while PAUSED, else this state."""
        return self.NAME

    # ---- loop -----------------------------------------------------------------------------------
    def execute(self, userdata=None):
        self.entered_at = self.clock.now()
        self._disarmed_unlanded_since = None
        self.on_entry(self.ctx.snapshot(), self.entered_at)
        period = 1.0 / self.cfg.loop_hz
        while True:
            if self.preempt_requested():
                self.service_preempt()
                return PREEMPTED
            outcome = self.tick(self.clock.now())
            if outcome is not None:
                if outcome not in self.get_registered_outcomes():
                    raise RuntimeError(f'{self.NAME} returned unregistered outcome {outcome!r}')
                return outcome
            self.clock.sleep(period)

    def tick(self, now) -> Optional[str]:
        """One loop: global checks and intents (in the outline's priority order), then step."""
        snap = self.ctx.snapshot()
        self._mark_coverage(snap)
        intents = self.ctx.take_intents()
        outcome = self.global_checks(snap, intents, now)
        while outcome is None and intents:
            outcome = self.handle_intent(intents.pop(0), now)
        if outcome is not None:
            self._requeue(intents)            # unhandled intents go to the next state
            return outcome
        return self.step(snap, now)

    # ---- hooks for states -----------------------------------------------------------------------
    def on_entry(self, snap, now) -> None:
        pass

    def step(self, snap, now) -> Optional[str]:
        return None

    def control_lost(self, status, snap, now) -> Optional[str]:
        """Airborne rule: must stay armed and in offboard. LANDING (and PAUSED from LANDING)
        use landing_control_lost instead."""
        if not px4.is_armed(status):
            return 'disarmed in flight'
        if not px4.is_offboard(status):
            return 'offboard lost: pilot or PX4 took control'
        return None

    def landing_control_lost(self, status, snap, now) -> Optional[str]:
        """While PX4 lands: a disarm with the landed flag (or the flag arriving within
        landed_disarm_grace_s) is the normal end of a landing; AUTO_LAND or OFFBOARD are ours."""
        landed = px4.is_landed(_msg(snap.land_detected))
        if px4.is_disarmed(status):
            if landed:
                self._disarmed_unlanded_since = None
                return None
            if self._disarmed_unlanded_since is None:
                self._disarmed_unlanded_since = now
            if now - self._disarmed_unlanded_since > self.cfg.landed_disarm_grace_s:
                return 'disarmed before landing'
            return None
        self._disarmed_unlanded_since = None
        if not (px4.is_auto_land(status) or px4.is_offboard(status)):
            return 'pilot or PX4 took control during landing'
        return None

    # ---- global checks, in priority order -------------------------------------------------------
    def global_checks(self, snap, intents: List[Stamped], now) -> Optional[str]:
        mode = self.mode()
        if ABORTED_OUT in self.GLOBALS:
            why = self._px4_control_lost(snap, now)
            if why:
                return self.abort(AbortCause.PX4_CONTROL, why)
            # An abort jumps the queue (it acts on the action alone).
            for intent in intents:
                if I.action_of(intent.msg) == I.ABORT:
                    intents.remove(intent)
                    self.ack(intent)
                    return self.abort(AbortCause.INTENT, 'abort intent')
            if mode in S.AIRBORNE and self._pose_lost(snap, now):
                return self.abort(AbortCause.POSE, 'position estimate invalid')
            if (mode in S.AIRBORNE and self.cfg.require_perception
                    and is_stale(snap.perception_heartbeat, now, self.cfg.perception_stale_s)):
                return self.abort(AbortCause.PERCEPTION, 'Perception heartbeat lost')
        if RETURN_OUT in self.GLOBALS:
            pct = px4.battery_percent(_msg(snap.battery_status))
            if mode in BATTERY_STATES and pct is not None and pct < self.cfg.battery_return_pct:
                return self.return_out(f'battery {pct:.0f}%: returning')
            if mode in CLOCK_STATES and \
                    self.ctx.mission_time_s(now) > self.cfg.force_return_at_s:
                return self.return_out('mission time up: returning')
        return None

    def _px4_control_lost(self, snap, now) -> Optional[str]:
        status = _msg(snap.vehicle_status)
        # A failsafe matters once we fly or the vehicle is armed (a disarmed vehicle on the
        # ground has nothing to hand back).
        if (self.ctx.offboard_engaged or px4.is_armed(status)) and px4.in_failsafe(status):
            return 'PX4 failsafe'
        if not self.ctx.offboard_engaged or self.mode() not in S.AIRBORNE:
            return None
        if is_stale(snap.vehicle_status, now, self.cfg.px4_status_stale_s):
            return 'PX4 link lost'
        return self.control_lost(status, snap, now)

    def _pose_lost(self, snap, now) -> bool:
        lp = snap.local_position
        if lp is None or is_stale(lp, now, self.cfg.pose_invalid_s):
            return True                  # already older than the limit
        if px4.position_valid(lp.msg):
            self.ctx.pose_bad_since = None
            return False
        if self.ctx.pose_bad_since is None:
            self.ctx.pose_bad_since = now
        return invalid_for(self.ctx.pose_bad_since, now, self.cfg.pose_invalid_s)

    # ---- intents (outline: Intent handling) -----------------------------------------------------
    def handle_intent(self, intent: Stamped, now) -> Optional[str]:
        """One intent, in arrival order. Returns an outcome if it changes state."""
        msg = intent.msg
        action = I.action_of(msg)
        if action not in I.ACTIONS:
            self.io.log('warn', f'intent {msg.intent_id}: unknown action {action!r}, '
                                'applied as update_only')
            action = I.UPDATE_ONLY
        if self.NAME in S.END_STATES:
            if action == I.ABORT:
                self.ack(intent)                         # applied, no change
                return None
            return self.reject(intent, I.REJECT_MISSION_OVER)
        if action == I.ABORT:            # only reached in a state without the aborted outcome
            self.ack(intent)
            return None
        if action == I.READ_CHIP:
            # TEMPORARY (Milestone 7): no on-die states yet, so never on the target die.
            return self.reject(intent, I.REJECT_READ_CHIP)
        if action == I.RESUME:
            return self.on_resume(intent, now)
        if action == I.PAUSE:
            return self.on_pause(intent, now)
        if action == I.RETURN_TO_START:
            return self.on_return_to_start(intent, now)
        if action == I.RETRIEVE:
            return self.on_retrieve(intent, now)
        return self.on_update_only(intent, now)

    def on_resume(self, intent, now):
        return self.reject(intent, I.REJECT_RESUME)

    def on_pause(self, intent, now):
        self.ack(intent)
        with self.ctx.lock:
            self.ctx.paused_from = self.NAME
            self.ctx.stored_intents = []
        return self.finish(PAUSE_OUT, 'paused')

    def on_return_to_start(self, intent, now):
        self.ack(intent)
        if RETURN_OUT in self.GLOBALS and self.mode() in RETURN_INTENT_STATES:
            return self.return_out('return to start')
        return None                                      # applied, no change

    def return_out(self, reason: str) -> str:
        """The `return` outcome for where we are: RETURNING when flying, LIFTOFF first on the
        die (then RETURNING), LANDING on the ground (PREFLIGHT: stop, disarm)."""
        mode = self.mode()
        if mode in S.ON_DIE:
            self.ctx.after_liftoff = S.RETURNING
        if self.NAME == S.PAUSED:
            if mode in GROUND_STATES:
                return self.finish(RETURN_GROUND, reason)
            if mode in S.ON_DIE:
                return self.finish(RETURN_LIFTOFF, reason)
        return self.finish(RETURN_OUT, reason)

    def on_retrieve(self, intent, now):
        msg = intent.msg
        color = I.color_of(msg)
        with self.ctx.lock:
            self.ctx.regions = I.merge_regions(self.ctx.regions, msg)
            first = not self.ctx.active_target
            new_color = bool(color) and not first and color != self.ctx.active_target
        self.ack(intent)
        if color and first and self.NAME == S.STANDBY:
            return self._start(color, now)
        if not new_color:
            return None                              # same color or none: regions only
        self.set_target(color)
        if self.NAME not in RETARGET_STATES:
            return None                              # e.g. PREFLIGHT, LANDING: target only
        snap = self.ctx.snapshot()
        seen = self.find_target(snap, near=self.field_xy(snap)) is not None
        if self.mode() in S.ON_DIE:                  # LIFTOFF first, then on
            self.ctx.after_liftoff = S.APPROACHING if seen else S.SEARCHING
        if seen:
            return self.finish(RETARGET_SEEN, f'retarget {color}: in the dice map')
        return self.finish(RETARGET_UNSEEN, f'retarget {color}: searching')

    # ---- targets, dice map and field frame ------------------------------------------------------
    def set_target(self, color: str) -> None:
        """New target color: forget the old die and the dice given up on (they were the old
        color). The coverage memory stays: searched ground is still searched."""
        with self.ctx.lock:
            self.ctx.active_target = color
            self.ctx.target_centroid = None
            self.forget_failures()

    def forget_failures(self) -> None:
        """Clear the given-up dice and the failure counts (new target, coverage reset)."""
        self.ctx.given_up_dice = []
        self.ctx.die_failures = []

    def find_target(self, snap, near=None):
        """The active target in Perception's dice map, skipping dice given up on.
        TEMPORARY (Milestone 9): every position fits the intent (no region checks yet)."""
        return find_die(_msg(snap.dice_map), self.ctx.active_target, near=near,
                        exclude=self.ctx.given_up_dice,
                        exclude_radius_m=self.cfg.die_match_radius_m)

    def give_up(self, point) -> None:
        """Exclude this die from now on (Perception confirmed its wrong color)."""
        if point is not None:
            with self.ctx.lock:
                self.ctx.given_up_dice.append(tuple(point))
                self.ctx.target_centroid = None

    def failed_attempt(self, point) -> int:
        """A lost / never-confirmed approach: count it; exclude the die only after
        max_die_attempts. Returns the die's failure count."""
        if point is None:
            return 0
        with self.ctx.lock:
            count = count_failure(self.ctx.die_failures, point, self.cfg.die_match_radius_m)
            self.ctx.target_centroid = None
        if count >= self.cfg.max_die_attempts:
            self.give_up(point)
        return count

    def field_xyz(self, lp):
        """PX4 NED position -> field (x, y, z); z is height above the PX4 origin."""
        return self.ctx.field_config.frame.to_field(lp.x, lp.y, lp.z)

    def field_xy(self, snap):
        lp = _msg(snap.local_position)
        if not px4.position_valid(lp) or self.ctx.field_config is None:
            return None
        return self.field_xyz(lp)[:2]

    def altitude(self, lp) -> float:
        """Height above the start position (PX4 NED z is down)."""
        start_z = self.ctx.start_position[2] if self.ctx.start_position else 0.0
        return start_z - lp.z

    def ned_at_height(self, x, y, height):
        """Field (x, y) at `height` above the start -> PX4 NED (n, e, d)."""
        n, e, _ = self.ctx.field_config.frame.to_ned(x, y)
        start_z = self.ctx.start_position[2] if self.ctx.start_position else 0.0
        return (n, e, start_z - height)

    def range_m(self, snap, now) -> Optional[float]:
        """Rangefinder distance to what is below (m), or None if there is no valid reading.
        rangefinder_source: distance_sensor (px4_msgs/DistanceSensor) or dist_bottom
        (vehicle_local_position.dist_bottom, only when dist_bottom_valid)."""
        if self.cfg.rangefinder_source == 'dist_bottom':
            lp = snap.local_position
            if lp is None or is_stale(lp, now, self.cfg.pose_invalid_s):
                return None
            ok = bool(getattr(lp.msg, 'dist_bottom_valid', False))
            return float(lp.msg.dist_bottom) if ok else None
        ds = snap.distance_sensor
        if ds is None or is_stale(ds, now, self.cfg.pose_invalid_s):
            return None
        d = float(ds.msg.current_distance)
        if not math.isfinite(d) or int(getattr(ds.msg, 'signal_quality', -1)) == 0:
            return None
        return d

    def _mark_coverage(self, snap) -> None:
        """Mark both cameras' projected footprints as searched, while airborne. Projects again
        only once the pose has moved half a coverage cell or turned (~5 deg), so hovering
        costs nothing."""
        odom = snap.odometry
        if (self.ctx.coverage is None or self.ctx.cameras is None or odom is None
                or self.mode() not in S.AIRBORNE):
            return
        position = [float(v) for v in odom.msg.position]
        q = [float(v) for v in odom.msg.q]
        last = self.ctx.coverage_pose
        if last is not None:
            moved = math.dist(position, last[0]) >= self.cfg.coverage_cell_m / 2.0
            turned = abs(sum(a * b for a, b in zip(q, last[1]))) < _COS_HALF_5_DEG
            if not (moved or turned):
                return
        self.ctx.coverage_pose = (position, q)
        polygons = footprints(self.ctx.cameras, position, q, self.ctx.field_config.frame,
                              self.ctx.field_config.bounds)
        for polygon in polygons.values():
            self.ctx.coverage.mark(polygon)

    def on_update_only(self, intent, now):
        with self.ctx.lock:
            self.ctx.regions = I.merge_regions(self.ctx.regions, intent.msg, hint=True)
        self.ack(intent)
        return None

    def _start(self, color, now) -> str:
        with self.ctx.lock:
            self.ctx.active_target = color
            self.ctx.mission_start_time = now
        return self.finish('start', f'retrieve {color}: starting')

    # ---- helpers --------------------------------------------------------------------------------
    def finish(self, outcome: str, reason: str) -> str:
        """Return `outcome`; `reason` goes on /mission/state with the transition."""
        self.ctx.pending_reason = reason
        return outcome

    def abort(self, cause: AbortCause, reason: str) -> str:
        with self.ctx.lock:
            self.ctx.abort_cause = cause.value
            self.ctx.abort_from = self.mode()
        return self.finish(ABORTED_OUT, reason)

    def _record_intent(self, intent: Stamped, result: str) -> None:
        intent_id = int(intent.msg.intent_id)
        with self.ctx.lock:
            if intent_id < self.ctx.last_intent_id:
                self.io.log('warn', f'intent_id went down ({self.ctx.last_intent_id} -> '
                                    f'{intent_id}): comms restarted? Handling it as new.')
            self.ctx.last_intent_id = intent_id
            self.ctx.last_intent_result = result

    def ack(self, intent: Stamped) -> None:
        """Applied: set last_intent_id/result and publish at once."""
        self._record_intent(intent, MissionState.RESULT_APPLIED)
        self.io.request_state_publish()

    def reject(self, intent: Stamped, reason: str) -> None:
        """One of the three rejections: result rejected, reason says why, publish at once."""
        self._record_intent(intent, MissionState.RESULT_REJECTED)
        with self.ctx.lock:
            self.ctx.reason = reason
        self.io.request_state_publish()
        return None

    def _requeue(self, intents: List[Stamped]) -> None:
        if intents:
            with self.ctx.lock:
                self.ctx.intent_queue.extendleft(reversed(intents))

    def _every(self, attr: str, now: float) -> bool:
        """True at most once per cmd_retry_s for the command tracked in `attr`."""
        last = getattr(self, attr, None)
        if last is None or now - last >= self.cfg.cmd_retry_s:
            setattr(self, attr, now)
            return True
        return False

    def _hold_here(self, snap) -> None:
        lp = _msg(snap.local_position)
        if px4.position_valid(lp):
            self.io.set_setpoint(lp.x, lp.y, lp.z, lp.heading)


# =================================================================================================

class Standby(MissionStateBase):
    """Disarmed, no setpoints. Waits for the first `retrieve` with a target color."""
    NAME = S.STANDBY
    OUTCOMES = ('start',)
    GLOBALS = (ABORTED_OUT, PAUSE_OUT)

    def on_entry(self, snap, now):
        self.io.clear_setpoint()


class Preflight(MissionStateBase):
    """Hold the current pose as the setpoint, wait for a stable estimate, record the start,
    then arm (or wait for the pilot to arm) and enter offboard."""
    NAME = S.PREFLIGHT
    OUTCOMES = ('ready', 'failed')
    GLOBALS = (ABORTED_OUT, RETURN_OUT, PAUSE_OUT)

    def on_entry(self, snap, now):
        self.valid_since = None
        self.stream_since = None
        self.first_cmd_at = None     # first arm/offboard request (deadline start)
        self.last_cmd_at = None
        self.ctx.start_position = None

    def step(self, snap, now):
        status = _msg(snap.vehicle_status)
        if px4.armed_offboard(status) and self.ctx.start_position is not None:
            self.ctx.offboard_engaged = True
            return self.finish('ready', 'armed in offboard')

        if self.first_cmd_at is not None and now - self.first_cmd_at > self.cfg.arm_timeout_s:
            with self.ctx.lock:
                self.ctx.abort_cause = AbortCause.PREFLIGHT_FAILED.value
                self.ctx.abort_from = self.NAME
            what = 'offboard' if self.cfg.require_pilot_arm else 'arming or offboard'
            return self.finish('failed', f'{what} refused for {self.cfg.arm_timeout_s:.0f} s')

        lp = snap.local_position
        valid = (lp is not None and not is_stale(lp, now, self.cfg.pose_invalid_s)
                 and px4.position_valid(lp.msg))
        if not valid:
            self.valid_since = None
            return None
        if self.valid_since is None:
            self.valid_since = now
        if self.ctx.start_position is None:
            # Hold the current pose until the start is recorded.
            p = lp.msg
            self.io.set_setpoint(p.x, p.y, p.z, p.heading)
            if self.stream_since is None:
                self.stream_since = now
            if (now - self.valid_since < self.cfg.ekf_stable_s
                    or now - self.stream_since < self.cfg.offboard_warmup_s):
                return None
            # Start position in PX4 local NED and in the field frame.
            self.ctx.start_position = (p.x, p.y, p.z, p.heading)
            if self.ctx.field_config is not None:
                self.ctx.start_field = self.field_xyz(p)

        if self.cfg.require_pilot_arm:
            if not px4.is_armed(status):
                return None              # the pilot arms; we never send an arm command
            if self._every('last_cmd_at', now):
                if self.first_cmd_at is None:
                    self.first_cmd_at = now
                self.io.set_offboard()
        elif self._every('last_cmd_at', now):
            if self.first_cmd_at is None:
                self.first_cmd_at = now
            self.io.set_offboard()
            if not px4.is_armed(status):
                self.io.arm()
        return None


class Takeoff(MissionStateBase):
    """Climb straight up from the start position to search altitude."""
    NAME = S.TAKEOFF
    OUTCOMES = ('at_altitude',)
    GLOBALS = FLYING_GLOBALS

    def on_entry(self, snap, now):
        x, y, z, yaw = self.ctx.start_position
        self.target_z = z - self.cfg.search_altitude_m          # NED: up is negative z
        self.io.set_setpoint(x, y, self.target_z, yaw)

    def step(self, snap, now):
        lp = _msg(snap.local_position)
        if px4.position_valid(lp) and abs(lp.z - self.target_z) <= self.cfg.altitude_tol_m:
            return self.finish('at_altitude', f'at {self.cfg.search_altitude_m:.1f} m')
        return None


class Searching(MissionStateBase):
    """Target check first: if Perception's dice map already holds the active color, leave at
    once. Otherwise fly the search pattern (search_pattern) at search altitude, skipping ground
    the coverage memory already holds, and leave when the dice map reports the active color.
    Field covered without the target: clear the coverage memory and search again."""
    NAME = S.SEARCHING
    OUTCOMES = ('target_known', 'target_found')
    GLOBALS = FLYING_GLOBALS

    def on_entry(self, snap, now):
        self.follower = None
        self.known = self.find_target(snap, near=self.field_xy(snap)) is not None
        if not self.known:
            self.plan(snap)

    def plan(self, snap, restarted=False) -> None:
        lp = _msg(snap.local_position)
        if not px4.position_valid(lp):
            self.follower = None
            return
        cfg, fc = self.cfg, self.ctx.field_config
        x, y, _ = self.field_xyz(lp)
        alt = cfg.search_altitude_m
        yaw = self.ctx.start_position[3] if self.ctx.start_position else lp.heading
        # The down camera's footprint at search altitude, at the constant search heading.
        wx, wy = down_footprint_extent(self.ctx.cameras, alt, yaw)
        across = wy if cfg.search_rows_along == 'x' else wx
        spacing = max(across * (1.0 - cfg.search_overlap), cfg.coverage_cell_m)
        path = plan_search(cfg.search_pattern, fc.area(cfg.wall_margin_m), (x, y),
                           self.ctx.coverage, max(min(wx, wy), cfg.coverage_cell_m), spacing,
                           cfg.wall_margin_m, obstacles_of(_msg(snap.obstacles)),
                           cfg.obstacle_clearance_m, rows_along=cfg.search_rows_along)
        if not path:
            if restarted:
                self.follower = None             # nothing to fly even after a reset: hover
                return
            with self.ctx.lock:
                self.ctx.coverage.clear()
                self.forget_failures()
                self.ctx.reason = 'target not found, searching again'
            self.io.request_state_publish()
            return self.plan(snap, restarted=True)
        waypoints = [self.ned_at_height(px, py, alt) for px, py in path]
        yaw = self.ctx.start_position[3] if self.ctx.start_position else lp.heading
        self.follower = PathFollower(waypoints, (lp.x, lp.y, lp.z), yaw, cfg.cruise_speed_mps,
                                     cfg.max_accel_mps2, cfg.acceptance_radius_m)

    def step(self, snap, now):
        color = self.ctx.active_target
        if self.known:
            return self.finish('target_known', f'{color} already in the dice map')
        if self.find_target(snap, near=self.field_xy(snap)) is not None:
            return self.finish('target_found', f'{color} found')
        if self.follower is None:
            self.plan(snap)
            if self.follower is None:
                return None
        lp = _msg(snap.local_position)
        position = (lp.x, lp.y, lp.z) if px4.position_valid(lp) else None
        (n, e, d), yaw = self.follower.update(now, position)
        self.io.set_setpoint(n, e, d, yaw)
        if self.follower.done:
            self.plan(snap)
        return None


class Approaching(MissionStateBase):
    """Fly an obstacle-aware path at search altitude to the target die's centroid (from the
    dice map), go straight down to hover_height_m, center on Perception's target_relative and
    run the color check (PENDING never transitions)."""
    NAME = S.APPROACHING
    OUTCOMES = ('over_target', 'wrong_color_known', 'wrong_color_unknown', 'lost')
    GLOBALS = FLYING_GLOBALS

    def on_entry(self, snap, now):
        self.phase = 'transit'
        self.follower = None
        self.arrived_at = None
        self.last_seen = None
        self.ctx.descent_retries = 0             # a fresh approach gets fresh descent retries
        here = self.field_xy(snap)
        self.goal = self.find_target(snap, near=here)
        if self.goal is None or here is None:
            self.phase = 'no_goal'
            return
        with self.ctx.lock:
            self.ctx.target_centroid = self.goal
        cfg, fc = self.cfg, self.ctx.field_config
        gx, gy = self.goal[0], self.goal[1]
        route, failed = sp.avoid_obstacles([(gx, gy)], here, obstacles_of(_msg(snap.obstacles)),
                                           cfg.obstacle_clearance_m, fc.area(0.0))
        if failed:
            self.io.log('warn', 'no obstacle-free route to the target; flying straight')
        waypoints = [self.ned_at_height(px, py, cfg.search_altitude_m) for px, py in route]
        self.hover_ned = self.ned_at_height(gx, gy, cfg.hover_height_m)
        waypoints.append(self.hover_ned)                  # then straight down over the die
        lp = _msg(snap.local_position)
        yaw = self.ctx.start_position[3] if self.ctx.start_position else lp.heading
        self.yaw = yaw
        self.follower = PathFollower(waypoints, (lp.x, lp.y, lp.z), yaw, cfg.cruise_speed_mps,
                                     cfg.max_accel_mps2, cfg.acceptance_radius_m)

    def step(self, snap, now):
        if self.phase == 'no_goal':
            return self.finish('lost', 'target no longer in the dice map')
        lp = _msg(snap.local_position)
        if not px4.position_valid(lp):
            return None
        if self.phase == 'transit':
            (n, e, d), yaw = self.follower.update(now, (lp.x, lp.y, lp.z))
            self.io.set_setpoint(n, e, d, yaw)
            if not self.follower.done:
                return None
            self.phase, self.arrived_at = 'over', now
        return self._over_die(snap, lp, now)

    def _over_die(self, snap, lp, now):
        cfg = self.cfg
        tr = snap.target_relative
        fresh = (tr is not None and bool(tr.msg.valid)
                 and not is_stale(tr, now, cfg.target_lost_s))
        centered = False
        if fresh:
            self.last_seen = now
            dx, dy = float(tr.msg.offset.x), float(tr.msg.offset.y)
            x, y, _ = self.field_xyz(lp)
            die = (x + dx, y + dy, self.goal[2])
            with self.ctx.lock:
                self.ctx.target_centroid = die
            n, e, d = self.ned_at_height(die[0], die[1], cfg.hover_height_m)
            self.io.set_setpoint(n, e, d, self.yaw)          # center over the die
            centered = is_centered(dx, dy, cfg.approach_center_tol_m)
        if now - (self.last_seen or self.arrived_at) > cfg.target_lost_s:
            n = self.failed_attempt(self.goal)
            return self.finish('lost', f'target lost ({n} of {cfg.max_die_attempts})')

        cc = snap.color_check          # only colors confirmed since arriving over this die
        color = confirmed_color(cc) if cc is not None and cc.rx_time >= self.arrived_at else None
        result = check_color(color, self.ctx.active_target)
        if result is ColorResult.WRONG:
            self.give_up(self.goal)
            other = self.find_target(snap, near=self.goal[:2])
            if other is not None:
                return self.finish('wrong_color_known', f'wrong color ({color}): going to target')
            return self.finish('wrong_color_unknown', f'wrong color ({color}): searching')
        if result is ColorResult.PENDING:
            if now - self.arrived_at > cfg.color_confirm_timeout_s:
                n = self.failed_attempt(self.goal)
                return self.finish('lost', f'no confirmed color ({n} of {cfg.max_die_attempts})')
            return None
        at_height = abs(self.altitude(lp) - cfg.hover_height_m) <= cfg.altitude_tol_m
        if fresh and centered and at_height:
            return self.finish('over_target', f'over the {self.ctx.active_target} die')
        return None


class Descending(MissionStateBase):
    """Staged descent onto the die. The touchdown point starts at the die's centroid and is
    re-centered on Perception's target_relative at every stage; a stage is left only once the
    sideways error is within that stage's tolerance (stage_lateral_tol_m; the last stage also
    within reader_lateral_tol_m). The color is re-checked while the die is in view. Below the
    last stage the camera is (nearly) blind: finish on the last good centering plus EKF, and
    confirm touchdown from the rangefinder, not only PX4's land detector.

    retry: stage tolerance not reached in stage_settle_s, or the die lost for target_lost_s:
    climb to hover_height_m and start again (up to descent_max_retries). failed: retries used
    (one failure for this die toward max_die_attempts), or a confirmed wrong color (the die is
    excluded), -> APPROACHING."""
    NAME = S.DESCENDING
    OUTCOMES = ('touchdown', 'retry', 'failed')
    GLOBALS = FLYING_GLOBALS

    def on_entry(self, snap, now):
        cfg = self.cfg
        self.die = self.ctx.target_centroid
        self.phase = 'align' if self.die is not None else 'no_target'
        self.touch = None if self.die is None else (self.die[0], self.die[1])
        self.stages = list(cfg.descent_stages_m)
        self.tols = list(cfg.stage_lateral_tol_m)
        self.tols[-1] = min(self.tols[-1], cfg.reader_lateral_tol_m)
        self.stage = 0
        self.hold_since = None
        self.last_seen = now
        self.below_since = None
        self.last_t = now
        self.retry_reason = ''
        lp = _msg(snap.local_position)
        self.height = self.altitude(lp) if px4.position_valid(lp) else cfg.hover_height_m
        self.yaw = self.ctx.start_position[3] if self.ctx.start_position else 0.0

    def control_lost(self, status, snap, now):
        return self.landing_control_lost(status, snap, now)

    def _setpoint(self, height):
        n, e, d = self.ned_at_height(self.touch[0], self.touch[1], height)
        self.io.set_setpoint(n, e, d, self.yaw)

    def step(self, snap, now):
        cfg = self.cfg
        if self.phase == 'no_target':
            return self.finish('failed', 'target unknown')
        lp = _msg(snap.local_position)
        if not px4.position_valid(lp):
            return None
        dt, self.last_t = max(0.0, now - self.last_t), now
        alt = self.altitude(lp)
        blind = self.ctx.blind_height_m or 0.0

        # Center on the die while the camera can still see it.
        err = None
        tr = snap.target_relative
        if (self.phase != 'blind' and alt > blind and tr is not None and bool(tr.msg.valid)
                and not is_stale(tr, now, cfg.target_lost_s)):
            dx, dy = float(tr.msg.offset.x), float(tr.msg.offset.y)
            x, y, _ = self.field_xyz(lp)
            self.touch = (x + dx, y + dy)
            with self.ctx.lock:
                self.ctx.target_centroid = (self.touch[0], self.touch[1], self.die[2])
            err = math.hypot(dx, dy)
            self.last_seen = now

        # Color re-check while the die is in view (only colors confirmed during this descent).
        cc = snap.color_check
        if cc is not None and cc.rx_time >= self.entered_at:
            if check_color(confirmed_color(cc), self.ctx.active_target) is ColorResult.WRONG:
                self.give_up(self.die)
                return self.finish('failed', f'wrong color ({confirmed_color(cc)}) on descent')

        if self.phase == 'climb':                       # retry: back up to hover height
            self._setpoint(cfg.hover_height_m)
            if abs(alt - cfg.hover_height_m) <= cfg.altitude_tol_m:
                return self.finish('retry', self.retry_reason)
            return None

        if self.phase == 'align':                       # to hover height over the die
            self.height = cfg.hover_height_m
            self._setpoint(self.height)
            if abs(alt - cfg.hover_height_m) <= cfg.altitude_tol_m:
                self.phase, self.last_seen = 'descend', now
            return None

        if self.phase in ('descend', 'hold') and now - self.last_seen > cfg.target_lost_s:
            return self._retry('die lost', now)

        if self.phase == 'descend':                     # ramp down to this stage's height
            goal = self.stages[self.stage]
            self.height = max(goal, self.height - cfg.descent_rate_mps * dt)
            self._setpoint(self.height)
            if abs(alt - goal) <= cfg.altitude_tol_m and self.height == goal:
                self.phase, self.hold_since = 'hold', now
            return None

        if self.phase == 'hold':                        # within tolerance before going on
            self._setpoint(self.stages[self.stage])
            if err is not None and err <= self.tols[self.stage]:
                self.stage += 1
                self.phase = 'descend' if self.stage < len(self.stages) else 'blind'
                return None
            if now - self.hold_since > cfg.stage_settle_s:
                return self._retry(f'off center at {self.stages[self.stage]:.2f} m', now)
            return None

        # blind: last good centering + EKF; touchdown from the rangefinder
        self.height = max(-0.1, self.height - cfg.descent_rate_mps * dt)
        self._setpoint(self.height)
        self.below_since = update_below_since(self.below_since, self.range_m(snap, now),
                                              cfg.touchdown_range_m, now)
        if touchdown_confirmed(self.below_since, now, cfg.touchdown_hold_s):
            with self.ctx.lock:
                self.ctx.touchdowns += 1
            return self.finish('touchdown', 'touchdown on the die')
        return None

    def _retry(self, reason, now):
        if self.ctx.descent_retries >= self.cfg.descent_max_retries:
            n = self.failed_attempt(self.die)
            return self.finish('failed', f'descent failed: {reason} '
                                         f'({n} of {self.cfg.max_die_attempts})')
        self.ctx.descent_retries += 1
        self.retry_reason = f'{reason}: retry {self.ctx.descent_retries} of ' \
                            f'{self.cfg.descent_max_retries}'
        self.phase = 'climb'
        return None


class OnDieTest(MissionStateBase):
    """TEMPORARY (Milestone 6 only; replaced by NFC_READY / READING in Milestone 7): stay on the
    die for on_die_test_s with no arming change (PX4 may auto-disarm after COM_DISARM_LAND),
    then LIFTOFF -> RETURNING. For landing-accuracy runs, LIFTOFF -> DESCENDING again until
    descent_test_count touchdowns."""
    NAME = S.ON_DIE_TEST
    OUTCOMES = ('done',)
    GLOBALS = FLYING_GLOBALS

    def step(self, snap, now):
        if now - self.entered_at < self.cfg.on_die_test_s:
            return None
        if self.ctx.touchdowns < self.cfg.descent_test_count:
            self.ctx.after_liftoff = S.DESCENDING
            self.ctx.descent_retries = 0
            return self.finish('done', f'touchdown {self.ctx.touchdowns} of '
                                       f'{self.cfg.descent_test_count}: descending again')
        self.ctx.after_liftoff = S.RETURNING
        return self.finish('done', 'on-die test done: returning')


class Liftoff(MissionStateBase):
    """Off the die: re-arm and re-enter offboard if disarmed (with require_pilot_arm, wait for
    the pilot; GNC never sends arm), climb straight up to hover_height_m, then go where the
    context says (after_liftoff)."""
    NAME = S.LIFTOFF
    OUTCOMES = tuple(LIFTOFF_TO.values())
    GLOBALS = FLYING_GLOBALS

    def on_entry(self, snap, now):
        lp = _msg(snap.local_position)
        self.engaged = px4.armed_offboard(_msg(snap.vehicle_status))
        self.first_cmd_at = self.last_cmd_at = None
        self.stream_since = now
        self.xyz = (lp.x, lp.y, lp.z) if px4.position_valid(lp) else None
        self.yaw = self.ctx.start_position[3] if self.ctx.start_position else 0.0
        if self.xyz is not None:
            self.io.set_setpoint(*self.xyz, self.yaw)    # hold here; starts the stream

    def control_lost(self, status, snap, now):
        if not self.engaged:
            return None                  # still on the die, re-arming
        return super().control_lost(status, snap, now)

    def step(self, snap, now):
        cfg = self.cfg
        status = _msg(snap.vehicle_status)
        lp = _msg(snap.local_position)
        if self.xyz is None:
            if not px4.position_valid(lp):
                return None
            self.xyz = (lp.x, lp.y, lp.z)
            self.io.set_setpoint(*self.xyz, self.yaw)
        if not self.engaged:
            if px4.armed_offboard(status):
                self.engaged = True
                self.ctx.offboard_engaged = True
            else:
                return self._rearm(status, now)
        start_z = self.ctx.start_position[2] if self.ctx.start_position else 0.0
        self.io.set_setpoint(self.xyz[0], self.xyz[1], start_z - cfg.hover_height_m, self.yaw)
        if px4.position_valid(lp) and \
                abs(self.altitude(lp) - cfg.hover_height_m) <= cfg.altitude_tol_m:
            after = self.ctx.after_liftoff or S.RETURNING
            return self.finish(LIFTOFF_TO[after], f'off the die: to {after}')
        return None

    def return_out(self, reason):
        if not self.engaged:             # still on the die: lift off first, then RETURNING
            self.ctx.after_liftoff = S.RETURNING
            return None
        return super().return_out(reason)

    def on_retrieve(self, intent, now):
        out = super().on_retrieve(intent, now)
        if out in (RETARGET_SEEN, RETARGET_UNSEEN) and not self.engaged:
            self.ctx.after_liftoff = S.APPROACHING if out == RETARGET_SEEN else S.SEARCHING
            self.ctx.pending_reason = ''
            return None                  # keep lifting off; then go to the new target
        return out

    def _rearm(self, status, now):
        cfg = self.cfg
        if self.first_cmd_at is not None and now - self.first_cmd_at > cfg.arm_timeout_s:
            return self.abort(AbortCause.LIFTOFF_FAILED,
                              f're-arm or offboard refused for {cfg.arm_timeout_s:.0f} s')
        if now - self.stream_since < cfg.offboard_warmup_s:
            return None
        if cfg.require_pilot_arm and not px4.is_armed(status):
            return None                  # the pilot arms; GNC never sends arm
        if self._every('last_cmd_at', now):
            if self.first_cmd_at is None:
                self.first_cmd_at = now
            self.io.set_offboard()
            if not cfg.require_pilot_arm and not px4.is_armed(status):
                self.io.arm()
        return None


class Returning(MissionStateBase):
    """Fly at search altitude to the start zone center (field frame), around obstacles, then
    LANDING. The simple version: battery and time thresholds come in Milestone 8."""
    NAME = S.RETURNING
    OUTCOMES = ('over_start',)
    GLOBALS = (ABORTED_OUT, PAUSE_OUT, RETARGET_SEEN, RETARGET_UNSEEN)

    def on_entry(self, snap, now):
        self.follower = None
        lp = _msg(snap.local_position)
        if not px4.position_valid(lp):
            return
        cfg, fc = self.cfg, self.ctx.field_config
        here = self.field_xyz(lp)[:2]
        route, failed = sp.avoid_obstacles([fc.start_center], here,
                                           obstacles_of(_msg(snap.obstacles)),
                                           cfg.obstacle_clearance_m, fc.area(0.0))
        if failed:
            self.io.log('warn', 'no obstacle-free route home; flying straight')
        alt = cfg.search_altitude_m
        waypoints = [self.ned_at_height(here[0], here[1], alt)]      # up first, then across
        waypoints += [self.ned_at_height(x, y, alt) for x, y in route]
        yaw = self.ctx.start_position[3] if self.ctx.start_position else lp.heading
        self.follower = PathFollower(waypoints, (lp.x, lp.y, lp.z), yaw, cfg.cruise_speed_mps,
                                     cfg.max_accel_mps2, cfg.acceptance_radius_m)

    def step(self, snap, now):
        if self.follower is None:
            self.on_entry(snap, now)
            if self.follower is None:
                return None
        lp = _msg(snap.local_position)
        position = (lp.x, lp.y, lp.z) if px4.position_valid(lp) else None
        (n, e, d), yaw = self.follower.update(now, position)
        self.io.set_setpoint(n, e, d, yaw)
        if self.follower.done:
            return self.finish('over_start', 'over the start zone')
        return None


class Landing(MissionStateBase):
    """Command PX4 land (in the start zone once RETURNING exists; in place for now) and wait
    for landed and disarmed. PX4 auto-disarms after landing, which must never read as an abort.
    Entered already on the ground (PREFLIGHT return, or resumed after landing): no land
    command; just disarm."""
    NAME = S.LANDING
    OUTCOMES = ('landed',)
    GLOBALS = (ABORTED_OUT, PAUSE_OUT)

    def on_entry(self, snap, now):
        self.last_land_at = now
        self.last_disarm_at = None
        if px4.is_landed(_msg(snap.land_detected)):
            self.io.clear_setpoint()
            self.stream_stopped = True
        else:
            self.stream_stopped = False
            self.io.land()

    def control_lost(self, status, snap, now):
        return self.landing_control_lost(status, snap, now)

    def step(self, snap, now):
        status = _msg(snap.vehicle_status)
        landed = px4.is_landed(_msg(snap.land_detected))
        if landed and px4.is_disarmed(status):
            return self.finish('landed', 'landed and disarmed')
        if px4.is_auto_land(status) and not self.stream_stopped:
            self.io.clear_setpoint()     # PX4 is landing; stop the offboard stream
            self.stream_stopped = True
        if px4.is_armed(status):
            if landed:
                if self._every('last_disarm_at', now):
                    self.io.disarm()     # in case PX4 auto-disarm is off
            elif not px4.is_auto_land(status) and self._every('last_land_at', now):
                self.io.land()
        return None


class Paused(MissionStateBase):
    """Hold where the pause found us; store new intents; leave by the resume rule.

    By paused_from:
      STANDBY            nothing changes (just shows PAUSED)
      PREFLIGHT          stop arming, stop the stream, disarm if armed
      airborne states    hover where it is
      LANDING            still descending: hover where it is (re-enter offboard);
                         already landed: stay landed (no setpoints, no offboard, no arming change)
      DESCENDING         stop descending and hover (landed already: stay landed)
      on the die         keep the motor stage: nothing changes
      LIFTOFF            not off the die yet: stay on it; climbing: hover
    Global checks judge by paused_from (mode()), so abort, return, battery, pose loss and the
    mission clock apply as they did in that state; the mission clock keeps running.
    """
    NAME = S.PAUSED
    GLOBALS = (ABORTED_OUT, RETURN_OUT)
    PAUSABLE = (S.STANDBY, S.PREFLIGHT, S.TAKEOFF, S.SEARCHING, S.APPROACHING, S.DESCENDING,
                S.ON_DIE_TEST, S.LIFTOFF, S.RETURNING, S.LANDING)
    OUTCOMES = tuple(resume_outcome(s) for s in PAUSABLE) + (
        RESUME_NEW_INTENT, RESUME_NEW_INTENT_LIFTOFF, RETURN_GROUND, RETURN_LIFTOFF)

    def mode(self):
        return self.ctx.paused_from

    def on_entry(self, snap, now):
        pf = self.ctx.paused_from
        self.last_cmd_at = None
        status = _msg(snap.vehicle_status)
        landed = px4.is_landed(_msg(snap.land_detected)) or px4.is_disarmed(status)
        if pf == S.STANDBY or pf in S.ON_DIE:
            self.hold = 'nothing'
        elif pf in (S.DESCENDING, S.LIFTOFF) and landed:
            self.hold = 'nothing'         # on the die: keep the motor stage, stay on it
        elif pf == S.PREFLIGHT:
            self.io.clear_setpoint()
            self.hold = 'disarm'
        elif pf == S.LANDING and (px4.is_landed(_msg(snap.land_detected))
                                  or px4.is_disarmed(status)):
            self.io.clear_setpoint()
            self.hold = 'landed'
        elif pf == S.LANDING:
            self._hold_here(snap)
            self.hold = 'leave_land'      # back to offboard to stop the descent
        else:
            self._hold_here(snap)
            self.hold = 'hover'

    def control_lost(self, status, snap, now):
        if self.ctx.paused_from in (S.LANDING, S.DESCENDING, S.LIFTOFF):
            return self.landing_control_lost(status, snap, now)
        return super().control_lost(status, snap, now)

    def step(self, snap, now):
        status = _msg(snap.vehicle_status)
        if self.hold == 'disarm':
            if px4.is_armed(status) and self._every('last_cmd_at', now):
                self.io.disarm()
        elif self.hold == 'leave_land':
            if px4.is_landed(_msg(snap.land_detected)):
                self.io.clear_setpoint()  # PX4 touched down first: stay landed
                self.hold = 'landed'
            elif px4.is_offboard(status):
                self.hold = 'hover'
            elif px4.is_armed(status) and self._every('last_cmd_at', now):
                self.io.set_offboard()
        return None

    def on_pause(self, intent, now):
        self.ack(intent)                  # already paused: applied, no change
        return None

    def on_return_to_start(self, intent, now):
        if self.ctx.paused_from == S.LANDING:
            # Leave the pause and continue LANDING. If PX4 already reports landed, LANDING
            # just finishes (no land command; disarm, RETURN_CONFIRMED).
            self.ack(intent)
            return self.finish(resume_outcome(S.LANDING), 'return to start: landing')
        return super().on_return_to_start(intent, now)

    def on_retrieve(self, intent, now):
        return self._store(intent)

    def on_update_only(self, intent, now):
        return self._store(intent)

    def _store(self, intent):
        with self.ctx.lock:
            self.ctx.stored_intents.append(intent)
        self.ack(intent)
        return None

    def on_resume(self, intent, now):
        """Resume rule. Extra fields on the resume are ignored."""
        self.ack(intent)
        with self.ctx.lock:
            stored, self.ctx.stored_intents = self.ctx.stored_intents, []
            pf = self.ctx.paused_from
        if not stored:
            return self.finish(resume_outcome(pf), 'resumed')
        started = False
        for item in stored:              # apply in arrival order
            msg = item.msg
            with self.ctx.lock:
                if I.action_of(msg) == I.RETRIEVE:
                    self.ctx.regions = I.merge_regions(self.ctx.regions, msg)
                    color = I.color_of(msg)
                    if color and not self.ctx.active_target and pf == S.STANDBY:
                        self.ctx.active_target = color
                        self.ctx.mission_start_time = now
                        started = True
                    elif color and color != self.ctx.active_target:
                        self.set_target(color)   # SEARCHING's entry check finds it if seen
                else:
                    self.ctx.regions = I.merge_regions(self.ctx.regions, msg, hint=True)
        if pf in GROUND_STATES:
            # Paused on the ground: back to paused_from with the stored intents applied; a
            # first retrieve from STANDBY starts PREFLIGHT.
            target = S.PREFLIGHT if started else pf
            return self.finish(resume_outcome(target), 'resumed with new intent')
        if pf in S.ON_DIE:               # LIFTOFF first, then SEARCHING
            self.ctx.after_liftoff = S.SEARCHING
            return self.finish(RESUME_NEW_INTENT_LIFTOFF, 'resumed with new intent')
        return self.finish(RESUME_NEW_INTENT, 'resumed with new intent')


class ReturnConfirmed(MissionStateBase):
    """Terminal: stop commands."""
    NAME = S.RETURN_CONFIRMED
    OUTCOMES = (MISSION_COMPLETE,)

    def step(self, snap, now):
        self.io.clear_setpoint()
        return self.finish(MISSION_COMPLETE, 'mission complete')


class Aborted(MissionStateBase):
    """Terminal. Depends on the cause:
      - PX4 or the pilot has control: stop commanding at once.
      - airborne (abort intent or pose loss): hold land_after_hold_s, then land in place, and
        finish when landed and disarmed (stop commanding if PX4 or the pilot takes over).
      - on the ground or on the die: stop setpoints, disarm if armed, stay.
    `abort_from` is the state the checks judged by (paused_from when aborted from PAUSED).
    """
    NAME = S.ABORTED
    OUTCOMES = (MISSION_ABORTED,)

    def on_entry(self, snap, now):
        cause, came_from = self.ctx.abort_cause, self.ctx.abort_from
        status = _msg(snap.vehicle_status)
        self.last_land_at = self.last_disarm_at = None
        self.stream_stopped = False
        if cause == AbortCause.PX4_CONTROL.value:
            self.io.clear_setpoint()
            self.phase = 'done'
        elif came_from in S.ON_DIE or cause == AbortCause.LIFTOFF_FAILED.value:
            self.io.clear_setpoint()     # on the die: disarm and stay
            self.phase = 'ground'
        elif came_from in S.AIRBORNE and self.ctx.offboard_engaged:
            if (came_from == S.LANDING or px4.is_auto_land(status)
                    or px4.is_landed(_msg(snap.land_detected))):
                self.phase = 'landing'
            else:
                self._hold_here(snap)
                self.phase = 'hold'
        else:
            self.io.clear_setpoint()
            self.phase = 'ground'

    def step(self, snap, now):
        status = _msg(snap.vehicle_status)
        landed = px4.is_landed(_msg(snap.land_detected))
        if self.phase == 'done':
            return MISSION_ABORTED
        if self.phase in ('hold', 'landing') and px4.is_armed(status) and (
                px4.in_failsafe(status)
                or not (px4.is_offboard(status) or px4.is_auto_land(status) or landed)):
            self.io.clear_setpoint()     # PX4 or the pilot took over: stop commanding
            return MISSION_ABORTED
        if self.phase == 'hold':
            if now - self.entered_at >= self.cfg.land_after_hold_s:
                self.phase = 'landing'
            else:
                return None
        if self.phase == 'landing':
            if landed and px4.is_disarmed(status):
                return MISSION_ABORTED
            if (px4.is_auto_land(status) or landed) and not self.stream_stopped:
                self.io.clear_setpoint()
                self.stream_stopped = True
            if px4.is_armed(status):
                if landed:
                    if self._every('last_disarm_at', now):
                        self.io.disarm()
                elif not px4.is_auto_land(status) and self._every('last_land_at', now):
                    self.io.land()
            return None
        # ground
        if px4.is_armed(status):
            if self._every('last_disarm_at', now):
                self.io.disarm()
            return None
        return MISSION_ABORTED


class MissionOver(MissionStateBase):
    """Not a SMACH state: answers intents after the machine has finished (in RETURN_CONFIRMED
    or ABORTED). Every intent except abort is rejected ("mission over"); abort is applied."""
    OUTCOMES = ()

    def __init__(self, ctx, io, cfg, clock):
        super().__init__(ctx, io, cfg, clock)
        self.NAME = ctx.state


# =================================================================================================

STATE_CLASSES = (Standby, Preflight, Takeoff, Searching, Approaching, Descending, OnDieTest,
                 Liftoff, Returning, Landing, Paused, ReturnConfirmed, Aborted)

# Global outcomes of the flying states (TAKEOFF through RETURNING).
_FLYING = {ABORTED_OUT: S.ABORTED, RETURN_OUT: S.RETURNING, PAUSE_OUT: S.PAUSED,
           RETARGET_SEEN: S.APPROACHING, RETARGET_UNSEEN: S.SEARCHING}
# ... and of the on-die states: return and retarget go through LIFTOFF (after_liftoff says where).
_ON_DIE = {ABORTED_OUT: S.ABORTED, RETURN_OUT: S.LIFTOFF, PAUSE_OUT: S.PAUSED,
           RETARGET_SEEN: S.LIFTOFF, RETARGET_UNSEEN: S.LIFTOFF}

# Each state's transitions (outcome -> next state label or container outcome).
TRANSITIONS = {
    S.STANDBY: {'start': S.PREFLIGHT, ABORTED_OUT: S.ABORTED, PAUSE_OUT: S.PAUSED},
    S.PREFLIGHT: {'ready': S.TAKEOFF, 'failed': S.ABORTED, ABORTED_OUT: S.ABORTED,
                  RETURN_OUT: S.LANDING,                  # on the ground: stop, disarm, done
                  PAUSE_OUT: S.PAUSED},
    S.TAKEOFF: {'at_altitude': S.SEARCHING, **_FLYING},
    S.SEARCHING: {'target_known': S.APPROACHING, 'target_found': S.APPROACHING, **_FLYING},
    S.APPROACHING: {'over_target': S.DESCENDING,
                    'wrong_color_known': S.APPROACHING, 'wrong_color_unknown': S.SEARCHING,
                    'lost': S.SEARCHING, **_FLYING},
    S.DESCENDING: {'touchdown': S.ON_DIE_TEST,            # TEMPORARY: NFC_READY in Milestone 7
                   'retry': S.DESCENDING, 'failed': S.APPROACHING, **_FLYING},
    S.ON_DIE_TEST: {'done': S.LIFTOFF, **_ON_DIE},        # TEMPORARY (Milestone 7)
    S.LIFTOFF: {**{out: state for state, out in LIFTOFF_TO.items()}, **_FLYING},
    S.RETURNING: {'over_start': S.LANDING, ABORTED_OUT: S.ABORTED, PAUSE_OUT: S.PAUSED,
                  RETARGET_SEEN: S.APPROACHING, RETARGET_UNSEEN: S.SEARCHING},
    S.LANDING: {'landed': S.RETURN_CONFIRMED, ABORTED_OUT: S.ABORTED, PAUSE_OUT: S.PAUSED},
    S.PAUSED: {**{resume_outcome(s): s for s in Paused.PAUSABLE},
               RESUME_NEW_INTENT: S.SEARCHING,
               RESUME_NEW_INTENT_LIFTOFF: S.LIFTOFF,
               ABORTED_OUT: S.ABORTED,
               RETURN_OUT: S.RETURNING,
               RETURN_GROUND: S.LANDING,
               RETURN_LIFTOFF: S.LIFTOFF},
    S.RETURN_CONFIRMED: {MISSION_COMPLETE: MISSION_COMPLETE},
    S.ABORTED: {MISSION_ABORTED: MISSION_ABORTED},
}


def build_state_machine(ctx: MissionContext, io: MissionIO, cfg: MissionConfig,
                        clock) -> smach.StateMachine:
    """The mission state machine. Run it with run_mission() (or sm.execute()); stop it with
    sm.request_preempt()."""
    if ctx.cameras is None:
        ctx.cameras = load_cameras(cfg.cameras_file or None)
    if ctx.blind_height_m is None:
        ctx.blind_height_m = load_blind_height(cfg.cameras_file or None)
    if ctx.coverage is None and ctx.field_config is not None:
        ctx.coverage = CoverageGrid(ctx.field_config.bounds, cfg.coverage_cell_m)
    sm = smach.StateMachine(outcomes=[MISSION_COMPLETE, MISSION_ABORTED, PREEMPTED])
    with sm:
        for cls in STATE_CLASSES:
            transitions = dict(TRANSITIONS[cls.NAME])
            transitions[PREEMPTED] = PREEMPTED
            smach.StateMachine.add(cls.NAME, cls(ctx, io, cfg, clock), transitions=transitions)

    def on_enter(userdata, active_states, *args):
        new = active_states[0]
        with ctx.lock:
            previous = ctx.state
            ctx.previous_state = previous
            ctx.state = new
            ctx.reason = ctx.pending_reason
            ctx.pending_reason = ''
            if previous == S.PAUSED and new != S.PAUSED:
                ctx.paused_from = ''
            reason = ctx.reason
        io.log('info', f'{previous or "(start)"} -> {new}' + (f': {reason}' if reason else ''))
        io.request_state_publish()

    sm.register_start_cb(on_enter)
    sm.register_transition_cb(on_enter)
    return sm


def run_mission(sm: smach.StateMachine, ctx: MissionContext, io: MissionIO, cfg: MissionConfig,
                clock, should_stop: Callable[[], bool]) -> str:
    """Run the machine; once it ends, keep answering intents (rejections) until `should_stop`."""
    outcome = sm.execute()
    if outcome == PREEMPTED:
        return outcome
    over = MissionOver(ctx, io, cfg, clock)
    period = 1.0 / cfg.loop_hz
    while not should_stop():
        for intent in ctx.take_intents():
            over.handle_intent(intent, clock.now())
        clock.sleep(period)
    return outcome
