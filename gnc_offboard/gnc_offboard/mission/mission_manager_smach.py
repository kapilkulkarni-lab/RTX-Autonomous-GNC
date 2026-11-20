"""mission_manager state machine (SMACH). No rclpy here: states act only through MissionIO and
MissionContext, and get time from an injected clock, so every state runs under pytest.

Built so far (Milestones 3-4):
    STANDBY -> PREFLIGHT -> TAKEOFF -> HOVER_TEST (TEMPORARY) -> LANDING -> RETURN_CONFIRMED
    plus ABORTED, PAUSED, the global checks and full intent handling.

TEMPORARY until later milestones (search for "TEMPORARY"; outline: Temporary stand-ins):
    - HOVER_TEST stands in for SEARCHING (Milestone 5), also as the resume_new_intent target.
    - The `return` outcome goes to LANDING in place; RETURNING arrives in Milestone 8.
    - retrieve with a new color only updates active_target (retarget: Milestone 5).
    - Region fields are stored only (Milestone 9). read_chip is always rejected (Milestone 7).
"""
import enum
from typing import Callable, List, Optional

import smach
from avc_msgs.msg import MissionState

from gnc_offboard.guidance import px4_interface as px4
from gnc_offboard.mission import intents as I
from gnc_offboard.mission import state_names as S
from gnc_offboard.mission.checks import invalid_for, is_stale
from gnc_offboard.mission.config import MissionConfig
from gnc_offboard.mission.context import MissionContext, Stamped
from gnc_offboard.mission.io import MissionIO

# Global outcomes and SMACH's preempt outcome.
ABORTED_OUT = 'aborted'
RETURN_OUT = 'return'
PAUSE_OUT = 'pause'
PREEMPTED = 'preempted'
RESUME_NEW_INTENT = 'resume_new_intent'

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


def resume_outcome(state: str) -> str:
    """PAUSED's outcome for going back to `state` (one per pausable state)."""
    return 'resume_' + state.lower()


class AbortCause(str, enum.Enum):
    PX4_CONTROL = 'px4_control'          # failsafe, offboard lost, pilot takeover, PX4 link lost
    INTENT = 'intent'                    # comms `abort`
    POSE = 'pose'                        # position estimate invalid
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
        if RETURN_OUT in self.GLOBALS:
            pct = px4.battery_percent(_msg(snap.battery_status))
            if mode in BATTERY_STATES and pct is not None and pct < self.cfg.battery_return_pct:
                return self.finish(RETURN_OUT, f'battery {pct:.0f}%: returning')
            if mode in CLOCK_STATES and \
                    self.ctx.mission_time_s(now) > self.cfg.force_return_at_s:
                return self.finish(RETURN_OUT, 'mission time up: returning')
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
            return self.finish(RETURN_OUT, 'return to start')
        return None                                      # applied, no change

    def on_retrieve(self, intent, now):
        msg = intent.msg
        color = I.color_of(msg)
        with self.ctx.lock:
            self.ctx.regions = I.merge_regions(self.ctx.regions, msg)
            first = not self.ctx.active_target
            if color and not first and color != self.ctx.active_target:
                # TEMPORARY (Milestone 5): a new color only updates the target, no retarget.
                self.ctx.active_target = color
        self.ack(intent)
        if color and first and self.NAME == S.STANDBY:
            return self._start(color, now)
        return None

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
            # Start position in PX4 local NED (field frame: Milestone 5).
            self.ctx.start_position = (p.x, p.y, p.z, p.heading)

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
    GLOBALS = (ABORTED_OUT, RETURN_OUT, PAUSE_OUT)

    def on_entry(self, snap, now):
        x, y, z, yaw = self.ctx.start_position
        self.target_z = z - self.cfg.search_altitude_m          # NED: up is negative z
        self.io.set_setpoint(x, y, self.target_z, yaw)

    def step(self, snap, now):
        lp = _msg(snap.local_position)
        if px4.position_valid(lp) and abs(lp.z - self.target_z) <= self.cfg.altitude_tol_m:
            return self.finish('at_altitude', f'at {self.cfg.search_altitude_m:.1f} m')
        return None


class HoverTest(MissionStateBase):
    """TEMPORARY (Milestone 3 only; replaced by SEARCHING in Milestone 5): hover in place for
    hover_test_s, then land in place."""
    NAME = S.HOVER_TEST
    OUTCOMES = ('done',)
    GLOBALS = (ABORTED_OUT, RETURN_OUT, PAUSE_OUT)

    def step(self, snap, now):
        if now - self.entered_at >= self.cfg.hover_test_s:
            return self.finish('done', 'hover test done: landing')
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
      TAKEOFF/HOVER_TEST hover where it is
      LANDING            still descending: hover where it is (re-enter offboard);
                         already landed: stay landed (no setpoints, no offboard, no arming change)
    Global checks judge by paused_from (mode()), so abort, return, battery, pose loss and the
    mission clock apply as they did in that state; the mission clock keeps running.
    """
    NAME = S.PAUSED
    GLOBALS = (ABORTED_OUT, RETURN_OUT)
    PAUSABLE = (S.STANDBY, S.PREFLIGHT, S.TAKEOFF, S.HOVER_TEST, S.LANDING)
    OUTCOMES = tuple(resume_outcome(s) for s in PAUSABLE) + (RESUME_NEW_INTENT,)

    def mode(self):
        return self.ctx.paused_from

    def on_entry(self, snap, now):
        pf = self.ctx.paused_from
        self.last_cmd_at = None
        status = _msg(snap.vehicle_status)
        if pf == S.STANDBY:
            self.hold = 'nothing'
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
        if self.ctx.paused_from == S.LANDING:
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
                    elif color:
                        self.ctx.active_target = color   # TEMPORARY (Milestone 5): no retarget
                else:
                    self.ctx.regions = I.merge_regions(self.ctx.regions, msg, hint=True)
        if pf in GROUND_STATES:
            # Paused on the ground: back to paused_from with the stored intents applied; a
            # first retrieve from STANDBY starts PREFLIGHT.
            target = S.PREFLIGHT if started else pf
            return self.finish(resume_outcome(target), 'resumed with new intent')
        # TEMPORARY (Milestone 5): SEARCHING; on the die, LIFTOFF first (Milestone 6).
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
      - on the ground: stop setpoints, disarm if armed.
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

STATE_CLASSES = (Standby, Preflight, Takeoff, HoverTest, Landing, Paused, ReturnConfirmed,
                 Aborted)

# Each state's transitions (outcome -> next state label or container outcome).
TRANSITIONS = {
    S.STANDBY: {'start': S.PREFLIGHT, ABORTED_OUT: S.ABORTED, PAUSE_OUT: S.PAUSED},
    S.PREFLIGHT: {'ready': S.TAKEOFF, 'failed': S.ABORTED, ABORTED_OUT: S.ABORTED,
                  RETURN_OUT: S.LANDING,                  # on the ground: stop, disarm, done
                  PAUSE_OUT: S.PAUSED},
    S.TAKEOFF: {'at_altitude': S.HOVER_TEST,              # TEMPORARY: SEARCHING in Milestone 5
                ABORTED_OUT: S.ABORTED,
                RETURN_OUT: S.LANDING,                    # TEMPORARY: RETURNING in Milestone 8
                PAUSE_OUT: S.PAUSED},
    S.HOVER_TEST: {'done': S.LANDING, ABORTED_OUT: S.ABORTED,
                   RETURN_OUT: S.LANDING,                 # TEMPORARY: RETURNING in Milestone 8
                   PAUSE_OUT: S.PAUSED},
    S.LANDING: {'landed': S.RETURN_CONFIRMED, ABORTED_OUT: S.ABORTED, PAUSE_OUT: S.PAUSED},
    S.PAUSED: {**{resume_outcome(s): s for s in Paused.PAUSABLE},
               RESUME_NEW_INTENT: S.HOVER_TEST,           # TEMPORARY: SEARCHING in Milestone 5
               ABORTED_OUT: S.ABORTED,
               RETURN_OUT: S.LANDING},                    # TEMPORARY: RETURNING in Milestone 8
    S.RETURN_CONFIRMED: {MISSION_COMPLETE: MISSION_COMPLETE},
    S.ABORTED: {MISSION_ABORTED: MISSION_ABORTED},
}


def build_state_machine(ctx: MissionContext, io: MissionIO, cfg: MissionConfig,
                        clock) -> smach.StateMachine:
    """The mission state machine. Run it with run_mission() (or sm.execute()); stop it with
    sm.request_preempt()."""
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
