"""mission_manager state machine (SMACH). No rclpy here: states act only through MissionIO and
MissionContext, and get time from an injected clock, so every state runs under pytest.

Milestone 3 builds the flight phases only:
    STANDBY -> PREFLIGHT -> TAKEOFF -> HOVER_TEST (TEMPORARY) -> LANDING -> RETURN_CONFIRMED
    plus ABORTED and the global checks.

TEMPORARY until later milestones (search for "TEMPORARY"):
    - HOVER_TEST stands in for SEARCHING (Milestone 5).
    - The `return` outcome goes to LANDING in place; RETURNING arrives in Milestone 8.
    - Intents: only `retrieve` (in STANDBY) and `abort` are handled; others are logged and left
      unacknowledged until Milestone 4.
"""
import enum
from typing import List, Optional

import smach
from avc_msgs.msg import MissionState

from gnc_offboard.guidance import px4_interface as px4
from gnc_offboard.mission import state_names as S
from gnc_offboard.mission.checks import invalid_for, is_stale
from gnc_offboard.mission.config import MissionConfig
from gnc_offboard.mission.context import MissionContext, Stamped
from gnc_offboard.mission.io import MissionIO

# Intent actions (avc_comms schema names). Milestone 4 moves intent handling to its own module.
ACTION_RETRIEVE = 'retrieve'
ACTION_ABORT = 'abort'

# Global outcomes and SMACH's preempt outcome.
ABORTED_OUT = 'aborted'
RETURN_OUT = 'return'
PREEMPTED = 'preempted'

# Container outcomes.
MISSION_COMPLETE = 'mission_complete'
MISSION_ABORTED = 'mission_aborted'

# Where the global `return` checks apply. Battery: airborne and on-die states. Mission clock: all
# states before RETURNING. Neither applies while already landing or returning.
BATTERY_STATES = (S.AIRBORNE | S.ON_DIE) - {S.LANDING, S.RETURNING}
CLOCK_STATES = frozenset(S.PUBLISHABLE) - {S.STANDBY, S.RETURNING, S.LANDING} - S.END_STATES


class AbortCause(str, enum.Enum):
    PX4_CONTROL = 'px4_control'          # failsafe, offboard lost, pilot takeover, PX4 link lost
    INTENT = 'intent'                    # comms `abort`
    POSE = 'pose'                        # position estimate invalid
    PREFLIGHT_FAILED = 'preflight_failed'


def _msg(stamped: Optional[Stamped]):
    return None if stamped is None else stamped.msg


class MissionStateBase(smach.State):
    """Common loop for every state: global checks first, then the state's own checks."""

    NAME = ''
    OUTCOMES = ()     # the state's own outcomes
    GLOBALS = ()      # global outcomes that apply to this state (ABORTED_OUT, RETURN_OUT)

    def __init__(self, ctx: MissionContext, io: MissionIO, cfg: MissionConfig, clock):
        super().__init__(outcomes=list(self.OUTCOMES) + list(self.GLOBALS) + [PREEMPTED])
        self.ctx, self.io, self.cfg, self.clock = ctx, io, cfg, clock
        self.entered_at = 0.0

    # ---- loop -----------------------------------------------------------------------------------
    def execute(self, userdata=None):
        self.entered_at = self.clock.now()
        self.on_entry(self.ctx.snapshot(), self.entered_at)
        period = 1.0 / self.cfg.loop_hz
        while True:
            if self.preempt_requested():
                self.service_preempt()
                return PREEMPTED
            now = self.clock.now()
            snap = self.ctx.snapshot()
            intents = self.ctx.take_intents()
            outcome = self.global_checks(snap, intents, now)
            while outcome is None and intents:
                outcome = self.on_intent(intents.pop(0), now)
            if outcome is None:
                outcome = self.step(snap, now)
            if outcome is not None:
                self._requeue(intents)          # unhandled intents go to the next state
                if outcome not in self.get_registered_outcomes():
                    raise RuntimeError(f'{self.NAME} returned unregistered outcome {outcome!r}')
                return outcome
            self.clock.sleep(period)

    # ---- hooks for states -----------------------------------------------------------------------
    def on_entry(self, snap, now) -> None:
        pass

    def step(self, snap, now) -> Optional[str]:
        return None

    def on_intent(self, intent: Stamped, now) -> Optional[str]:
        """Intents a state does not handle: logged, not acknowledged (TEMPORARY, Milestone 4)."""
        self.io.log('warn', f'intent {intent.msg.intent_id} action {intent.msg.action!r} '
                            f'not handled in {self.NAME} yet (Milestone 4)')
        return None

    def control_lost(self, status, snap, now) -> Optional[str]:
        """Airborne rule: must stay armed and in offboard. LANDING overrides this."""
        if not px4.is_armed(status):
            return 'disarmed in flight'
        if not px4.is_offboard(status):
            return 'offboard lost: pilot or PX4 took control'
        return None

    # ---- global checks, in priority order -------------------------------------------------------
    def global_checks(self, snap, intents: List[Stamped], now) -> Optional[str]:
        if ABORTED_OUT in self.GLOBALS:
            why = self._px4_control_lost(snap, now)
            if why:
                return self.abort(AbortCause.PX4_CONTROL, why)
            for intent in intents:
                if intent.msg.action == ACTION_ABORT:
                    intents.remove(intent)
                    self.ack(intent, applied=True)
                    return self.abort(AbortCause.INTENT, 'abort intent')
            if self.NAME in S.AIRBORNE and self._pose_lost(snap, now):
                return self.abort(AbortCause.POSE, 'position estimate invalid')
        if RETURN_OUT in self.GLOBALS:
            pct = px4.battery_percent(_msg(snap.battery_status))
            if (self.NAME in BATTERY_STATES and pct is not None
                    and pct < self.cfg.battery_return_pct):
                return self.finish(RETURN_OUT, f'battery {pct:.0f}%: returning')
            if self.NAME in CLOCK_STATES and \
                    self.ctx.mission_time_s(now) > self.cfg.force_return_at_s:
                return self.finish(RETURN_OUT, 'mission time up: returning')
        return None

    def _px4_control_lost(self, snap, now) -> Optional[str]:
        status = _msg(snap.vehicle_status)
        # A failsafe matters once we fly or the vehicle is armed (a disarmed vehicle on the
        # ground has nothing to hand back).
        if (self.ctx.offboard_engaged or px4.is_armed(status)) and px4.in_failsafe(status):
            return 'PX4 failsafe'
        if not self.ctx.offboard_engaged or self.NAME not in S.AIRBORNE:
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

    # ---- helpers --------------------------------------------------------------------------------
    def finish(self, outcome: str, reason: str) -> str:
        """Return `outcome`; `reason` goes on /mission/state with the transition."""
        self.ctx.pending_reason = reason
        return outcome

    def abort(self, cause: AbortCause, reason: str) -> str:
        with self.ctx.lock:
            self.ctx.abort_cause = cause.value
            self.ctx.abort_from = self.NAME
        return self.finish(ABORTED_OUT, reason)

    def ack(self, intent: Stamped, applied: bool) -> None:
        with self.ctx.lock:
            self.ctx.last_intent_id = int(intent.msg.intent_id)
            self.ctx.last_intent_result = (MissionState.RESULT_APPLIED if applied
                                           else MissionState.RESULT_REJECTED)
        self.io.request_state_publish()

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


# =================================================================================================

class Standby(MissionStateBase):
    """Disarmed, no setpoints. Waits for the first `retrieve` with a target color."""
    NAME = S.STANDBY
    OUTCOMES = ('start',)
    GLOBALS = (ABORTED_OUT,)

    def on_entry(self, snap, now):
        self.io.clear_setpoint()

    def on_intent(self, intent, now):
        msg = intent.msg
        if msg.action == ACTION_RETRIEVE and msg.target_color:
            with self.ctx.lock:
                self.ctx.active_target = msg.target_color
                self.ctx.mission_start_time = now
            self.ack(intent, applied=True)
            return self.finish('start', f'retrieve {msg.target_color}: starting')
        return super().on_intent(intent, now)


class Preflight(MissionStateBase):
    """Hold the current pose as the setpoint, wait for a stable estimate, record the start,
    then arm (or wait for the pilot to arm) and enter offboard."""
    NAME = S.PREFLIGHT
    OUTCOMES = ('ready', 'failed')
    GLOBALS = (ABORTED_OUT, RETURN_OUT)

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
    GLOBALS = (ABORTED_OUT, RETURN_OUT)

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
    GLOBALS = (ABORTED_OUT, RETURN_OUT)

    def step(self, snap, now):
        if now - self.entered_at >= self.cfg.hover_test_s:
            return self.finish('done', 'hover test done: landing')
        return None


class Landing(MissionStateBase):
    """Command PX4 land (in the start zone once RETURNING exists; in place for now) and wait
    for landed and disarmed. PX4 auto-disarms after landing, which must never read as an abort."""
    NAME = S.LANDING
    OUTCOMES = ('landed',)
    GLOBALS = (ABORTED_OUT,)

    def on_entry(self, snap, now):
        self.disarmed_unlanded_since = None
        self.stream_stopped = False
        self.last_land_at = now
        self.last_disarm_at = None
        self.io.land()

    def control_lost(self, status, snap, now):
        landed = px4.is_landed(_msg(snap.land_detected))
        if px4.is_disarmed(status):
            if landed:
                return None              # normal end of a landing (PX4 auto-disarm)
            # Disarm seen before the landed flag: give the flag a moment to arrive.
            if self.disarmed_unlanded_since is None:
                self.disarmed_unlanded_since = now
            if now - self.disarmed_unlanded_since > self.cfg.landed_disarm_grace_s:
                return 'disarmed before landing'
            return None
        self.disarmed_unlanded_since = None
        if not (px4.is_auto_land(status) or px4.is_offboard(status)):
            return 'pilot or PX4 took control during landing'
        return None

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
            if came_from == S.LANDING or px4.is_auto_land(status):
                self.phase = 'landing'
            else:
                lp = _msg(snap.local_position)
                if px4.position_valid(lp):
                    self.io.set_setpoint(lp.x, lp.y, lp.z, lp.heading)   # hold here
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
                or not (px4.is_offboard(status) or px4.is_auto_land(status))):
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
            if px4.is_auto_land(status) and not self.stream_stopped:
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


# =================================================================================================

STATE_CLASSES = (Standby, Preflight, Takeoff, HoverTest, Landing, ReturnConfirmed, Aborted)

# Each state's transitions (outcome -> next state label or container outcome).
TRANSITIONS = {
    S.STANDBY: {'start': S.PREFLIGHT, ABORTED_OUT: S.ABORTED},
    S.PREFLIGHT: {'ready': S.TAKEOFF, 'failed': S.ABORTED, ABORTED_OUT: S.ABORTED,
                  RETURN_OUT: S.LANDING},                 # TEMPORARY: RETURNING in Milestone 8
    S.TAKEOFF: {'at_altitude': S.HOVER_TEST,              # TEMPORARY: SEARCHING in Milestone 5
                ABORTED_OUT: S.ABORTED, RETURN_OUT: S.LANDING},
    S.HOVER_TEST: {'done': S.LANDING, ABORTED_OUT: S.ABORTED, RETURN_OUT: S.LANDING},
    S.LANDING: {'landed': S.RETURN_CONFIRMED, ABORTED_OUT: S.ABORTED},
    S.RETURN_CONFIRMED: {MISSION_COMPLETE: MISSION_COMPLETE},
    S.ABORTED: {MISSION_ABORTED: MISSION_ABORTED},
}


def build_state_machine(ctx: MissionContext, io: MissionIO, cfg: MissionConfig,
                        clock) -> smach.StateMachine:
    """The mission state machine. Run it with sm.execute(); stop it with sm.request_preempt()."""
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
            reason = ctx.reason
        io.log('info', f'{previous or "(start)"} -> {new}' + (f': {reason}' if reason else ''))
        io.request_state_publish()

    sm.register_start_cb(on_enter)
    sm.register_transition_cb(on_enter)
    return sm
