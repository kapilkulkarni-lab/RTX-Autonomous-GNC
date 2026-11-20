"""Global checks from every Milestone 3 state they apply to, their priority, and where they
must not fire."""
import pytest

from conftest import (heartbeat, AUTO_LAND, OFFBOARD, POSCTL, battery, intent, land_detected, position,
                      status)
from gnc_offboard.mission import state_names as S
from gnc_offboard.mission.mission_manager_smach import (
    ABORTED_OUT, PAUSE_OUT, RETURN_OUT, STATE_CLASSES, TRANSITIONS, AbortCause, TargetHover, Landing,
    Preflight, Standby, Takeoff)

START = (0.0, 0.0, 0.0, 0.0)
FLYING = [Takeoff, TargetHover, Landing]     # airborne states in this milestone


def fly(ctx, feed, clock, cls):
    """Put the vehicle in the air in the state `cls` expects; nothing else would end it soon."""
    ctx.offboard_engaged = True
    ctx.start_position = START
    ctx.mission_start_time = clock.now()
    nav = AUTO_LAND if cls is Landing else OFFBOARD
    z = -3.0 if cls is Takeoff else -1.5     # Takeoff target is -1.5: stay away from it
    feed.set(vehicle_status=status(True, nav), local_position=position(z=z),
             land_detected=land_detected(False), battery_status=battery(0.9),
             perception_heartbeat=heartbeat())


def run(cls, ctx, io, cfg, clock):
    return cls(ctx, io, cfg, clock).execute()


# ---- 1. PX4 failsafe / offboard lost / pilot takeover / link lost -------------------------------

@pytest.mark.parametrize('cls', FLYING)
@pytest.mark.parametrize('fault, reason', [
    (lambda feed: feed.set(vehicle_status=status(True, OFFBOARD, failsafe=True)), 'PX4 failsafe'),
    (lambda feed: feed.stop('vehicle_status'), 'PX4 link lost'),
])
def test_px4_faults_abort_everywhere_airborne(cls, fault, reason, ctx, io, cfg, clock, feed):
    fly(ctx, feed, clock, cls)
    feed.at(1.0, lambda: fault(feed))
    assert run(cls, ctx, io, cfg, clock) == ABORTED_OUT
    assert ctx.abort_cause == AbortCause.PX4_CONTROL.value
    assert ctx.pending_reason == reason


@pytest.mark.parametrize('cls', [Takeoff, TargetHover])
@pytest.mark.parametrize('new_status, reason', [
    (status(True, POSCTL), 'offboard lost: pilot or PX4 took control'),
    (status(True, AUTO_LAND), 'offboard lost: pilot or PX4 took control'),
    (status(False, OFFBOARD), 'disarmed in flight'),
])
def test_offboard_lost_or_disarm_aborts(cls, new_status, reason, ctx, io, cfg, clock, feed):
    fly(ctx, feed, clock, cls)
    feed.at(1.0, lambda: feed.set(vehicle_status=new_status))
    assert run(cls, ctx, io, cfg, clock) == ABORTED_OUT
    assert ctx.pending_reason == reason


def test_link_lost_after_stale_time_only(ctx, io, cfg, clock, feed):
    fly(ctx, feed, clock, TargetHover)
    feed.at(1.0, lambda: feed.stop('vehicle_status'))
    t0 = clock.now()
    assert run(TargetHover, ctx, io, cfg, clock) == ABORTED_OUT
    assert clock.now() - t0 == pytest.approx(1.0 + cfg.px4_status_stale_s, abs=0.06)


def test_failsafe_ignored_when_disarmed_on_ground(ctx, io, cfg, clock, feed):
    feed.set(vehicle_status=status(False, POSCTL, failsafe=True))
    feed.at(1.0, lambda: ctx.update_input('intent', intent('retrieve', 'blue'), clock.now()))
    assert run(Standby, ctx, io, cfg, clock) == 'start'


def test_failsafe_aborts_preflight_once_armed(ctx, io, cfg, clock, feed):
    ctx.mission_start_time = clock.now()
    feed.set(vehicle_status=status(True, POSCTL, failsafe=True), local_position=position())
    assert run(Preflight, ctx, io, cfg, clock) == ABORTED_OUT
    assert ctx.abort_cause == AbortCause.PX4_CONTROL.value


def test_not_offboard_is_fine_before_offboard_engaged(ctx, io, cfg, clock, fake_px4):
    fake_px4()                                    # POSCTL, disarmed until PREFLIGHT arms it
    ctx.mission_start_time = clock.now()
    assert run(Preflight, ctx, io, cfg, clock) == 'ready'


# ---- 2. abort intent ----------------------------------------------------------------------------

@pytest.mark.parametrize('cls', [Standby, Preflight, Takeoff, TargetHover, Landing])
def test_abort_intent_from_every_state(cls, ctx, io, cfg, clock, feed):
    if cls in FLYING:
        fly(ctx, feed, clock, cls)
    else:
        ctx.mission_start_time = clock.now()
        feed.set(vehicle_status=status(False), local_position=position(valid=False))
    feed.at(0.5, lambda: ctx.update_input('intent', intent('abort', '', 9), clock.now()))
    assert run(cls, ctx, io, cfg, clock) == ABORTED_OUT
    assert ctx.abort_cause == AbortCause.INTENT.value
    assert ctx.abort_from == cls.NAME
    assert (ctx.last_intent_id, ctx.last_intent_result) == (9, 'applied')
    assert 'request_state_publish' in io.names()


def test_abort_intent_with_empty_fields(ctx, io, cfg, clock):
    """A stop-word abort arrives with every other field empty."""
    ctx.update_input('intent', intent('abort', '', 4), clock.now())
    assert run(Standby, ctx, io, cfg, clock) == ABORTED_OUT


def test_abort_wins_over_retrieve_in_same_batch(ctx, io, cfg, clock):
    ctx.update_input('intent', intent('retrieve', 'blue', 1), clock.now())
    ctx.update_input('intent', intent('abort', '', 2), clock.now())
    assert run(Standby, ctx, io, cfg, clock) == ABORTED_OUT
    assert ctx.active_target == ''
    assert [i.msg.intent_id for i in ctx.intent_queue] == [1]    # left for the next state


# ---- 3. position invalid ------------------------------------------------------------------------

@pytest.mark.parametrize('cls', FLYING)
def test_position_invalid_aborts_after_limit(cls, ctx, io, cfg, clock, feed):
    fly(ctx, feed, clock, cls)
    z = -3.0 if cls is Takeoff else -1.5
    feed.at(1.0, lambda: feed.set(local_position=position(z=z, valid=False)))
    t0 = clock.now()
    assert run(cls, ctx, io, cfg, clock) == ABORTED_OUT
    assert ctx.abort_cause == AbortCause.POSE.value
    assert clock.now() - t0 == pytest.approx(1.0 + cfg.pose_invalid_s, abs=0.11)


def test_short_position_dropout_is_tolerated(ctx, io, cfg, clock, feed):
    fly(ctx, feed, clock, TargetHover)
    feed.at(1.0, lambda: feed.set(local_position=position(z=-1.5, valid=False)))
    feed.at(1.0 + cfg.pose_invalid_s / 2, lambda: feed.set(local_position=position(z=-1.5)))
    feed.at(3.0, lambda: feed.set(local_position=position(z=-1.5, valid=False)))
    feed.at(3.0 + cfg.pose_invalid_s / 2, lambda: feed.set(local_position=position(z=-1.5)))
    assert run(TargetHover, ctx, io, cfg, clock) == RETURN_OUT
    assert ctx.pending_reason.startswith('target confirmed')   # its own end, no check fired


def test_stale_position_aborts(ctx, io, cfg, clock, feed):
    fly(ctx, feed, clock, TargetHover)
    feed.at(1.0, lambda: feed.stop('local_position'))
    assert run(TargetHover, ctx, io, cfg, clock) == ABORTED_OUT
    assert ctx.abort_cause == AbortCause.POSE.value


def test_position_check_not_on_ground(ctx, io, cfg, clock, feed):
    """PREFLIGHT just waits for a valid estimate; it does not abort."""
    feed.set(vehicle_status=status(False), local_position=position(valid=False))
    ctx.mission_start_time = clock.now()
    clock.limit = clock.now() + 10.0
    with pytest.raises(TimeoutError):
        run(Preflight, ctx, io, cfg, clock)


# ---- 4. battery ---------------------------------------------------------------------------------

@pytest.mark.parametrize('cls', [Takeoff, TargetHover])
def test_low_battery_returns(cls, ctx, io, cfg, clock, feed):
    fly(ctx, feed, clock, cls)
    feed.at(1.0, lambda: feed.set(battery_status=battery(cfg.battery_return_pct / 100 - 0.01)))
    assert run(cls, ctx, io, cfg, clock) == RETURN_OUT
    assert ctx.pending_reason.startswith('battery 29%')


def test_low_battery_ignored_while_landing(ctx, io, cfg, clock, feed):
    fly(ctx, feed, clock, Landing)
    feed.set(battery_status=battery(0.05))
    feed.at(2.0, lambda: feed.set(land_detected=land_detected(True),
                                  vehicle_status=status(False, POSCTL)))
    assert run(Landing, ctx, io, cfg, clock) == 'landed'


@pytest.mark.parametrize('remaining', [-1.0, float('nan')])
def test_unknown_battery_ignored(remaining, ctx, io, cfg, clock, feed):
    fly(ctx, feed, clock, TargetHover)
    feed.set(battery_status=battery(remaining))
    assert run(TargetHover, ctx, io, cfg, clock) == RETURN_OUT
    assert ctx.pending_reason.startswith('target confirmed')   # its own end, no check fired


def test_battery_not_checked_on_ground(ctx, io, cfg, clock, fake_px4, feed):
    fake_px4()
    ctx.mission_start_time = clock.now()
    # FakePx4 reports 0.9 each tick; this hook runs after it and reports 5% instead.
    clock.hooks.append(lambda now: ctx.update_input('battery_status', battery(0.05), now))
    assert run(Preflight, ctx, io, cfg, clock) == 'ready'


# ---- 5. mission clock ---------------------------------------------------------------------------

@pytest.mark.parametrize('cls', [Preflight, Takeoff, TargetHover])
def test_mission_clock_forces_return(cls, ctx, io, cfg, clock, feed):
    if cls is Preflight:
        feed.set(vehicle_status=status(False), local_position=position(valid=False))
    else:
        fly(ctx, feed, clock, cls)
    ctx.mission_start_time = clock.now() - cfg.force_return_at_s + 0.5
    assert run(cls, ctx, io, cfg, clock) == RETURN_OUT
    assert ctx.pending_reason == 'mission time up: returning'


def test_mission_clock_not_checked_in_landing(ctx, io, cfg, clock, feed):
    fly(ctx, feed, clock, Landing)
    ctx.mission_start_time = clock.now() - 10 * cfg.force_return_at_s
    feed.at(1.0, lambda: feed.set(land_detected=land_detected(True),
                                  vehicle_status=status(False, POSCTL)))
    assert run(Landing, ctx, io, cfg, clock) == 'landed'


def test_mission_clock_not_running_in_standby(ctx, io, cfg, clock, feed):
    feed.at(1.0, lambda: ctx.update_input('intent', intent('retrieve', 'red'), clock.now()))
    assert run(Standby, ctx, io, cfg, clock) == 'start'


# ---- priority -----------------------------------------------------------------------------------

def test_priority_px4_before_abort_intent(ctx, io, cfg, clock, feed):
    fly(ctx, feed, clock, TargetHover)
    feed.set(vehicle_status=status(True, OFFBOARD, failsafe=True))
    ctx.update_input('intent', intent('abort', '', 3), clock.now())
    assert run(TargetHover, ctx, io, cfg, clock) == ABORTED_OUT
    assert ctx.abort_cause == AbortCause.PX4_CONTROL.value
    assert len(ctx.intent_queue) == 1             # abort intent not consumed here


def test_priority_abort_intent_before_return(ctx, io, cfg, clock, feed):
    fly(ctx, feed, clock, TargetHover)
    feed.set(battery_status=battery(0.01))
    ctx.mission_start_time = clock.now() - 10 * cfg.force_return_at_s
    ctx.update_input('intent', intent('abort', '', 3), clock.now())
    assert run(TargetHover, ctx, io, cfg, clock) == ABORTED_OUT
    assert ctx.abort_cause == AbortCause.INTENT.value


def test_priority_pose_before_battery(ctx, io, cfg, clock, feed):
    fly(ctx, feed, clock, TargetHover)
    feed.set(battery_status=battery(0.01))
    feed.stop('local_position')
    clock.t += 2 * cfg.pose_invalid_s             # position already stale on the first loop
    feed.tick(clock.now())                        # everything else fresh
    assert run(TargetHover, ctx, io, cfg, clock) == ABORTED_OUT
    assert ctx.abort_cause == AbortCause.POSE.value


def test_priority_battery_before_clock(ctx, io, cfg, clock, feed):
    fly(ctx, feed, clock, TargetHover)
    feed.set(battery_status=battery(0.01))
    ctx.mission_start_time = clock.now() - 10 * cfg.force_return_at_s
    assert run(TargetHover, ctx, io, cfg, clock) == RETURN_OUT
    assert ctx.pending_reason.startswith('battery')


# ---- wiring -------------------------------------------------------------------------------------

def test_return_goes_to_landing_for_now():
    """TEMPORARY until Milestone 8 (RETURNING)."""
    for name, targets in TRANSITIONS.items():
        if RETURN_OUT in targets:
            assert targets[RETURN_OUT] == S.LANDING, name


def test_global_outcomes_registered_and_wired(ctx, io, cfg, clock):
    for cls in STATE_CLASSES:
        state = cls(ctx, io, cfg, clock)
        assert set(cls.GLOBALS) <= set(state.get_registered_outcomes())
        assert set(cls.OUTCOMES) | set(cls.GLOBALS) == set(TRANSITIONS[cls.NAME])
    assert Standby.GLOBALS == (ABORTED_OUT, PAUSE_OUT)          # no return on the ground
    assert Landing.GLOBALS == (ABORTED_OUT, PAUSE_OUT)          # already landing in place
