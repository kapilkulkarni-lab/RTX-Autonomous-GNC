"""Every outcome of every Milestone 3 state, with FakeIO and a fake clock."""
import pytest

from conftest import (flight, heartbeat, AUTO_LAND, OFFBOARD, POSCTL, battery, intent, land_detected, position,
                      status)
from gnc_offboard.mission import state_names as S
from gnc_offboard.mission.mission_manager_smach import (
    MISSION_ABORTED, MISSION_COMPLETE, PREEMPTED, AbortCause, Aborted, Landing,
    Preflight, ReturnConfirmed, Standby, Takeoff, build_state_machine)

START = (1.0, 2.0, -0.1, 0.5)   # PX4 NED start position + yaw


def run(cls, ctx, io, cfg, clock):
    return cls(ctx, io, cfg, clock).execute()


def airborne(ctx, feed, clock, z=-1.5, nav=OFFBOARD):
    ctx.offboard_engaged = True
    ctx.start_position = START
    ctx.mission_start_time = clock.now()
    feed.set(vehicle_status=status(True, nav), local_position=position(1.0, 2.0, z),
             land_detected=land_detected(False), battery_status=battery(0.9),
             perception_heartbeat=heartbeat())


# ---- STANDBY ------------------------------------------------------------------------------------

def test_standby_starts_on_retrieve_with_color(ctx, io, cfg, clock):
    ctx.update_input('intent', intent('retrieve', 'blue', intent_id=7), clock.now())
    assert run(Standby, ctx, io, cfg, clock) == 'start'
    assert ctx.active_target == 'blue'
    assert ctx.mission_start_time == clock.now()
    assert (ctx.last_intent_id, ctx.last_intent_result) == (7, 'applied')
    assert io.names() == ['clear_setpoint', 'request_state_publish']
    assert ctx.pending_reason == 'retrieve blue: starting'


def test_standby_waits_for_retrieve_with_a_color(ctx, io, cfg, clock, feed):
    """update_only and a retrieve without a color are applied but do not start the mission."""
    ctx.update_input('intent', intent('update_only', intent_id=1), clock.now())
    ctx.update_input('intent', intent('retrieve', '', intent_id=2), clock.now())
    feed.at(1.0, lambda: ctx.update_input('intent', intent('retrieve', 'red', 3), clock.now()))
    t0 = clock.now()
    assert run(Standby, ctx, io, cfg, clock) == 'start'
    assert clock.now() - t0 >= 1.0 - 1e-6         # only the retrieve with a color started it
    assert ctx.last_intent_id == 3
    assert io.names().count('request_state_publish') == 3     # each intent published at once
    assert 'arm' not in io.names()


# ---- PREFLIGHT ----------------------------------------------------------------------------------

def test_preflight_arms_and_enters_offboard(ctx, io, cfg, clock, fake_px4):
    px4 = fake_px4()
    t0 = clock.now()
    assert run(Preflight, ctx, io, cfg, clock) == 'ready'
    assert ctx.offboard_engaged and px4.armed and px4.nav == OFFBOARD
    assert ctx.start_position == (0.0, 0.0, 0.0, 0.0)
    assert io.names().count('arm') >= 1 and 'set_offboard' in io.names()
    first_cmd = io.names().index('set_offboard')
    assert io.calls[0][0] == 'set_setpoint'           # stream starts before any command
    assert 'arm' not in io.names()[:first_cmd]
    # Commands only after the estimate has been stable for ekf_stable_s.
    assert clock.now() - t0 >= cfg.ekf_stable_s


def test_preflight_waits_while_position_invalid(ctx, io, make_cfg, clock, feed):
    feed.set(vehicle_status=status(False), local_position=position(valid=False))
    clock.limit = clock.now() + 30.0
    with pytest.raises(TimeoutError):
        run(Preflight, ctx, io, make_cfg(), clock)
    assert io.calls == []


def test_preflight_restarts_stability_window_when_position_drops(ctx, io, cfg, clock, feed):
    feed.set(vehicle_status=status(False), local_position=position())
    feed.at(cfg.ekf_stable_s - 0.5, lambda: feed.set(local_position=position(valid=False)))
    feed.at(cfg.ekf_stable_s - 0.3, lambda: feed.set(local_position=position()))
    clock.limit = clock.now() + 2 * cfg.ekf_stable_s - 0.5
    with pytest.raises(TimeoutError):
        run(Preflight, ctx, io, cfg, clock)
    assert 'arm' not in io.names()


def test_preflight_fails_when_arming_refused(ctx, io, cfg, clock, fake_px4):
    fake_px4(accept_arm=False)
    assert run(Preflight, ctx, io, cfg, clock) == 'failed'
    assert ctx.abort_cause == AbortCause.PREFLIGHT_FAILED.value
    # Retried every cmd_retry_s until arm_timeout_s.
    assert io.names().count('arm') == pytest.approx(cfg.arm_timeout_s / cfg.cmd_retry_s + 1, abs=1)


def test_require_pilot_arm_never_sends_arm(ctx, io, make_cfg, clock, fake_px4, feed):
    cfg = make_cfg(require_pilot_arm=True)
    px4 = fake_px4()
    feed.at(8.0, px4.pilot_arm)
    assert run(Preflight, ctx, io, cfg, clock) == 'ready'
    assert 'arm' not in io.names()
    assert 'set_offboard' in io.names()
    assert ctx.offboard_engaged


def test_require_pilot_arm_waits_without_timeout(ctx, io, make_cfg, clock, fake_px4):
    cfg = make_cfg(require_pilot_arm=True)
    fake_px4()
    clock.limit = clock.now() + 5 * cfg.arm_timeout_s
    with pytest.raises(TimeoutError):            # still waiting for the pilot, not 'failed'
        run(Preflight, ctx, io, cfg, clock)
    assert 'arm' not in io.names() and 'set_offboard' not in io.names()


def test_require_pilot_arm_fails_if_offboard_refused(ctx, io, make_cfg, clock, feed):
    cfg = make_cfg(require_pilot_arm=True)
    feed.set(vehicle_status=status(True, POSCTL), local_position=position())
    assert run(Preflight, ctx, io, cfg, clock) == 'failed'
    assert 'arm' not in io.names()


# ---- TAKEOFF ------------------------------------------------------------------------------------

def test_takeoff_climbs_from_start_to_search_altitude(ctx, io, cfg, clock, feed):
    airborne(ctx, feed, clock, z=-0.1)
    target_z = START[2] - cfg.search_altitude_m
    feed.at(1.0, lambda: feed.set(local_position=position(1.0, 2.0, target_z + 0.3)))
    feed.at(2.0, lambda: feed.set(local_position=position(1.0, 2.0, target_z + 0.1)))
    t0 = clock.now()
    assert run(Takeoff, ctx, io, cfg, clock) == 'at_altitude'
    assert io.calls[0] == ('set_setpoint', (1.0, 2.0, target_z, 0.5))
    assert clock.now() - t0 == pytest.approx(2.0, abs=0.06)


# ---- LANDING ------------------------------------------------------------------------------------

def landing_setup(ctx, feed, clock):
    airborne(ctx, feed, clock)
    feed.at(0.1, lambda: feed.set(vehicle_status=status(True, AUTO_LAND)))


def test_landing_normal_landed_then_auto_disarm(ctx, io, cfg, clock, feed):
    """PX4 order: landed flag first, auto-disarm ~2 s later. Must give landed, not aborted."""
    landing_setup(ctx, feed, clock)
    feed.at(3.0, lambda: feed.set(land_detected=land_detected(True)))
    feed.at(5.0, lambda: feed.set(vehicle_status=status(False, POSCTL)))
    assert run(Landing, ctx, io, cfg, clock) == 'landed'
    assert io.names()[0] == 'land'
    assert 'clear_setpoint' in io.names()         # stream stopped once PX4 was in AUTO_LAND


def test_landing_disarm_and_landed_in_same_update(ctx, io, cfg, clock, feed):
    landing_setup(ctx, feed, clock)
    feed.at(3.0, lambda: feed.set(vehicle_status=status(False, POSCTL),
                                  land_detected=land_detected(True)))
    assert run(Landing, ctx, io, cfg, clock) == 'landed'


def test_landing_disarm_just_before_landed_flag(ctx, io, cfg, clock, feed):
    landing_setup(ctx, feed, clock)
    feed.at(3.0, lambda: feed.set(vehicle_status=status(False, POSCTL)))
    feed.at(3.0 + cfg.landed_disarm_grace_s / 2,
            lambda: feed.set(land_detected=land_detected(True)))
    assert run(Landing, ctx, io, cfg, clock) == 'landed'


def test_landing_disarm_without_landed_aborts(ctx, io, cfg, clock, feed):
    landing_setup(ctx, feed, clock)
    feed.at(3.0, lambda: feed.set(vehicle_status=status(False, POSCTL)))
    t0 = clock.now()
    assert run(Landing, ctx, io, cfg, clock) == 'aborted'
    assert ctx.abort_cause == AbortCause.PX4_CONTROL.value
    assert ctx.pending_reason == 'disarmed before landing'
    assert clock.now() - t0 > 3.0 + cfg.landed_disarm_grace_s


def test_landing_resends_land_until_px4_lands(ctx, io, cfg, clock, feed):
    airborne(ctx, feed, clock)                    # stays in OFFBOARD at first
    feed.at(2.5, lambda: feed.set(vehicle_status=status(True, AUTO_LAND)))
    feed.at(4.0, lambda: feed.set(land_detected=land_detected(True),
                                  vehicle_status=status(False, POSCTL)))
    assert run(Landing, ctx, io, cfg, clock) == 'landed'
    assert io.names().count('land') == 3          # t=0, 1, 2


def test_landing_disarms_if_px4_does_not(ctx, io, cfg, clock, feed):
    landing_setup(ctx, feed, clock)
    feed.at(2.0, lambda: feed.set(land_detected=land_detected(True)))
    feed.at(2.5, lambda: feed.set(vehicle_status=status(False, POSCTL))
            if 'disarm' in io.names() else None)
    assert run(Landing, ctx, io, cfg, clock) == 'landed'
    assert 'disarm' in io.names()


def test_landing_pilot_takeover_aborts(ctx, io, cfg, clock, feed):
    landing_setup(ctx, feed, clock)
    feed.at(1.0, lambda: feed.set(vehicle_status=status(True, POSCTL)))
    assert run(Landing, ctx, io, cfg, clock) == 'aborted'
    assert ctx.abort_cause == AbortCause.PX4_CONTROL.value


# ---- RETURN_CONFIRMED ---------------------------------------------------------------------------

def test_return_confirmed_stops_commands(ctx, io, cfg, clock):
    assert run(ReturnConfirmed, ctx, io, cfg, clock) == MISSION_COMPLETE
    assert io.names() == ['clear_setpoint']


# ---- ABORTED ------------------------------------------------------------------------------------

def test_aborted_px4_control_stops_commanding_at_once(ctx, io, cfg, clock, feed):
    airborne(ctx, feed, clock, nav=POSCTL)
    ctx.abort_cause, ctx.abort_from = AbortCause.PX4_CONTROL.value, S.SEARCHING
    assert run(Aborted, ctx, io, cfg, clock) == MISSION_ABORTED
    assert io.names() == ['clear_setpoint']


def test_aborted_airborne_holds_then_lands_in_place(ctx, io, cfg, clock, feed):
    airborne(ctx, feed, clock, z=-1.6)
    ctx.abort_cause, ctx.abort_from = AbortCause.INTENT.value, S.SEARCHING
    t0 = clock.now()

    def px4_lands():
        assert clock.now() - t0 >= cfg.land_after_hold_s
        feed.set(vehicle_status=status(True, AUTO_LAND))
        feed.at(clock.now() - feed.t0 + 2.0, lambda: feed.set(
            land_detected=land_detected(True), vehicle_status=status(False, POSCTL)))

    original_land = io.land
    io.land = lambda: (original_land(), px4_lands())
    assert run(Aborted, ctx, io, cfg, clock) == MISSION_ABORTED
    assert io.calls[0] == ('set_setpoint', (1.0, 2.0, -1.6, 0.0))   # hold here
    assert io.names().count('land') == 1


def test_aborted_on_ground_disarms(ctx, io, cfg, clock, feed):
    feed.set(vehicle_status=status(True, POSCTL), local_position=position())
    feed.at(1.5, lambda: feed.set(vehicle_status=status(False, POSCTL)))
    ctx.abort_cause, ctx.abort_from = AbortCause.PREFLIGHT_FAILED.value, S.PREFLIGHT
    assert run(Aborted, ctx, io, cfg, clock) == MISSION_ABORTED
    assert io.names() == ['clear_setpoint', 'disarm', 'disarm']


def test_aborted_from_standby_finishes_at_once(ctx, io, cfg, clock, feed):
    feed.set(vehicle_status=status(False))
    ctx.abort_cause, ctx.abort_from = AbortCause.INTENT.value, S.STANDBY
    assert run(Aborted, ctx, io, cfg, clock) == MISSION_ABORTED
    assert 'disarm' not in io.names() and 'land' not in io.names()


def test_aborted_stops_if_pilot_takes_over_during_hold(ctx, io, cfg, clock, feed):
    airborne(ctx, feed, clock)
    ctx.abort_cause, ctx.abort_from = AbortCause.POSE.value, S.TAKEOFF
    feed.at(1.0, lambda: feed.set(vehicle_status=status(True, POSCTL)))
    assert run(Aborted, ctx, io, cfg, clock) == MISSION_ABORTED
    assert 'land' not in io.names()
    assert io.names()[-1] == 'clear_setpoint'


# ---- whole machine ------------------------------------------------------------------------------

def transitions(io):
    return [text.split(' -> ')[1].split(':')[0] for level, text in io.logs
            if level == 'info' and ' -> ' in text]


def test_full_flight_standby_to_return_confirmed(ctx, io, cfg, clock, fake_px4,
                                                 fake_perception):
    flight(ctx, io, cfg, clock, fake_px4, fake_perception, 'yellow')
    sm = build_state_machine(ctx, io, cfg, clock)
    assert sm.execute() == MISSION_COMPLETE
    assert transitions(io) == [S.STANDBY, S.PREFLIGHT, S.TAKEOFF, S.SEARCHING, S.APPROACHING,
                               S.DESCENDING, S.ON_DIE_TEST, S.LIFTOFF, S.RETURNING, S.LANDING,
                               S.RETURN_CONFIRMED]
    assert ctx.state == S.RETURN_CONFIRMED and ctx.previous_state == S.LANDING
    assert ctx.reason == 'landed and disarmed'
    assert io.setpoint is None


def test_full_flight_abort_intent_while_approaching(ctx, io, cfg, clock, fake_px4, fake_perception):
    px4, _ = flight(ctx, io, cfg, clock, fake_px4, fake_perception, 'red')

    def abort_when_hovering(now):
        if ctx.state == S.APPROACHING and not ctx.intent_queue and ctx.last_intent_id == 1:
            ctx.update_input('intent', intent('abort', '', 2), now)
    clock.hooks.append(abort_when_hovering)
    assert build_state_machine(ctx, io, cfg, clock).execute() == MISSION_ABORTED
    assert transitions(io)[-2:] == [S.APPROACHING, S.ABORTED]
    assert (ctx.last_intent_id, ctx.last_intent_result) == (2, 'applied')
    assert ctx.reason == 'abort intent'
    assert not px4.armed and px4.landed           # held, then landed in place


def test_full_flight_with_pilot_arm(ctx, io, make_cfg, clock, fake_px4, fake_perception, feed):
    """The pilot arms for takeoff, and again for LIFTOFF after PX4's auto-disarm on the die;
    GNC never sends arm."""
    cfg = make_cfg(require_pilot_arm=True)
    px4, _ = flight(ctx, io, cfg, clock, fake_px4, fake_perception, 'black')
    feed.at(6.0, px4.pilot_arm)
    rearmed = []

    def pilot_rearms_on_the_die(now):
        if ctx.state == S.LIFTOFF and not px4.armed and not rearmed:
            rearmed.append(now)
            px4.pilot_arm()
    clock.hooks.append(pilot_rearms_on_the_die)
    assert build_state_machine(ctx, io, cfg, clock).execute() == MISSION_COMPLETE
    assert 'arm' not in io.names()
    assert rearmed                                  # LIFTOFF waited for the pilot on the die


def test_preempt_stops_the_machine(ctx, io, cfg, clock):
    sm = build_state_machine(ctx, io, cfg, clock)
    clock.hooks.append(lambda now: sm.request_preempt() if now > clock.limit - 590 else None)
    assert sm.execute() == PREEMPTED
