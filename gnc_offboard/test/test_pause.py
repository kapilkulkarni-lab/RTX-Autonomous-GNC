"""PAUSED: pause and resume from every pausable state, the ground/air behavior, the resume rule,
and the global checks that still apply while paused."""
import pytest

from conftest import (AUTO_LAND, OFFBOARD, POSCTL, battery, intent, land_detected, position,
                      status, transitions_of)
from gnc_offboard.mission import state_names as S
from gnc_offboard.mission.intents import RegionFields
from gnc_offboard.mission.mission_manager_smach import (
    ABORTED_OUT, MISSION_ABORTED, MISSION_COMPLETE, RESUME_NEW_INTENT, RETURN_OUT, AbortCause,
    Paused, TRANSITIONS, build_state_machine, resume_outcome)


def msg(action, color='', intent_id=1, **fields):
    m = intent(action, color, intent_id)
    for k, v in fields.items():
        setattr(m, k, v)
    return m


class Ids:
    def __init__(self):
        self.n = 1

    def __call__(self):
        self.n += 1
        return self.n


@pytest.fixture
def ids():
    return Ids()


def fly_with_pause(ctx, io, cfg, clock, fake_px4, script, ids, pause_when, during=None,
                   resume_after_s=2.0, **px4_kw):
    """Full flight; pause when `pause_when(ctx, px4)`; optionally send `during` intents while
    paused; resume after `resume_after_s`. Returns (outcome, px4, calls made while paused)."""
    px4 = fake_px4(io, **px4_kw)
    ctx.update_input('intent', msg('retrieve', 'blue', 1), clock.now())
    marks = {}
    script.when(lambda c: pause_when(c, px4), msg('pause', intent_id=ids()),
                then=lambda: marks.update(paused_at=clock.now(), calls_at=len(io.calls)))
    script.when(lambda c: c.state == S.PAUSED, None, then=lambda: marks.update(z=px4.z))
    for m in during or []:
        script.when(lambda c: c.state == S.PAUSED, m)
    script.when(lambda c: c.state == S.PAUSED and clock.now() - marks['paused_at']
                >= resume_after_s, None,
                then=lambda: marks.update(calls_end=len(io.calls), z_end=px4.z))
    script.when(lambda c: True, msg('resume', intent_id=ids()))
    outcome = build_state_machine(ctx, io, cfg, clock).execute()
    paused_calls = [n for n, _ in io.calls[marks['calls_at']:marks['calls_end']]
                    if n != 'request_state_publish']
    return outcome, px4, paused_calls, marks


def published_while_paused(rio):
    return [m for m in rio.published if m.state == S.PAUSED]


# ---- pause and resume (no new intent) from every pausable state ---------------------------------

def test_pause_resume_standby(ctx, rio, cfg, clock, fake_px4, script, ids):
    fake_px4(rio)
    script.when(lambda c: c.state == S.STANDBY, msg('pause', intent_id=1))
    script.when(lambda c: c.state == S.PAUSED, msg('resume', intent_id=2))
    script.when(lambda c: c.state == S.STANDBY and c.last_intent_id == 2,
                msg('retrieve', 'red', 3))
    assert build_state_machine(ctx, rio, cfg, clock).execute() == MISSION_COMPLETE
    assert transitions_of(rio)[:4] == [S.STANDBY, S.PAUSED, S.STANDBY, S.PREFLIGHT]
    assert 'arm' not in [n for n, _ in rio.calls[:rio.names().index('set_setpoint')]]
    paused = published_while_paused(rio)
    assert paused and all(m.previous_state == S.STANDBY for m in paused)


def test_pause_resume_preflight_disarms_then_starts_over(ctx, rio, cfg, clock, fake_px4, script,
                                                        ids):
    outcome, px4, during, _ = fly_with_pause(
        ctx, rio, cfg, clock, fake_px4, script, ids,
        pause_when=lambda c, p: c.state == S.PREFLIGHT and p.armed)
    assert outcome == MISSION_COMPLETE
    assert 'disarm' in during and 'arm' not in during and 'set_offboard' not in during
    assert during[0] == 'clear_setpoint'            # stream stopped
    names = transitions_of(rio)
    assert names[:5] == [S.STANDBY, S.PREFLIGHT, S.PAUSED, S.PREFLIGHT, S.TAKEOFF]
    assert all(m.previous_state == S.PREFLIGHT for m in published_while_paused(rio))


def test_pause_resume_takeoff_hovers_where_it_is(ctx, rio, cfg, clock, fake_px4, script, ids):
    outcome, px4, during, marks = fly_with_pause(
        ctx, rio, cfg, clock, fake_px4, script, ids,
        pause_when=lambda c, p: c.state == S.TAKEOFF and p.z < -0.6)
    assert outcome == MISSION_COMPLETE
    assert marks['z_end'] == pytest.approx(marks['z'], abs=0.06)   # held, not climbing
    assert marks['z'] > -1.0                         # paused mid-climb
    names = transitions_of(rio)
    assert names[3:6] == [S.PAUSED, S.TAKEOFF, S.HOVER_TEST]       # TAKEOFF starts over


def test_pause_resume_hover_test_restarts_its_timer(ctx, rio, cfg, clock, fake_px4, script, ids):
    entered = {}
    clock.hooks.append(lambda now: entered.setdefault(len(transitions_of(rio)), (
        transitions_of(rio)[-1] if transitions_of(rio) else '', now)))
    outcome, px4, during, marks = fly_with_pause(
        ctx, rio, cfg, clock, fake_px4, script, ids,
        pause_when=lambda c, p: c.state == S.HOVER_TEST)
    assert outcome == MISSION_COMPLETE
    assert transitions_of(rio)[3:7] == [S.HOVER_TEST, S.PAUSED, S.HOVER_TEST, S.LANDING]
    assert 'land' not in during
    # The resumed hover lasted a full hover_test_s.
    resumed = next(t for name, t in entered.values() if name == S.HOVER_TEST and t > marks[
        'paused_at'])
    landing = next(t for name, t in entered.values() if name == S.LANDING)
    assert landing - resumed >= cfg.hover_test_s - 0.1


def test_pause_while_descending_hovers_then_resumes_landing(ctx, rio, cfg, clock, fake_px4,
                                                             script, ids):
    outcome, px4, during, marks = fly_with_pause(
        ctx, rio, cfg, clock, fake_px4, script, ids,
        pause_when=lambda c, p: c.state == S.LANDING and p.nav == AUTO_LAND and p.z > -1.0)
    assert outcome == MISSION_COMPLETE
    assert 'set_offboard' in during and 'set_setpoint' in during    # back to offboard, hover
    assert marks['z_end'] == pytest.approx(marks['z'], abs=0.1)      # stopped descending
    assert marks['z_end'] < -0.2                                     # still in the air
    assert transitions_of(rio)[-3:] == [S.PAUSED, S.LANDING, S.RETURN_CONFIRMED]
    assert all(m.previous_state == S.LANDING for m in published_while_paused(rio))


def test_pause_after_landed_stays_on_the_ground(ctx, rio, cfg, clock, fake_px4, script, ids):
    """Pause in LANDING once PX4 reports landed: no offboard request, no setpoints, no arming
    change. Resume goes back to LANDING to finish."""
    outcome, px4, during, marks = fly_with_pause(
        ctx, rio, cfg, clock, fake_px4, script, ids,
        pause_when=lambda c, p: c.state == S.LANDING and p.landed and p.armed,
        resume_after_s=3.0, auto_disarm_s=1.0)
    assert outcome == MISSION_COMPLETE
    assert not {'set_offboard', 'set_setpoint', 'arm', 'disarm'} & set(during)
    assert marks['z_end'] == 0.0 and px4.landed
    assert transitions_of(rio)[-3:] == [S.PAUSED, S.LANDING, S.RETURN_CONFIRMED]


def test_paused_landing_px4_touches_down_before_offboard(ctx, io, cfg, clock, feed):
    """Paused while descending, but PX4 lands before offboard is accepted: stay landed."""
    ctx.offboard_engaged, ctx.paused_from = True, S.LANDING
    feed.set(vehicle_status=status(True, AUTO_LAND), local_position=position(z=-0.3),
             land_detected=land_detected(False), battery_status=battery(0.9))
    feed.at(0.5, lambda: feed.set(land_detected=land_detected(True)))
    feed.at(2.0, lambda: feed.set(vehicle_status=status(False, POSCTL)))
    clock.limit = clock.now() + 6.0
    state = Paused(ctx, io, cfg, clock)
    with pytest.raises(TimeoutError):               # stays paused, no abort
        state.execute()
    assert state.hold == 'landed'
    assert io.names().count('set_offboard') <= 1      # stopped asking once landed
    assert io.names()[-1] == 'clear_setpoint'


# ---- resume rule --------------------------------------------------------------------------------

def test_new_intent_while_paused_airborne_resumes_to_hover_test(ctx, rio, cfg, clock, fake_px4,
                                                              script, ids):
    """TEMPORARY: SEARCHING in Milestone 5. The new color applies at resume, not before."""
    targets = []
    clock.hooks.append(lambda now: targets.append(ctx.active_target)
                       if ctx.state == S.PAUSED else None)
    outcome, px4, during, marks = fly_with_pause(
        ctx, rio, cfg, clock, fake_px4, script, ids,
        pause_when=lambda c, p: c.state == S.TAKEOFF and p.z < -0.6,
        during=[msg('retrieve', 'yellow', 50, avoid_mask=2)])
    assert outcome == MISSION_COMPLETE
    assert set(targets) == {'blue'}                 # stored while paused
    assert ctx.active_target == 'yellow' and ctx.regions == RegionFields(avoid_mask=2)
    names = transitions_of(rio)
    i = names.index(S.PAUSED)
    assert names[i + 1] == S.HOVER_TEST             # not back to TAKEOFF
    assert ctx.reason != '' and 'resumed with new intent' in [
        m.reason for m in rio.published if m.state == S.HOVER_TEST]


def test_update_only_while_paused_counts_as_new_intent(ctx, io, cfg, clock, feed):
    ctx.offboard_engaged, ctx.paused_from, ctx.active_target = True, S.HOVER_TEST, 'blue'
    feed.set(vehicle_status=status(True, OFFBOARD), local_position=position(z=-1.5),
             land_detected=land_detected(False), battery_status=battery(0.9))
    ctx.update_input('intent', msg('update_only', 'red', 2, target_quadrant=4), clock.now())
    feed.at(0.5, lambda: ctx.update_input('intent', msg('resume', intent_id=3), clock.now()))
    assert Paused(ctx, io, cfg, clock).execute() == RESUME_NEW_INTENT
    assert ctx.regions == RegionFields(target_quadrant=4, quadrant_hint_color='red')
    assert ctx.active_target == 'blue'


def test_resume_extra_fields_ignored(ctx, io, cfg, clock, feed):
    ctx.offboard_engaged, ctx.paused_from, ctx.active_target = True, S.HOVER_TEST, 'blue'
    feed.set(vehicle_status=status(True, OFFBOARD), local_position=position(z=-1.5),
             land_detected=land_detected(False), battery_status=battery(0.9))
    ctx.update_input('intent', msg('resume', 'black', 3, avoid_mask=15, target_quadrant=1),
                     clock.now())
    assert Paused(ctx, io, cfg, clock).execute() == resume_outcome(S.HOVER_TEST)
    assert ctx.active_target == 'blue' and ctx.regions is None
    assert (ctx.last_intent_id, ctx.last_intent_result) == (3, 'applied')


def test_stored_first_retrieve_in_standby_starts_preflight(ctx, io, cfg, clock):
    ctx.paused_from = S.STANDBY
    ctx.update_input('intent', msg('retrieve', 'red', 1), clock.now())
    ctx.update_input('intent', msg('resume', intent_id=2), clock.now())
    assert Paused(ctx, io, cfg, clock).execute() == resume_outcome(S.PREFLIGHT)
    assert ctx.active_target == 'red' and ctx.mission_start_time == clock.now()


@pytest.mark.parametrize('paused_from', [S.STANDBY, S.PREFLIGHT])
def test_new_intent_paused_on_ground_resumes_to_paused_from(paused_from, ctx, io, cfg, clock,
                                                            feed):
    ctx.paused_from = paused_from
    if paused_from == S.PREFLIGHT:
        ctx.active_target = 'blue'
    feed.set(vehicle_status=status(False), local_position=position())
    ctx.update_input('intent', msg('update_only', intent_id=1, avoid_mask=8), clock.now())
    ctx.update_input('intent', msg('resume', intent_id=2), clock.now())
    assert Paused(ctx, io, cfg, clock).execute() == resume_outcome(paused_from)
    assert ctx.regions == RegionFields(avoid_mask=8)


def test_resume_targets_are_wired():
    wiring = TRANSITIONS[S.PAUSED]
    for state in Paused.PAUSABLE:
        assert wiring[resume_outcome(state)] == state
    assert wiring[RESUME_NEW_INTENT] == S.HOVER_TEST          # TEMPORARY until Milestone 5
    for state in Paused.PAUSABLE:
        assert TRANSITIONS[state]['pause'] == S.PAUSED
    for end in S.END_STATES:
        assert 'pause' not in TRANSITIONS[end]


# ---- global checks while paused -----------------------------------------------------------------

def paused_airborne(ctx, feed, clock, paused_from=S.HOVER_TEST):
    ctx.offboard_engaged, ctx.paused_from, ctx.active_target = True, paused_from, 'blue'
    ctx.start_position = (0.0, 0.0, 0.0, 0.0)
    ctx.mission_start_time = clock.now()
    feed.set(vehicle_status=status(True, OFFBOARD), local_position=position(z=-1.5),
             land_detected=land_detected(False), battery_status=battery(0.9))


def test_abort_while_paused_airborne_holds_then_lands(ctx, io, cfg, clock, fake_px4, script,
                                                      ids):
    px4 = fake_px4()
    ctx.update_input('intent', msg('retrieve', 'blue', 1), clock.now())
    script.when(lambda c: c.state == S.HOVER_TEST, msg('pause', intent_id=2))
    script.when(lambda c: c.state == S.PAUSED, msg('abort', intent_id=3))
    assert build_state_machine(ctx, io, cfg, clock).execute() == MISSION_ABORTED
    assert transitions_of(io)[-2:] == [S.PAUSED, S.ABORTED]
    assert ctx.abort_from == S.HOVER_TEST
    assert px4.landed and not px4.armed             # held, then landed in place


def test_abort_while_paused_on_ground(ctx, io, cfg, clock, feed):
    ctx.paused_from = S.STANDBY
    ctx.update_input('intent', msg('abort', intent_id=2), clock.now())
    assert Paused(ctx, io, cfg, clock).execute() == ABORTED_OUT
    assert ctx.abort_from == S.STANDBY


@pytest.mark.parametrize('paused_from', [S.TAKEOFF, S.HOVER_TEST])
def test_return_to_start_while_paused_airborne(paused_from, ctx, io, cfg, clock, feed):
    paused_airborne(ctx, feed, clock, paused_from)
    ctx.update_input('intent', msg('return_to_start', intent_id=2), clock.now())
    assert Paused(ctx, io, cfg, clock).execute() == RETURN_OUT


@pytest.mark.parametrize('landed', [False, True])
def test_return_to_start_while_paused_from_landing_continues_landing(landed, ctx, io, cfg,
                                                                     clock, feed):
    """Leaves the pause and continues LANDING (applied), descending or already landed."""
    paused_airborne(ctx, feed, clock, S.LANDING)
    feed.set(land_detected=land_detected(landed))
    ctx.update_input('intent', msg('return_to_start', intent_id=2), clock.now())
    assert Paused(ctx, io, cfg, clock).execute() == resume_outcome(S.LANDING)
    assert (ctx.last_intent_id, ctx.last_intent_result) == (2, 'applied')
    assert ctx.pending_reason == 'return to start: landing'


def test_return_to_start_paused_while_descending_lands(ctx, rio, cfg, clock, fake_px4, script,
                                                       ids):
    px4 = fake_px4(rio)
    ctx.update_input('intent', msg('retrieve', 'blue', 1), clock.now())
    script.when(lambda c: c.state == S.LANDING and px4.nav == AUTO_LAND and px4.z > -1.0,
                msg('pause', intent_id=2))
    script.when(lambda c: c.state == S.PAUSED and px4.nav == OFFBOARD,
                msg('return_to_start', intent_id=3))
    assert build_state_machine(ctx, rio, cfg, clock).execute() == MISSION_COMPLETE
    assert transitions_of(rio)[-4:] == [S.LANDING, S.PAUSED, S.LANDING, S.RETURN_CONFIRMED]
    assert px4.landed and not px4.armed


def test_return_to_start_paused_after_landed_finishes(ctx, rio, cfg, clock, fake_px4, script,
                                                      ids):
    """Already landed: resumes LANDING, which disarms and ends in RETURN_CONFIRMED; it never
    sends land or asks for offboard again."""
    px4 = fake_px4(rio, auto_disarm_s=1000.0)      # PX4 auto-disarm off: GNC disarms
    px4.accept_disarm = False                      # still armed on the ground when paused
    ctx.update_input('intent', msg('retrieve', 'blue', 1), clock.now())
    marks = {}
    script.when(lambda c: c.state == S.LANDING and px4.landed and px4.armed,
                msg('pause', intent_id=2))
    script.when(lambda c: c.state == S.PAUSED, msg('return_to_start', intent_id=3),
                then=lambda: (marks.update(calls_at=len(rio.calls)),
                              setattr(px4, 'accept_disarm', True)))
    assert build_state_machine(ctx, rio, cfg, clock).execute() == MISSION_COMPLETE
    assert transitions_of(rio)[-4:] == [S.LANDING, S.PAUSED, S.LANDING, S.RETURN_CONFIRMED]
    after = [n for n, _ in rio.calls[marks['calls_at']:]]
    assert 'disarm' in after
    assert not {'land', 'set_offboard', 'set_setpoint', 'arm'} & set(after)
    assert px4.landed and not px4.armed and px4.z == 0.0


def test_return_to_start_while_paused_from_preflight(ctx, io, cfg, clock, feed):
    ctx.paused_from, ctx.mission_start_time = S.PREFLIGHT, clock.now()
    feed.set(vehicle_status=status(False), local_position=position())
    ctx.update_input('intent', msg('return_to_start', intent_id=2), clock.now())
    assert Paused(ctx, io, cfg, clock).execute() == RETURN_OUT   # -> LANDING on the ground


def test_return_to_start_while_paused_from_standby_no_change(ctx, io, cfg, clock):
    ctx.paused_from = S.STANDBY
    ctx.update_input('intent', msg('return_to_start', intent_id=2), clock.now())
    clock.limit = clock.now() + 2.0
    with pytest.raises(TimeoutError):
        Paused(ctx, io, cfg, clock).execute()
    assert ctx.last_intent_result == 'applied'


def test_low_battery_while_paused_returns(ctx, io, cfg, clock, feed):
    paused_airborne(ctx, feed, clock)
    feed.at(1.0, lambda: feed.set(battery_status=battery(0.1)))
    assert Paused(ctx, io, cfg, clock).execute() == RETURN_OUT
    assert ctx.pending_reason.startswith('battery')


def test_mission_clock_keeps_running_while_paused(ctx, io, cfg, clock, feed):
    paused_airborne(ctx, feed, clock)
    ctx.mission_start_time = clock.now() - cfg.force_return_at_s + 1.0
    t0 = clock.now()
    assert Paused(ctx, io, cfg, clock).execute() == RETURN_OUT
    assert clock.now() - t0 == pytest.approx(1.0, abs=0.06)
    assert ctx.pending_reason == 'mission time up: returning'


def test_pose_loss_while_paused_aborts(ctx, io, cfg, clock, feed):
    paused_airborne(ctx, feed, clock)
    feed.at(1.0, lambda: feed.set(local_position=position(z=-1.5, valid=False)))
    assert Paused(ctx, io, cfg, clock).execute() == ABORTED_OUT
    assert ctx.abort_cause == AbortCause.POSE.value


def test_px4_takeover_while_paused_aborts(ctx, io, cfg, clock, feed):
    paused_airborne(ctx, feed, clock)
    feed.at(1.0, lambda: feed.set(vehicle_status=status(True, POSCTL)))
    assert Paused(ctx, io, cfg, clock).execute() == ABORTED_OUT
    assert ctx.abort_cause == AbortCause.PX4_CONTROL.value


def test_paused_on_ground_ignores_battery_and_pose(ctx, io, cfg, clock, feed):
    ctx.paused_from = S.STANDBY
    feed.set(vehicle_status=status(False), local_position=position(valid=False),
             battery_status=battery(0.05))
    clock.limit = clock.now() + 5.0
    with pytest.raises(TimeoutError):
        Paused(ctx, io, cfg, clock).execute()
    assert io.calls == []                           # STANDBY: just shows PAUSED


def test_paused_hover_holds_current_position(ctx, io, cfg, clock, feed):
    paused_airborne(ctx, feed, clock)
    feed.set(local_position=position(1.0, 2.0, -1.2, heading=0.4))
    clock.limit = clock.now() + 1.0
    with pytest.raises(TimeoutError):
        Paused(ctx, io, cfg, clock).execute()
    assert io.names() == ['set_setpoint']
    assert io.calls[0][1] == pytest.approx((1.0, 2.0, -1.2, 0.4))


def test_read_chip_and_resume_while_paused(ctx, io, cfg, clock, feed):
    paused_airborne(ctx, feed, clock)
    ctx.update_input('intent', msg('read_chip', 'blue', 7), clock.now())
    state = Paused(ctx, io, cfg, clock)
    state.entered_at = clock.now()
    state.on_entry(ctx.snapshot(), clock.now())
    assert state.tick(clock.now()) is None
    assert (ctx.last_intent_id, ctx.last_intent_result) == (7, 'rejected')
    assert ctx.stored_intents == []                 # a rejected intent is not a new intent
