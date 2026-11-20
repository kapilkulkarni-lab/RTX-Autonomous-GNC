"""Milestone 6: DESCENDING, ON_DIE_TEST (temporary), LIFTOFF, RETURNING, and the on-die global
behavior, against FakePx4 (ground contact, land detector, PX4's auto-disarm after landing in
offboard) and FakePerception (blind height, optional noise)."""
import math
from types import SimpleNamespace

import pytest

from conftest import (OFFBOARD, POSCTL, battery, flight, heartbeat, intent, land_detected,
                      position, rangefinder, status, transitions_of)
from gnc_offboard.mission import state_names as S
from gnc_offboard.mission.mission_manager_smach import (
    MISSION_ABORTED, MISSION_COMPLETE, RETARGET_SEEN, RETARGET_UNSEEN, RETURN_OUT, TRANSITIONS,
    AbortCause, Descending, Liftoff, OnDieTest, Returning, build_state_machine)

RED = (2.0, 2.0)


def lines(io):
    return [t for lvl, t in io.logs if lvl == 'info' and ' -> ' in t]


def run_to_end(ctx, io, cfg, clock):
    return build_state_machine(ctx, io, cfg, clock).execute()


def entered(io, state):
    return [t for t in lines(io) if t.split(' -> ')[1].split(':')[0] == state]


def relative(ctx, px4, die, bias=(0.0, 0.0), valid=True):
    """target_relative as Perception would report it, plus a constant bias."""
    x, y, _ = ctx.field_config.frame.to_field(px4.x, px4.y, px4.z)
    return SimpleNamespace(valid=valid, offset=SimpleNamespace(
        x=die[0] - x + bias[0], y=die[1] - y + bias[1], z=px4.z))


# ---- full mission -------------------------------------------------------------------------------

def test_full_mission_lands_on_the_die_and_returns_to_start(ctx, io, cfg, clock, fake_px4,
                                                           fake_perception):
    px4, _ = flight(ctx, io, cfg, clock, fake_px4, fake_perception, 'red')
    assert run_to_end(ctx, io, cfg, clock) == MISSION_COMPLETE
    assert transitions_of(io)[5:] == [S.DESCENDING, S.ON_DIE_TEST, S.LIFTOFF, S.RETURNING,
                                      S.LANDING, S.RETURN_CONFIRMED]
    home = ctx.field_config.frame.to_field(px4.x, px4.y, px4.z)
    assert math.dist(home[:2], ctx.field_config.start_center) <= cfg.acceptance_radius_m + 0.01
    assert ctx.touchdowns == 1
    # PX4 auto-disarmed on the die (on_die_test_s > COM_DISARM_LAND): LIFTOFF re-armed.
    liftoff = io.names().index('arm', io.names().index('arm') + 1)
    assert liftoff > 0


def test_descent_stages_and_touchdown_on_rangefinder(ctx, io, cfg, clock, fake_px4,
                                                     fake_perception):
    px4, _ = flight(ctx, io, cfg, clock, fake_px4, fake_perception, 'red')
    heights = []
    clock.hooks.append(lambda now: heights.append((now, -px4.z))
                       if ctx.state == S.DESCENDING else None)
    run_to_end(ctx, io, cfg, clock)
    # It paused at each stage height (several samples within tolerance of each).
    for stage in cfg.descent_stages_m:
        assert sum(abs(h - stage) < 0.02 for _, h in heights) >= 3, stage
    # Descending between stages never faster than descent_rate_mps (with a little slack).
    rates = [(h0 - h1) / (t1 - t0) for (t0, h0), (t1, h1) in zip(heights, heights[1:])
             if t1 > t0 and h0 < cfg.hover_height_m - 0.05]
    assert max(rates) <= cfg.descent_rate_mps * 1.6
    assert 'DESCENDING -> ON_DIE_TEST: touchdown on the die' in lines(io)


def test_landing_accuracy_with_perception_noise(ctx, io, make_cfg, clock, fake_px4,
                                                fake_perception):
    """20 descents with 2 cm noise on target_relative, as in camera_fov's default."""
    cfg = make_cfg(descent_test_count=20, on_die_test_s=1.0)
    px4, _ = flight(ctx, io, cfg, clock, fake_px4, fake_perception, 'red', noise_m=0.02)
    clock.limit += 3000
    offsets = []
    clock.hooks.append(lambda now: offsets.append(math.dist(
        ctx.field_config.frame.to_field(px4.x, px4.y, px4.z)[:2], RED))
        if ctx.state == S.ON_DIE_TEST and (not offsets or len(offsets) < ctx.touchdowns) else None)
    assert run_to_end(ctx, io, cfg, clock) == MISSION_COMPLETE
    assert ctx.touchdowns == 20 and len(offsets) == 20
    assert sum(offsets) / 20 < 0.03 and max(offsets) < 0.06
    assert len(entered(io, S.ON_DIE_TEST)) == 20


# ---- DESCENDING: retry and failed ---------------------------------------------------------------

def test_off_center_retries_then_fails_back_to_approaching(ctx, io, cfg, clock, fake_px4,
                                                          fake_perception, script):
    """Perception keeps reporting the die 7 cm off, alternating sides: that passes stage 1
    (10 cm) but never stage 2 (5 cm)."""
    px4, _ = flight(ctx, io, cfg, clock, fake_px4, fake_perception, 'red')
    side = [1.0]

    def jitter(now):
        if ctx.state == S.DESCENDING:
            side[0] = -side[0]
            ctx.update_input('target_relative', SimpleNamespace(valid=True, offset=SimpleNamespace(
                x=0.07 * side[0], y=0.0, z=px4.z)), now)
    clock.hooks.append(jitter)
    script.when(lambda c: c.state == S.APPROACHING and c.die_failures, intent('abort', '', 2))
    assert run_to_end(ctx, io, cfg, clock) == MISSION_ABORTED
    descents = [t for t in lines(io) if t.startswith('DESCENDING -> ')]
    assert descents == [
        'DESCENDING -> DESCENDING: off center at 0.50 m: retry 1 of 2',
        'DESCENDING -> DESCENDING: off center at 0.50 m: retry 2 of 2',
        'DESCENDING -> APPROACHING: descent failed: off center at 0.50 m (1 of 2)']


def test_die_lost_mid_descent_retries(ctx, io, cfg, clock, fake_px4, fake_perception, script):
    px4, perception = flight(ctx, io, cfg, clock, fake_px4, fake_perception, 'red')
    script.when(lambda c: c.state == S.DESCENDING and -px4.z < 0.8, None,
                then=lambda: setattr(perception, 'drop', True))
    script.when(lambda c: c.descent_retries == 1, None,
                then=lambda: setattr(perception, 'drop', False))
    assert run_to_end(ctx, io, cfg, clock) == MISSION_COMPLETE
    assert 'DESCENDING -> DESCENDING: die lost: retry 1 of 2' in lines(io)
    assert ctx.touchdowns == 1


def test_retry_climbs_back_to_hover_height(ctx, io, cfg, clock, fake_px4, fake_perception,
                                           script):
    px4, perception = flight(ctx, io, cfg, clock, fake_px4, fake_perception, 'red')
    script.when(lambda c: c.state == S.DESCENDING and -px4.z < 0.6, None,
                then=lambda: setattr(perception, 'drop', True))
    heights = []
    script.when(lambda c: len(entered(io, S.DESCENDING)) == 2, None,      # re-entered: retry
                then=lambda: (heights.append(-px4.z), setattr(perception, 'drop', False)))
    run_to_end(ctx, io, cfg, clock)
    assert heights and heights[0] == pytest.approx(cfg.hover_height_m, abs=cfg.altitude_tol_m)


def test_wrong_color_on_descent_excludes_the_die(ctx, io, cfg, clock, fake_px4, fake_perception,
                                                 script):
    px4, _ = flight(ctx, io, cfg, clock, fake_px4, fake_perception, 'red')
    injected, excluded = [], []

    def wrong_once(now):
        if ctx.state == S.DESCENDING and -px4.z < 0.7 and not injected:
            injected.append(now)
            ctx.update_input('color_check', SimpleNamespace(color_guess='blue', confidence=1.0),
                             now)
        if ctx.given_up_dice and not excluded:
            excluded.append(list(ctx.given_up_dice))
    clock.hooks.append(wrong_once)
    run_to_end(ctx, io, cfg, clock)
    assert 'DESCENDING -> APPROACHING: wrong color (blue) on descent' in lines(io)
    assert math.dist(excluded[0][0][:2], RED) < 0.05             # excluded at once


# ---- DESCENDING: unit -----------------------------------------------------------------------------

def hover_over_die(ctx, feed, clock, height=1.0, die=RED, **inputs):
    ctx.active_target = 'red'
    ctx.offboard_engaged = True
    ctx.start_position = (0.0, 0.0, 0.0, 0.0)
    ctx.mission_start_time = clock.now()
    ctx.target_centroid = (*die, 0.0)
    n, e, _ = ctx.field_config.frame.to_ned(*die)
    feed.set(vehicle_status=status(True, OFFBOARD), local_position=position(n, e, -height),
             land_detected=land_detected(False), battery_status=battery(0.9),
             perception_heartbeat=heartbeat(), distance_sensor=rangefinder(height), **inputs)
    return n, e


def test_no_target_fails(ctx, io, cfg, clock, feed):
    hover_over_die(ctx, feed, clock)
    ctx.target_centroid = None
    assert Descending(ctx, io, cfg, clock).execute() == 'failed'
    assert ctx.pending_reason == 'target unknown'


def test_land_detector_alone_is_not_touchdown(ctx, io, cfg, clock, fake_px4, fake_perception):
    """PX4 lands (and would auto-disarm), but the rangefinder still reads high: no touchdown."""
    px4, _ = flight(ctx, io, cfg, clock, fake_px4, fake_perception, 'red',
                    px4_kw={'auto_disarm_s': 1000.0})
    clock.hooks.append(lambda now: ctx.update_input('distance_sensor', rangefinder(0.4), now))
    clock.limit = clock.now() + 200
    with pytest.raises(Exception, match='FakeClock passed its limit'):
        run_to_end(ctx, io, cfg, clock)
    assert ctx.state == S.DESCENDING and px4.landed and ctx.touchdowns == 0


def test_rangefinder_source_dist_bottom(ctx, io, make_cfg, clock, fake_px4, fake_perception):
    cfg = make_cfg(rangefinder_source='dist_bottom')
    px4, _ = flight(ctx, io, cfg, clock, fake_px4, fake_perception, 'red')
    clock.hooks.append(lambda now: ctx.update_input('distance_sensor', rangefinder(5.0, 0), now))
    assert run_to_end(ctx, io, cfg, clock) == MISSION_COMPLETE
    assert ctx.touchdowns == 1


def test_rangefinder_readings(ctx, io, make_cfg, clock, feed):
    cfg = make_cfg()
    hover_over_die(ctx, feed, clock)
    state = Descending(ctx, io, cfg, clock)
    snap = ctx.snapshot()
    assert state.range_m(snap, clock.now()) == pytest.approx(1.0)
    ctx.update_input('distance_sensor', rangefinder(0.3, quality=0), clock.now())
    assert state.range_m(ctx.snapshot(), clock.now()) is None               # invalid signal
    db = Descending(ctx, io, make_cfg(rangefinder_source='dist_bottom'), clock)
    lp = position(z=-0.7)
    ctx.update_input('local_position', lp, clock.now())
    assert db.range_m(ctx.snapshot(), clock.now()) == pytest.approx(0.7)
    lp.dist_bottom_valid = False
    ctx.update_input('local_position', lp, clock.now())
    assert db.range_m(ctx.snapshot(), clock.now()) is None


def test_target_relative_ignored_below_blind_height(ctx, io, cfg, clock, feed):
    n, e = hover_over_die(ctx, feed, clock, height=0.1,
                          target_relative=SimpleNamespace(valid=True, offset=SimpleNamespace(
                              x=0.5, y=0.5, z=-0.1)))
    state = Descending(ctx, io, cfg, clock)
    state.entered_at = clock.now()
    state.on_entry(ctx.snapshot(), clock.now())
    state.phase = 'blind'
    state.tick(clock.now())
    assert ctx.target_centroid[:2] == pytest.approx(RED)                   # not moved by it


def test_disarm_with_landed_flag_is_not_an_abort(ctx, io, cfg, clock, feed):
    hover_over_die(ctx, feed, clock, height=0.0)
    feed.set(vehicle_status=status(False, POSCTL), land_detected=land_detected(True),
             distance_sensor=rangefinder(0.0))
    state = Descending(ctx, io, cfg, clock)
    state.entered_at = clock.now()
    state.on_entry(ctx.snapshot(), clock.now())
    assert state.tick(clock.now()) is None
    assert ctx.abort_cause == ''


def test_approaching_resets_descent_retries():
    from gnc_offboard.mission.mission_manager_smach import Approaching
    import inspect
    assert 'descent_retries = 0' in inspect.getsource(Approaching.on_entry)


# ---- ON_DIE_TEST, LIFTOFF ------------------------------------------------------------------------

def on_die(ctx, feed, clock):
    hover_over_die(ctx, feed, clock, height=0.0)
    feed.set(vehicle_status=status(False, POSCTL), land_detected=land_detected(True),
             distance_sensor=rangefinder(0.0))


def test_on_die_test_then_liftoff_to_returning(ctx, io, cfg, clock, feed):
    on_die(ctx, feed, clock)
    ctx.touchdowns = 1
    t0 = clock.now()
    assert OnDieTest(ctx, io, cfg, clock).execute() == 'done'
    assert clock.now() - t0 == pytest.approx(cfg.on_die_test_s, abs=0.06)
    assert ctx.after_liftoff == S.RETURNING
    assert io.calls == []                       # no arming change, no setpoints on the die


@pytest.mark.parametrize('after, outcome', [(S.DESCENDING, 'to_descending'),
                                            (S.SEARCHING, 'to_searching'),
                                            (S.APPROACHING, 'to_approaching'),
                                            (S.RETURNING, 'to_returning')])
def test_liftoff_rearms_climbs_and_goes_where_the_context_says(after, outcome, ctx, io, cfg,
                                                               clock, fake_px4):
    px4 = fake_px4(io)                          # on the ground, disarmed (as after auto-disarm)
    ctx.offboard_engaged, ctx.start_position = True, (0.0, 0.0, 0.0, 0.0)
    ctx.after_liftoff = after
    assert Liftoff(ctx, io, cfg, clock).execute() == outcome
    assert 'arm' in io.names() and 'set_offboard' in io.names()
    assert -px4.z == pytest.approx(cfg.hover_height_m, abs=cfg.altitude_tol_m)
    assert TRANSITIONS[S.LIFTOFF][outcome] == after


def test_liftoff_with_pilot_arm_never_sends_arm(ctx, io, make_cfg, clock, fake_px4, feed):
    cfg = make_cfg(require_pilot_arm=True)
    px4 = fake_px4(io)
    ctx.offboard_engaged, ctx.start_position = True, (0.0, 0.0, 0.0, 0.0)
    feed.at(4.0, px4.pilot_arm)
    assert Liftoff(ctx, io, cfg, clock).execute() == 'to_returning'
    assert 'arm' not in io.names()


def test_liftoff_rearm_refused_aborts_and_stays_on_the_die(ctx, io, cfg, clock, fake_px4):
    px4 = fake_px4(io, accept_arm=False)
    ctx.offboard_engaged, ctx.start_position = True, (0.0, 0.0, 0.0, 0.0)
    assert Liftoff(ctx, io, cfg, clock).execute() == 'aborted'
    assert ctx.abort_cause == AbortCause.LIFTOFF_FAILED.value
    from gnc_offboard.mission.mission_manager_smach import Aborted
    io.calls.clear()
    assert Aborted(ctx, io, cfg, clock).execute() == MISSION_ABORTED
    assert 'land' not in io.names() and not px4.armed


def test_liftoff_on_the_die_retarget_or_return_waits_for_liftoff(ctx, io, cfg, clock, fake_px4,
                                                                feed):
    fake_px4(io, accept_arm=False)                # on the die, never re-arms
    ctx.offboard_engaged, ctx.start_position = True, (0.0, 0.0, 0.0, 0.0)
    ctx.active_target, ctx.after_liftoff = 'red', S.RETURNING
    state = Liftoff(ctx, io, cfg, clock)
    state.entered_at = clock.now()
    state.on_entry(ctx.snapshot(), clock.now())
    ctx.update_input('intent', intent('retrieve', 'yellow', 5), clock.now())
    assert state.tick(clock.now()) is None                                  # still on the die
    assert ctx.active_target == 'yellow' and ctx.after_liftoff == S.SEARCHING
    feed.set(battery_status=battery(0.05))
    assert state.tick(clock.now()) is None and ctx.after_liftoff == S.RETURNING


# ---- on-die global behavior ---------------------------------------------------------------------

def test_abort_on_the_die_disarms_and_stays(ctx, io, cfg, clock, fake_px4, fake_perception,
                                            script):
    px4, _ = flight(ctx, io, cfg, clock, fake_px4, fake_perception, 'red',
                    px4_kw={'auto_disarm_s': 1000.0})               # still armed on the die
    script.when(lambda c: c.state == S.ON_DIE_TEST, intent('abort', '', 2))
    assert run_to_end(ctx, io, cfg, clock) == MISSION_ABORTED
    assert transitions_of(io)[-2:] == [S.ON_DIE_TEST, S.ABORTED]
    after = io.names()[io.names().index('request_state_publish', len(io.names()) - 8):]
    assert 'disarm' in io.names() and 'land' not in after
    assert not px4.armed and px4.landed


def test_low_battery_on_the_die_lifts_off_then_returns(ctx, io, cfg, clock, fake_px4,
                                                       fake_perception, script):
    px4, _ = flight(ctx, io, cfg, clock, fake_px4, fake_perception, 'red')
    script.when(lambda c: c.state == S.ON_DIE_TEST, None,
                then=lambda: clock.hooks.append(
                    lambda now: ctx.update_input('battery_status', battery(0.1), now)))
    assert run_to_end(ctx, io, cfg, clock) == MISSION_COMPLETE
    assert 'ON_DIE_TEST -> LIFTOFF: battery 10%: returning' in lines(io)
    assert any(t.startswith('LIFTOFF -> RETURNING') for t in lines(io))


@pytest.mark.parametrize('color, then', [('black', S.APPROACHING), ('yellow', S.SEARCHING)])
def test_retarget_on_the_die_lifts_off_first(color, then, ctx, io, cfg, clock, fake_px4,
                                             fake_perception, script):
    px4, perception = flight(ctx, io, cfg, clock, fake_px4, fake_perception, 'red')

    def freeze_map():           # black in Perception's map, yellow not; no new detections
        perception.model.reported = {i: d.color for i, d in enumerate(perception.model.dice)
                                     if d.color in ('black', 'red')}
        perception.drop = True
    script.when(lambda c: c.state == S.ON_DIE_TEST, None, then=freeze_map)
    script.when(lambda c: c.state == S.ON_DIE_TEST, intent('retrieve', color, 2))
    script.when(lambda c: c.state == then, intent('abort', '', 3))
    run_to_end(ctx, io, cfg, clock)
    names = transitions_of(io)
    i = names.index(S.ON_DIE_TEST)
    assert names[i + 1:i + 3] == [S.LIFTOFF, then]


def test_pause_on_the_die_and_new_intent_lifts_off_to_searching(ctx, io, cfg, clock, fake_px4,
                                                                fake_perception, script):
    flight(ctx, io, cfg, clock, fake_px4, fake_perception, 'red')
    script.when(lambda c: c.state == S.ON_DIE_TEST, intent('pause', '', 2))
    script.when(lambda c: c.state == S.PAUSED, intent('update_only', '', 3))
    script.when(lambda c: c.last_intent_id == 3, intent('resume', '', 4))
    script.when(lambda c: c.state == S.SEARCHING and c.last_intent_id == 4, intent('abort', '', 5))
    run_to_end(ctx, io, cfg, clock)
    names = transitions_of(io)
    i = names.index(S.PAUSED)
    assert names[i - 1:i + 3] == [S.ON_DIE_TEST, S.PAUSED, S.LIFTOFF, S.SEARCHING]


def test_paused_on_the_die_return_lifts_off(ctx, io, cfg, clock, feed):
    from gnc_offboard.mission.mission_manager_smach import RETURN_LIFTOFF, Paused
    on_die(ctx, feed, clock)
    ctx.paused_from = S.ON_DIE_TEST
    ctx.update_input('intent', intent('return_to_start', '', 2), clock.now())
    assert Paused(ctx, io, cfg, clock).execute() == RETURN_LIFTOFF
    assert ctx.after_liftoff == S.RETURNING


# ---- RETURNING ----------------------------------------------------------------------------------

def test_returning_flies_home_at_search_altitude(ctx, io, cfg, clock, fake_px4):
    px4 = fake_px4(io)
    px4.armed, px4.nav, px4.z, px4.took_off = True, OFFBOARD, -1.0, True
    n, e, _ = ctx.field_config.frame.to_ned(*RED)
    px4.x, px4.y = n, e
    px4.publish(clock.now())                     # flying, before RETURNING's first look
    ctx.offboard_engaged, ctx.start_position = True, (0.0, 0.0, 0.0, 0.0)
    ctx.update_input('perception_heartbeat', heartbeat(), clock.now())
    clock.hooks.append(lambda now: ctx.update_input('perception_heartbeat', heartbeat(), now))
    peak = []
    clock.hooks.append(lambda now: peak.append(-px4.z))
    assert Returning(ctx, io, cfg, clock).execute() == 'over_start'
    home = ctx.field_config.frame.to_field(px4.x, px4.y, px4.z)
    assert math.dist(home[:2], ctx.field_config.start_center) <= cfg.acceptance_radius_m + 0.01
    assert max(peak) == pytest.approx(cfg.search_altitude_m, abs=0.05)


def test_return_to_start_while_returning_is_no_change(ctx, io, cfg, clock, feed):
    hover_over_die(ctx, feed, clock, height=2.5)
    state = Returning(ctx, io, cfg, clock)
    state.entered_at = clock.now()
    state.on_entry(ctx.snapshot(), clock.now())
    ctx.update_input('intent', intent('return_to_start', '', 3), clock.now())
    assert state.tick(clock.now()) is None and ctx.last_intent_result == 'applied'


def test_return_from_flying_states_goes_to_returning():
    for name in (S.TAKEOFF, S.SEARCHING, S.APPROACHING, S.DESCENDING, S.LIFTOFF):
        assert TRANSITIONS[name][RETURN_OUT] == S.RETURNING
    assert TRANSITIONS[S.RETURNING]['over_start'] == S.LANDING
    assert TRANSITIONS[S.ON_DIE_TEST][RETARGET_SEEN] == S.LIFTOFF
    assert TRANSITIONS[S.ON_DIE_TEST][RETARGET_UNSEEN] == S.LIFTOFF
