"""SEARCHING, APPROACHING, TARGET_HOVER, retarget, the Perception heartbeat check, and full runs
against FakePx4 + FakePerception (the shared sim/perception_model.py)."""
import math
from types import SimpleNamespace

import pytest

from conftest import (OFFBOARD, battery, flight, heartbeat, intent, land_detected, position,
                      status, transitions_of)
from gnc_offboard.mission import state_names as S
from gnc_offboard.mission.mission_manager_smach import (
    ABORTED_OUT, MISSION_ABORTED, MISSION_COMPLETE, RETARGET_SEEN, RETARGET_UNSEEN,
    TRANSITIONS, AbortCause, Approaching, Searching, Takeoff, TargetHover, build_state_machine)
from gnc_offboard.sim.perception_model import Die

RED, BLACK, YELLOW = (2.0, 2.0), (-4.5, -1.0), (3.0, -3.0)     # test world (data/field_manifest.json)


def dmap(*dice):
    return SimpleNamespace(dice=[SimpleNamespace(color=c, centroid=SimpleNamespace(x=x, y=y, z=0.0))
                                 for c, (x, y) in dice])


def lines(io):
    return [text for level, text in io.logs if level == 'info' and ' -> ' in text]


def near(point, xy, tol=0.3):
    return point is not None and math.hypot(point[0] - xy[0], point[1] - xy[1]) <= tol


def airborne(ctx, feed, clock, target='red', dice=()):
    ctx.active_target = target
    ctx.offboard_engaged = True
    ctx.start_position = (0.0, 0.0, 0.0, 0.0)
    ctx.mission_start_time = clock.now()
    feed.set(vehicle_status=status(True, OFFBOARD), local_position=position(z=-1.5),
             land_detected=land_detected(False), battery_status=battery(0.9),
             perception_heartbeat=heartbeat(), dice_map=dmap(*dice))


def make(cls, ctx, io, cfg, clock):
    state = cls(ctx, io, cfg, clock)
    state.entered_at = clock.now()
    ctx.state = cls.NAME
    state.on_entry(ctx.snapshot(), clock.now())
    return state


def run_to_end(ctx, io, cfg, clock):
    return build_state_machine(ctx, io, cfg, clock).execute()


# ---- full runs ----------------------------------------------------------------------------------

def test_happy_path_to_the_target(ctx, io, cfg, clock, fake_px4, fake_perception):
    px4, _ = flight(ctx, io, cfg, clock, fake_px4, fake_perception, 'red')
    hovered = []
    clock.hooks.append(lambda now: hovered.append(ctx.field_config.frame.to_field(
        px4.x, px4.y, px4.z)) if ctx.state == S.TARGET_HOVER else None)
    assert run_to_end(ctx, io, cfg, clock) == MISSION_COMPLETE
    assert transitions_of(io) == [S.STANDBY, S.PREFLIGHT, S.TAKEOFF, S.SEARCHING, S.APPROACHING,
                                  S.TARGET_HOVER, S.LANDING, S.RETURN_CONFIRMED]
    assert near(ctx.target_centroid, RED, 0.05)
    x, y, z = hovered[0]
    assert near((x, y), RED, cfg.approach_center_tol_m + 0.01)          # centered over it
    assert z == pytest.approx(cfg.hover_height_m, abs=cfg.altitude_tol_m)
    assert ctx.start_field == pytest.approx((-6.096, -6.096, 0.0))       # field frame
    assert ctx.coverage.count > 0
    assert 'SEARCHING -> APPROACHING: red found' in lines(io)


@pytest.mark.parametrize('pattern', ['lawnmower', 'zone_sweep'])
def test_other_search_patterns_also_find_it(pattern, ctx, io, make_cfg, clock, fake_px4,
                                            fake_perception):
    cfg = make_cfg(search_pattern=pattern)
    flight(ctx, io, cfg, clock, fake_px4, fake_perception, 'red')
    assert run_to_end(ctx, io, cfg, clock) == MISSION_COMPLETE
    assert near(ctx.target_centroid, RED, 0.05)


def test_wrong_color_first_then_the_real_target(ctx, io, cfg, clock, fake_px4, fake_perception):
    """The first die seen is reported red; a close look shows its true color, Perception
    corrects its map, and the search goes on to the real red die."""
    flight(ctx, io, cfg, clock, fake_px4, fake_perception, 'red', wrong_color_first='red')
    assert run_to_end(ctx, io, cfg, clock) == MISSION_COMPLETE
    wrong = [t for t in lines(io) if 'wrong color' in t]
    assert len(wrong) == 1 and wrong[0].startswith('APPROACHING -> ')
    assert near(ctx.target_centroid, RED, 0.05)
    assert len(ctx.given_up_dice) == 1 and not near(ctx.given_up_dice[0], RED)


def test_wrong_color_known_goes_straight_to_the_target(ctx, io, cfg, clock, fake_px4,
                                                       fake_perception):
    px4 = fake_px4(io, perception=False)
    fake_perception(px4, dice=[Die('black', -5.0, -4.0), Die('red', 3.0, 3.0)], view_all=True,
                    wrong_color_first='red')
    ctx.update_input('intent', intent('retrieve', 'red', 1), clock.now())
    assert run_to_end(ctx, io, cfg, clock) == MISSION_COMPLETE
    names = transitions_of(io)
    assert names[3:7] == [S.SEARCHING, S.APPROACHING, S.APPROACHING, S.TARGET_HOVER]
    assert 'SEARCHING -> APPROACHING: red already in the dice map' in lines(io)
    assert 'APPROACHING -> APPROACHING: wrong color (black): going to target' in lines(io)
    assert near(ctx.target_centroid, (3.0, 3.0), 0.05)


def test_target_lost_retried_until_max_attempts(ctx, io, cfg, clock, fake_px4, fake_perception,
                                                script):
    """Lost once: the die stays eligible, so SEARCHING's entry check sends the drone back.
    Lost max_die_attempts (2) times: skipped."""
    px4, perception = flight(ctx, io, cfg, clock, fake_px4, fake_perception, 'red')
    script.when(lambda c: c.state == S.APPROACHING, None,
                then=lambda: setattr(perception, 'drop', True))
    script.when(lambda c: c.state == S.SEARCHING and c.given_up_dice, intent('abort', '', 2))
    assert run_to_end(ctx, io, cfg, clock) == MISSION_ABORTED
    out = [t for t in lines(io) if t.startswith(('APPROACHING -> ', 'SEARCHING -> APPROACHING'))]
    assert out == ['SEARCHING -> APPROACHING: red found',
                   'APPROACHING -> SEARCHING: target lost (1 of 2)',
                   'SEARCHING -> APPROACHING: red already in the dice map',
                   'APPROACHING -> SEARCHING: target lost (2 of 2)']
    assert len(ctx.given_up_dice) == 1 and near(ctx.given_up_dice[0], RED)


def test_color_never_confirmed_times_out(ctx, io, cfg, clock, fake_px4, fake_perception, script):
    """PENDING never transitions; only color_confirm_timeout_s ends the wait."""
    # Never low enough to confirm (hover height is 1.0 m).
    flight(ctx, io, cfg, clock, fake_px4, fake_perception, 'red', confirm_max_alt_m=0.5)
    script.when(lambda c: c.state == S.SEARCHING and c.given_up_dice, intent('abort', '', 2))
    assert run_to_end(ctx, io, cfg, clock) == MISSION_ABORTED
    left = [t for t in lines(io) if t.startswith('APPROACHING -> ')]
    assert left == ['APPROACHING -> SEARCHING: no confirmed color (1 of 2)',
                    'APPROACHING -> SEARCHING: no confirmed color (2 of 2)']
    assert near(ctx.given_up_dice[0], RED)


def test_retarget_mid_search_unseen(ctx, io, cfg, clock, fake_px4, fake_perception, script):
    flight(ctx, io, cfg, clock, fake_px4, fake_perception, 'red')
    script.when(lambda c: c.state == S.SEARCHING and c.coverage.count > 20
                and not any(d.color == 'yellow' for d in c.snapshot().dice_map.msg.dice),
                intent('retrieve', 'yellow', 2))
    assert run_to_end(ctx, io, cfg, clock) == MISSION_COMPLETE
    assert 'SEARCHING -> SEARCHING: retarget yellow: searching' in lines(io)
    assert ctx.active_target == 'yellow' and near(ctx.target_centroid, YELLOW, 0.05)


def test_retarget_mid_search_seen(ctx, io, cfg, clock, fake_px4, fake_perception, script):
    flight(ctx, io, cfg, clock, fake_px4, fake_perception, 'red')
    script.when(lambda c: c.state == S.SEARCHING
                and any(d.color == 'black' for d in c.snapshot().dice_map.msg.dice),
                intent('retrieve', 'black', 2))
    assert run_to_end(ctx, io, cfg, clock) == MISSION_COMPLETE
    assert 'SEARCHING -> APPROACHING: retarget black: in the dice map' in lines(io)
    assert near(ctx.target_centroid, BLACK, 0.05)


def test_resume_with_new_color_already_seen(ctx, io, cfg, clock, fake_px4, fake_perception,
                                            script):
    """Pause, retrieve a die already in the map, resume: SEARCHING, then straight to it."""
    flight(ctx, io, cfg, clock, fake_px4, fake_perception, 'red')
    script.when(lambda c: c.state == S.SEARCHING
                and any(d.color == 'black' for d in c.snapshot().dice_map.msg.dice),
                intent('pause', '', 2))
    script.when(lambda c: c.state == S.PAUSED, intent('retrieve', 'black', 3))
    script.when(lambda c: c.last_intent_id == 3, intent('resume', '', 4))
    assert run_to_end(ctx, io, cfg, clock) == MISSION_COMPLETE
    assert 'PAUSED -> SEARCHING: resumed with new intent' in lines(io)
    assert 'SEARCHING -> APPROACHING: black already in the dice map' in lines(io)
    assert near(ctx.target_centroid, BLACK, 0.05)


def test_perception_dropout_aborts(ctx, io, cfg, clock, fake_px4, fake_perception, script):
    px4, perception = flight(ctx, io, cfg, clock, fake_px4, fake_perception, 'red')
    script.when(lambda c: c.state == S.SEARCHING, None,
                then=lambda: setattr(perception, 'beating', False))
    assert run_to_end(ctx, io, cfg, clock) == MISSION_ABORTED
    assert 'SEARCHING -> ABORTED: Perception heartbeat lost' in lines(io)
    assert ctx.abort_cause == AbortCause.PERCEPTION.value
    assert px4.landed and not px4.armed                 # held, then landed in place


def test_no_perception_check_when_not_required(ctx, io, make_cfg, clock, fake_px4,
                                               fake_perception):
    cfg = make_cfg(require_perception=False)
    px4, perception = flight(ctx, io, cfg, clock, fake_px4, fake_perception, 'red')
    perception.beating = False
    assert run_to_end(ctx, io, cfg, clock) == MISSION_COMPLETE


def test_mission_target_follows_the_die(ctx, io, cfg, clock, fake_px4, fake_perception):
    flight(ctx, io, cfg, clock, fake_px4, fake_perception, 'red')
    seen = []
    clock.hooks.append(lambda now: seen.append((ctx.active_target, ctx.target_centroid)))
    run_to_end(ctx, io, cfg, clock)
    assert ('red', None) in seen                         # target set before the die is known
    assert any(c is not None and near(c, RED, 0.05) for _, c in seen)


# ---- SEARCHING ----------------------------------------------------------------------------------

def test_searching_entry_check_target_known(ctx, io, cfg, clock, feed):
    airborne(ctx, feed, clock, dice=[('red', RED)])
    assert make(Searching, ctx, io, cfg, clock).tick(clock.now()) == 'target_known'
    assert 'set_setpoint' not in io.names()             # leaves at once, flies nothing


def test_die_lost_fewer_than_max_attempts_stays_eligible(ctx, io, cfg, clock, feed):
    airborne(ctx, feed, clock, dice=[('red', RED)])
    ctx.die_failures = [[(*RED, 0.0), cfg.max_die_attempts - 1]]
    assert make(Searching, ctx, io, cfg, clock).tick(clock.now()) == 'target_known'


def test_max_die_attempts_one_skips_after_first_loss(ctx, io, make_cfg, clock, fake_px4,
                                                     fake_perception, script):
    cfg = make_cfg(max_die_attempts=1)
    px4, perception = flight(ctx, io, cfg, clock, fake_px4, fake_perception, 'red')
    script.when(lambda c: c.state == S.APPROACHING, None,
                then=lambda: setattr(perception, 'drop', True))
    script.when(lambda c: c.state == S.SEARCHING and c.given_up_dice, intent('abort', '', 2))
    assert run_to_end(ctx, io, cfg, clock) == MISSION_ABORTED
    assert [t for t in lines(io) if t.startswith('APPROACHING -> ')] == [
        'APPROACHING -> SEARCHING: target lost (1 of 1)']


def test_wrong_color_excluded_at_once(ctx, io, cfg, clock, fake_px4, fake_perception):
    """A wrong color Perception confirmed is never retried, whatever max_die_attempts is."""
    flight(ctx, io, cfg, clock, fake_px4, fake_perception, 'red', wrong_color_first='red')
    assert run_to_end(ctx, io, cfg, clock) == MISSION_COMPLETE
    assert len(ctx.given_up_dice) == 1 and ctx.die_failures == []


def test_searching_skips_dice_given_up_on(ctx, io, cfg, clock, feed):
    airborne(ctx, feed, clock, dice=[('red', RED)])
    ctx.given_up_dice = [RED]
    state = make(Searching, ctx, io, cfg, clock)
    assert state.tick(clock.now()) is None and state.follower is not None


def test_searching_target_found_while_flying(ctx, io, cfg, clock, feed):
    airborne(ctx, feed, clock)
    feed.at(2.0, lambda: feed.set(dice_map=dmap(('blue', (0, 0)), ('red', RED))))
    assert Searching(ctx, io, cfg, clock).execute() == 'target_found'
    assert io.names().count('set_setpoint') > 10        # it was flying the pattern


def test_searching_field_covered_clears_and_searches_again(ctx, io, cfg, clock, feed):
    airborne(ctx, feed, clock)
    ctx.coverage.mark(0.0, 0.0, 20.0)
    ctx.given_up_dice = [(1.0, 1.0, 0.0)]
    ctx.die_failures = [[(2.0, 2.0, 0.0), 1]]
    state = make(Searching, ctx, io, cfg, clock)
    assert ctx.reason == 'target not found, searching again'
    assert ctx.given_up_dice == [] and ctx.die_failures == [] and state.follower is not None
    assert 'request_state_publish' in io.names()


def test_coverage_marked_only_while_airborne(ctx, io, cfg, clock, feed):
    airborne(ctx, feed, clock)
    make(Takeoff, ctx, io, cfg, clock).tick(clock.now())
    assert ctx.coverage.count > 0
    ctx.coverage.clear()
    feed.set(local_position=position(z=-0.05))          # on the ground
    make(Takeoff, ctx, io, cfg, clock).tick(clock.now())
    assert ctx.coverage.count == 0


# ---- APPROACHING --------------------------------------------------------------------------------

def test_approaching_without_a_goal_is_lost(ctx, io, cfg, clock, feed):
    airborne(ctx, feed, clock)
    assert make(Approaching, ctx, io, cfg, clock).tick(clock.now()) == 'lost'
    assert ctx.pending_reason == 'target no longer in the dice map'


def test_approaching_flies_at_search_altitude_then_down(ctx, io, cfg, clock, feed):
    airborne(ctx, feed, clock, dice=[('red', RED)])
    state = make(Approaching, ctx, io, cfg, clock)
    waypoints = state.follower.waypoints
    n, e, _ = ctx.field_config.frame.to_ned(*RED)
    assert waypoints[-1] == pytest.approx((n, e, -cfg.hover_height_m))
    assert waypoints[-2] == pytest.approx((n, e, -cfg.search_altitude_m))
    assert ctx.target_centroid == pytest.approx((*RED, 0.0))


def test_approaching_routes_around_obstacles(ctx, io, cfg, clock, feed):
    airborne(ctx, feed, clock, dice=[('red', RED)])
    # An obstacle on the straight line from the start (-6.1, -6.1) to the die.
    feed.set(obstacles=SimpleNamespace(obstacles=[SimpleNamespace(x=-2.0, y=-2.0, radius=0.5)]))
    state = make(Approaching, ctx, io, cfg, clock)
    assert len(state.follower.waypoints) > 2            # detour before the die


# ---- retarget -----------------------------------------------------------------------------------

@pytest.mark.parametrize('cls', [Takeoff, Searching, Approaching, TargetHover])
@pytest.mark.parametrize('color, outcome', [('black', RETARGET_SEEN), ('yellow', RETARGET_UNSEEN)])
def test_retarget_from_every_flying_state(cls, color, outcome, ctx, io, cfg, clock, feed):
    airborne(ctx, feed, clock, dice=[('black', BLACK)])
    ctx.given_up_dice, ctx.target_centroid = [(9.0, 9.0, 0.0)], (1.0, 1.0, 0.0)
    ctx.die_failures = [[(8.0, 8.0, 0.0), 1]]
    state = make(cls, ctx, io, cfg, clock)
    ctx.update_input('intent', intent('retrieve', color, 5), clock.now())
    assert state.tick(clock.now()) == outcome
    assert ctx.active_target == color and ctx.last_intent_result == 'applied'
    assert ctx.given_up_dice == [] and ctx.die_failures == [] and ctx.target_centroid is None


def test_retarget_wiring():
    for name in (S.TAKEOFF, S.SEARCHING, S.APPROACHING, S.TARGET_HOVER):
        assert TRANSITIONS[name][RETARGET_SEEN] == S.APPROACHING
        assert TRANSITIONS[name][RETARGET_UNSEEN] == S.SEARCHING
    for name in (S.STANDBY, S.PREFLIGHT, S.LANDING, S.PAUSED):
        assert RETARGET_SEEN not in TRANSITIONS[name]


# ---- Perception heartbeat check -----------------------------------------------------------------

@pytest.mark.parametrize('cls', [Takeoff, Searching, TargetHover])
def test_perception_stale_aborts_airborne(cls, ctx, io, cfg, clock, feed):
    airborne(ctx, feed, clock)
    feed.set(local_position=position(z=-0.5))     # below search altitude: TAKEOFF keeps climbing
    feed.at(1.0, lambda: feed.stop('perception_heartbeat'))
    t0 = clock.now()
    assert cls(ctx, io, cfg, clock).execute() == ABORTED_OUT
    assert ctx.abort_cause == AbortCause.PERCEPTION.value
    assert clock.now() - t0 == pytest.approx(1.0 + cfg.perception_stale_s, abs=0.11)
