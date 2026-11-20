"""Every row of the outline's Intent handling table, the three rejections, intent IDs, empty
fields, and intents after the mission is over."""
from types import SimpleNamespace

import pytest

from conftest import (Hover, heartbeat, AUTO_LAND, OFFBOARD, POSCTL, battery, intent, land_detected, position,
                      status, transitions_of)
from gnc_offboard.mission import intents as I
from gnc_offboard.mission import state_names as S
from gnc_offboard.mission.intents import RegionFields
from gnc_offboard.mission.mission_manager_smach import (
    ABORTED_OUT, MISSION_COMPLETE, PAUSE_OUT, RETURN_OUT, Aborted, Landing,
    MissionOver, Paused, Preflight, ReturnConfirmed, Standby, Takeoff, build_state_machine,
    run_mission)

START = (0.0, 0.0, 0.0, 0.0)
NOT_END = [Standby, Preflight, Takeoff, Hover, Landing]


def full(action, color='', intent_id=1, **fields):
    m = intent(action, color, intent_id)
    for k, v in fields.items():
        setattr(m, k, v)
    return m


def prepare(cls, ctx, feed, clock):
    """Put the world in a state where `cls` can run and nothing ends it by itself soon."""
    if cls in (Standby, Preflight):
        if cls is Preflight:
            ctx.active_target = 'blue'
            ctx.mission_start_time = clock.now()
        feed.set(vehicle_status=status(False), local_position=position(valid=False),
                 land_detected=land_detected(True))
        return
    ctx.active_target = 'blue'
    ctx.offboard_engaged = True
    ctx.start_position = START
    ctx.mission_start_time = clock.now()
    nav = AUTO_LAND if cls is Landing else OFFBOARD
    feed.set(vehicle_status=status(True, nav), local_position=position(z=-3.0),
             land_detected=land_detected(False), battery_status=battery(0.9),
             perception_heartbeat=heartbeat())


def make(cls, ctx, io, cfg, clock):
    state = cls(ctx, io, cfg, clock)
    state.entered_at = clock.now()
    ctx.state = cls.NAME
    state.on_entry(ctx.snapshot(), clock.now())
    return state


def send(ctx, clock, *msgs):
    for m in msgs:
        ctx.update_input('intent', m, clock.now())


# ---- the three rejections and acknowledgement ---------------------------------------------------

@pytest.mark.parametrize('cls', NOT_END)
def test_read_chip_rejected_not_on_the_die(cls, ctx, io, cfg, clock, feed):
    prepare(cls, ctx, feed, clock)
    state = make(cls, ctx, io, cfg, clock)
    send(ctx, clock, full('read_chip', 'blue', 4))
    assert state.tick(clock.now()) is None
    assert (ctx.last_intent_id, ctx.last_intent_result) == (4, 'rejected')
    assert ctx.reason == 'read_chip rejected: not on the die'
    assert 'request_state_publish' in io.names()


@pytest.mark.parametrize('cls', NOT_END)
def test_resume_rejected_when_not_paused(cls, ctx, io, cfg, clock, feed):
    prepare(cls, ctx, feed, clock)
    state = make(cls, ctx, io, cfg, clock)
    send(ctx, clock, full('resume', intent_id=5))
    assert state.tick(clock.now()) is None
    assert (ctx.last_intent_id, ctx.last_intent_result) == (5, 'rejected')
    assert ctx.reason == 'resume ignored: not paused'


@pytest.mark.parametrize('cls', [ReturnConfirmed, Aborted, MissionOver])
@pytest.mark.parametrize('action', ['retrieve', 'read_chip', 'pause', 'resume',
                                    'return_to_start', 'update_only', ''])
def test_everything_but_abort_rejected_after_the_end(cls, action, ctx, io, cfg, clock):
    ctx.state = S.ABORTED if cls is not ReturnConfirmed else S.RETURN_CONFIRMED
    state = cls(ctx, io, cfg, clock)
    send(ctx, clock, full(action, 'red', 8))
    state.handle_intent(ctx.take_intents()[0], clock.now())
    assert (ctx.last_intent_id, ctx.last_intent_result) == (8, 'rejected')
    assert ctx.reason == 'mission over: intent ignored'


@pytest.mark.parametrize('end', [S.RETURN_CONFIRMED, S.ABORTED])
def test_abort_after_the_end_applied_no_change(end, ctx, io, cfg, clock):
    ctx.state, ctx.reason = end, 'mission complete'
    over = MissionOver(ctx, io, cfg, clock)
    send(ctx, clock, full('abort', intent_id=9))
    assert over.handle_intent(ctx.take_intents()[0], clock.now()) is None
    assert (ctx.last_intent_id, ctx.last_intent_result) == (9, 'applied')
    assert ctx.state == end and ctx.reason == 'mission complete'


def test_only_three_rejection_reasons():
    reasons = {v for k, v in vars(I).items() if k.startswith('REJECT_')}
    assert reasons == {'read_chip rejected: not on the die', 'resume ignored: not paused',
                       'mission over: intent ignored'}


def test_every_handled_intent_publishes_at_once(ctx, io, cfg, clock, feed):
    prepare(Landing, ctx, feed, clock)            # LANDING: a new color changes no state
    state = make(Landing, ctx, io, cfg, clock)
    send(ctx, clock, full('update_only', intent_id=1), full('retrieve', 'blue', 2),
         full('read_chip', intent_id=3), full('resume', intent_id=4),
         full('retrieve', 'red', 5))
    assert state.tick(clock.now()) is None
    assert io.names().count('request_state_publish') == 5
    assert (ctx.last_intent_id, ctx.last_intent_result) == (5, 'applied')


# ---- retrieve -----------------------------------------------------------------------------------

def test_retrieve_first_starts_the_mission(ctx, io, cfg, clock):
    send(ctx, clock, full('retrieve', 'Blue', 1))
    assert Standby(ctx, io, cfg, clock).execute() == 'start'
    assert ctx.active_target == 'blue' and ctx.mission_start_time == clock.now()
    assert ctx.last_intent_result == 'applied'


@pytest.mark.parametrize('cls', [Preflight, Takeoff, Hover, Landing])
def test_retrieve_same_color_applies_regions_no_state_change(cls, ctx, io, cfg, clock, feed):
    prepare(cls, ctx, feed, clock)
    state = make(cls, ctx, io, cfg, clock)
    send(ctx, clock, full('retrieve', 'blue', 3, avoid_mask=1, target_quadrant=8))
    assert state.tick(clock.now()) is None
    assert ctx.active_target == 'blue'
    assert ctx.regions == RegionFields(avoid_mask=1, target_quadrant=8)
    assert ctx.last_intent_result == 'applied'


@pytest.mark.parametrize('cls', [Preflight, Landing])
def test_retrieve_new_color_outside_retarget_states_updates_target(cls, ctx, io, cfg, clock,
                                                                   feed):
    """Retarget applies TAKEOFF through RETURNING; elsewhere only the target changes."""
    prepare(cls, ctx, feed, clock)
    state = make(cls, ctx, io, cfg, clock)
    send(ctx, clock, full('retrieve', 'yellow', 3))
    assert state.tick(clock.now()) is None
    assert ctx.active_target == 'yellow' and ctx.last_intent_result == 'applied'


def test_retrieve_without_color_is_no_change(ctx, io, cfg, clock, feed):
    prepare(Hover, ctx, feed, clock)
    state = make(Hover, ctx, io, cfg, clock)
    send(ctx, clock, full('retrieve', '', 3))
    assert state.tick(clock.now()) is None
    assert ctx.active_target == 'blue' and ctx.last_intent_result == 'applied'


# ---- abort --------------------------------------------------------------------------------------

@pytest.mark.parametrize('cls', NOT_END)
def test_abort_from_every_state_acts_on_action_alone(cls, ctx, io, cfg, clock, feed):
    prepare(cls, ctx, feed, clock)
    state = make(cls, ctx, io, cfg, clock)
    send(ctx, clock, full('abort', intent_id=2))      # stop word: every other field empty
    assert state.tick(clock.now()) == ABORTED_OUT
    assert (ctx.last_intent_id, ctx.last_intent_result) == (2, 'applied')


def test_abort_from_paused(ctx, io, cfg, clock, feed):
    prepare(Hover, ctx, feed, clock)
    ctx.paused_from = S.SEARCHING
    state = make(Paused, ctx, io, cfg, clock)
    send(ctx, clock, full('abort', intent_id=2))
    assert state.tick(clock.now()) == ABORTED_OUT
    assert ctx.abort_from == S.SEARCHING          # ABORTED treats it as airborne


# ---- return_to_start ----------------------------------------------------------------------------

@pytest.mark.parametrize('cls', [Takeoff, Hover])
def test_return_to_start_airborne(cls, ctx, io, cfg, clock, feed):
    prepare(cls, ctx, feed, clock)
    state = make(cls, ctx, io, cfg, clock)
    send(ctx, clock, full('return_to_start', intent_id=6))
    assert state.tick(clock.now()) == RETURN_OUT
    assert ctx.pending_reason == 'return to start' and ctx.last_intent_result == 'applied'


def test_return_to_start_in_preflight_returns(ctx, io, cfg, clock, feed):
    prepare(Preflight, ctx, feed, clock)
    state = make(Preflight, ctx, io, cfg, clock)
    send(ctx, clock, full('return_to_start', intent_id=6))
    assert state.tick(clock.now()) == RETURN_OUT


@pytest.mark.parametrize('cls', [Standby, Landing])
def test_return_to_start_applied_no_change(cls, ctx, io, cfg, clock, feed):
    """STANDBY: nothing to return from. LANDING: already landing in place."""
    prepare(cls, ctx, feed, clock)
    state = make(cls, ctx, io, cfg, clock)
    send(ctx, clock, full('return_to_start', intent_id=6))
    assert state.tick(clock.now()) is None
    assert (ctx.last_intent_id, ctx.last_intent_result) == (6, 'applied')


def test_return_to_start_from_preflight_stops_disarms_and_completes(ctx, io, cfg, clock,
                                                                    fake_px4, script):
    px4 = fake_px4(accept_arm=True)
    send(ctx, clock, full('retrieve', 'red', 1))
    # Arrives once PREFLIGHT has armed but before offboard (PX4 armed on the ground).
    script.when(lambda c: c.state == S.PREFLIGHT and px4.armed and px4.nav == POSCTL,
                full('return_to_start', intent_id=2))
    px4.accept_offboard = False
    assert build_state_machine(ctx, io, cfg, clock).execute() == MISSION_COMPLETE
    assert transitions_of(io)[-3:] == [S.PREFLIGHT, S.LANDING, S.RETURN_CONFIRMED]
    assert not px4.armed and px4.landed
    assert 'land' not in io.names()                 # on the ground: no land command, disarm


# ---- pause --------------------------------------------------------------------------------------

@pytest.mark.parametrize('cls', NOT_END)
def test_pause_from_every_state(cls, ctx, io, cfg, clock, feed):
    prepare(cls, ctx, feed, clock)
    state = make(cls, ctx, io, cfg, clock)
    send(ctx, clock, full('pause', intent_id=3))   # stop word: every other field empty
    assert state.tick(clock.now()) == PAUSE_OUT
    assert ctx.paused_from == cls.NAME and ctx.last_intent_result == 'applied'


def test_pause_while_paused_applied_no_change(ctx, io, cfg, clock, feed):
    prepare(Hover, ctx, feed, clock)
    ctx.paused_from = S.SEARCHING
    state = make(Paused, ctx, io, cfg, clock)
    send(ctx, clock, full('pause', intent_id=4))
    assert state.tick(clock.now()) is None
    assert (ctx.last_intent_id, ctx.last_intent_result) == (4, 'applied')
    assert ctx.paused_from == S.SEARCHING


# ---- update_only and empty fields ---------------------------------------------------------------

@pytest.mark.parametrize('cls', NOT_END)
def test_update_only_stores_regions_no_state_change(cls, ctx, io, cfg, clock, feed):
    prepare(cls, ctx, feed, clock)
    target = ctx.active_target
    state = make(cls, ctx, io, cfg, clock)
    send(ctx, clock, full('update_only', 'black', 2, target_quadrant=2, stay_mask=9))
    assert state.tick(clock.now()) is None
    assert ctx.regions == RegionFields(stay_mask=9, target_quadrant=2, quadrant_hint_color='black')
    assert ctx.active_target == target              # a color on update_only is never a retarget


def test_empty_action_is_update_only(ctx, io, cfg, clock, feed):
    prepare(Hover, ctx, feed, clock)
    state = make(Hover, ctx, io, cfg, clock)
    send(ctx, clock, full('', 'red', 2, avoid_mask=4))
    assert state.tick(clock.now()) is None
    assert ctx.active_target == 'blue' and ctx.regions.avoid_mask == 4
    assert ctx.last_intent_result == 'applied'


def test_unknown_action_applied_as_update_only_with_warning(ctx, io, cfg, clock, feed):
    prepare(Hover, ctx, feed, clock)
    state = make(Hover, ctx, io, cfg, clock)
    send(ctx, clock, full('dance', intent_id=2))
    assert state.tick(clock.now()) is None
    assert ctx.last_intent_result == 'applied'
    assert any('unknown action' in text for _, text in io.logs)


def test_empty_fields_mean_no_change():
    regions = I.merge_regions(None, full('retrieve', 'red', 1, avoid_mask=1, until_mask=4))
    regions = I.merge_regions(regions, full('update_only', '', 2, stay_mask=2))
    assert regions == RegionFields(avoid_mask=1, stay_mask=2, until_mask=4)
    assert I.merge_regions(regions, full('update_only', 'red', 3)) == regions  # no quadrant


def test_action_and_color_normalized():
    assert I.action_of(SimpleNamespace(action='')) == 'update_only'
    assert I.action_of(SimpleNamespace(action=' Pause ')) == 'pause'
    assert I.color_of(SimpleNamespace(target_color=' BLUE')) == 'blue'


# ---- intent IDs ---------------------------------------------------------------------------------

def test_ids_5_6_1_2_all_applied_and_echoed(ctx, rio, cfg, clock, feed):
    prepare(Hover, ctx, feed, clock)
    state = make(Hover, ctx, rio, cfg, clock)
    for i in (5, 6, 1, 2):
        send(ctx, clock, full('update_only', intent_id=i))
        state.tick(clock.now())
    assert [m.last_intent_id for m in rio.published] == [5, 6, 1, 2]
    assert all(m.last_intent_result == 'applied' for m in rio.published)
    warnings = [text for level, text in rio.logs if level == 'warn']
    assert len(warnings) == 1 and '6 -> 1' in warnings[0]


def test_same_id_twice_is_still_new(ctx, io, cfg, clock, feed):
    prepare(Landing, ctx, feed, clock)
    state = make(Landing, ctx, io, cfg, clock)
    send(ctx, clock, full('retrieve', 'red', 3), full('retrieve', 'black', 3))
    state.tick(clock.now())
    assert ctx.active_target == 'black'
    assert not [t for lvl, t in io.logs if lvl == 'warn']


# ---- after the machine ends ---------------------------------------------------------------------

def test_intents_answered_after_the_machine_finishes(ctx, rio, cfg, clock, fake_px4, script):
    fake_px4(rio)
    send(ctx, clock, full('retrieve', 'blue', 1))
    stop = []
    script.when(lambda c: c.state == S.RETURN_CONFIRMED, full('pause', intent_id=2))
    script.when(lambda c: c.last_intent_id == 2, full('abort', intent_id=3))
    script.when(lambda c: c.last_intent_id == 3, None, then=lambda: stop.append(1))
    sm = build_state_machine(ctx, rio, cfg, clock)
    assert run_mission(sm, ctx, rio, cfg, clock, should_stop=lambda: bool(stop)) == \
        MISSION_COMPLETE
    last = rio.published[-2:]
    assert [(m.last_intent_id, m.last_intent_result, m.reason) for m in last] == [
        (2, 'rejected', 'mission over: intent ignored'),
        (3, 'applied', 'mission over: intent ignored')]
    assert all(m.state == S.RETURN_CONFIRMED for m in last)
