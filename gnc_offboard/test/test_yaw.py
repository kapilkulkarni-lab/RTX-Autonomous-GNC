"""face_center: the yaw target (nose at the field center), the deadband, the rate limit, the
held phases (over the die, DESCENDING, on the die, LANDING), and row spacing from the down
camera's narrowest width over all yaws."""
import math

import pytest

from conftest import CAMERAS_YAML, flight
from gnc_offboard.frames import wrap_pi
from gnc_offboard.guidance.yaw import YawCommand, face_point_yaw, field_center, rate_limited
from gnc_offboard.mission import state_names as S
from gnc_offboard.mission.cameras import (down_footprint_extent, down_footprint_min_width,
                                          load_cameras)
from gnc_offboard.mission.mission_manager_smach import Searching, build_state_machine

NE = math.radians(45.0)          # NED yaw from the SW start toward the field center


def yaws_by_state(io, ctx, clock):
    """Record (state, time, yaw) of every setpoint the mission sends."""
    out = []
    real = io.set_setpoint

    def spy(x, y, z, yaw):
        out.append((ctx.state, clock.now(), yaw))
        real(x, y, z, yaw)
    io.set_setpoint = spy
    return out


# ---- pure ---------------------------------------------------------------------------------------

@pytest.mark.parametrize('drone, expected', [((-5.0, -5.0), NE), ((-5.0, 0.0), math.pi / 2),
                                             ((0.0, -5.0), 0.0), ((5.0, 0.0), -math.pi / 2),
                                             ((0.0, 5.0), -math.pi)])
def test_face_point_yaw_points_the_nose_at_the_center(drone, expected):
    yaw = face_point_yaw(drone, (0.0, 0.0), last_yaw=1.0, deadband_m=1.0)
    assert wrap_pi(yaw - expected) == pytest.approx(0.0, abs=1e-9)


def test_deadband_keeps_the_last_yaw():
    assert face_point_yaw((0.5, 0.5), (0.0, 0.0), last_yaw=0.3, deadband_m=1.0) == 0.3
    assert face_point_yaw((1.5, 0.0), (0.0, 0.0), last_yaw=0.3, deadband_m=1.0) == \
        pytest.approx(-math.pi / 2)


def test_rate_limit_takes_the_short_way():
    rate = math.radians(45.0)
    assert rate_limited(0.0, 1.0, rate, 0.1) == pytest.approx(rate * 0.1)
    assert rate_limited(0.0, 0.01, rate, 0.1) == pytest.approx(0.01)        # reached
    # From 170 deg to -170 deg is 20 deg through 180, not 340 deg back.
    y = rate_limited(math.radians(170), math.radians(-170), rate, 0.1)
    assert math.degrees(wrap_pi(y)) == pytest.approx(174.5, abs=1e-6)


def test_yaw_command_is_continuous():
    cmd = YawCommand(0.0, now=0.0)
    for k in range(1, 11):
        cmd.face(k * 0.1, (-5.0, -5.0), (0.0, 0.0), 1.0, math.radians(45.0))
    assert cmd.yaw == pytest.approx(math.radians(45.0), abs=1e-9)             # 1 s at 45 deg/s
    assert cmd.hold(5.0) == cmd.yaw
    assert cmd.face(5.05, (-5.0, -5.0), (0.0, 0.0), 1.0, math.radians(45.0)) == cmd.yaw


def test_field_center():
    assert field_center((-6.858, -6.858, 6.858, 6.858)) == (0.0, 0.0)
    assert field_center((0, 0, 10, 4)) == (5.0, 2.0)


# ---- row spacing --------------------------------------------------------------------------------

def test_spacing_uses_the_narrowest_width_over_all_yaws():
    cams = load_cameras(CAMERAS_YAML)
    w = down_footprint_min_width(cams, 2.5)
    assert w == pytest.approx(8.66, abs=0.05)
    assert w <= min(down_footprint_extent(cams, 2.5, math.radians(a))[1]
                    for a in (0, 30, 45, 60, 90)) + 1e-9


def test_searching_spacing_does_not_depend_on_the_start_heading(ctx, io, cfg, clock, feed,
                                                                monkeypatch):
    from conftest import OFFBOARD, battery, heartbeat, land_detected, position, status
    import gnc_offboard.mission.mission_manager_smach as smach_mod
    seen = []
    monkeypatch.setattr(smach_mod, 'plan_search',
                        lambda *a, **k: seen.append(a[5]) or [(0.0, 0.0)])
    for heading in (0.0, NE):
        ctx.active_target, ctx.offboard_engaged = 'red', True
        ctx.start_position = (0.0, 0.0, 0.0, heading)
        feed.set(vehicle_status=status(True, OFFBOARD), local_position=position(z=-2.5),
                 land_detected=land_detected(False), battery_status=battery(0.9),
                 perception_heartbeat=heartbeat())
        state = Searching(ctx, io, cfg, clock)
        state.on_entry(ctx.snapshot(), clock.now())
    assert len(seen) == 2 and seen[0] == seen[1]
    assert seen[0] == pytest.approx(8.66 * (1 - cfg.search_overlap), abs=0.05)


# ---- in flight ----------------------------------------------------------------------------------

def test_flying_phases_face_the_center_with_the_rate_limit(ctx, io, cfg, clock, fake_px4,
                                                           fake_perception):
    flight(ctx, io, cfg, clock, fake_px4, fake_perception, 'red')
    log = yaws_by_state(io, ctx, clock)
    build_state_machine(ctx, io, cfg, clock).execute()
    takeoff = [(t, y) for s, t, y in log if s == S.TAKEOFF]
    # Starts at the spawn heading (0) and turns toward the center (NE from the SW corner).
    assert takeoff[0][1] == pytest.approx(0.0, abs=1e-9)
    assert takeoff[-1][1] == pytest.approx(NE, abs=math.radians(1))
    # Never faster than face_center_max_rate_dps, across every state change.
    limit = math.radians(cfg.face_center_max_rate_dps)
    for (_, t0, y0), (_, t1, y1) in zip(log, log[1:]):
        assert abs(wrap_pi(y1 - y0)) <= limit * (t1 - t0) + 1e-6
    # RETURNING ends at the start zone, facing the center again.
    returning = [y for s, _, y in log if s == S.RETURNING]
    assert returning[-1] == pytest.approx(NE, abs=math.radians(2))


def test_precision_phases_hold_the_yaw(ctx, io, cfg, clock, fake_px4, fake_perception):
    flight(ctx, io, cfg, clock, fake_px4, fake_perception, 'red')
    log = yaws_by_state(io, ctx, clock)
    build_state_machine(ctx, io, cfg, clock).execute()
    descending = {round(y, 9) for s, _, y in log if s == S.DESCENDING}
    assert len(descending) == 1                                      # no rotation at all
    on_die = [y for s, _, y in log if s in (S.ON_DIE_TEST, S.LANDING)]
    assert on_die == []                                              # no setpoints at all
    # APPROACHING's last leg (straight down over the die) holds the same yaw.
    approach = [y for s, _, y in log if s == S.APPROACHING]
    assert round(approach[-1], 9) in descending


def test_face_center_off_holds_the_start_heading(ctx, io, make_cfg, clock, fake_px4,
                                                 fake_perception):
    cfg = make_cfg(face_center=False)
    flight(ctx, io, cfg, clock, fake_px4, fake_perception, 'red')
    log = yaws_by_state(io, ctx, clock)
    build_state_machine(ctx, io, cfg, clock).execute()
    assert {round(y, 9) for _, _, y in log} == {0.0}


def test_deadband_near_the_center_holds(ctx, io, cfg, clock, feed):
    from conftest import OFFBOARD, battery, heartbeat, land_detected, position, status
    from gnc_offboard.mission.mission_manager_smach import Takeoff
    ctx.offboard_engaged, ctx.start_position = True, (0.0, 0.0, 0.0, 0.2)
    n, e, _ = ctx.field_config.frame.to_ned(0.4, 0.3)               # 0.5 m from the center
    feed.set(vehicle_status=status(True, OFFBOARD), local_position=position(n, e, -0.5),
             land_detected=land_detected(False), battery_status=battery(0.9),
             perception_heartbeat=heartbeat())
    state = Takeoff(ctx, io, cfg, clock)
    state.entered_at = clock.now()
    state.on_entry(ctx.snapshot(), clock.now())
    for _ in range(20):
        clock.t += 0.05
        state.tick(clock.now())
    assert {round(args[3], 9) for name, args in io.calls if name == 'set_setpoint'} == {0.2}
