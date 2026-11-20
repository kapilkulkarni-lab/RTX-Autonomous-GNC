"""Shared fixtures for the mission_manager tests: config from the real mission.yaml, a fake
clock, FakeIO, PX4 message helpers and a tiny PX4 stand-in for full state machine runs."""
import dataclasses
from pathlib import Path
from types import SimpleNamespace

import pytest

from px4_msgs.msg import (BatteryStatus, VehicleLandDetected, VehicleLocalPosition,  # noqa: E402
                          VehicleStatus)

from gnc_offboard.mission.clock import FakeClock  # noqa: E402
from gnc_offboard.mission.config import MissionConfig, load_yaml_params  # noqa: E402
from gnc_offboard.mission.context import MissionContext  # noqa: E402
from gnc_offboard.mission.io import FakeIO  # noqa: E402

CONFIG_YAML = Path(__file__).resolve().parents[1] / 'config' / 'mission.yaml'

ARMED, DISARMED = VehicleStatus.ARMING_STATE_ARMED, VehicleStatus.ARMING_STATE_DISARMED
OFFBOARD = VehicleStatus.NAVIGATION_STATE_OFFBOARD
AUTO_LAND = VehicleStatus.NAVIGATION_STATE_AUTO_LAND
POSCTL = VehicleStatus.NAVIGATION_STATE_POSCTL


def status(armed=False, nav=POSCTL, failsafe=False):
    m = VehicleStatus()
    m.arming_state = ARMED if armed else DISARMED
    m.nav_state = nav
    m.failsafe = failsafe
    return m


def position(x=0.0, y=0.0, z=0.0, heading=0.0, valid=True):
    m = VehicleLocalPosition()
    m.x, m.y, m.z, m.heading = float(x), float(y), float(z), float(heading)
    m.xy_valid = m.z_valid = valid
    return m


def land_detected(landed):
    m = VehicleLandDetected()
    m.landed = landed
    return m


def battery(remaining):
    m = BatteryStatus()
    m.remaining = float(remaining)
    return m


def intent(action, color='', intent_id=1):
    return SimpleNamespace(action=action, target_color=color, intent_id=intent_id,
                           avoid_mask=0, stay_mask=0, until_mask=0, target_quadrant=0)


@pytest.fixture
def cfg():
    return MissionConfig.from_dict(load_yaml_params(CONFIG_YAML))


@pytest.fixture
def make_cfg(cfg):
    return lambda **changes: dataclasses.replace(cfg, **changes)


@pytest.fixture
def clock():
    return FakeClock(limit_s=600.0)


@pytest.fixture
def ctx():
    return MissionContext()


@pytest.fixture
def io():
    return FakeIO()


class Feeder:
    """Keeps PX4 inputs fresh: re-stores the current messages on every tick, and lets a test
    schedule changes at given times (seconds after the run starts)."""

    def __init__(self, ctx, clock):
        self.ctx, self.clock = ctx, clock
        self.current = {}
        self.schedule = []          # (time, callable)
        self.t0 = clock.now()
        clock.hooks.append(self.tick)

    def set(self, **inputs):
        self.current.update(inputs)
        for name, msg in inputs.items():
            if msg is not None:
                self.ctx.update_input(name, msg, self.clock.now())

    def stop(self, name):
        """Stop refreshing an input (it goes stale)."""
        self.current.pop(name, None)

    def at(self, t, fn):
        self.schedule.append((self.t0 + t, fn))

    def tick(self, now):
        due = [item for item in self.schedule if item[0] <= now + 1e-9]
        for item in due:
            self.schedule.remove(item)
            item[1]()
        for name, msg in self.current.items():
            self.ctx.update_input(name, msg, now)


@pytest.fixture
def feed(ctx, clock):
    return Feeder(ctx, clock)


class FakePx4:
    """Minimal PX4 for full runs: follows FakeIO commands. Arm and offboard are accepted
    (offboard only while a setpoint streams), z moves toward the setpoint, land descends to
    the ground, then PX4 auto-disarms after `auto_disarm_s`."""

    def __init__(self, ctx, io, clock, accept_arm=True, auto_disarm_s=2.0, climb_mps=1.0):
        self.ctx, self.io, self.clock = ctx, io, clock
        self.accept_arm, self.auto_disarm_s, self.climb = accept_arm, auto_disarm_s, climb_mps
        self.armed, self.nav, self.z, self.landed = False, POSCTL, 0.0, True
        self.landed_at = None
        self.seen = 0
        clock.hooks.append(self.tick)
        self.publish(clock.now())

    def pilot_arm(self):
        self.armed = True

    def tick(self, now):
        dt = 0.05
        for name, args in self.io.calls[self.seen:]:
            if name == 'arm' and self.accept_arm:
                self.armed = True
            elif name == 'disarm' and self.landed:
                self.armed = False
            elif name == 'set_offboard' and self.armed and self.io.setpoint is not None:
                self.nav = OFFBOARD
            elif name == 'land' and self.armed:
                self.nav = AUTO_LAND
        self.seen = len(self.io.calls)
        if self.armed and self.nav == OFFBOARD and self.io.setpoint is not None:
            target = self.io.setpoint[2]
            step = max(-self.climb * dt, min(self.climb * dt, target - self.z))
            self.z += step
            self.landed = self.z >= -0.02 and target >= -0.02
        elif self.armed and self.nav == AUTO_LAND:
            self.z = min(0.0, self.z + 0.5 * dt)
            if self.z >= 0.0 and not self.landed:
                self.landed, self.landed_at = True, now
        if self.landed and self.armed and self.nav == AUTO_LAND and self.landed_at is not None \
                and now - self.landed_at >= self.auto_disarm_s:
            self.armed, self.nav = False, POSCTL          # PX4 auto-disarm
        self.publish(now)

    def publish(self, now):
        self.ctx.update_input('vehicle_status', status(self.armed, self.nav), now)
        self.ctx.update_input('local_position', position(z=self.z), now)
        self.ctx.update_input('land_detected', land_detected(self.landed), now)
        self.ctx.update_input('battery_status', battery(0.9), now)


@pytest.fixture
def fake_px4(ctx, io, clock):
    return lambda **kw: FakePx4(ctx, io, clock, **kw)
