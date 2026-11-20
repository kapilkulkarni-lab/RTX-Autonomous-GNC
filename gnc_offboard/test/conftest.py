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
TEST_MANIFEST = Path(__file__).resolve().parent / 'data' / 'field_manifest.json'
CAMERAS_YAML = Path(__file__).resolve().parents[1] / 'config' / 'cameras.yaml'

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
    m.dist_bottom, m.dist_bottom_valid = max(0.0, -float(z)), valid
    return m


def rangefinder(distance, quality=100):
    return SimpleNamespace(current_distance=float(distance), signal_quality=quality)


def land_detected(landed):
    m = VehicleLandDetected()
    m.landed = landed
    return m


def heartbeat():
    return SimpleNamespace(vio_ok=True, detector_ok=True)


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
def field_config():
    from gnc_offboard.mission.field import FieldConfig
    return FieldConfig.load(TEST_MANIFEST)


@pytest.fixture
def cameras():
    from gnc_offboard.mission.cameras import load_cameras
    return load_cameras(CAMERAS_YAML)


@pytest.fixture
def ctx(field_config, cfg, cameras):
    from gnc_offboard.camera_model import CoverageGrid
    from gnc_offboard.mission.cameras import load_blind_height
    c = MissionContext()
    c.field_config = field_config
    c.coverage = CoverageGrid(field_config.bounds, cfg.coverage_cell_m)
    c.cameras = cameras
    c.blind_height_m = load_blind_height(CAMERAS_YAML)
    return c


from gnc_offboard.mission.mission_manager_smach import (  # noqa: E402
    FLYING_GLOBALS, RETURN_OUT, MissionStateBase)
from gnc_offboard.mission import state_names as _S  # noqa: E402


class Hover(MissionStateBase):
    """Test probe: an airborne state (reports as SEARCHING: airborne, battery, clock and retarget
    checks apply) that only hovers and returns after HOLD_S. For testing global checks and
    intents on their own, without SEARCHING's planning."""
    NAME = _S.SEARCHING
    GLOBALS = FLYING_GLOBALS
    HOLD_S = 5.0

    def step(self, snap, now):
        if now - self.entered_at >= self.HOLD_S:
            return self.finish(RETURN_OUT, 'target confirmed: landing (temporary)')
        return None


def odometry(x=0.0, y=0.0, z=0.0, yaw=0.0):
    """VehicleOdometry-like: PX4 NED position and a level attitude with heading yaw."""
    import math
    return SimpleNamespace(position=[x, y, z],
                           q=[math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)])


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
        self.accept_offboard = True
        self.accept_disarm = True
        self.armed, self.nav, self.z, self.landed = False, POSCTL, 0.0, True
        self.x = self.y = 0.0
        self.yaw = 0.0
        self.took_off = False              # PX4 auto-disarms on landing only after a takeoff
        self.landed_since = None
        self.speed = 3.0                  # m/s horizontal limit (setpoints move slower)
        self.landed_at = None
        self.seen = 0
        clock.hooks.append(self.tick)
        self.publish(clock.now())

    def pilot_arm(self):
        self.armed, self.took_off = True, False       # PX4: "taken off since arming" resets

    def tick(self, now):
        dt = 0.05
        for name, args in self.io.calls[self.seen:]:
            if name == 'arm' and self.accept_arm and not self.armed:
                self.armed, self.took_off = True, False
            elif name == 'disarm' and self.landed and self.accept_disarm:
                self.armed, self.nav = False, POSCTL
            elif name == 'set_offboard' and self.armed and self.io.setpoint is not None \
                    and self.accept_offboard:
                self.nav = OFFBOARD
            elif name == 'land' and self.armed:
                self.nav = AUTO_LAND
        self.seen = len(self.io.calls)
        if self.armed and self.nav == OFFBOARD and self.io.setpoint is not None:
            tx, ty, target = self.io.setpoint[:3]
            self.yaw = self.io.setpoint[3]
            step = max(-self.climb * dt, min(self.climb * dt, target - self.z))
            self.z = min(0.0, self.z + step)               # the ground (or the die) stops it
            if self.z < -0.02:                            # off the ground: free to move sideways
                dx, dy = tx - self.x, ty - self.y
                d = (dx * dx + dy * dy) ** 0.5
                k = min(1.0, self.speed * dt / d) if d > 1e-9 else 0.0
                self.x, self.y = self.x + k * dx, self.y + k * dy
        elif self.armed and self.nav == AUTO_LAND:
            self.z = min(0.0, self.z + 0.5 * dt)
        if self.armed and self.z < -0.3:
            self.took_off = True
        self.landed = self.z >= -0.02                     # land detector
        self.landed_since = (self.landed_since or now) if self.landed else None
        self.landed_at = self.landed_since
        # PX4 COM_DISARM_LAND: disarm after auto_disarm_s landed, in offboard too (once it flew).
        if self.landed and self.armed and self.took_off and \
                now - self.landed_since >= self.auto_disarm_s:
            self.armed, self.nav = False, POSCTL
        self.publish(now)

    def publish(self, now):
        self.ctx.update_input('vehicle_status', status(self.armed, self.nav), now)
        self.ctx.update_input('local_position', position(self.x, self.y, self.z), now)
        self.ctx.update_input('odometry', odometry(self.x, self.y, self.z, self.yaw), now)
        self.ctx.update_input('distance_sensor', rangefinder(-self.z), now)
        self.ctx.update_input('land_detected', land_detected(self.landed), now)
        self.ctx.update_input('battery_status', battery(0.9), now)


@pytest.fixture
def fake_px4(ctx, io, clock, cfg):
    """fake_px4(io=..., perception=True, **options): follows the given IO (default: the `io`
    fixture). With perception (default), a FakePerception runs on it too (px4.perception)."""
    def make(io_=None, perception=True, **kw):
        px4 = FakePx4(ctx, io if io_ is None else io_, clock, **kw)
        px4.perception = FakePerception(ctx, px4, clock, cfg) if perception else None
        return px4
    return make


class RecordingIO(FakeIO):
    """FakeIO that also builds the /mission/state message the node would publish on every
    request_state_publish (same builder as the node)."""

    def __init__(self, ctx, clock):
        super().__init__()
        from gnc_offboard.mission.state_msg import MissionStateBuilder
        self._ctx, self._clock, self._builder = ctx, clock, MissionStateBuilder()
        self.published = []

    def request_state_publish(self):
        super().request_state_publish()
        c = self._ctx
        if c.state:
            self.published.append(self._builder.build(
                state=c.state, previous_state=c.previous_state, paused_from=c.paused_from,
                reason=c.reason, active_target=c.active_target,
                mission_time_s=c.mission_time_s(self._clock.now()),
                last_intent_id=c.last_intent_id, last_intent_result=c.last_intent_result))


@pytest.fixture
def rio(ctx, clock):
    return RecordingIO(ctx, clock)


class Script:
    """Send intents when conditions hold: script.when(cond, intent_msg). Each fires once, in
    order (the next one is only armed after the previous one fired)."""

    def __init__(self, ctx, clock):
        self.ctx, self.clock, self.steps, self.fired = ctx, clock, [], []
        clock.hooks.append(self.tick)

    def when(self, cond, msg, then=None):
        self.steps.append((cond, msg, then))
        return self

    def tick(self, now):
        if self.steps and self.steps[0][0](self.ctx):
            cond, msg, then = self.steps.pop(0)
            self.fired.append((now, self.ctx.state, msg))
            if msg is not None:
                self.ctx.update_input('intent', msg, now)
            if then:
                then()


@pytest.fixture
def script(ctx, clock):
    return Script(ctx, clock)


def transitions_of(io):
    """State names entered, from the transition log lines."""
    return [text.split(' -> ')[1].split(':')[0] for level, text in io.logs
            if level == 'info' and ' -> ' in text]


class FakePerception:
    """Perception stand-in for full runs: the shared PerceptionModel (sim/perception_model.py)
    seeing through both cameras of config/cameras.yaml, projected from FakePx4's pose exactly as
    camera_fov does (dice map: either camera; target_relative and color: down camera). Publishes
    into the context on every tick. Switches: drop (no detections), beating (heartbeat)."""

    def __init__(self, ctx, px4, clock, cfg, manifest_path=TEST_MANIFEST, dice=None,
                 view_all=False, **model_kw):
        import json
        from gnc_offboard import search_planners as sp
        from gnc_offboard.sim.perception_model import PerceptionModel, square_view
        self.ctx, self.px4, self.cfg, self.sp, self.square_view = ctx, px4, cfg, sp, square_view
        model_kw.setdefault('blind_height_m', ctx.blind_height_m or 0.0)
        model_kw.setdefault('seed', 7)
        if dice is not None:
            self.model = PerceptionModel(dice, **model_kw)
        else:
            self.model = PerceptionModel.from_manifest(
                json.loads(Path(manifest_path).read_text()), **model_kw)
        self.view_all = view_all            # every die visible from anywhere (scenario tests)
        self.drop, self.beating = False, True
        self.obstacles = []
        clock.hooks.append(self.tick)
        self.tick(clock.now())

    def drone(self):
        x, y, _ = self.ctx.field_config.frame.to_field(self.px4.x, self.px4.y, self.px4.z)
        return (x, y, -self.px4.z)

    def tick(self, now):
        from gnc_offboard.camera_model import contains_point
        from gnc_offboard.mission.cameras import footprints
        drone = self.drone()
        if self.view_all:
            view = close = lambda x, y: True  # noqa: E731
        else:
            pose = (self.px4.x, self.px4.y, self.px4.z, self.px4.yaw)
            last = getattr(self, '_pose', None)
            if last is None or max(abs(a - b) for a, b in zip(pose, last)) > 0.05:
                odom = odometry(*pose)
                self._polys = footprints(self.ctx.cameras, odom.position, odom.q,
                                         self.ctx.field_config.frame,
                                         self.ctx.field_config.bounds)
                self._pose = pose
            polys = self._polys
            view = lambda x, y: any(contains_point(p, (x, y)) for p in polys.values())  # noqa: E731
            close = lambda x, y: contains_point(polys['down'], (x, y))  # noqa: E731
        target = self.ctx.target_centroid
        out = self.model.update(drone, view, target[:2] if target else None, drop=self.drop,
                                close_view=close)
        up = self.ctx.update_input
        up('dice_map', SimpleNamespace(dice=[
            SimpleNamespace(color=c, centroid=SimpleNamespace(x=p[0], y=p[1], z=p[2]))
            for c, p in out.dice]), now)
        rel = out.target_relative
        up('target_relative', SimpleNamespace(
            valid=rel is not None,
            offset=SimpleNamespace(x=rel[0], y=rel[1], z=rel[2]) if rel else
            SimpleNamespace(x=0.0, y=0.0, z=0.0)), now)
        if out.color:
            up('color_check', SimpleNamespace(color_guess=out.color, confidence=1.0), now)
        up('obstacles', SimpleNamespace(obstacles=[
            SimpleNamespace(x=x, y=y, radius=r) for x, y, r in self.obstacles]), now)
        if self.beating:
            up('perception_heartbeat', heartbeat(), now)


@pytest.fixture
def fake_perception(ctx, clock, cfg):
    """fake_perception(px4, **PerceptionModel options)"""
    return lambda px4, **kw: FakePerception(ctx, px4, clock, cfg, **kw)


def flight(ctx, io, cfg, clock, fake_px4, fake_perception, color='red', px4_kw=None, **perc_kw):
    """Start a full run: fake PX4 + fake Perception + a retrieve for `color`."""
    px4 = fake_px4(io, perception=False, **(px4_kw or {}))
    perception = fake_perception(px4, **perc_kw)
    ctx.update_input('intent', intent('retrieve', color, 1), clock.now())
    return px4, perception
