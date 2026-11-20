"""Pure Milestone 5 modules: field config, coverage memory, frontier from a coverage memory,
search patterns, dice-map helpers, path follower, and the shared Perception model."""
import math
from pathlib import Path
from types import SimpleNamespace

import pytest

from conftest import TEST_MANIFEST
from gnc_offboard import search_planners as sp
from gnc_offboard.guidance.path_follower import PathFollower
from gnc_offboard.mission.coverage import CoverageMap
from gnc_offboard.mission.dice import count_failure, dice_of, find_die, obstacles_of
from gnc_offboard.mission.field import FieldConfig
from gnc_offboard.mission.search import PATTERNS, area_covered, plan_search
from gnc_offboard.sim.perception_model import Die, PerceptionModel, square_view

FIELD_YAML = Path(__file__).resolve().parents[1] / 'config' / 'field.yaml'
BOUNDS = (-6.858, -6.858, 6.858, 6.858)


# ---- field config -------------------------------------------------------------------------------

def test_field_from_manifest():
    fc = FieldConfig.load(TEST_MANIFEST)
    assert fc.bounds == BOUNDS
    assert fc.start_center == (-6.096, -6.096)
    assert fc.px4_origin == (-6.096, -6.096)
    assert fc.frame.to_field(0.0, 0.0, 0.0) == (-6.096, -6.096, 0.0)   # PX4 origin = spawn


def test_field_from_hardware_yaml_has_the_same_shape():
    fc = FieldConfig.load(FIELD_YAML)
    assert fc.bounds == BOUNDS and fc.start_zone[0] == -6.858
    assert 'TBD' in FIELD_YAML.read_text()


def test_field_area_and_bad_bounds():
    fc = FieldConfig.load(TEST_MANIFEST)
    a = fc.area(1.0)
    assert (a.xmin, a.xmax) == pytest.approx((-5.858, 5.858))
    with pytest.raises(ValueError):
        fc.area(7.0)
    with pytest.raises(ValueError):
        FieldConfig.from_yaml({'field': {'bounds': [1, 0, 0, 1]}, 'start_zone': {
            'bounds': [0, 0, 1, 1], 'center': [0, 0]}, 'px4_origin': [0, 0]})


# ---- coverage memory ----------------------------------------------------------------------------

def test_coverage_marks_square_footprint_once():
    cov = CoverageMap(BOUNDS, 0.5)
    new = cov.mark(0.0, 0.0, 2.0)
    assert new == 16 and cov.mark(0.0, 0.0, 2.0) == 0          # 4 x 4 cells, no double count
    assert cov.is_seen(0.4, -0.4) and not cov.is_seen(1.6, 0.0)
    assert not cov.is_seen(100.0, 0.0)
    cov.clear()
    assert cov.count == 0 and not cov.is_seen(0.0, 0.0)


def test_coverage_ignores_bad_input():
    cov = CoverageMap(BOUNDS, 0.5)
    assert cov.mark(float('nan'), 0.0, 2.0) == 0 and cov.mark(0.0, 0.0, 0.0) == 0
    with pytest.raises(ValueError):
        CoverageMap(BOUNDS, 0.0)


# ---- frontier from coverage memory, and patterns ------------------------------------------------

def area():
    return FieldConfig.load(TEST_MANIFEST).area(1.0)


def test_frontier_skips_covered_ground():
    cov = CoverageMap(BOUNDS, 0.5)
    for x in [i * 0.5 for i in range(-14, 15)]:         # cover the whole west half
        for y in [j * 0.5 for j in range(-14, 15)]:
            if x < 0:
                cov.mark(x, y, 0.5)
    fp = sp.footprint(1.5, 70.0)
    path = plan_search('frontier', area(), (-5.0, -5.0), cov, fp, 1.5, 1.0, [], 0.5)
    assert path
    # Every leg ends on new ground: no waypoint deep inside the covered half.
    assert all(x > -fp for x, _ in path)
    fresh = sp.frontier(area(), (-5.0, -5.0), fp, pad=1.0, cell=0.5)
    assert sp.path_length((-5.0, -5.0), path) < sp.path_length((-5.0, -5.0), fresh)


def test_frontier_all_covered_returns_nothing():
    cov = CoverageMap(BOUNDS, 0.5)
    cov.mark(0.0, 0.0, 20.0)
    assert plan_search('frontier', area(), (0.0, 0.0), cov, 2.0, 1.5, 1.0, [], 0.5) == []
    assert area_covered(cov, area())


@pytest.mark.parametrize('pattern', PATTERNS)
def test_every_pattern_plans_inside_the_area(pattern):
    cov = CoverageMap(BOUNDS, 0.5)
    a = area()
    path = plan_search(pattern, a, (-6.0, -6.0), cov, sp.footprint(1.5, 70.0), 1.6, 1.0,
                       [(0.0, 0.0, 0.5)], 0.5)
    assert len(path) > 2 and all(a.contains(p) for p in path)
    assert plan_search(pattern, a, (0, 0), _full(), 2.0, 1.6, 1.0, [], 0.5) == []


def _full():
    cov = CoverageMap(BOUNDS, 0.5)
    cov.mark(0.0, 0.0, 20.0)
    return cov


def test_unknown_pattern_rejected():
    with pytest.raises(ValueError, match='search_pattern'):
        plan_search('spiral', area(), (0, 0), CoverageMap(BOUNDS, 0.5), 2.0, 1.6, 1.0, [], 0.5)


def test_frontier_without_seen_unchanged():
    a = area()
    assert sp.frontier(a, (-5, -5), 2.0, pad=1.0) == sp.frontier(a, (-5, -5), 2.0, pad=1.0,
                                                                 seen=None)


# ---- dice map helpers ---------------------------------------------------------------------------

def dmap(*dice):
    return SimpleNamespace(dice=[SimpleNamespace(color=c, centroid=SimpleNamespace(x=x, y=y, z=0.0))
                                 for c, x, y in dice])


def test_find_die_nearest_and_excluded():
    m = dmap(('red', 4.0, 0.0), ('Red ', 1.0, 0.0), ('blue', 0.0, 0.0))
    assert dice_of(m)[1][0] == 'red'
    assert find_die(m, 'red', near=(0, 0)) == (1.0, 0.0, 0.0)
    assert find_die(m, 'red', near=(0, 0), exclude=[(1.1, 0.0)], exclude_radius_m=0.3) == \
        (4.0, 0.0, 0.0)
    assert find_die(m, 'black') is None and find_die(m, '') is None and find_die(None, 'red') is None
    assert find_die(m, 'red', fits=lambda p: p[0] > 2) == (4.0, 0.0, 0.0)


def test_count_failure_per_die():
    failures = []
    assert count_failure(failures, (2.0, 2.0, 0.0), 0.3) == 1
    assert count_failure(failures, (2.1, 2.1, 0.0), 0.3) == 2     # same die, moved a little
    assert count_failure(failures, (-4.0, 1.0, 0.0), 0.3) == 1    # another die
    assert failures == [[(2.0, 2.0, 0.0), 2], [(-4.0, 1.0, 0.0), 1]]


def test_obstacles_of():
    msg = SimpleNamespace(obstacles=[SimpleNamespace(x=1, y=2, radius=0.4)])
    assert obstacles_of(msg) == [(1.0, 2.0, 0.4)] and obstacles_of(None) == []


# ---- path follower ------------------------------------------------------------------------------

def test_path_follower_visits_waypoints_in_order():
    f = PathFollower([(2.0, 0.0, -1.5), (2.0, 2.0, -1.5)], (0.0, 0.0, -1.5), 0.3, 1.0, 0.5, 0.25)
    t, pos, visited = 0.0, (0.0, 0.0, -1.5), []
    while not f.done and t < 60:
        sp_, yaw = f.update(t, pos)
        pos = sp_                                        # perfect tracking
        if f.target not in visited and f.target is not None:
            visited.append(f.target)
        t += 0.05
    assert f.done and visited == [(2.0, 0.0, -1.5), (2.0, 2.0, -1.5)]
    assert yaw == 0.3 and f.update(t, pos)[0] == (2.0, 2.0, -1.5)


def test_path_follower_waits_until_vehicle_arrives():
    f = PathFollower([(1.0, 0.0, 0.0)], (0.0, 0.0, 0.0), 0.0, 1.0, 1.0, 0.2)
    for k in range(200):
        f.update(k * 0.05, (0.0, 0.0, 0.0))             # vehicle stuck at the start
    assert not f.done
    f.update(20.0, (0.95, 0.0, 0.0))
    assert f.done


# ---- shared Perception model --------------------------------------------------------------------

DICE = [Die('black', -4.5, -1.0), Die('red', 2.0, 2.0)]


def test_model_reports_only_seen_dice():
    m = PerceptionModel(DICE)
    out = m.update((0.0, 0.0, 1.5), square_view((0.0, 0.0), 2.0))
    assert out.dice == [] and out.color is None and out.target_relative is None
    out = m.update((2.0, 2.0, 1.5), square_view((2.0, 2.0), 2.0))
    assert out.dice == [('red', (2.0, 2.0, 0.0))]
    out = m.update((0.0, 0.0, 1.5), square_view((0.0, 0.0), 2.0))
    assert out.dice == [('red', (2.0, 2.0, 0.0))]       # remembered once seen


def test_model_confirms_color_only_close_and_low():
    m = PerceptionModel(DICE, confirm_radius_m=0.3, confirm_max_alt_m=1.3)
    view = square_view((2.1, 2.0), 2.0)
    assert m.update((2.1, 2.0, 1.5), view).color is None          # too high
    assert m.update((2.5, 2.0, 1.0), square_view((2.5, 2.0), 2.0)).color is None   # too far
    assert m.update((2.1, 2.0, 1.0), view).color == 'red'


def test_model_wrong_color_first_corrected_by_close_look():
    m = PerceptionModel(DICE, wrong_color_first='red')
    out = m.update((-4.5, -1.0, 1.5), square_view((-4.5, -1.0), 2.0))
    assert out.dice == [('red', (-4.5, -1.0, 0.0))]                 # black reported as red
    out = m.update((-4.5, -1.0, 1.0), square_view((-4.5, -1.0), 2.0))
    assert out.color == 'black' and out.dice == [('black', (-4.5, -1.0, 0.0))]


def test_model_target_relative_and_drop():
    m = PerceptionModel(DICE)
    view = square_view((1.5, 1.8), 3.0)
    out = m.update((1.5, 1.8, 1.0), view, target_xy=(2.1, 2.1))
    assert out.target_relative == pytest.approx((0.5, 0.2, -1.0))
    assert m.update((1.5, 1.8, 1.0), view, target_xy=(-4.0, -4.0)).target_relative is None
    dropped = m.update((1.5, 1.8, 1.0), view, target_xy=(2.1, 2.1), drop=True)
    assert dropped.target_relative is None and dropped.color is None
    assert dropped.dice == [('red', (2.0, 2.0, 0.0))]               # the map itself stays


def test_model_from_manifest():
    import json
    m = PerceptionModel.from_manifest(json.loads(TEST_MANIFEST.read_text()))
    assert {d.color for d in m.dice} == {'black', 'blue', 'red', 'yellow'}
    assert math.isclose(m.dice[0].x, -4.5)
