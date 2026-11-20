"""Pure Milestone 5 modules: field config, coverage memory, frontier from a coverage memory,
search patterns, dice-map helpers, path follower, and the shared Perception model."""
import math
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from conftest import TEST_MANIFEST
from gnc_offboard import search_planners as sp
from gnc_offboard.frames import FieldFrame
from gnc_offboard.camera_model import CameraModel, CoverageGrid
from gnc_offboard.guidance.path_follower import PathFollower
from gnc_offboard.mission.cameras import (down_footprint_extent, footprints, load_cameras,
                                          yaw_quaternion)
from gnc_offboard.mission.dice import count_failure, dice_of, find_die, obstacles_of
from gnc_offboard.mission.field import FieldConfig
from gnc_offboard.mission.search import (PATTERNS, area_covered, lawnmower_rows,
                                         lawnmower_skipping, plan_search, row_covered)
from gnc_offboard.sim.perception_model import Die, PerceptionModel, square_view

FIELD_YAML = Path(__file__).resolve().parents[1] / 'config' / 'field.yaml'
CAMERAS_YAML = Path(__file__).resolve().parents[1] / 'config' / 'cameras.yaml'
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


# ---- cameras ------------------------------------------------------------------------------------

def test_camera_config_values():
    cams = load_cameras(CAMERAS_YAML)
    assert set(cams) == {'d455', 'down'}
    assert (cams['d455'].hfov_deg, cams['d455'].vfov_deg, cams['d455'].pitch_down_deg,
            cams['d455'].max_range_m) == (90.0, 65.0, 45.0, 6.0)
    assert (cams['down'].hfov_deg, cams['down'].vfov_deg, cams['down'].pitch_down_deg) == \
        (120.0, 120.0, 90.0)
    text = CAMERAS_YAML.read_text()
    assert text.count('TBD') >= 8                      # tilt, lens, offsets, ranges


def test_camera_config_rejects_missing_or_extra(tmp_path):
    import yaml
    data = yaml.safe_load(CAMERAS_YAML.read_text())
    bad = tmp_path / 'c.yaml'
    del data['cameras']['down']['roll_deg']
    bad.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError, match='down'):
        load_cameras(bad)
    data = yaml.safe_load(CAMERAS_YAML.read_text())
    data['cameras']['third'] = data['cameras']['down']
    bad.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError, match='exactly'):
        load_cameras(bad)


def test_two_camera_projection_at_search_altitude():
    cams = load_cameras(CAMERAS_YAML)
    frame = FieldFrame(0.0, 0.0)
    polys = footprints(cams, (0.0, 0.0, -2.5), yaw_quaternion(0.0), frame, (-50, -50, 50, 50))
    down, d455 = polys['down'], polys['d455']
    # Down: 120 deg square under the drone, 2 * 2.5 * tan(60) = 8.66 m, clipped by the 6 m range.
    assert np.ptp(down[:, 0]) == pytest.approx(2 * 2.5 * math.tan(math.radians(60)), abs=0.1)
    assert abs(down[:, 0].mean()) < 0.05 and abs(down[:, 1].mean()) < 0.05
    assert np.linalg.norm(down, axis=1).max() <= 6.0 + 1e-6
    # D455: 45 deg down, facing north (yaw 0): its footprint lies ahead, within 6 m.
    assert d455[:, 1].min() > 0.5 and d455[:, 1].max() <= 6.0 + 1e-6


def test_down_footprint_extent_follows_heading():
    cams = load_cameras(CAMERAS_YAML)
    wx, wy = down_footprint_extent(cams, 2.5, 0.0)
    assert wx == pytest.approx(wy, abs=0.05) and wx == pytest.approx(8.66, abs=0.1)
    wx45, _ = down_footprint_extent(cams, 2.5, math.pi / 4)     # rotated square: wider band
    assert wx45 > wx
    small = CameraModel(hfov_deg=60, vfov_deg=60, pitch_down_deg=90, forward_m=0)
    assert down_footprint_extent({'down': small}, 1.0, 0.0)[0] == pytest.approx(
        2 * math.tan(math.radians(30)), abs=1e-6)


def test_projection_below_ground_sees_nothing():
    cams = load_cameras(CAMERAS_YAML)
    polys = footprints(cams, (0.0, 0.0, 0.5), yaw_quaternion(0.0), FieldFrame(0, 0), BOUNDS)
    assert all(len(p) == 0 for p in polys.values())


# ---- coverage memory (camera_model.CoverageGrid) ------------------------------------------------

def grid():
    return CoverageGrid(BOUNDS, 0.5)


def square(cx, cy, side):
    h = side / 2
    return np.array([[cx - h, cy - h], [cx + h, cy - h], [cx + h, cy + h], [cx - h, cy + h]])


def test_coverage_marks_both_camera_footprints():
    cams = load_cameras(CAMERAS_YAML)
    g = grid()
    polys = footprints(cams, (0.0, 0.0, -2.5), yaw_quaternion(0.0), FieldFrame(0.0, 0.0), BOUNDS)
    for poly in polys.values():
        g.mark(poly)
    assert g.is_seen(0.0, 0.0) and g.is_seen(0.0, 5.0)         # under it, and the D455 ahead
    assert not g.is_seen(0.0, -5.5) and not g.is_seen(6.0, 6.0)
    only_down = grid()
    only_down.mark(polys['down'])
    assert g.count > only_down.count


def test_coverage_grid_mark_clear_and_union_update():
    g = grid()
    g.mark(square(0, 0, 2.0))
    assert g.count == 16 and g.is_seen(0.4, -0.4) and not g.is_seen(1.6, 0.0)
    assert not g.is_seen(100.0, 0.0)
    g.update_many([square(-3, 0, 1.0), square(3, 0, 1.0)], 0.0)
    assert g.is_seen(-3, 0) and g.is_seen(3, 0) and g.in_view[g.searched].sum() == 8
    g.clear()
    assert g.count == 0 and not g.is_seen(0, 0)
    g.update(square(0, 0, 1.0), 0.0)                           # single-camera API still works
    assert g.count == 4


# ---- search patterns, row skipping --------------------------------------------------------------

def area():
    return FieldConfig.load(TEST_MANIFEST).area(1.0)


def full():
    g = grid()
    g.mark(square(0, 0, 20.0))
    return g


def test_lawnmower_rows_match_the_planner():
    from gnc_offboard import search_planners as sp
    rows = lawnmower_rows(area(), (-6.0, -6.0), 3.0)
    flat = [p for row in rows for p in row]
    assert flat[0] == pytest.approx((-5.858, -5.858))         # starts at the nearest corner
    assert all(abs(a[1] - b[1]) < 1e-9 for a, b in rows)      # rows along x
    assert len(rows) == len(sp._row_positions(area().height, 3.0))
    rows_y = lawnmower_rows(area(), (-6.0, -6.0), 3.0, 'y')
    assert all(abs(a[0] - b[0]) < 1e-9 for a, b in rows_y)
    with pytest.raises(ValueError):
        lawnmower_rows(area(), (0, 0), 3.0, 'z')


def test_covered_rows_are_skipped():
    g = grid()
    rows = lawnmower_rows(area(), (-6.0, -6.0), 3.0)
    first, second = rows[0], rows[1]
    g.mark(np.array([[-7, first[0][1] - 1.6], [7, first[0][1] - 1.6],
                     [7, first[0][1] + 1.6], [-7, first[0][1] + 1.6]]))   # the first row's swath
    assert row_covered(g, first, 3.0) and not row_covered(g, second, 3.0)
    path = lawnmower_skipping(area(), (-6.0, -6.0), g, 3.0, 'x')
    assert all(abs(p[1] - first[0][1]) > 1e-6 for p in path)  # first row not flown
    assert path[0][1] == pytest.approx(second[0][1])
    fresh = lawnmower_skipping(area(), (-6.0, -6.0), grid(), 3.0, 'x')
    assert len(path) == len(fresh) - 2


def test_partly_covered_row_is_flown():
    g = grid()
    first = lawnmower_rows(area(), (-6.0, -6.0), 3.0)[0]
    g.mark(np.array([[-7, -7], [0, -7], [0, -3], [-7, -3]]))   # west half of the swath only
    assert not row_covered(g, first, 3.0)
    assert lawnmower_skipping(area(), (-6.0, -6.0), g, 3.0, 'x')[0][1] == pytest.approx(
        first[0][1])


def test_rows_entered_at_the_nearer_end():
    path = lawnmower_skipping(area(), (5.5, -6.0), grid(), 3.0, 'x')
    assert path[0][0] == pytest.approx(5.858)                  # starts on the east side


def test_frontier_skips_covered_ground():
    g = grid()
    g.mark(np.array([[-7, -7], [0, -7], [0, 7], [-7, 7]]))    # the whole west half
    fp = 3.0
    path = plan_search('frontier', area(), (-5.0, -5.0), g, fp, 2.4, 1.0, [], 0.5)
    assert path and all(x > -fp for x, _ in path)
    fresh = sp_frontier_fresh(fp)
    assert sp.path_length((-5.0, -5.0), path) < sp.path_length((-5.0, -5.0), fresh)


def sp_frontier_fresh(fp):
    return sp.frontier(area(), (-5.0, -5.0), fp, pad=1.0, cell=0.5)


def test_all_covered_returns_nothing():
    for pattern in PATTERNS:
        assert plan_search(pattern, area(), (0.0, 0.0), full(), 3.0, 2.4, 1.0, [], 0.5) == []
    assert area_covered(full(), area())


@pytest.mark.parametrize('pattern', PATTERNS)
def test_every_pattern_plans_inside_the_area(pattern):
    a = area()
    path = plan_search(pattern, a, (-6.0, -6.0), grid(), 3.0, 2.4, 1.0, [(0.0, 0.0, 0.5)], 0.5)
    assert len(path) > 2 and all(a.contains(p) for p in path)


def test_unknown_pattern_rejected():
    with pytest.raises(ValueError, match='search_pattern'):
        plan_search('spiral', area(), (0, 0), grid(), 2.0, 1.6, 1.0, [], 0.5)


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
