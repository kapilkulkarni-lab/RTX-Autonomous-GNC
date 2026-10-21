import math

import numpy as np
import pytest

from gnc_offboard.camera_model import CameraModel, CoverageGrid, contains_point
from gnc_offboard.frames import FieldFrame


FRAME = FieldFrame(0, 0)
BOUNDS = (-100, -100, 100, 100)


def project(model, q=(1, 0, 0, 0), position=(0, 0, -2), bounds=BOUNDS):
    return model.project(position, q, FRAME, bounds)


def test_nadir_has_analytic_square_footprint():
    model = CameraModel(hfov_deg=90, vfov_deg=90, pitch_down_deg=90, forward_m=0)
    origin, polygon = project(model)
    np.testing.assert_allclose(origin, (0, 0, 2))
    np.testing.assert_allclose(np.min(polygon, axis=0), (-2, -2), atol=1e-10)
    np.testing.assert_allclose(np.max(polygon, axis=0), (2, 2), atol=1e-10)


def test_45_degree_front_mount_points_north_and_yaw_rotates_it():
    model = CameraModel(vfov_deg=30, forward_m=0)
    _, polygon = project(model)
    assert polygon[:, 1].min() == pytest.approx(2/math.tan(math.radians(60)))
    assert polygon[:, 1].max() == pytest.approx(2/math.tan(math.radians(30)))
    _, turned = project(model, q=(0, 0, 0, 1))
    np.testing.assert_allclose(turned.min(axis=0), -polygon.max(axis=0), atol=1e-9)
    np.testing.assert_allclose(turned.max(axis=0), -polygon.min(axis=0), atol=1e-9)


def test_translation_rotates_with_vehicle_and_changes_height():
    model = CameraModel(forward_m=.3, right_m=.1, down_m=.2)
    origin, _ = project(model)
    np.testing.assert_allclose(origin, (.1, .3, 1.8))
    origin, _ = project(model, q=(0, 0, 0, 1))
    np.testing.assert_allclose(origin, (-.1, -.3, 1.8))


def test_vehicle_pitch_affects_projection():
    model = CameraModel(pitch_down_deg=45, forward_m=0)
    # PX4 positive pitch raises the nose: a 45 degree nose-up pose looks at horizon.
    q = (math.cos(math.pi/8), 0, math.sin(math.pi/8), 0)
    _, polygon = project(model, q=q)
    assert len(polygon) >= 3
    assert polygon[:, 1].max() > 20


def test_horizon_is_finite_and_upward_camera_sees_nothing():
    _, polygon = project(CameraModel(pitch_down_deg=0, max_range_m=10, forward_m=0))
    assert len(polygon) >= 3
    assert np.isfinite(polygon).all()
    assert np.linalg.norm(polygon, axis=1).max() <= 10+1e-9
    _, upward = project(CameraModel(pitch_down_deg=-90))
    assert len(upward) == 0
    _, underground = project(CameraModel(), position=(0, 0, 1))
    assert len(underground) == 0


def test_field_clipping_and_persistent_coverage():
    bounds = (-1, -1, 1, 1)
    model = CameraModel(pitch_down_deg=90, forward_m=0, hfov_deg=90, vfov_deg=90)
    _, polygon = project(model, bounds=bounds)
    assert (polygon >= -1).all() and (polygon <= 1).all()
    grid = CoverageGrid(bounds, .25)
    grid.update(polygon)
    assert grid.area_m2 == 4
    grid.update(np.empty((0, 2)))
    assert grid.area_m2 == 4


@pytest.mark.parametrize('kwargs', [dict(hfov_deg=180), dict(vfov_deg=0),
                                    dict(pitch_down_deg=float('nan')), dict(max_range_m=-1)])
def test_invalid_parameters(kwargs):
    with pytest.raises(ValueError):
        CameraModel(**kwargs)


def test_invalid_quaternion_rejected():
    with pytest.raises(ValueError):
        project(CameraModel(), q=(0, 0, 0, 0))


def test_target_uses_exact_footprint_including_boundary():
    polygon = np.array([[0, 0], [1, 0], [1, .01], [0, .01]])
    assert contains_point(polygon, (.5, .005))
    assert contains_point(polygon, (1, .005))
    assert not contains_point(polygon, (.5, .1))
    assert not contains_point([], (0, 0))
    assert not contains_point(np.array([[0, 0], [0, 0], [0, 0]]), (0, 0))


def test_coverage_area_clips_partial_edge_cells():
    grid = CoverageGrid((0, 0, 1.2, 1.2), .5)
    grid.searched[:] = True
    assert grid.area_m2 == pytest.approx(1.44)
