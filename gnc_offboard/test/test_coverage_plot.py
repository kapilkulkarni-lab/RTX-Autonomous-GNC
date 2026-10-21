import pytest

from gnc_offboard.coverage_plot import coverage_summary


def test_old_export_area_is_clipped_and_target_status_is_estimated():
    data = {'bounds': [0, 0, 1.2, 1.2], 'resolution_m': .5,
            'searched_cell_centres': [[1.25, 1.25]]}
    manifest = {'scenario': {'initial_target': 'yellow'},
                'targets': [{'color': 'yellow', 'x': 1.1, 'y': 1.1}]}
    area, arena, percent, status = coverage_summary(data, manifest)
    assert area == pytest.approx(.04)
    assert arena == pytest.approx(1.44)
    assert percent == pytest.approx(100*.04/1.44)
    assert 'inside searched grid (estimate)' in status


def test_exact_target_result_takes_precedence_over_grid_estimate():
    data = {'bounds': [0, 0, 1, 1], 'resolution_m': .5,
            'searched_cell_centres': [], 'target_found': True}
    manifest = {'scenario': {'initial_target': 'yellow'},
                'targets': [{'color': 'yellow', 'x': .1, 'y': .1}]}
    assert 'YES' in coverage_summary(data, manifest)[3]
    data['target_found'] = None
    assert 'unknown' in coverage_summary(data, manifest)[3]
