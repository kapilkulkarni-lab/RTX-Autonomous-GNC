import math
from types import SimpleNamespace

import pytest

from gnc_offboard.mission.checks import (
    ColorResult, centering_error, check_color, confirmed_color, invalid_for, is_centered,
    is_stale, read_complete, stage_tolerance, touchdown_confirmed, update_below_since)
from gnc_offboard.mission.context import Stamped


def heartbeat(echo, rx_time):
    return Stamped(SimpleNamespace(last_state_seq=0, last_nfc_read_id=echo, link_ok=True), rx_time)


# --- read_complete (option B) ---

def test_read_complete_when_echo_matches_after_publish():
    assert read_complete(3, published_at=10.0, heartbeat=heartbeat(3, 10.2))


def test_read_not_complete_without_heartbeat():
    assert not read_complete(3, published_at=10.0, heartbeat=None)


def test_read_not_complete_on_echo_of_other_read_id():
    assert not read_complete(3, published_at=10.0, heartbeat=heartbeat(2, 10.2))
    assert not read_complete(3, published_at=10.0, heartbeat=heartbeat(4, 10.2))


def test_read_not_complete_from_heartbeat_before_publish():
    # GNC restarted: read_id 1 again, but comms is still echoing the old run's 1.
    assert not read_complete(1, published_at=10.0, heartbeat=heartbeat(1, 9.9))
    assert not read_complete(1, published_at=10.0, heartbeat=heartbeat(1, 10.0))
    assert read_complete(1, published_at=10.0, heartbeat=heartbeat(1, 10.1))


def test_read_never_completes_on_zero_echo_and_has_no_timeout():
    assert not read_complete(1, published_at=0.0, heartbeat=heartbeat(0, 1e6))
    assert not read_complete(0, published_at=0.0, heartbeat=heartbeat(0, 1.0))


# --- check_color ---

@pytest.mark.parametrize('confirmed, target, expected', [
    ('blue', 'blue', ColorResult.RIGHT),
    ('Blue ', 'blue', ColorResult.RIGHT),
    ('red', 'blue', ColorResult.WRONG),
    ('', 'blue', ColorResult.PENDING),
    (None, 'blue', ColorResult.PENDING),
    ('  ', 'blue', ColorResult.PENDING),
])
def test_check_color(confirmed, target, expected):
    assert check_color(confirmed, target) is expected


def test_no_nothing_result():
    assert {r.name for r in ColorResult} == {'RIGHT', 'WRONG', 'PENDING'}


def test_confirmed_color_reads_only_color_field():
    low = Stamped(SimpleNamespace(color_guess='blue', confidence=0.01), 1.0)
    assert check_color(confirmed_color(low), 'blue') is ColorResult.RIGHT
    assert confirmed_color(None) is None


# --- staleness ---

def test_is_stale():
    assert is_stale(None, now=5.0, max_age_s=1.0)
    s = Stamped(object(), rx_time=4.0)
    assert not is_stale(s, now=4.9, max_age_s=1.0)
    assert not is_stale(s, now=5.0, max_age_s=1.0)
    assert is_stale(s, now=5.01, max_age_s=1.0)


def test_heartbeat_that_stops_updating_goes_stale():
    s = Stamped(object(), rx_time=0.0)
    ages = [t / 10 for t in range(0, 30)]
    first_stale = next(t for t in ages if is_stale(s, t, max_age_s=2.0))
    assert math.isclose(first_stale, 2.1)


def test_invalid_for():
    assert not invalid_for(None, now=100.0, limit_s=1.0)
    assert not invalid_for(10.0, now=11.0, limit_s=1.0)
    assert invalid_for(10.0, now=11.01, limit_s=1.0)


# --- centering ---

def test_centering_error():
    assert math.isclose(centering_error(0.03, 0.04), 0.05)
    assert is_centered(0.03, 0.04, 0.05)
    assert not is_centered(0.03, 0.04, 0.049)


@pytest.mark.parametrize('height, tol', [
    (1.5, 0.10), (1.0, 0.10), (0.75, 0.10), (0.5, 0.05), (0.3, 0.05), (0.25, 0.03), (0.1, 0.03)])
def test_stage_tolerance(height, tol):
    assert stage_tolerance(height, [1.0, 0.5, 0.25], [0.10, 0.05, 0.03]) == tol


def test_stage_tolerance_rejects_bad_config():
    with pytest.raises(ValueError):
        stage_tolerance(1.0, [1.0, 0.5], [0.1])
    with pytest.raises(ValueError):
        stage_tolerance(1.0, [0.5, 1.0], [0.1, 0.05])
    with pytest.raises(ValueError):
        stage_tolerance(1.0, [], [])


# --- touchdown ---

def test_touchdown_from_rangefinder_hold():
    since = None
    # 0.25 s steps are exact in binary, so the hold boundary is exact too.
    for i, r in enumerate([0.30, 0.10, 0.04, 0.04, 0.04, 0.04]):
        since = update_below_since(since, r, touchdown_range_m=0.05, now=i * 0.25)
    # Low from t=0.5 on.
    assert since == 0.5
    assert touchdown_confirmed(since, now=1.0, touchdown_hold_s=0.5)
    assert not touchdown_confirmed(since, now=0.75, touchdown_hold_s=0.5)


def test_touchdown_resets_on_bounce_or_missing_reading():
    since = update_below_since(None, 0.04, 0.05, now=1.0)
    assert since == 1.0
    assert update_below_since(since, 0.04, 0.05, now=1.2) == 1.0
    assert update_below_since(since, 0.08, 0.05, now=1.3) is None
    assert update_below_since(since, None, 0.05, now=1.3) is None
    assert update_below_since(since, float('nan'), 0.05, now=1.3) is None
    assert not touchdown_confirmed(None, now=10.0, touchdown_hold_s=0.5)
