import pytest
from avc_msgs.msg import MissionState

from gnc_offboard.mission import state_names as S
from gnc_offboard.mission.state_msg import MissionStateBuilder, clean_reason


@pytest.mark.parametrize('state', S.ALL)
def test_every_state_publishes_its_name(state):
    kwargs = {'paused_from': S.SEARCHING} if state == S.PAUSED else {}
    msg = MissionStateBuilder().build(state=state, **kwargs)
    assert isinstance(msg, MissionState)
    assert msg.state == state


def test_comms_states_use_comms_strings():
    b = MissionStateBuilder()
    assert b.build(state=S.SEARCHING).state == MissionState.STATE_SEARCHING
    assert b.build(state=S.PAUSED, paused_from=S.READING).state == MissionState.STATE_PAUSED


def test_seq_goes_up_on_every_publish():
    b = MissionStateBuilder()
    seqs = [b.build(state=S.STANDBY).seq for _ in range(5)]
    assert seqs == [1, 2, 3, 4, 5]


def test_seq_wraps_to_one_not_zero():
    b = MissionStateBuilder()
    b.seq = 2 ** 32 - 1
    assert b.build(state=S.STANDBY).seq == 1


def test_previous_state_is_paused_from_while_paused():
    b = MissionStateBuilder()
    msg = b.build(state=S.PAUSED, previous_state=S.TAKEOFF, paused_from=S.DESCENDING)
    assert msg.previous_state == S.DESCENDING
    msg = b.build(state=S.SEARCHING, previous_state=S.TAKEOFF, paused_from=S.DESCENDING)
    assert msg.previous_state == S.TAKEOFF


def test_paused_needs_paused_from():
    b = MissionStateBuilder()
    with pytest.raises(ValueError):
        b.build(state=S.PAUSED)
    with pytest.raises(ValueError):
        b.build(state=S.PAUSED, paused_from=S.PAUSED)


def test_reason_under_80_chars_no_pipe():
    long = 'read failed at idle | retrying motors off ' * 5
    msg = MissionStateBuilder().build(state=S.READING, reason=long)
    assert len(msg.reason) < 80
    assert '|' not in msg.reason
    assert msg.reason.startswith('read failed at idle retrying motors off')


def test_clean_reason():
    assert clean_reason('a|b') == 'ab'
    assert clean_reason('line one\nline two') == 'line one line two'
    assert clean_reason(None) == ''
    assert len(clean_reason('x' * 80)) == 79
    assert clean_reason('x' * 79) == 'x' * 79


def test_intent_fields():
    b = MissionStateBuilder()
    msg = b.build(state=S.STANDBY, last_intent_id=6,
                  last_intent_result=MissionState.RESULT_REJECTED,
                  reason='resume ignored: not paused')
    assert msg.last_intent_id == 6
    assert msg.last_intent_result == 'rejected'
    assert b.build(state=S.STANDBY).last_intent_result == ''
    with pytest.raises(ValueError):
        b.build(state=S.STANDBY, last_intent_result='ok')


def test_unknown_states_rejected():
    b = MissionStateBuilder()
    with pytest.raises(ValueError):
        b.build(state='HOVERING')
    with pytest.raises(ValueError):
        b.build(state=S.STANDBY, previous_state='HOVERING')


def test_stamp_and_other_fields():
    msg = MissionStateBuilder().build(state=S.SEARCHING, active_target='blue',
                                      mission_time_s=12.5, stamp_ns=3_250_000_001)
    assert (msg.header.stamp.sec, msg.header.stamp.nanosec) == (3, 250_000_001)
    assert msg.active_target == 'blue'
    assert msg.mission_time_s == pytest.approx(12.5)


def test_temporary_on_die_test_is_publishable():
    b = MissionStateBuilder()
    msg = b.build(state=S.ON_DIE_TEST, previous_state=S.TAKEOFF)
    assert msg.state == 'ON_DIE_TEST'
    assert b.build(state=S.LANDING, previous_state=S.ON_DIE_TEST).previous_state == 'ON_DIE_TEST'
