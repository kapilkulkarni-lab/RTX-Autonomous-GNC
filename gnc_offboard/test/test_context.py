import dataclasses
import threading

import pytest

from gnc_offboard.mission.context import INPUT_NAMES, Inputs, MissionContext, Stamped
from gnc_offboard.mission.io import FakeIO


def test_inputs_start_empty():
    snap = MissionContext().snapshot()
    assert all(getattr(snap, n) is None for n in INPUT_NAMES)


def test_update_stores_whole_message_and_receive_time():
    ctx = MissionContext()
    msg = object()
    ctx.update_input('llm_heartbeat', msg, now=3.5)
    s = ctx.snapshot().llm_heartbeat
    assert s == Stamped(msg, 3.5)
    assert s.msg is msg


def test_unknown_input_rejected():
    with pytest.raises(KeyError):
        MissionContext().update_input('nope', object(), now=0.0)


def test_snapshot_is_frozen_and_unaffected_by_later_updates():
    ctx = MissionContext()
    ctx.update_input('color_check', 'first', now=1.0)
    snap = ctx.snapshot()
    ctx.update_input('color_check', 'second', now=2.0)
    assert snap.color_check.msg == 'first'
    with pytest.raises(dataclasses.FrozenInstanceError):
        snap.color_check = None


def test_mission_time_and_read_ids():
    ctx = MissionContext()
    assert ctx.mission_time_s(100.0) == 0.0
    ctx.mission_start_time = 100.0
    assert ctx.mission_time_s(130.0) == 30.0
    assert [ctx.next_read_id() for _ in range(3)] == [1, 2, 3]


def test_concurrent_updates_and_snapshots():
    ctx = MissionContext()

    def writer():
        for i in range(2000):
            ctx.update_input('vehicle_status', i, now=float(i))

    t = threading.Thread(target=writer)
    t.start()
    for _ in range(2000):
        s = ctx.snapshot().vehicle_status
        assert s is None or s.msg == s.rx_time
    t.join()
    assert ctx.snapshot().vehicle_status.msg == 1999


def test_inputs_match_names():
    assert tuple(f.name for f in dataclasses.fields(Inputs)) == INPUT_NAMES


def test_fake_io_records_calls_in_order():
    io = FakeIO()
    io.arm()
    io.set_offboard()
    io.set_setpoint(1.0, 2.0, 1.5, 0.0)
    io.nfc_command('start')
    io.gripper_command('close')
    io.publish_nfc_result('msg')
    io.request_state_publish()
    assert io.names() == ['arm', 'set_offboard', 'set_setpoint', 'nfc_command',
                          'gripper_command', 'publish_nfc_result', 'request_state_publish']
    assert io.calls[2] == ('set_setpoint', (1.0, 2.0, 1.5, 0.0))
    with pytest.raises(ValueError):
        io.nfc_command('go')
    with pytest.raises(ValueError):
        io.gripper_command('grab')
