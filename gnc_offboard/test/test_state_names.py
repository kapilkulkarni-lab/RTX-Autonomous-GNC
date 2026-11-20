import ast
from pathlib import Path

from avc_msgs.msg import MissionState

from gnc_offboard.mission import state_names as S

MISSION_DIR = Path(S.__file__).parent
# Packages that may not hard-code a mission state name. (search_base.WaypointMission has its
# own unrelated internal states.)
CHECKED_DIRS = [MISSION_DIR, MISSION_DIR.parent / 'ros']


def test_fifteen_unique_states():
    assert len(S.ALL) == 15
    assert len(set(S.ALL)) == 15


def test_comms_states_come_from_avc_msgs():
    comms = {getattr(MissionState, n) for n in dir(MissionState) if n.startswith('STATE_')}
    assert len(comms) == 13
    assert comms <= set(S.ALL)
    assert set(S.ALL) - comms == {S.DESCENDING, S.LIFTOFF}


def test_placeholders():
    assert S.DESCENDING == 'DESCENDING'
    assert S.LIFTOFF == 'LIFTOFF'


def test_groups():
    assert S.ON_DIE == {S.NFC_READY, S.READING, S.RETRIEVING}
    assert S.END_STATES == {S.RETURN_CONFIRMED, S.ABORTED}


def test_temporary_target_hover_kept_out_of_all():
    assert S.TARGET_HOVER == 'TARGET_HOVER'
    assert S.TARGET_HOVER not in S.ALL
    assert S.PUBLISHABLE == S.ALL + (S.TARGET_HOVER,)
    assert not hasattr(S, 'HOVER_TEST')         # removed in Milestone 5
    assert S.TARGET_HOVER in S.AIRBORNE


def test_no_other_module_hard_codes_a_state_name():
    names = set(S.PUBLISHABLE)
    offenders = []
    for path in (p for d in CHECKED_DIRS for p in d.glob('*.py')):
        if path.name == 'state_names.py':
            continue
        for node in ast.walk(ast.parse(path.read_text())):
            if isinstance(node, ast.Constant) and node.value in names:
                offenders.append(f'{path.name}:{node.lineno} {node.value!r}')
    assert not offenders, offenders
