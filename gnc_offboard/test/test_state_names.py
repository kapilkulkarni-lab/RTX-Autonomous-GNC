import ast
from pathlib import Path

from avc_msgs.msg import MissionState

from gnc_offboard.mission import state_names as S

MISSION_DIR = Path(S.__file__).parent


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


def test_no_other_module_hard_codes_a_state_name():
    names = set(S.ALL)
    offenders = []
    for path in MISSION_DIR.glob('*.py'):
        if path.name == 'state_names.py':
            continue
        for node in ast.walk(ast.parse(path.read_text())):
            if isinstance(node, ast.Constant) and node.value in names:
                offenders.append(f'{path.name}:{node.lineno} {node.value!r}')
    assert not offenders, offenders
