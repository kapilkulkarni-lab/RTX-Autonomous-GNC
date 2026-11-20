from pathlib import Path

import pytest
from rclpy.qos import DurabilityPolicy as D, HistoryPolicy, ReliabilityPolicy as R

from gnc_offboard.mission import qos

R_, BE = R.RELIABLE, R.BEST_EFFORT
V, TL = D.VOLATILE, D.TRANSIENT_LOCAL

# The outline's Interfaces tables, plus the agreed placeholder rule for the TBD rows.
EXPECTED = {
    # From the tables.
    '/mission/intent': (R_, V, 10),
    '/llm/heartbeat': (R_, V, 10),
    '/mission/state': (R_, TL, 1),
    '/mission/nfc_result': (R_, TL, 10),
    # Placeholders: unknown publisher -> best effort, volatile, 10.
    '/perception/dice_map': (BE, V, 10),
    '/perception/target_relative': (BE, V, 10),
    '/perception/color_check': (BE, V, 10),
    '/perception/obstacle_detections': (BE, V, 10),
    '/perception/heartbeat': (BE, V, 10),
    '/hw/nfc/status': (BE, V, 10),
    '/hw/nfc/result': (BE, V, 10),
    '/hw/gripper/status': (BE, V, 10),
    # Placeholders: our commands -> reliable, volatile, 10.
    '/hw/nfc/command': (R_, V, 10),
    '/hw/gripper/command': (R_, V, 10),
    # Placeholder: /mission/target -> reliable, transient local, 1.
    '/mission/target': (R_, TL, 1),
}


@pytest.mark.parametrize('topic', sorted(EXPECTED))
def test_profile_matches(topic):
    p = qos.BY_TOPIC[topic]
    assert (p.reliability, p.durability, p.depth) == EXPECTED[topic]
    assert p.history == HistoryPolicy.KEEP_LAST


def test_every_topic_covered():
    assert set(qos.BY_TOPIC) == set(EXPECTED)


def test_px4_best_effort():
    assert qos.PX4.reliability == BE
    assert qos.PX4.durability == TL and qos.PX4.depth == 1


def test_tbd_topics_are_exactly_the_placeholders():
    from_tables = {'/mission/intent', '/llm/heartbeat', '/mission/state', '/mission/nfc_result'}
    assert qos.TBD_TOPICS == set(EXPECTED) - from_tables


def test_every_placeholder_line_is_marked():
    src = (Path(qos.__file__)).read_text().splitlines()
    names = ['PERCEPTION_DICE_MAP', 'PERCEPTION_TARGET_RELATIVE', 'PERCEPTION_COLOR_CHECK',
             'PERCEPTION_OBSTACLES', 'PERCEPTION_HEARTBEAT', 'HW_NFC_STATUS', 'HW_NFC_RESULT',
             'HW_GRIPPER_STATUS', 'HW_NFC_COMMAND', 'HW_GRIPPER_COMMAND', 'MISSION_TARGET']
    for name in names:
        line = next(l for l in src if l.startswith(f'{name} = '))
        assert '# TBD: not in Interfaces table' in line, name


def test_unknown_publisher_subscriptions_connect_to_anything():
    # A BEST_EFFORT + VOLATILE request is satisfied by any publisher offer.
    for topic in ['/perception/dice_map', '/perception/target_relative', '/perception/color_check',
                  '/perception/obstacle_detections', '/perception/heartbeat',
                  '/hw/nfc/status', '/hw/nfc/result', '/hw/gripper/status']:
        p = qos.BY_TOPIC[topic]
        assert p.reliability == BE and p.durability == V, topic
