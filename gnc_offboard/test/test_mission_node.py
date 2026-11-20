"""mission_manager node and config: parameters come from mission.yaml; state publishing works
without PX4."""
import dataclasses
import threading

import pytest
import yaml

from conftest import CONFIG_YAML, intent
from gnc_offboard.mission import state_names as S
from gnc_offboard.mission.config import MissionConfig, load_yaml_params


def test_yaml_has_exactly_the_config_fields():
    params = load_yaml_params(CONFIG_YAML)
    assert set(params) == set(MissionConfig.names())


def test_yaml_values_and_markers():
    cfg = MissionConfig.from_dict(load_yaml_params(CONFIG_YAML))
    assert cfg.require_pilot_arm is False                 # SITL default
    assert cfg.force_return_at_s == 360.0
    assert (cfg.loop_hz, cfg.setpoint_hz, cfg.state_pub_hz) == (20.0, 20.0, 5.0)
    text = CONFIG_YAML.read_text()
    assert 'force_return_at_s: 360.0      # TBD' in text
    assert 'TEMPORARY' in text and 'hover_test_s' in text


def test_config_rejects_missing_unknown_and_bad_bool():
    params = load_yaml_params(CONFIG_YAML)
    with pytest.raises(ValueError, match='missing'):
        MissionConfig.from_dict({k: v for k, v in params.items() if k != 'loop_hz'})
    with pytest.raises(ValueError, match='unknown'):
        MissionConfig.from_dict({**params, 'surprise': 1})
    with pytest.raises(ValueError, match='require_pilot_arm'):
        MissionConfig.from_dict({**params, 'require_pilot_arm': 'no'})


@pytest.fixture
def node():
    rclpy = pytest.importorskip('rclpy')
    from gnc_offboard.ros.mission_manager_node import MissionManagerNode
    rclpy.init(args=['--ros-args', '-p', 'hover_test_s:=7.5'])
    n = MissionManagerNode(config_path=str(CONFIG_YAML))
    yield n
    n.stop_mission()
    n.destroy_node()
    rclpy.shutdown()


def test_node_loads_parameters_with_overrides(node, cfg):
    assert node.cfg == dataclasses.replace(cfg, hover_test_s=7.5)


def test_node_publishes_state_without_px4(node):
    sent = []
    node.state_pub.publish = sent.append
    node.publish_state()
    assert sent == []                              # nothing before the machine starts
    node.ctx.state, node.ctx.reason = S.STANDBY, 'waiting | for retrieve'
    node.publish_state()
    node.publish_state()
    assert [m.seq for m in sent] == [1, 2]
    assert sent[0].state == S.STANDBY and sent[0].reason == 'waiting for retrieve'


def test_node_runs_machine_in_worker_thread_and_preempts(node):
    """Without PX4 the machine sits in STANDBY; a retrieve moves it to PREFLIGHT, which waits
    for a position estimate. Shutdown preempts the worker cleanly."""
    sent = []
    node.state_pub.publish = sent.append
    node.start_mission()
    assert node.worker.daemon and node.worker.is_alive()
    node.ctx.update_input('intent', intent('retrieve', 'blue', 1), node.clock.now())
    for _ in range(100):
        if node.ctx.state == S.PREFLIGHT:
            break
        threading.Event().wait(0.02)
    assert node.ctx.state == S.PREFLIGHT
    assert any(m.state == S.PREFLIGHT and m.last_intent_id == 1 for m in sent)
    node.stop_mission()
    assert not node.worker.is_alive()
    assert node.sm_outcome == 'preempted'


def test_installed_yaml_is_a_ros_params_file():
    data = yaml.safe_load(CONFIG_YAML.read_text())
    assert list(data) == ['mission_manager'] and 'ros__parameters' in data['mission_manager']
