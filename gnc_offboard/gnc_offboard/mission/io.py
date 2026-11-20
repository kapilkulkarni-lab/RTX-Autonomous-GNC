"""MissionIO: the only way SMACH states act on the world.

States never call rclpy. The node implements MissionIO with real publishers (Milestone 3);
tests and sims use FakeIO, which records every call.
"""
import abc
from dataclasses import dataclass, field
from typing import Any, List, Tuple


class MissionIO(abc.ABC):

    @abc.abstractmethod
    def set_setpoint(self, x: float, y: float, z: float, yaw: float) -> None:
        """Set the position setpoint (field ENU, meters, radians) the node streams to PX4."""

    @abc.abstractmethod
    def arm(self) -> None:
        ...

    @abc.abstractmethod
    def disarm(self) -> None:
        ...

    @abc.abstractmethod
    def set_offboard(self) -> None:
        ...

    @abc.abstractmethod
    def land(self) -> None:
        ...

    @abc.abstractmethod
    def nfc_command(self, command: str) -> None:
        """/hw/nfc/command: 'start' or 'cancel'."""

    @abc.abstractmethod
    def gripper_command(self, command: str) -> None:
        """/hw/gripper/command: 'open', 'close' or 'release'."""

    @abc.abstractmethod
    def publish_nfc_result(self, msg: Any) -> None:
        """/mission/nfc_result: publish (or republish) an avc_msgs/NfcResult."""

    @abc.abstractmethod
    def request_state_publish(self) -> None:
        """Ask the node to publish /mission/state now (state change or handled intent)."""


NFC_COMMANDS = ('start', 'cancel')
GRIPPER_COMMANDS = ('open', 'close', 'release')


@dataclass
class FakeIO(MissionIO):
    """Records every call as (method name, args) in order, for tests and sim stand-ins."""

    calls: List[Tuple[str, tuple]] = field(default_factory=list)

    def _record(self, name: str, *args: Any) -> None:
        self.calls.append((name, args))

    def names(self) -> List[str]:
        return [name for name, _ in self.calls]

    def set_setpoint(self, x, y, z, yaw):
        self._record('set_setpoint', x, y, z, yaw)

    def arm(self):
        self._record('arm')

    def disarm(self):
        self._record('disarm')

    def set_offboard(self):
        self._record('set_offboard')

    def land(self):
        self._record('land')

    def nfc_command(self, command):
        if command not in NFC_COMMANDS:
            raise ValueError(f'unknown NFC command {command!r}')
        self._record('nfc_command', command)

    def gripper_command(self, command):
        if command not in GRIPPER_COMMANDS:
            raise ValueError(f'unknown gripper command {command!r}')
        self._record('gripper_command', command)

    def publish_nfc_result(self, msg):
        self._record('publish_nfc_result', msg)

    def request_state_publish(self):
        self._record('request_state_publish')
