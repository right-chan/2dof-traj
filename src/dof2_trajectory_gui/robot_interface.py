from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Tuple


@dataclass
class JointState:
    q1: float
    q2: float


class RobotInterface(ABC):
    @abstractmethod
    def connect(self) -> None:
        raise NotImplementedError

    @abstractmethod
    def send_joint_target(self, q1: float, q2: float) -> None:
        raise NotImplementedError

    @abstractmethod
    def get_joint_state(self) -> JointState:
        raise NotImplementedError


class MockRobotInterface(RobotInterface):
    def __init__(self) -> None:
        self._state = JointState(0.0, 0.0)

    def connect(self) -> None:
        return

    def send_joint_target(self, q1: float, q2: float) -> None:
        self._state = JointState(q1=q1, q2=q2)

    def get_joint_state(self) -> JointState:
        return self._state


class RealRobotInterface(RobotInterface):
    def connect(self) -> None:
        # TODO: Replace with real robot initialization.
        raise NotImplementedError("Implement hardware connection here.")

    def send_joint_target(self, q1: float, q2: float) -> None:
        # TODO: Replace with the motor command API.
        raise NotImplementedError("Implement motor command output here.")

    def get_joint_state(self) -> JointState:
        # TODO: Replace with encoder feedback API.
        raise NotImplementedError("Implement encoder state reading here.")
