from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from .trajectory import Trajectory


@dataclass
class StepCommand:
    index: int
    t: float
    z1: float
    q1: float
    q2: float


class TrajectoryController:
    def __init__(self, fs: float = 50.0) -> None:
        self.fs = fs
        self._traj: Optional[Trajectory] = None
        self._index = 0
        self._running = False

    @property
    def running(self) -> bool:
        return self._running

    @property
    def trajectory(self) -> Optional[Trajectory]:
        return self._traj

    def load(self, traj: Trajectory) -> None:
        self._traj = traj
        self._index = 0
        self._running = False

    def start(self) -> None:
        if self._traj is None:
            raise RuntimeError("No trajectory loaded.")
        if self._index >= self._traj.n - 1:
            self._index = 0
        self._running = True

    def stop(self) -> None:
        self._running = False

    def reset(self) -> None:
        self._index = 0
        self._running = False

    def current_index(self) -> int:
        return self._index

    def step(self) -> Optional[StepCommand]:
        if not self._running or self._traj is None:
            return None

        idx = self._index
        cmd = StepCommand(
            index=idx,
            t=float(self._traj.t[idx]),
            z1=float(self._traj.z1[idx]),
            q1=float(self._traj.q1[idx]),
            q2=float(self._traj.q2[idx]),
        )

        if idx >= self._traj.n - 1:
            self._running = False
        else:
            self._index += 1

        return cmd
