from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .kinematics import ArmConfig, inverse_kinematics


@dataclass
class Trajectory:
    t: np.ndarray
    z1: np.ndarray
    q1: np.ndarray
    q2: np.ndarray
    fs: float

    @property
    def n(self) -> int:
        return int(self.t.size)

    @property
    def duration(self) -> float:
        return float(self.t[-1]) if self.n > 0 else 0.0


def quintic_blend(t: np.ndarray, duration: float) -> np.ndarray:
    if duration <= 0.0:
        raise ValueError("duration must be > 0")
    s = np.clip(t / duration, 0.0, 1.0)
    return 10.0 * s**3 - 15.0 * s**4 + 6.0 * s**5


def plan_z1_trajectory(
    cfg: ArmConfig,
    z1_start: float,
    z1_target: float,
    duration: float,
    fs: float = 50.0,
    elbow: str = "down",
) -> Trajectory:
    if fs <= 0.0:
        raise ValueError("fs must be > 0")
    if duration <= 0.0:
        raise ValueError("duration must be > 0")

    n = max(2, int(round(duration * fs)) + 1)
    t = np.linspace(0.0, duration, n)

    blend = quintic_blend(t, duration)
    z1 = z1_start + (z1_target - z1_start) * blend

    q1 = np.zeros_like(z1)
    q2 = np.zeros_like(z1)

    for i, z in enumerate(z1):
        try:
            q1[i], q2[i] = inverse_kinematics(float(z), cfg, elbow=elbow)
        except ValueError as exc:
            raise ValueError(
                f"Trajectory unreachable at index={i}, t={t[i]:.3f}s, z1={z:.6f}m. {exc}"
            ) from exc

    return Trajectory(t=t, z1=z1, q1=q1, q2=q2, fs=fs)
