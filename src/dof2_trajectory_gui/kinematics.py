from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Tuple


@dataclass(frozen=True)
class ArmConfig:
    l1: float
    l2: float
    x2: float
    wheel_diameter: float
    x1: float = 0.0

    @property
    def wheel_radius(self) -> float:
        return 0.5 * self.wheel_diameter

    @property
    def z2(self) -> float:
        return self.wheel_radius


def _wrap_to_pi(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def _distance_squared(z1: float, cfg: ArmConfig) -> float:
    dx = cfg.x2 - cfg.x1
    dz = cfg.z2 - z1
    return dx * dx + dz * dz


def is_reachable(z1: float, cfg: ArmConfig, tol: float = 1e-9) -> bool:
    d = math.sqrt(_distance_squared(z1, cfg))
    min_r = abs(cfg.l1 - cfg.l2) - tol
    max_r = (cfg.l1 + cfg.l2) + tol
    return min_r <= d <= max_r


def inverse_kinematics(
    z1: float,
    cfg: ArmConfig,
    elbow: str = "down",
    tol: float = 1e-9,
) -> Tuple[float, float]:
    if elbow not in {"down", "up"}:
        raise ValueError("elbow must be either 'down' or 'up'")

    dx = cfg.x2 - cfg.x1
    dz = cfg.z2 - z1
    d2 = dx * dx + dz * dz

    c2 = (d2 - cfg.l1 * cfg.l1 - cfg.l2 * cfg.l2) / (2.0 * cfg.l1 * cfg.l2)
    if c2 < -1.0 - tol or c2 > 1.0 + tol:
        raise ValueError(
            f"Unreachable z1={z1:.6f} m. cos(q2)={c2:.6f} is outside [-1, 1]."
        )
    c2 = max(-1.0, min(1.0, c2))

    s2_mag = math.sqrt(max(0.0, 1.0 - c2 * c2))
    s2 = s2_mag if elbow == "down" else -s2_mag
    q2 = math.atan2(s2, c2)

    # Internal geometric angle from +x axis.
    theta1 = math.atan2(dz, dx) - math.atan2(cfg.l2 * s2, cfg.l1 + cfg.l2 * c2)

    # User convention:
    # q1 = 0 -> link points to the left (-x),
    # positive direction is counter-clockwise.
    q1 = _wrap_to_pi(theta1 - math.pi)
    return q1, q2


def forward_kinematics(
    z1: float,
    q1: float,
    q2: float,
    cfg: ArmConfig,
) -> Tuple[Tuple[float, float], Tuple[float, float], Tuple[float, float]]:
    base_x = cfg.x1
    base_z = z1

    # Convert from user convention(q1=0 at left) to geometric angle.
    theta1 = q1 + math.pi
    theta2 = theta1 + q2

    joint2_x = base_x + cfg.l1 * math.cos(theta1)
    joint2_z = base_z + cfg.l1 * math.sin(theta1)

    end_x = joint2_x + cfg.l2 * math.cos(theta2)
    end_z = joint2_z + cfg.l2 * math.sin(theta2)

    return (base_x, base_z), (joint2_x, joint2_z), (end_x, end_z)
