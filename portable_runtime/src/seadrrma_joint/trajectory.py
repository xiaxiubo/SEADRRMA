from __future__ import annotations

import math
from dataclasses import dataclass

from .config import TrajectoryConfig


@dataclass(frozen=True)
class TrajectoryPoint:
    position_rad: float
    velocity_rad_s: float


def evaluate_trajectory(config: TrajectoryConfig, time_s: float) -> TrajectoryPoint:
    if config.kind != "sine":
        raise ValueError(f"unsupported trajectory kind: {config.kind}")
    omega = 2.0 * math.pi * config.frequency_hz
    phase = omega * time_s + config.phase_rad
    return TrajectoryPoint(
        position_rad=config.offset_rad + config.amplitude_rad * math.sin(phase),
        velocity_rad_s=config.amplitude_rad * omega * math.cos(phase),
    )

