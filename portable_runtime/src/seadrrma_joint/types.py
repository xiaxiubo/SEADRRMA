from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class JointSample:
    timestamp_s: float
    motor_position_rad: float
    load_position_rad: float
    motor_velocity_rad_s: float
    load_velocity_rad_s: float
    spring_deflection_rad: float
    spring_velocity_rad_s: float


@dataclass(frozen=True)
class ModelOutput:
    action: float
    inertia_hat: float
    inertia_raw: float
    inertia_short: float
    inertia_long: float
    jump_gate: float
    confidence: float

