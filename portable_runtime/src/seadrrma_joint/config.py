from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any


@dataclass(frozen=True)
class TrajectoryConfig:
    kind: str = "sine"
    amplitude_rad: float = 0.1
    frequency_hz: float = 0.2
    offset_rad: float = 0.0
    phase_rad: float = 0.0


@dataclass(frozen=True)
class Calibration:
    motor_position_sign: float = 1.0
    load_position_sign: float = 1.0
    spring_deflection_sign: float = 1.0
    motor_position_offset_rad: float = 0.0
    load_position_offset_rad: float = 0.0
    spring_deflection_offset_rad: float = 0.0


@dataclass(frozen=True)
class JointProfile:
    control_period_s: float = 0.005
    history_length: int = 100
    startup_duration_s: float = 0.6
    model_torque_scale_nm: float = 61.0
    torque_limit_nm: float = 8.0
    torque_slew_limit_nm_per_s: float = 400.0
    position_limit_rad: float = 1.2
    velocity_limit_rad_s: float = 5.0
    spring_deflection_limit_rad: float = 0.03
    watchdog_limit_s: float = 0.02
    output_ramp_s: float = 2.0
    trajectory: TrajectoryConfig = field(default_factory=TrajectoryConfig)
    calibration: Calibration = field(default_factory=Calibration)
    plant_reference: dict[str, float] = field(default_factory=dict)

    def validate(self) -> None:
        if self.control_period_s <= 0.0:
            raise ValueError("control_period_s must be positive")
        if self.history_length <= 0:
            raise ValueError("history_length must be positive")
        for name in (
            "model_torque_scale_nm",
            "torque_limit_nm",
            "position_limit_rad",
            "velocity_limit_rad_s",
            "spring_deflection_limit_rad",
            "watchdog_limit_s",
        ):
            if getattr(self, name) <= 0.0:
                raise ValueError(f"{name} must be positive")


def load_profile(path: str | Path) -> JointProfile:
    payload: dict[str, Any] = json.loads(Path(path).read_text(encoding="utf-8"))
    payload["trajectory"] = TrajectoryConfig(**payload.get("trajectory", {}))
    payload["calibration"] = Calibration(**payload.get("calibration", {}))
    profile = JointProfile(**payload)
    profile.validate()
    return profile

