from __future__ import annotations

import math
from dataclasses import dataclass

from .config import JointProfile
from .types import JointSample


class SafetyViolation(RuntimeError):
    pass


@dataclass
class TorqueLimiter:
    profile: JointProfile
    previous_torque_nm: float = 0.0

    def reset(self) -> None:
        self.previous_torque_nm = 0.0

    def validate_sample(self, sample: JointSample) -> None:
        values = (
            sample.motor_position_rad,
            sample.load_position_rad,
            sample.motor_velocity_rad_s,
            sample.load_velocity_rad_s,
            sample.spring_deflection_rad,
            sample.spring_velocity_rad_s,
        )
        if not all(math.isfinite(value) for value in values):
            raise SafetyViolation("non-finite sensor sample")
        if abs(sample.load_position_rad) > self.profile.position_limit_rad:
            raise SafetyViolation("load position limit exceeded")
        if max(abs(sample.motor_velocity_rad_s), abs(sample.load_velocity_rad_s)) > (
            self.profile.velocity_limit_rad_s
        ):
            raise SafetyViolation("velocity limit exceeded")
        if abs(sample.spring_deflection_rad) > self.profile.spring_deflection_limit_rad:
            raise SafetyViolation("spring deflection limit exceeded")

    def limit(self, requested_nm: float, elapsed_s: float) -> float:
        if not math.isfinite(requested_nm):
            raise SafetyViolation("non-finite torque request")
        limit = self.profile.torque_limit_nm
        if self.profile.output_ramp_s > 0.0:
            limit *= min(1.0, max(0.0, elapsed_s / self.profile.output_ramp_s))
        requested_nm = min(limit, max(-limit, requested_nm))
        max_delta = self.profile.torque_slew_limit_nm_per_s * self.profile.control_period_s
        requested_nm = min(
            self.previous_torque_nm + max_delta,
            max(self.previous_torque_nm - max_delta, requested_nm),
        )
        self.previous_torque_nm = requested_nm
        return requested_nm

