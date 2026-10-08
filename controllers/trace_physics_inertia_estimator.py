from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class TRACEPhysicsInertiaConfig:
    window_samples: int = 300
    torque_inertia_scale: float = 4.21881667
    viscous_nm_per_rad_s: float = 7.63785533
    coulomb_positive_nm: float = 5.40749114
    coulomb_negative_nm: float = -5.18903976
    torque_bias_nm: float = -0.04947318
    velocity_deadband_rad_s: float = 0.03
    minimum_denominator: float = 1e-6
    clip_min_kgm2: float = 0.03
    clip_max_kgm2: float = 1.0


class TRACEPhysicsInertiaEstimator:
    """Sliding least-squares inertia estimate after friction compensation."""

    def __init__(
        self,
        config: TRACEPhysicsInertiaConfig | None = None,
    ) -> None:
        self.config = config or TRACEPhysicsInertiaConfig()
        if self.config.window_samples <= 0:
            raise ValueError("window_samples must be positive.")
        self.terms: deque[tuple[float, float]] = deque(
            maxlen=self.config.window_samples
        )
        self.numerator = 0.0
        self.denominator = 0.0
        self.last_estimate_kgm2: float | None = None

    def reset(self) -> None:
        self.terms.clear()
        self.numerator = 0.0
        self.denominator = 0.0
        self.last_estimate_kgm2 = None

    def update(
        self,
        *,
        action_torque_nm: float,
        load_velocity_rad_s: float,
        reference_acceleration_rad_s2: float,
        excitation_valid: bool,
    ) -> tuple[float, bool]:
        if not excitation_valid:
            if self.last_estimate_kgm2 is None:
                return 0.0, False
            return self.last_estimate_kgm2, True

        values = (
            action_torque_nm,
            load_velocity_rad_s,
            reference_acceleration_rad_s2,
        )
        if not all(math.isfinite(float(value)) for value in values):
            raise ValueError("Physics inertia estimator received non-finite data.")

        velocity = float(load_velocity_rad_s)
        coulomb = 0.0
        if velocity > self.config.velocity_deadband_rad_s:
            coulomb = self.config.coulomb_positive_nm
        elif velocity < -self.config.velocity_deadband_rad_s:
            coulomb = self.config.coulomb_negative_nm

        residual_torque = (
            float(action_torque_nm)
            - self.config.viscous_nm_per_rad_s * velocity
            - coulomb
            - self.config.torque_bias_nm
        )
        scaled_acceleration = (
            self.config.torque_inertia_scale
            * float(reference_acceleration_rad_s2)
        )
        term = (
            scaled_acceleration * residual_torque,
            scaled_acceleration * scaled_acceleration,
        )
        if len(self.terms) == self.terms.maxlen:
            old_numerator, old_denominator = self.terms[0]
            self.numerator -= old_numerator
            self.denominator -= old_denominator
        self.terms.append(term)
        self.numerator += term[0]
        self.denominator += term[1]

        if (
            len(self.terms) < self.config.window_samples
            or self.denominator < self.config.minimum_denominator
        ):
            return 0.0, False

        estimate = self.numerator / self.denominator
        estimate = max(
            min(estimate, self.config.clip_max_kgm2),
            self.config.clip_min_kgm2,
        )
        self.last_estimate_kgm2 = float(estimate)
        return self.last_estimate_kgm2, True


__all__ = [
    "TRACEPhysicsInertiaConfig",
    "TRACEPhysicsInertiaEstimator",
]
