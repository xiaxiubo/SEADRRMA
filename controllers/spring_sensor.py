from __future__ import annotations

import math

from dataclasses import dataclass


@dataclass(slots=True)
class SpringSensorConfig:
    counts_per_rev: int = 1 << 19
    zero_counts: int = 0
    sign: int = 1
    spring_stiffness_nm_per_rad: float = 2400.0


@dataclass(frozen=True, slots=True)
class SpringEstimate:
    raw_counts: int
    delta_theta_rad: float
    spring_torque_nm: float


class SpringSensorEstimator:
    def __init__(self, config: SpringSensorConfig | None = None) -> None:
        self.config = config or SpringSensorConfig()

    def tare_zero(self, raw_counts: int) -> None:
        self.config.zero_counts = int(raw_counts)

    def tare_zero_from_samples(self, samples: list[int] | tuple[int, ...]) -> int:
        if not samples:
            raise ValueError("samples must not be empty")
        zero_counts = round(sum(samples) / len(samples))
        self.tare_zero(zero_counts)
        return zero_counts

    def counts_diff_wrapped(self, raw_counts: int) -> int:
        diff = int(raw_counts) - self.config.zero_counts
        half = self.config.counts_per_rev // 2
        if diff >= half:
            diff -= self.config.counts_per_rev
        elif diff < -half:
            diff += self.config.counts_per_rev
        return diff

    def delta_theta_rad(self, raw_counts: int) -> float:
        diff = self.counts_diff_wrapped(raw_counts)
        return self.config.sign * diff * (2.0 * math.pi / self.config.counts_per_rev)

    def spring_torque_nm(self, raw_counts: int) -> float:
        return self.config.spring_stiffness_nm_per_rad * self.delta_theta_rad(raw_counts)

    def estimate(self, raw_counts: int) -> SpringEstimate:
        delta_theta = self.delta_theta_rad(raw_counts)
        return SpringEstimate(
            raw_counts=int(raw_counts),
            delta_theta_rad=delta_theta,
            spring_torque_nm=self.config.spring_stiffness_nm_per_rad * delta_theta,
        )


__all__ = [
    "SpringEstimate",
    "SpringSensorConfig",
    "SpringSensorEstimator",
]
