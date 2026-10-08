from __future__ import annotations

import numpy as np
import pytest

from seadrrma_joint.config import JointProfile, TrajectoryConfig
from seadrrma_joint.history import ObservationHistory
from seadrrma_joint.safety import SafetyViolation, TorqueLimiter
from seadrrma_joint.trajectory import evaluate_trajectory
from seadrrma_joint.types import JointSample


def test_history_keeps_newest_sample_last() -> None:
    history = ObservationHistory(3, 2)
    history.append(np.asarray([1.0, 2.0]))
    history.append(np.asarray([3.0, 4.0]))
    np.testing.assert_allclose(history.array[0], [[0.0, 0.0], [1.0, 2.0], [3.0, 4.0]])


def test_sine_trajectory_derivative() -> None:
    config = TrajectoryConfig(amplitude_rad=0.2, frequency_hz=0.5)
    point = evaluate_trajectory(config, 0.0)
    assert point.position_rad == pytest.approx(0.0)
    assert point.velocity_rad_s == pytest.approx(0.2 * np.pi)


def test_torque_limiter_clips_and_slews() -> None:
    profile = JointProfile(
        torque_limit_nm=2.0,
        torque_slew_limit_nm_per_s=100.0,
        control_period_s=0.01,
        output_ramp_s=0.0,
    )
    limiter = TorqueLimiter(profile)
    assert limiter.limit(10.0, 1.0) == pytest.approx(1.0)
    assert limiter.limit(10.0, 1.01) == pytest.approx(2.0)


def test_position_limit_raises() -> None:
    profile = JointProfile(position_limit_rad=0.5)
    limiter = TorqueLimiter(profile)
    sample = JointSample(0.0, 0.0, 0.6, 0.0, 0.0, 0.0, 0.0)
    with pytest.raises(SafetyViolation):
        limiter.validate_sample(sample)

