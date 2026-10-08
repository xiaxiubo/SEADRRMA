from __future__ import annotations

import time

import numpy as np

from .backend import JointBackend
from .config import JointProfile
from .logger import CsvLogger
from .model import TraceOnnxController
from .safety import SafetyViolation, TorqueLimiter
from .trajectory import evaluate_trajectory


def build_observation(sample, target, previous_action: float) -> np.ndarray:
    return np.asarray(
        [
            target.position_rad - sample.load_position_rad,
            target.velocity_rad_s - sample.load_velocity_rad_s,
            target.position_rad - sample.motor_position_rad,
            target.velocity_rad_s - sample.motor_velocity_rad_s,
            sample.spring_deflection_rad,
            sample.spring_velocity_rad_s,
            target.position_rad,
            target.velocity_rad_s,
            previous_action,
        ],
        dtype=np.float32,
    )


def run_experiment(
    backend: JointBackend,
    controller: TraceOnnxController,
    profile: JointProfile,
    duration_s: float,
    logger: CsvLogger,
    enable_output: bool,
) -> None:
    limiter = TorqueLimiter(profile)
    controller.reset()
    backend.disable()
    previous_action = 0.0
    start = time.perf_counter()
    deadline = start
    previous_tick = start
    try:
        while True:
            now = time.perf_counter()
            elapsed = now - start
            if elapsed >= duration_s:
                break
            period_ms = (now - previous_tick) * 1000.0
            previous_tick = now
            schedule_lateness_ms = max(0.0, (now - deadline) * 1000.0)
            sample = backend.read()
            limiter.validate_sample(sample)
            target = evaluate_trajectory(profile.trajectory, elapsed)
            observation = build_observation(sample, target, previous_action)
            startup = elapsed < profile.startup_duration_s
            output = controller.step(observation, startup)
            action = float(np.clip(output.action, -1.0, 1.0))
            torque = limiter.limit(action * profile.model_torque_scale_nm, elapsed)
            backend.write_torque(torque if enable_output else 0.0, enable_output)
            previous_action = action

            compute_ms = (time.perf_counter() - now) * 1000.0
            missed = compute_ms > profile.control_period_s * 1000.0
            logger.write(
                {
                    "time_s": elapsed,
                    "compute_ms": compute_ms,
                    "period_ms": period_ms,
                    "schedule_lateness_ms": schedule_lateness_ms,
                    "deadline_missed": int(missed),
                    "output_enabled": int(enable_output),
                    "target_position_rad": target.position_rad,
                    "target_velocity_rad_s": target.velocity_rad_s,
                    "motor_position_rad": sample.motor_position_rad,
                    "load_position_rad": sample.load_position_rad,
                    "motor_velocity_rad_s": sample.motor_velocity_rad_s,
                    "load_velocity_rad_s": sample.load_velocity_rad_s,
                    "spring_deflection_rad": sample.spring_deflection_rad,
                    "spring_velocity_rad_s": sample.spring_velocity_rad_s,
                    "normalized_action": action,
                    "command_torque_nm": torque if enable_output else 0.0,
                    "inertia_hat": output.inertia_hat,
                    "inertia_raw": output.inertia_raw,
                    "inertia_short": output.inertia_short,
                    "inertia_long": output.inertia_long,
                    "jump_gate": output.jump_gate,
                    "confidence": output.confidence,
                }
            )

            deadline += profile.control_period_s
            remaining = deadline - time.perf_counter()
            if remaining > 0.0:
                time.sleep(remaining)
            elif -remaining > profile.watchdog_limit_s:
                raise SafetyViolation(
                    f"control loop exceeded watchdog by {-remaining * 1000.0:.2f} ms"
                )
    finally:
        backend.disable()
        backend.close()
