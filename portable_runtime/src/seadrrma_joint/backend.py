from __future__ import annotations

import importlib
import time
from typing import Protocol

from .config import JointProfile
from .types import JointSample


class JointBackend(Protocol):
    def read(self) -> JointSample:
        ...

    def write_torque(self, torque_nm: float, enable: bool) -> None:
        ...

    def disable(self) -> None:
        ...

    def close(self) -> None:
        ...


class MockJointBackend:
    """Small deterministic plant used only to exercise runtime plumbing."""

    def __init__(self, profile: JointProfile) -> None:
        self.dt = profile.control_period_s
        self.motor_position = 0.0
        self.load_position = 0.0
        self.motor_velocity = 0.0
        self.load_velocity = 0.0
        self.command_nm = 0.0
        self.enabled = False

    def read(self) -> JointSample:
        spring = self.motor_position - self.load_position
        return JointSample(
            timestamp_s=time.perf_counter(),
            motor_position_rad=self.motor_position,
            load_position_rad=self.load_position,
            motor_velocity_rad_s=self.motor_velocity,
            load_velocity_rad_s=self.load_velocity,
            spring_deflection_rad=spring,
            spring_velocity_rad_s=self.motor_velocity - self.load_velocity,
        )

    def write_torque(self, torque_nm: float, enable: bool) -> None:
        self.command_nm = torque_nm if enable else 0.0
        self.enabled = enable
        # This is not the research simulator. It only keeps the integration test finite.
        motor_acc = 2.0 * self.command_nm - 8.0 * self.motor_velocity
        load_acc = 25.0 * (self.motor_position - self.load_position) - 2.0 * self.load_velocity
        self.motor_velocity += motor_acc * self.dt
        self.load_velocity += load_acc * self.dt
        self.motor_position += self.motor_velocity * self.dt
        self.load_position += self.load_velocity * self.dt

    def disable(self) -> None:
        self.write_torque(0.0, False)

    def close(self) -> None:
        self.disable()


def load_backend(spec: str, profile: JointProfile) -> JointBackend:
    if spec == "mock":
        return MockJointBackend(profile)
    if ":" not in spec:
        raise ValueError("backend must be 'mock' or 'module:factory'")
    module_name, factory_name = spec.split(":", 1)
    factory = getattr(importlib.import_module(module_name), factory_name)
    return factory(profile)
