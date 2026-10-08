from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np


# 参考当前 SEA 参数做一个简化版的两惯量模型：
# motor side / load side / spring coupling
DEFAULT_LOAD_INERTIA = 0.3 * 0.3 * 0.3


@dataclass(frozen=True, slots=True)
class SeaModelParams:
    # SEA 简化物理参数，全部使用关节侧 SI 单位。
    spring_stiffness: float = 2400.0
    motor_damping: float = 0.0955
    load_damping: float = 0.0
    motor_inertia: float = 0.417
    load_inertia: float = DEFAULT_LOAD_INERTIA


@dataclass(frozen=True, slots=True)
class SeaState:
    # 控制器使用的 4 维状态，单位全部为 SI。
    theta_m_rad: float
    dtheta_m_rad_s: float
    theta_l_rad: float
    dtheta_l_rad_s: float

    def as_vector(self) -> np.ndarray:
        return np.array(
            [
                self.theta_m_rad,
                self.dtheta_m_rad_s,
                self.theta_l_rad,
                self.dtheta_l_rad_s,
            ],
            dtype=float,
        )

    @classmethod
    def from_vector(cls, vector: np.ndarray) -> "SeaState":
        values = np.asarray(vector, dtype=float).reshape(4)
        return cls(
            theta_m_rad=float(values[0]),
            dtheta_m_rad_s=float(values[1]),
            theta_l_rad=float(values[2]),
            dtheta_l_rad_s=float(values[3]),
        )


DEFAULT_MODEL_PARAMS = SeaModelParams()


def build_continuous_state_matrices(model_params: SeaModelParams) -> tuple[np.ndarray, np.ndarray]:
    # 连续时间 SEA 两惯量模型：
    # x = [theta_m, dtheta_m, theta_l, dtheta_l]
    # u = motor torque command
    ks = model_params.spring_stiffness
    bm = model_params.motor_damping
    bl = model_params.load_damping
    jm = model_params.motor_inertia
    jl = model_params.load_inertia

    a_matrix = np.array(
        [
            [0.0, 1.0, 0.0, 0.0],
            [-ks / jm, -bm / jm, ks / jm, 0.0],
            [0.0, 0.0, 0.0, 1.0],
            [ks / jl, 0.0, -ks / jl, -bl / jl],
        ],
        dtype=float,
    )
    b_matrix = np.array([[0.0], [1.0 / jm], [0.0], [0.0]], dtype=float)
    return a_matrix, b_matrix


__all__ = [
    "DEFAULT_MODEL_PARAMS",
    "DEFAULT_LOAD_INERTIA",
    "SeaModelParams",
    "SeaState",
    "build_continuous_state_matrices",
]
