from __future__ import annotations

from collections import deque
from pathlib import Path

import numpy as np
import onnxruntime as ort


TRACE_HISTORY_LENGTH = 100
TRACE_OBSERVATION_DIM = 9
TRACE_STARTUP_OVERRIDE_SECONDS = 0.6


class TRACEOnnxController:
    """Pure ONNX Runtime wrapper for the deployed TRACE controller."""

    def __init__(
        self,
        model_path: str | Path,
        max_torque_nm: float = 61.0,
        history_length: int = TRACE_HISTORY_LENGTH,
        obs_dim: int = TRACE_OBSERVATION_DIM,
        num_threads: int = 4,
        startup_override_seconds: float = TRACE_STARTUP_OVERRIDE_SECONDS,
    ) -> None:
        self.model_path = Path(model_path)
        if not self.model_path.exists():
            raise FileNotFoundError(f"TRACE ONNX model not found: {self.model_path}")
        if startup_override_seconds < 0.0:
            raise ValueError("startup_override_seconds must be non-negative")

        self.max_torque_nm = float(max_torque_nm)
        self.history_length = int(history_length)
        self.obs_dim = int(obs_dim)
        self.startup_override_seconds = float(startup_override_seconds)
        self.prev_action = 0.0
        self.last_startup_override = 1.0
        self.history: deque[np.ndarray] = deque(maxlen=self.history_length)
        self.input_names = ("history", "base_obs", "startup_override")
        self.output_names = (
            "action",
            "j_hat",
            "j_raw",
            "j_short",
            "j_long",
            "jump_gate",
            "confidence",
        )

        session_options = ort.SessionOptions()
        session_options.intra_op_num_threads = max(int(num_threads), 1)
        session_options.inter_op_num_threads = 1
        session_options.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
        session_options.graph_optimization_level = (
            ort.GraphOptimizationLevel.ORT_ENABLE_ALL
        )
        self.session = ort.InferenceSession(
            str(self.model_path),
            sess_options=session_options,
            providers=["CPUExecutionProvider"],
        )
        self._validate_interface()
        self.reset()

    def _validate_interface(self) -> None:
        actual_inputs = tuple(item.name for item in self.session.get_inputs())
        actual_outputs = tuple(item.name for item in self.session.get_outputs())
        if actual_inputs != self.input_names:
            raise RuntimeError(
                f"Unexpected TRACE ONNX inputs: {actual_inputs}; "
                f"expected {self.input_names}"
            )
        if actual_outputs != self.output_names:
            raise RuntimeError(
                f"Unexpected TRACE ONNX outputs: {actual_outputs}; "
                f"expected {self.output_names}"
            )

        expected_input_shapes = {
            "history": [1, self.history_length, self.obs_dim],
            "base_obs": [1, self.obs_dim],
            "startup_override": [1, 1],
        }
        for item in self.session.get_inputs():
            if list(item.shape) != expected_input_shapes[item.name]:
                raise RuntimeError(
                    f"Unexpected shape for {item.name}: {item.shape}; "
                    f"expected {expected_input_shapes[item.name]}"
                )
            if item.type != "tensor(float)":
                raise RuntimeError(
                    f"Unexpected type for {item.name}: {item.type}; "
                    "expected tensor(float)"
                )

    def reset(self) -> None:
        self.prev_action = 0.0
        self.last_startup_override = 1.0
        self.history.clear()
        for _ in range(self.history_length):
            self.history.append(np.zeros((self.obs_dim,), dtype=np.float32))

    @staticmethod
    def build_observation(
        *,
        pos_error_load: float,
        vel_error_load: float,
        pos_error_motor: float,
        vel_error_motor: float,
        spring_defl: float,
        spring_vel: float,
        target_pos: float,
        target_vel: float,
    ) -> np.ndarray:
        return np.asarray(
            [
                pos_error_load,
                vel_error_load,
                pos_error_motor,
                vel_error_motor,
                spring_defl,
                spring_vel,
                target_pos,
                target_vel,
            ],
            dtype=np.float32,
        )

    def run_inference(
        self,
        sensor_obs_8: np.ndarray,
        elapsed_s: float,
        estimator_sensor_obs_8: np.ndarray | None = None,
    ) -> tuple[float, float, float, float, float, float, float, float]:
        obs_8 = np.asarray(sensor_obs_8, dtype=np.float32).reshape(8)
        base_obs = np.asarray([*obs_8, self.prev_action], dtype=np.float32)
        if estimator_sensor_obs_8 is None:
            estimator_obs_8 = obs_8
        else:
            estimator_obs_8 = np.asarray(
                estimator_sensor_obs_8,
                dtype=np.float32,
            ).reshape(8)
        history_obs = np.asarray(
            [*estimator_obs_8, self.prev_action],
            dtype=np.float32,
        )
        self.history.append(history_obs)
        startup_override = float(elapsed_s < self.startup_override_seconds)
        self.last_startup_override = startup_override
        outputs = self.session.run(
            list(self.output_names),
            {
                "history": np.asarray(self.history, dtype=np.float32)[None, :, :],
                "base_obs": base_obs[None, :],
                "startup_override": np.asarray(
                    [[startup_override]],
                    dtype=np.float32,
                ),
            },
        )

        action_norm = float(np.asarray(outputs[0], dtype=np.float32).reshape(-1)[0])
        action_norm = max(min(action_norm, 1.0), -1.0)
        self.prev_action = action_norm
        predicted_torque_nm = action_norm * self.max_torque_nm
        diagnostics = [
            float(np.asarray(output, dtype=np.float32).reshape(-1)[0])
            for output in outputs[1:]
        ]
        return action_norm, predicted_torque_nm, *diagnostics


__all__ = [
    "TRACE_HISTORY_LENGTH",
    "TRACE_OBSERVATION_DIM",
    "TRACE_STARTUP_OVERRIDE_SECONDS",
    "TRACEOnnxController",
]
