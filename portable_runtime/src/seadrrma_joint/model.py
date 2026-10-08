from __future__ import annotations

from pathlib import Path

import numpy as np
import onnxruntime as ort

from .history import ObservationHistory
from .types import ModelOutput


EXPECTED_INPUTS = ("history", "base_obs", "startup_override")
EXPECTED_OUTPUTS = (
    "action",
    "j_hat",
    "j_raw",
    "j_short",
    "j_long",
    "jump_gate",
    "confidence",
)


class TraceOnnxController:
    def __init__(
        self,
        model_path: str | Path,
        history_length: int = 100,
        threads: int = 2,
    ) -> None:
        options = ort.SessionOptions()
        options.intra_op_num_threads = threads
        options.inter_op_num_threads = 1
        self.session = ort.InferenceSession(
            str(model_path),
            sess_options=options,
            providers=["CPUExecutionProvider"],
        )
        input_names = tuple(item.name for item in self.session.get_inputs())
        output_names = tuple(item.name for item in self.session.get_outputs())
        if input_names != EXPECTED_INPUTS:
            raise ValueError(
                f"unexpected ONNX inputs {input_names}; expected {EXPECTED_INPUTS}"
            )
        if output_names != EXPECTED_OUTPUTS:
            raise ValueError(
                f"unexpected ONNX outputs {output_names}; expected {EXPECTED_OUTPUTS}"
            )
        self.history = ObservationHistory(history_length, 9)

    def reset(self) -> None:
        self.history.reset()

    def step(self, observation: np.ndarray, startup: bool) -> ModelOutput:
        base_obs = np.asarray(observation, dtype=np.float32).reshape(1, 9)
        self.history.append(base_obs[0])
        flag = np.asarray([[1.0 if startup else 0.0]], dtype=np.float32)
        outputs = self.session.run(
            list(EXPECTED_OUTPUTS),
            {
                "history": self.history.array,
                "base_obs": base_obs,
                "startup_override": flag,
            },
        )
        values = [float(np.asarray(item).reshape(-1)[0]) for item in outputs]
        if not np.all(np.isfinite(values)):
            raise FloatingPointError("non-finite ONNX output")
        return ModelOutput(*values)
