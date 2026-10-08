from __future__ import annotations

import numpy as np


class ObservationHistory:
    def __init__(self, length: int, dimension: int = 9) -> None:
        self._buffer = np.zeros((1, length, dimension), dtype=np.float32)

    @property
    def array(self) -> np.ndarray:
        return self._buffer

    def reset(self) -> None:
        self._buffer.fill(0.0)

    def append(self, observation: np.ndarray) -> None:
        value = np.asarray(observation, dtype=np.float32).reshape(-1)
        if value.shape != (self._buffer.shape[-1],):
            raise ValueError(
                f"expected observation shape {(self._buffer.shape[-1],)}, got {value.shape}"
            )
        self._buffer[:, :-1] = self._buffer[:, 1:]
        self._buffer[:, -1] = value

