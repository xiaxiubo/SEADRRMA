from __future__ import annotations

from collections import deque
import json
import math
from pathlib import Path

import numpy as np


class TRACEInertiaCalibrator:
    """Causal NumPy post-calibrator for TRACE inertia diagnostics."""

    def __init__(self, artifact_path: str | Path) -> None:
        self.artifact_path = Path(artifact_path)
        artifact = json.loads(self.artifact_path.read_text(encoding="utf-8"))
        self.feature_names = tuple(artifact["feature_names"])
        self.windows = tuple(int(value) for value in artifact["windows"])
        self.base_columns = tuple(artifact["base_columns"])
        self.rolling_columns = tuple(artifact["rolling_columns"])
        self.abs_rolling_columns = tuple(artifact["abs_rolling_columns"])
        self.minimum_samples = int(artifact["minimum_samples"])
        self.clip_min_kgm2 = float(artifact["clip_min_kgm2"])
        self.clip_max_kgm2 = float(artifact["clip_max_kgm2"])
        self.output_filter_tau_s = float(artifact["output_filter_tau_s"])
        self.minimum_velocity_absmean_rad_s = float(
            artifact["minimum_velocity_absmean_rad_s"]
        )
        self.scaler_mean = np.asarray(artifact["scaler_mean"], dtype=np.float64)
        self.scaler_scale = np.asarray(
            artifact["scaler_scale"],
            dtype=np.float64,
        )
        self.ridge_coef = np.asarray(artifact["ridge_coef"], dtype=np.float64)
        self.ridge_intercept = float(artifact["ridge_intercept"])
        expected_shape = (len(self.feature_names),)
        for name, values in (
            ("scaler_mean", self.scaler_mean),
            ("scaler_scale", self.scaler_scale),
            ("ridge_coef", self.ridge_coef),
        ):
            if values.shape != expected_shape:
                raise ValueError(
                    f"{name} shape {values.shape} does not match "
                    f"{expected_shape}."
                )
        if np.any(self.scaler_scale <= 0.0):
            raise ValueError("All scaler values must be positive.")
        self.history = {
            column: deque(maxlen=self.minimum_samples)
            for column in self.base_columns
        }
        self.filtered_output_kgm2: float | None = None

    def reset(self) -> None:
        for values in self.history.values():
            values.clear()
        self.filtered_output_kgm2 = None

    def ready(self) -> bool:
        return all(
            len(values) >= self.minimum_samples
            for values in self.history.values()
        )

    @staticmethod
    def _sample_std(values: np.ndarray) -> float:
        if values.size < 2:
            return 0.0
        return float(np.std(values, ddof=1))

    def _feature_vector(self) -> np.ndarray:
        history_arrays = {
            column: np.fromiter(
                self.history[column],
                dtype=np.float64,
                count=len(self.history[column]),
            )
            for column in self.base_columns
        }
        features: dict[str, float] = {
            column: float(history_arrays[column][-1])
            for column in self.base_columns
        }
        for window in self.windows:
            for column in self.rolling_columns:
                values = history_arrays[column][-window:]
                features[f"{column}_mean{window}"] = float(np.mean(values))
                features[f"{column}_std{window}"] = self._sample_std(values)
            for column in self.abs_rolling_columns:
                values = history_arrays[column][-window:]
                features[f"{column}_absmean{window}"] = float(
                    np.mean(np.abs(values))
                )
        return np.asarray(
            [features[name] for name in self.feature_names],
            dtype=np.float64,
        )

    def update(
        self,
        sample: dict[str, float],
        dt_s: float,
    ) -> tuple[float, float, bool]:
        for column in self.base_columns:
            value = float(sample[column])
            if not math.isfinite(value):
                raise ValueError(f"Non-finite calibrator input {column}={value}")
            self.history[column].append(value)

        raw_j_hat = float(sample["J_hat"])
        if not self.ready():
            self.filtered_output_kgm2 = None
            return raw_j_hat, raw_j_hat, False
        velocity_history = np.asarray(
            self.history["meas_vel_l_rad_s"],
            dtype=np.float64,
        )
        if (
            float(np.mean(np.abs(velocity_history)))
            < self.minimum_velocity_absmean_rad_s
        ):
            self.filtered_output_kgm2 = None
            return raw_j_hat, raw_j_hat, False

        feature_vector = self._feature_vector()
        standardized = (
            feature_vector - self.scaler_mean
        ) / self.scaler_scale
        calibrated = float(
            self.ridge_intercept + np.dot(self.ridge_coef, standardized)
        )
        calibrated = float(
            np.clip(
                calibrated,
                self.clip_min_kgm2,
                self.clip_max_kgm2,
            )
        )
        if self.filtered_output_kgm2 is None:
            self.filtered_output_kgm2 = calibrated
        else:
            alpha = 1.0 - math.exp(
                -max(float(dt_s), 1e-6) / self.output_filter_tau_s
            )
            self.filtered_output_kgm2 += alpha * (
                calibrated - self.filtered_output_kgm2
            )
        return calibrated, self.filtered_output_kgm2, True


__all__ = ["TRACEInertiaCalibrator"]
