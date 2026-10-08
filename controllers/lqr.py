from __future__ import annotations

from dataclasses import dataclass, asdict
from pathlib import Path
from typing import Any

import numpy as np
import yaml
from scipy.linalg import solve_continuous_are

if __package__ in {None, ""}:
    import sys

    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from controllers.sea_model import DEFAULT_MODEL_PARAMS, SeaModelParams, SeaState, build_continuous_state_matrices
else:
    from .sea_model import DEFAULT_MODEL_PARAMS, SeaModelParams, SeaState, build_continuous_state_matrices


DEFAULT_LQR_PARAM_PATH = Path(__file__).with_name("lqr_params.yaml")


@dataclass(frozen=True, slots=True)
class LqrWeights:
    # 第一版先用一个简单的权重配置，后面需要的话再按实验结果细调。
    q_diag: tuple[float, float, float, float] = (10.0, 1.0, 100.0, 1.0)
    r_value: float = 0.1


DEFAULT_LQR_WEIGHTS = LqrWeights()


class LqrController:
    # 简化版 SEA LQR 控制器：
    # - 生成连续时间 LQR 增益 K
    # - 保存 / 读取 YAML 参数文件
    # - 由状态 s 计算动作 a
    def __init__(
        self,
        model_params: SeaModelParams = DEFAULT_MODEL_PARAMS,
        weights: LqrWeights = DEFAULT_LQR_WEIGHTS,
        param_path: str | Path = DEFAULT_LQR_PARAM_PATH,
    ) -> None:
        self.model_params = model_params
        self.weights = weights
        self.param_path = Path(param_path)
        self.a_matrix, self.b_matrix = build_continuous_state_matrices(model_params)
        self.q_matrix = np.diag(weights.q_diag)
        self.r_matrix = np.array([[weights.r_value]], dtype=float)
        self.k_matrix: np.ndarray | None = None

    def compute_lqr_matrices(self) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        # 方便外部查看模型矩阵与权重矩阵。
        return self.a_matrix, self.b_matrix, self.q_matrix, self.r_matrix

    def generate_k_matrix(self) -> np.ndarray:
        # 生成连续时间 LQR 增益 K。
        p_matrix = solve_continuous_are(self.a_matrix, self.b_matrix, self.q_matrix, self.r_matrix)
        self.k_matrix = np.linalg.solve(self.r_matrix, self.b_matrix.T @ p_matrix)
        return self.k_matrix

    def ensure_gain(self) -> np.ndarray:
        # 运行时优先从 YAML 读取；如果当前实例已加载过就直接复用。
        if self.k_matrix is None:
            self.load_yaml()
        if self.k_matrix is None:
            raise RuntimeError(
                f"Missing LQR parameters. Run `python -m controllers.lqr` first to generate {self.param_path}."
            )
        return self.k_matrix

    def save_yaml(self, path: str | Path | None = None) -> Path:
        # 把增益和必要元信息保存到 YAML。
        gain = self.k_matrix if self.k_matrix is not None else self.generate_k_matrix()
        path = self._normalize_yaml_path(path or self.param_path)
        path.parent.mkdir(parents=True, exist_ok=True)

        payload = {
            "version": 1,
            "model_params": asdict(self.model_params),
            "weights": asdict(self.weights),
            "K": gain.tolist(),
        }
        with path.open("w", encoding="utf-8") as f:
            yaml.safe_dump(payload, f, allow_unicode=True, sort_keys=False)
        self.param_path = path
        return path

    def load_yaml(self, path: str | Path | None = None) -> np.ndarray | None:
        # 从 YAML 恢复增益 K。
        path = self._normalize_yaml_path(path or self.param_path)
        if not path.exists():
            self.k_matrix = None
            return None

        with path.open("r", encoding="utf-8") as f:
            payload = yaml.safe_load(f) or {}

        k_value = payload.get("K")
        if k_value is None:
            raise ValueError(f"Missing `K` in YAML file: {path}")

        self.k_matrix = np.asarray(k_value, dtype=float)
        self.param_path = path
        return self.k_matrix

    def control(
        self,
        state: SeaState | np.ndarray,
        reference: SeaState | np.ndarray | None = None,
    ) -> float:
        # 控制流程唯一调用的接口：先从 YAML 取 K，再根据状态计算控制量。
        k_matrix = self.ensure_gain()
        state_vec = self._to_vector(state)
        ref_vec = np.zeros(4, dtype=float) if reference is None else self._to_vector(reference)
        error = state_vec - ref_vec
        action = -(k_matrix @ error.reshape(-1, 1))[0, 0]
        return float(action)

    def export_yaml_snapshot(self) -> dict[str, Any]:
        # 给调试和日志用的轻量快照。
        gain = self.ensure_gain()
        return {
            "version": 1,
            "param_path": str(self.param_path),
            "model_params": asdict(self.model_params),
            "weights": asdict(self.weights),
            "K": gain.tolist(),
        }

    def _to_vector(self, value: SeaState | np.ndarray) -> np.ndarray:
        if isinstance(value, SeaState):
            return value.as_vector()
        return np.asarray(value, dtype=float).reshape(4)

    def _normalize_yaml_path(self, path: str | Path) -> Path:
        normalized = Path(path)
        if normalized.suffix.lower() not in {".yaml", ".yml"}:
            normalized = normalized.with_suffix(".yaml")
        return normalized


def main() -> None:
    # 单独运行时：求解 K 并保存到 controllers/lqr_params.yaml。
    controller = LqrController()
    gain = controller.generate_k_matrix()
    path = controller.save_yaml()
    print(f"Saved LQR parameters to {path}")
    print("K =")
    print(gain)


if __name__ == "__main__":
    main()


__all__ = [
    "DEFAULT_LQR_PARAM_PATH",
    "DEFAULT_LQR_WEIGHTS",
    "LqrController",
    "LqrWeights",
]
