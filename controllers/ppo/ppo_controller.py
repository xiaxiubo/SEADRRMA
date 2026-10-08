from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import torch


OBS_DIM = 9
MODEL_DIR = Path(__file__).resolve().parent
LEGACY_ACTOR_PATH = MODEL_DIR / "actor_only.ts"
LEGACY_STATS_PATH = MODEL_DIR / "obs_norm_stats.json"
DEFAULT_ACTOR_PATH = MODEL_DIR / "actor_only_retrained.ts"
DEFAULT_STATS_PATH = MODEL_DIR / "obs_norm_stats_retrained.json"


class PpoActorController:
    """TorchScript PPO actor wrapper for Python deployment."""

    def __init__(
        self,
        actor_path: str | Path = DEFAULT_ACTOR_PATH,
        stats_path: str | Path = DEFAULT_STATS_PATH,
        device: str = "cpu",
        warmup_steps: int = 3,
    ) -> None:
        self.actor_path = Path(actor_path)
        self.stats_path = Path(stats_path)
        self.device = torch.device(device)

        self.actor = torch.jit.load(str(self.actor_path), map_location=self.device)
        self.actor.eval()

        with self.stats_path.open("r", encoding="utf-8") as f:
            stats = json.load(f)

        self.obs_mean = np.array(stats["mean"], dtype=np.float32)
        self.obs_var = np.array(stats["var"], dtype=np.float32)
        self._obs_std = np.sqrt(self.obs_var + float(stats["epsilon"]))
        self._obs_mean_t = torch.as_tensor(self.obs_mean, dtype=torch.float32, device=self.device)
        self._obs_std_t = torch.as_tensor(self._obs_std, dtype=torch.float32, device=self.device)
        self.clip_obs = float(stats["clip_obs"])
        self.eps = float(stats["epsilon"])
        self.action_scale = float(stats["action_scale"])
        self.prev_action_norm = 0.0

        self._validate_config()
        if warmup_steps > 0:
            self.warmup(warmup_steps)

    def _validate_config(self) -> None:
        if self.obs_mean.shape != (OBS_DIM,):
            raise ValueError(f"obs mean must have shape ({OBS_DIM},), got {self.obs_mean.shape}")
        if self.obs_var.shape != (OBS_DIM,):
            raise ValueError(f"obs var must have shape ({OBS_DIM},), got {self.obs_var.shape}")
        if self.action_scale <= 0.0:
            raise ValueError(f"action_scale must be positive, got {self.action_scale}")
        if self.clip_obs <= 0.0:
            raise ValueError(f"clip_obs must be positive, got {self.clip_obs}")
        if self.eps <= 0.0:
            raise ValueError(f"epsilon must be positive, got {self.eps}")

    def normalize_obs(self, obs_raw: np.ndarray) -> np.ndarray:
        obs_raw = np.asarray(obs_raw, dtype=np.float32)
        if obs_raw.shape != (OBS_DIM,):
            raise ValueError(f"obs_raw must have shape ({OBS_DIM},), got {obs_raw.shape}")

        obs = (obs_raw - self.obs_mean) / self._obs_std
        obs = np.clip(obs, -self.clip_obs, self.clip_obs)
        return obs.astype(np.float32)

    def run_actor(self, obs_raw: np.ndarray) -> tuple[float, float]:
        """
        Run one PPO actor inference step.

        The input observation order is fixed:
        [pos_error_load, vel_error_load, pos_error_motor, vel_error_motor,
         spring_defl, spring_vel, target_pos, target_vel, prev_action_norm].

        Returns:
            action_norm: clipped normalized action in [-1, 1]
            torque_cmd_nm: torque command in Nm
        """
        obs_tensor = torch.as_tensor(obs_raw, dtype=torch.float32, device=self.device)
        if obs_tensor.shape != (OBS_DIM,):
            raise ValueError(f"obs_raw must have shape ({OBS_DIM},), got {tuple(obs_tensor.shape)}")
        obs_tensor = (obs_tensor - self._obs_mean_t) / self._obs_std_t
        obs_tensor = torch.clamp(obs_tensor, -self.clip_obs, self.clip_obs).unsqueeze(0)

        with torch.inference_mode():
            action = self.actor(obs_tensor)

        if action.shape != (1, 1):
            raise ValueError(f"actor output must have shape (1, 1), got {tuple(action.shape)}")

        action_norm = float(torch.clamp(action[0, 0], -1.0, 1.0).item())
        torque_cmd_nm = action_norm * self.action_scale
        self.prev_action_norm = action_norm
        return action_norm, torque_cmd_nm

    def build_observation(
        self,
        pos_error_load: float,
        vel_error_load: float,
        pos_error_motor: float,
        vel_error_motor: float,
        spring_defl: float,
        spring_vel: float,
        target_pos: float,
        target_vel: float,
        out: np.ndarray | None = None,
    ) -> np.ndarray:
        """Build the 9-D raw observation with the last normalized action."""
        if out is None:
            out = np.empty((OBS_DIM,), dtype=np.float32)
        elif out.shape != (OBS_DIM,):
            raise ValueError(f"out must have shape ({OBS_DIM},), got {out.shape}")
        out[0] = pos_error_load
        out[1] = vel_error_load
        out[2] = pos_error_motor
        out[3] = vel_error_motor
        out[4] = spring_defl
        out[5] = spring_vel
        out[6] = target_pos
        out[7] = target_vel
        out[8] = self.prev_action_norm
        return out

    def reset_prev_action(self, value: float = 0.0) -> None:
        self.prev_action_norm = float(np.clip(value, -1.0, 1.0))

    def warmup(self, warmup_steps: int = 3) -> None:
        if warmup_steps <= 0:
            return
        saved_prev_action = self.prev_action_norm
        zero_obs = np.zeros((OBS_DIM,), dtype=np.float32)
        for _ in range(warmup_steps):
            self.run_actor(zero_obs)
        self.prev_action_norm = saved_prev_action


def self_test() -> int:
    controller = PpoActorController()
    obs_raw = controller.build_observation(
        pos_error_load=0.0,
        vel_error_load=0.0,
        pos_error_motor=0.0,
        vel_error_motor=0.0,
        spring_defl=0.0,
        spring_vel=0.0,
        target_pos=0.0,
        target_vel=0.0,
    )
    action_norm, torque_cmd_nm = controller.run_actor(obs_raw)
    print(
        (
            f"self-test ok: actor={controller.actor_path} stats={controller.stats_path} "
            f"obs_shape={obs_raw.shape} action_norm={action_norm:.6f} "
            f"torque_cmd_nm={torque_cmd_nm:.6f} prev_action_norm={controller.prev_action_norm:.6f}"
        ),
        flush=True,
    )
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="PPO actor controller helper")
    parser.add_argument("--self-test", action="store_true", help="Run one zero-observation inference test.")
    args = parser.parse_args()

    _ = args.self_test
    return self_test()


if __name__ == "__main__":
    raise SystemExit(main())
