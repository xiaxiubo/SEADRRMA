from __future__ import annotations

from collections import deque
import os
from pathlib import Path

import numpy as np
import onnxruntime as ort
import torch
import torch.nn as nn
import torch.nn.functional as F
from stable_baselines3 import PPO


OBS_DIM = 9
HIST_LEN = 100
LATENT_DIM = 8
MODEL_DIR = Path(__file__).resolve().parent


class CausalConv1dBlock(nn.Module):
    def __init__(self, in_ch: int, out_ch: int, kernel: int, dilation: int = 1):
        super().__init__()
        self.pad = (kernel - 1) * dilation
        self.conv = nn.Conv1d(in_ch, out_ch, kernel_size=kernel, dilation=dilation)
        self.norm = nn.LayerNorm(out_ch)
        self.act = nn.ELU()
        self.res = nn.Conv1d(in_ch, out_ch, 1) if in_ch != out_ch else nn.Identity()

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        out = self.conv(F.pad(x, (self.pad, 0)))
        out = self.norm(out.transpose(1, 2)).transpose(1, 2)
        return self.act(out + self.res(x))


class TCNBranch(nn.Module):
    def __init__(self):
        super().__init__()
        self.tcn = nn.Sequential(
            CausalConv1dBlock(OBS_DIM, 64, kernel=8, dilation=1),
            CausalConv1dBlock(64, 128, kernel=8, dilation=2),
            CausalConv1dBlock(128, 128, kernel=8, dilation=4),
            CausalConv1dBlock(128, 64, kernel=4, dilation=8),
            CausalConv1dBlock(64, 32, kernel=4, dilation=16),
        )
        self.head = nn.Linear(32, LATENT_DIM)

    def forward(self, history: torch.Tensor) -> torch.Tensor:
        features = self.tcn(history.permute(0, 2, 1))
        return self.head(features[:, :, -1])


class RmaActorController:
    """Single-branch RMA student plus Teacher policy actor for deployment."""

    def __init__(
        self,
        teacher_path: str | Path = MODEL_DIR / "teacher_model.zip",
        student_path: str | Path = MODEL_DIR / "tcn.pth",
        device: str = "cpu",
        action_scale: float = 61.0,
    ):
        self.actor_path = Path(teacher_path)
        self.student_path = Path(student_path)
        self.device = torch.device(device)
        self.action_scale = float(action_scale)

        self.teacher = PPO.load(str(self.actor_path), device=device)
        self.teacher.policy.set_training_mode(False)
        self.tcn = TCNBranch().to(self.device)
        self.tcn.load_state_dict(torch.load(self.student_path, map_location=self.device))
        self.tcn.eval()

        self.obs_mean = torch.zeros(OBS_DIM, dtype=torch.float32, device=self.device)
        self.obs_var = torch.tensor(
            [
                4.17431031e-02,
                1.62242092e00,
                4.17410270e-02,
                1.13738142e00,
                2.15834654e-05,
                7.31311142e-01,
                8.19175153e-02,
                2.97369780e00,
                1.22189974e-01,
            ],
            dtype=torch.float32,
            device=self.device,
        )
        self.obs_std = torch.sqrt(self.obs_var + 1e-8)
        self.clip_obs = 10.0
        self.latent_update_interval = max(1, int(os.environ.get("RMA_LATENT_UPDATE_INTERVAL", "10")))
        self.latent_step = 0
        self.latent: torch.Tensor | None = None
        self.history: deque[np.ndarray] = deque(maxlen=HIST_LEN)
        self.prev_action_norm = 0.0
        self.reset_prev_action()

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
        if out is None:
            out = np.empty(OBS_DIM, dtype=np.float32)
        out[:] = (
            pos_error_load,
            vel_error_load,
            pos_error_motor,
            vel_error_motor,
            spring_defl,
            spring_vel,
            target_pos,
            target_vel,
            self.prev_action_norm,
        )
        return out

    def reset_prev_action(self, value: float = 0.0) -> None:
        self.prev_action_norm = float(np.clip(value, -1.0, 1.0))
        self.latent_step = 0
        self.latent = None
        zero = np.zeros(OBS_DIM, dtype=np.float32)
        self.history = deque([zero.copy() for _ in range(HIST_LEN)], maxlen=HIST_LEN)

    def run_actor(self, obs_raw: np.ndarray) -> tuple[float, float]:
        obs = np.asarray(obs_raw, dtype=np.float32)
        self.history.append(obs.copy())
        history_t = torch.as_tensor(
            np.asarray(self.history),
            dtype=torch.float32,
            device=self.device,
        ).unsqueeze(0)
        obs_t = torch.as_tensor(obs, dtype=torch.float32, device=self.device)
        obs_norm = torch.clamp((obs_t - self.obs_mean) / self.obs_std, -self.clip_obs, self.clip_obs)

        with torch.inference_mode():
            if self.latent is None or self.latent_step % self.latent_update_interval == 0:
                self.latent = self.tcn(history_t)
            policy_features = torch.cat((obs_norm.unsqueeze(0), self.latent), dim=1)
            latent_pi = self.teacher.policy.mlp_extractor.policy_net(policy_features)
            action = self.teacher.policy.action_net(latent_pi)

        self.latent_step += 1
        action_norm = float(torch.clamp(action[0, 0], -1.0, 1.0).item())
        self.prev_action_norm = action_norm
        return action_norm, action_norm * self.action_scale


class RmaOnnxActorController:
    """ONNX Runtime implementation of the same RMA deployment policy."""

    def __init__(
        self,
        tcn_path: str | Path = MODEL_DIR / "rma_tcn.onnx",
        actor_path: str | Path = MODEL_DIR / "rma_actor.onnx",
        action_scale: float = 61.0,
    ):
        self.actor_path = Path(actor_path)
        self.tcn_path = Path(tcn_path)
        options = ort.SessionOptions()
        options.intra_op_num_threads = 1
        options.inter_op_num_threads = 1
        options.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
        self.tcn = ort.InferenceSession(str(self.tcn_path), sess_options=options, providers=["CPUExecutionProvider"])
        self.actor = ort.InferenceSession(
            str(self.actor_path), sess_options=options, providers=["CPUExecutionProvider"]
        )
        self.action_scale = float(action_scale)
        self.obs_mean = np.zeros(OBS_DIM, dtype=np.float32)
        self.obs_std = np.sqrt(
            np.asarray(
                [
                    4.17431031e-02,
                    1.62242092e00,
                    4.17410270e-02,
                    1.13738142e00,
                    2.15834654e-05,
                    7.31311142e-01,
                    8.19175153e-02,
                    2.97369780e00,
                    1.22189974e-01,
                ],
                dtype=np.float32,
            )
            + 1e-8
        )
        self.history_array = np.zeros((1, HIST_LEN, OBS_DIM), dtype=np.float32)
        self.prev_action_norm = 0.0

    def build_observation(self, **kwargs) -> np.ndarray:
        out = kwargs.get("out")
        if out is None:
            out = np.empty(OBS_DIM, dtype=np.float32)
        out[:] = (
            kwargs["pos_error_load"],
            kwargs["vel_error_load"],
            kwargs["pos_error_motor"],
            kwargs["vel_error_motor"],
            kwargs["spring_defl"],
            kwargs["spring_vel"],
            kwargs["target_pos"],
            kwargs["target_vel"],
            self.prev_action_norm,
        )
        return out

    def reset_prev_action(self, value: float = 0.0) -> None:
        self.prev_action_norm = float(np.clip(value, -1.0, 1.0))
        self.history_array.fill(0.0)

    def run_actor(self, obs_raw: np.ndarray) -> tuple[float, float]:
        self.history_array[:, :-1, :] = self.history_array[:, 1:, :]
        self.history_array[0, -1, :] = np.asarray(obs_raw, dtype=np.float32)
        latent = self.tcn.run(None, {"history": self.history_array})[0]
        obs_norm = np.clip(
            (np.asarray(obs_raw, dtype=np.float32) - self.obs_mean) / self.obs_std,
            -10.0,
            10.0,
        ).reshape(1, -1)
        features = np.concatenate((obs_norm, latent), axis=1).astype(np.float32, copy=False)
        action = self.actor.run(None, {"features": features})[0]
        action_norm = float(np.clip(action[0, 0], -1.0, 1.0))
        self.prev_action_norm = action_norm
        return action_norm, action_norm * self.action_scale
