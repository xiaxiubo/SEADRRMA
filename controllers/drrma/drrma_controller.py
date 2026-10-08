from __future__ import annotations

import json
from pathlib import Path
from collections import deque

import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F


OBS_DIM = 9
HIST_LEN = 100
MODEL_DIR = Path(__file__).resolve().parent
DEFAULT_FAST_PATH = MODEL_DIR / "fast_attn.pt"
DEFAULT_SLOW_PATH = MODEL_DIR / "slow_tcn.pt"
DEFAULT_TEACHER_PATH = MODEL_DIR / "teacher_actor.ts"
DEFAULT_STATS_PATH = MODEL_DIR / "obs_norm_stats.json"


# ================================================================
#  Fast Attention Branch (copied from student_model.py)
# ================================================================
class FastAttentionBranch(nn.Module):
    def __init__(
        self,
        hist_dim: int = 9,
        d_model: int = 128,
        n_heads: int = 8,
        hist_len: int = 100,
    ):
        super().__init__()
        self.d_model = d_model
        self.hist_len = hist_len

        self.obs_encoder = nn.Sequential(
            nn.Linear(hist_dim, d_model),
            nn.ELU(),
            nn.Linear(d_model, d_model),
            nn.ELU(),
            nn.Linear(d_model, d_model),
            nn.ELU(),
        )
        self.pos_embed = nn.Parameter(torch.zeros(hist_len, d_model))
        self.attn = nn.MultiheadAttention(
            embed_dim=d_model,
            num_heads=n_heads,
            batch_first=True,
        )
        self.head = nn.Sequential(
            nn.Linear(d_model * 3, 128),
            nn.ELU(),
            nn.Linear(128, 64),
            nn.ELU(),
            nn.Linear(64, 1),
            nn.Softplus(),
        )


    def forward(self, H: torch.Tensor):
        B, T, _ = H.shape
        x = self.obs_encoder(H)
        pos = self.pos_embed[-T:].unsqueeze(0)
        x = x + pos

        latest = x[:, -1:, :]
        ctx, attn_w = self.attn(
            query=latest,
            key=x,
            value=x,
            need_weights=True,
            average_attn_weights=True,
        )
        ctx = ctx.squeeze(1)
        latest = latest.squeeze(1)

        features = torch.cat([ctx, latest, ctx - latest], dim=-1)
        J_hat = self.head(features)
        return J_hat, attn_w


# ================================================================
#  Causal Conv1D Block
# ================================================================
class CausalConv1dBlock(nn.Module):
    def __init__(self, in_ch: int, out_ch: int, kernel: int, dilation: int = 1):
        super().__init__()
        self.pad = (kernel - 1) * dilation
        self.conv = nn.Conv1d(
            in_ch, out_ch, kernel_size=kernel,
            dilation=dilation, padding=0
        )
        self.norm = nn.LayerNorm(out_ch)
        self.act = nn.ELU()
        self.res = nn.Conv1d(in_ch, out_ch, 1) if in_ch != out_ch else nn.Identity()

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        x_pad = F.pad(x, (self.pad, 0))
        out = self.conv(x_pad)
        out = self.norm(out.transpose(1, 2)).transpose(1, 2)
        out = self.act(out + self.res(x))
        return out


# ================================================================
#  Slow TCN Branch
# ================================================================
class SlowTCNBranch(nn.Module):
    def __init__(self, hist_dim: int = 9, z_dim: int = 8, hist_len: int = 100):
        super().__init__()
        self.hist_len = hist_len
        self.tcn = nn.Sequential(
            CausalConv1dBlock(hist_dim, 64, kernel=8, dilation=1),
            CausalConv1dBlock(64, 128, kernel=8, dilation=2),
            CausalConv1dBlock(128, 128, kernel=8, dilation=4),
            CausalConv1dBlock(128, 64, kernel=4, dilation=8),
            CausalConv1dBlock(64, 32, kernel=4, dilation=16),
        )
        self.head = nn.Linear(32, z_dim)


    def forward(self, H: torch.Tensor) -> torch.Tensor:
        x = H.permute(0, 2, 1)
        x = self.tcn(x)
        x = x[:, :, -1]
        return self.head(x)


# ================================================================
#  DRRMA Controller (Deployment Version)
# ================================================================
class DRRMAController:
    """Lightweight DRRMA controller for real-time deployment."""

    def __init__(
        self,
        fast_path: str | Path = DEFAULT_FAST_PATH,
        slow_path: str | Path = DEFAULT_SLOW_PATH,
        teacher_path: str | Path = DEFAULT_TEACHER_PATH,
        stats_path: str | Path = DEFAULT_STATS_PATH,
        device: str = "cpu",
        warmup_steps: int = 3,
    ) -> None:
        self.fast_path = Path(fast_path)
        self.slow_path = Path(slow_path)
        self.teacher_path = Path(teacher_path)
        self.stats_path = Path(stats_path)
        self.device = torch.device(device)

        # Load student models
        self.fast = FastAttentionBranch().to(self.device)
        self.fast.load_state_dict(torch.load(str(self.fast_path), map_location=self.device))
        self.fast.eval()

        self.slow = SlowTCNBranch().to(self.device)
        self.slow.load_state_dict(torch.load(str(self.slow_path), map_location=self.device))
        self.slow.eval()

        # Load teacher actor (TorchScript)
        self.teacher = torch.jit.load(str(self.teacher_path), map_location=self.device)
        self.teacher.eval()

        # Load normalization stats
        with self.stats_path.open("r", encoding="utf-8") as f:
            stats = json.load(f)

        self.obs_mean = np.array(stats["mean"][:OBS_DIM], dtype=np.float32)
        self.obs_var = np.array(stats["var"][:OBS_DIM], dtype=np.float32)
        self._obs_std = np.sqrt(self.obs_var + float(stats["epsilon"]))
        self._obs_mean_t = torch.as_tensor(self.obs_mean, dtype=torch.float32, device=self.device)
        self._obs_std_t = torch.as_tensor(self._obs_std, dtype=torch.float32, device=self.device)
        self.clip_obs = float(stats["clip_obs"])
        self.eps = float(stats["epsilon"])
        self.action_scale = float(stats["action_scale"])

        # J_hat normalization stats (from VecNormalize dim 9)
        self.j_hat_mean = float(stats.get("j_hat_mean", 0.541))
        self.j_hat_std = float(stats.get("j_hat_std", 0.284))

        # History buffer (FIFO queue)
        self.hist_buf = deque(
            [np.zeros(OBS_DIM, dtype=np.float32)] * HIST_LEN, maxlen=HIST_LEN
        )

        # State variables
        self.prev_action_norm = 0.0
        self.nominal_inertia = 0.3  # kg·m²
        self.warmup_time = 0.6  # seconds

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

    def reset(self) -> None:
        """Reset history buffer and state variables."""
        self.hist_buf = deque(
            [np.zeros(OBS_DIM, dtype=np.float32)] * HIST_LEN, maxlen=HIST_LEN
        )
        self.prev_action_norm = 0.0

    def _update_hist(self, obs: np.ndarray) -> None:
        """Update history buffer with new observation."""
        self.hist_buf.append(obs[:OBS_DIM].astype(np.float32))

    def run_inference(self, obs_raw: np.ndarray, elapsed_time: float) -> tuple[float, float, float]:
        """
        Run one DRRMA inference step.

        Args:
            obs_raw: (9,) raw observation [pos_error_load, vel_error_load, pos_error_motor,
                     vel_error_motor, spring_defl, spring_vel, target_pos, target_vel, prev_action]
            elapsed_time: elapsed time in seconds since start

        Returns:
            action_norm: normalized action in [-1, 1]
            torque_cmd_nm: torque command in Nm
            J_hat: estimated inertia in kg·m²
        """
        # Update history buffer
        self._update_hist(obs_raw)

        # Build history tensor
        H = torch.tensor(
            np.array(self.hist_buf, dtype=np.float32), dtype=torch.float32
        ).unsqueeze(0).to(self.device)  # (1, 100, 9)

        # Student inference
        with torch.inference_mode():
            J_hat_t, attn_w_t = self.fast(H)
            z_hat_t = self.slow(H)

        J_hat_predicted = J_hat_t.squeeze().cpu().item()
        z_hat = z_hat_t.squeeze().cpu().numpy()


        # Use nominal inertia during warmup period
        if elapsed_time <= self.warmup_time:
            J_hat = self.nominal_inertia
        else:
            J_hat = J_hat_predicted

        # Normalize base observation (first 9 dims)
        obs_tensor = torch.as_tensor(obs_raw, dtype=torch.float32, device=self.device)
        obs_tensor = (obs_tensor - self._obs_mean_t) / self._obs_std_t
        obs_tensor = torch.clamp(obs_tensor, -self.clip_obs, self.clip_obs).unsqueeze(0)

        # Normalize inertia using training statistics
        J_normalized = (J_hat - self.j_hat_mean) / self.j_hat_std
        J_normalized = np.clip(J_normalized, -self.clip_obs, self.clip_obs)
        J_tensor = torch.tensor([[J_normalized]], dtype=torch.float32, device=self.device)

        # Concatenate [base_obs(9), J_hat(1), z_hat(8)] for teacher
        z_tensor = torch.as_tensor(z_hat, dtype=torch.float32, device=self.device).unsqueeze(0)
        teacher_input = torch.cat([obs_tensor, J_tensor, z_tensor], dim=1)  # (1, 18)

        # Teacher inference
        with torch.inference_mode():
            action = self.teacher(teacher_input)

        if action.shape != (1, 1):
            raise ValueError(f"teacher output must have shape (1, 1), got {tuple(action.shape)}")

        action_norm = float(torch.clamp(action[0, 0], -1.0, 1.0).item())
        torque_cmd_nm = action_norm * self.action_scale
        self.prev_action_norm = action_norm

        return action_norm, torque_cmd_nm, J_hat

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
        """Warmup the models with zero observations."""
        if warmup_steps <= 0:
            return
        saved_prev_action = self.prev_action_norm
        zero_obs = np.zeros((OBS_DIM,), dtype=np.float32)
        for _ in range(warmup_steps):
            self.run_inference(zero_obs, elapsed_time=0.0)
        self.prev_action_norm = saved_prev_action


def self_test() -> int:
    """Self-test function."""
    controller = DRRMAController()
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
    action_norm, torque_cmd_nm, J_hat = controller.run_inference(obs_raw, elapsed_time=0.0)
    print(
        f"self-test ok: fast={controller.fast_path} slow={controller.slow_path} "
        f"teacher={controller.teacher_path} obs_shape={obs_raw.shape} "
        f"action_norm={action_norm:.6f} torque_cmd_nm={torque_cmd_nm:.6f} J_hat={J_hat:.6f}",
        flush=True,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(self_test())

