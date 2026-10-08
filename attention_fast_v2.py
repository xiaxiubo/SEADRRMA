from __future__ import annotations

import torch
import torch.nn as nn
import torch.nn.functional as F


class ImprovedAttentionFastBranch(nn.Module):
    """Dual-timescale attention estimator using measurable SEA dynamics only."""

    def __init__(
        self,
        hist_len: int = 100,
        short_len: int = 20,
        d_model: int = 128,
        n_heads: int = 8,
        control_dt: float = 0.005,
        spring_stiffness: float = 2400.0,
        j_min: float = 0.03,
        j_max: float = 1.0,
    ) -> None:
        super().__init__()
        self.hist_len = int(hist_len)
        self.short_len = int(short_len)
        self.control_dt = float(control_dt)
        self.spring_stiffness = float(spring_stiffness)
        self.j_min = float(j_min)
        self.j_max = float(j_max)
        self.d_model = int(d_model)

        # No reference position/phase and no privileged friction parameters.
        self.temporal_stem = nn.Sequential(
            nn.Conv1d(8, 64, kernel_size=5, padding=2),
            nn.ELU(),
            nn.Conv1d(64, d_model, kernel_size=3, padding=1),
            nn.ELU(),
        )
        self.position_embedding = nn.Parameter(torch.zeros(hist_len, d_model))
        self.short_attention = nn.MultiheadAttention(d_model, n_heads, batch_first=True)
        self.long_attention = nn.MultiheadAttention(d_model, n_heads, batch_first=True)
        branch_features = d_model * 3
        self.short_head = nn.Sequential(
            nn.Linear(branch_features, 128), nn.ELU(), nn.Linear(128, 64), nn.ELU(), nn.Linear(64, 1)
        )
        self.long_head = nn.Sequential(
            nn.Linear(branch_features, 128), nn.ELU(), nn.Linear(128, 64), nn.ELU(), nn.Linear(64, 1)
        )
        self.gate_head = nn.Sequential(
            nn.Linear(d_model * 4, 128),
            nn.ELU(),
            nn.Linear(128, 64),
            nn.ELU(),
            nn.Linear(64, 2),
        )

    def _bounded_inertia(self, raw: torch.Tensor) -> torch.Tensor:
        return self.j_min + (self.j_max - self.j_min) * torch.sigmoid(raw)

    def dynamics_features(self, history: torch.Tensor) -> torch.Tensor:
        if history.ndim != 3 or history.shape[-1] != 9:
            raise ValueError(f"Expected history shape (B, T, 9), got {tuple(history.shape)}")
        target_velocity = history[..., 7]
        load_velocity = target_velocity - history[..., 1]
        motor_velocity = target_velocity - history[..., 3]
        load_acceleration = torch.zeros_like(load_velocity)
        motor_acceleration = torch.zeros_like(motor_velocity)
        load_acceleration[:, 1:] = (load_velocity[:, 1:] - load_velocity[:, :-1]) / self.control_dt
        motor_acceleration[:, 1:] = (motor_velocity[:, 1:] - motor_velocity[:, :-1]) / self.control_dt
        load_acceleration[:, 0] = load_acceleration[:, 1]
        motor_acceleration[:, 0] = motor_acceleration[:, 1]
        spring_deflection = history[..., 4]
        spring_velocity = history[..., 5]
        spring_torque = -self.spring_stiffness * spring_deflection
        previous_action = history[..., 8]
        return torch.stack(
            [
                load_velocity / 10.0,
                load_acceleration / 50.0,
                motor_velocity / 10.0,
                motor_acceleration / 50.0,
                spring_deflection / 0.02,
                spring_velocity / 10.0,
                spring_torque / 50.0,
                previous_action,
            ],
            dim=-1,
        )

    @staticmethod
    def _branch_features(context: torch.Tensor, latest: torch.Tensor) -> torch.Tensor:
        return torch.cat([context, latest, context - latest], dim=-1)

    def forward_with_aux(self, history: torch.Tensor, need_weights: bool = False):
        sequence = self.dynamics_features(history).transpose(1, 2)
        encoded = self.temporal_stem(sequence).transpose(1, 2)
        encoded = encoded + self.position_embedding[-encoded.shape[1] :].unsqueeze(0)
        latest_query = encoded[:, -1:, :]
        latest = latest_query.squeeze(1)
        short_encoded = encoded[:, -min(self.short_len, encoded.shape[1]) :, :]
        short_context, short_weights = self.short_attention(
            latest_query,
            short_encoded,
            short_encoded,
            need_weights=need_weights,
            average_attn_weights=True,
        )
        long_context, long_weights = self.long_attention(
            latest_query,
            encoded,
            encoded,
            need_weights=need_weights,
            average_attn_weights=True,
        )
        short_context = short_context.squeeze(1)
        long_context = long_context.squeeze(1)
        short_estimate = self._bounded_inertia(
            self.short_head(self._branch_features(short_context, latest))
        )
        long_estimate = self._bounded_inertia(
            self.long_head(self._branch_features(long_context, latest))
        )
        gate_output = self.gate_head(
            torch.cat([short_context, long_context, latest, short_context - long_context], dim=-1)
        )
        jump_gate = torch.sigmoid(gate_output[:, :1])
        confidence = torch.sigmoid(gate_output[:, 1:2])
        estimate = jump_gate * short_estimate + (1.0 - jump_gate) * long_estimate

        attention_weights = None
        if need_weights:
            short_weights = short_weights.squeeze(1)
            long_weights = long_weights.squeeze(1)
            short_padded = F.pad(short_weights, (encoded.shape[1] - short_weights.shape[1], 0))
            attention_weights = torch.stack([short_padded, long_weights], dim=1)
        aux = {
            "short_estimate": short_estimate,
            "long_estimate": long_estimate,
            "jump_gate": jump_gate,
            "jump_logit": gate_output[:, :1],
            "confidence": confidence,
            "confidence_logit": gate_output[:, 1:2],
            "attention_weights": attention_weights,
        }
        return estimate, aux

    def forward(self, history: torch.Tensor, need_weights: bool = True):
        estimate, aux = self.forward_with_aux(history, need_weights=need_weights)
        return estimate, aux["attention_weights"]
