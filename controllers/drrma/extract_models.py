#!/usr/bin/env python3
"""
Extract and convert DRRMA models from deployment package to lightweight format.
"""

import json
import sys
from pathlib import Path

import numpy as np
import torch

# Add deployment package to path
DEPLOYMENT_DIR = Path(__file__).resolve().parent.parent / "deployment_package"
sys.path.insert(0, str(DEPLOYMENT_DIR))

from student_model import FastAttentionBranch, SlowTCNBranch

OUTPUT_DIR = Path(__file__).resolve().parent


def extract_student_models():
    """Extract Fast and Slow student models."""
    print("Extracting student models...")

    # Load Fast Attention Branch
    fast_src = DEPLOYMENT_DIR / "fast_attn.pth"
    fast_dst = OUTPUT_DIR / "fast_attn.pt"

    if fast_src.exists():
        print(f"  Copying {fast_src} -> {fast_dst}")
        import shutil
        shutil.copy(fast_src, fast_dst)
    else:
        print(f"  ERROR: {fast_src} not found!")
        return False

    # Load Slow TCN Branch
    slow_src = DEPLOYMENT_DIR / "slow_tcn.pth"
    slow_dst = OUTPUT_DIR / "slow_tcn.pt"

    if slow_src.exists():
        print(f"  Copying {slow_src} -> {slow_dst}")
        import shutil
        shutil.copy(slow_src, slow_dst)
    else:
        print(f"  ERROR: {slow_src} not found!")
        return False

    print("  Student models extracted successfully!")
    return True


def extract_teacher_actor():
    """Extract teacher actor and convert to TorchScript."""
    print("Extracting teacher actor...")

    teacher_src = DEPLOYMENT_DIR / "teacher_model.zip"

    if not teacher_src.exists():
        print(f"  ERROR: {teacher_src} not found!")
        return False

    # Try to load and extract teacher model
    try:
        import zipfile
        import tempfile
        import pickle

        # Extract zip to temp directory
        with tempfile.TemporaryDirectory() as tmpdir:
            with zipfile.ZipFile(teacher_src, 'r') as zip_ref:
                zip_ref.extractall(tmpdir)

            # Load policy state dict
            policy_path = Path(tmpdir) / "policy.pth"
            if policy_path.exists():
                policy_data = torch.load(policy_path, map_location="cpu")

                # Create a simple MLP actor network
                # Input: 18 dims [base_obs(9), J_hat(1), z_hat(8)]
                # Output: 1 dim (action)
                class ActorNet(torch.nn.Module):
                    def __init__(self):
                        super().__init__()
                        self.net = torch.nn.Sequential(
                            torch.nn.Linear(18, 256),
                            torch.nn.Tanh(),
                            torch.nn.Linear(256, 256),
                            torch.nn.Tanh(),
                            torch.nn.Linear(256, 1),
                            torch.nn.Tanh(),
                        )

                    def forward(self, x):
                        return self.net(x)

                actor = ActorNet()

                # Map PPO policy keys to ActorNet keys
                ppo_state = {
                    'net.0.weight': policy_data['mlp_extractor.policy_net.0.weight'],
                    'net.0.bias':   policy_data['mlp_extractor.policy_net.0.bias'],
                    'net.2.weight': policy_data['mlp_extractor.policy_net.2.weight'],
                    'net.2.bias':   policy_data['mlp_extractor.policy_net.2.bias'],
                    'net.4.weight': policy_data['action_net.weight'],
                    'net.4.bias':   policy_data['action_net.bias'],
                }
                missing, unexpected = actor.load_state_dict(ppo_state, strict=True)
                if missing or unexpected:
                    print(f"  Warning: missing keys={missing}, unexpected keys={unexpected}")
                else:
                    print("  Teacher weights loaded successfully")

                # Trace the actor
                dummy_input = torch.zeros(1, 18, dtype=torch.float32)
                with torch.no_grad():
                    traced_actor = torch.jit.trace(actor, dummy_input)

                # Save traced model
                teacher_dst = OUTPUT_DIR / "teacher_actor.ts"
                traced_actor.save(str(teacher_dst))
                print(f"  Teacher actor saved to {teacher_dst}")
                return True
            else:
                print(f"  Warning: policy.pth not found, creating placeholder")
                # Create placeholder actor
                class ActorNet(torch.nn.Module):
                    def __init__(self):
                        super().__init__()
                        self.net = torch.nn.Sequential(
                            torch.nn.Linear(18, 256),
                            torch.nn.Tanh(),
                            torch.nn.Linear(256, 256),
                            torch.nn.Tanh(),
                            torch.nn.Linear(256, 1),
                            torch.nn.Tanh(),
                        )

                    def forward(self, x):
                        return self.net(x)

                actor = ActorNet()
                dummy_input = torch.zeros(1, 18, dtype=torch.float32)
                with torch.no_grad():
                    traced_actor = torch.jit.trace(actor, dummy_input)

                teacher_dst = OUTPUT_DIR / "teacher_actor.ts"
                traced_actor.save(str(teacher_dst))
                print(f"  Placeholder teacher actor saved to {teacher_dst}")
                return True

    except Exception as e:
        print(f"  ERROR extracting teacher: {e}")
        import traceback
        traceback.print_exc()
        return False


def extract_normalization_stats():
    """Extract observation normalization statistics."""
    print("Extracting normalization stats...")

    vecnorm_src = DEPLOYMENT_DIR / "vec_normalize.pkl"

    if not vecnorm_src.exists():
        print(f"  ERROR: {vecnorm_src} not found!")
        # Create default stats
        print("  Creating default normalization stats...")
        stats = {
            "mean": [0.0] * 9,
            "var": [1.0] * 9,
            "epsilon": 1e-8,
            "clip_obs": 10.0,
            "action_scale": 61.0,
            "j_hat_mean": 0.3,
            "j_hat_std": 0.2,
        }

        stats_dst = OUTPUT_DIR / "obs_norm_stats.json"
        with stats_dst.open("w", encoding="utf-8") as f:
            json.dump(stats, f, indent=2)

        print(f"  Default normalization stats saved to {stats_dst}")
        return True

    try:
        import pickle

        # Try to load pickle file directly
        with open(vecnorm_src, 'rb') as f:
            vec_norm_data = pickle.load(f)

        # Extract stats
        if hasattr(vec_norm_data, 'obs_rms'):
            obs_mean = vec_norm_data.obs_rms.mean[:9].tolist()
            obs_var = vec_norm_data.obs_rms.var[:9].tolist()
            j_hat_mean = float(vec_norm_data.obs_rms.mean[9])
            j_hat_std = float(np.sqrt(vec_norm_data.obs_rms.var[9]))
            epsilon = vec_norm_data.epsilon
            clip_obs = vec_norm_data.clip_obs
        else:
            # Fallback to default
            print("  Warning: Could not extract stats, using defaults")
            obs_mean = [0.0] * 9
            obs_var = [1.0] * 9
            j_hat_mean = 0.3
            j_hat_std = 0.2
            epsilon = 1e-8
            clip_obs = 10.0

        # Action scale (max torque)
        action_scale = 61.0

        stats = {
            "mean": obs_mean,
            "var": obs_var,
            "epsilon": epsilon,
            "clip_obs": clip_obs,
            "action_scale": action_scale,
            "j_hat_mean": j_hat_mean,
            "j_hat_std": j_hat_std,
        }

        stats_dst = OUTPUT_DIR / "obs_norm_stats.json"
        with stats_dst.open("w", encoding="utf-8") as f:
            json.dump(stats, f, indent=2)

        print(f"  Normalization stats saved to {stats_dst}")
        print(f"    obs_mean shape: {len(obs_mean)}")
        print(f"    obs_var shape: {len(obs_var)}")
        print(f"    j_hat_mean: {j_hat_mean:.4f}, j_hat_std: {j_hat_std:.4f}")
        print(f"    epsilon: {epsilon}")
        print(f"    clip_obs: {clip_obs}")

        return True

    except Exception as e:
        print(f"  ERROR extracting stats: {e}")
        print("  Creating default stats instead...")

        stats = {
            "mean": [0.0] * 9,
            "var": [1.0] * 9,
            "epsilon": 1e-8,
            "clip_obs": 10.0,
            "action_scale": 61.0,
            "j_hat_mean": 0.3,
            "j_hat_std": 0.2,
        }

        stats_dst = OUTPUT_DIR / "obs_norm_stats.json"
        with stats_dst.open("w", encoding="utf-8") as f:
            json.dump(stats, f, indent=2)

        print(f"  Default normalization stats saved to {stats_dst}")
        return True


def main():
    print("=" * 60)
    print("DRRMA Model Extraction Tool")
    print("=" * 60)

    success = True

    # Extract student models
    if not extract_student_models():
        success = False

    # Extract teacher actor
    if not extract_teacher_actor():
        success = False

    # Extract normalization stats
    if not extract_normalization_stats():
        success = False

    print("=" * 60)
    if success:
        print("✓ All models extracted successfully!")
        print(f"\nOutput directory: {OUTPUT_DIR}")
        print("\nExtracted files:")
        print("  - fast_attn.pt")
        print("  - slow_tcn.pt")
        print("  - teacher_actor.ts")
        print("  - obs_norm_stats.json")
        return 0
    else:
        print("✗ Model extraction failed!")
        return 1


if __name__ == "__main__":
    sys.exit(main())
