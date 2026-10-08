#!/usr/bin/env python3
"""
Plot FSEA torque-excitation test logs.
"""
from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd

from plot_common import require_columns, warn_if_flat_signal


REQUIRED_COLUMNS = [
    "time_s",
    "command_torque_nm",
    "action_nm",
    "spring_delta_theta_rad",
    "spring_torque_nm",
    "meas_theta_m_rad",
    "meas_theta_l_rad",
    "meas_dtheta_m_rad_s",
    "meas_dtheta_l_rad_s",
]


def plot_fsea_excitation_results(csv_path: Path, output_dir: Path | None = None) -> Path | None:
    df = pd.read_csv(csv_path)
    require_columns(df, REQUIRED_COLUMNS, csv_path)

    warn_if_flat_signal(df, "action_nm", "Applied torque command")
    warn_if_flat_signal(df, "spring_torque_nm", "Estimated spring torque")
    warn_if_flat_signal(df, "spring_delta_theta_rad", "Spring deflection")
    warn_if_flat_signal(df, "meas_theta_l_rad", "Load-side position")

    dt = df["time_s"].diff().dropna()
    time_s = df["time_s"] - df["time_s"].iloc[0]
    motor_load_offset = df["meas_theta_m_rad"].iloc[0] - df["meas_theta_l_rad"].iloc[0]
    aligned_motor = df["meas_theta_m_rad"] - motor_load_offset
    load_relative = df["meas_theta_l_rad"] - df["meas_theta_l_rad"].iloc[0]
    motor_relative = aligned_motor - aligned_motor.iloc[0]
    fig, axes = plt.subplots(4, 2, figsize=(15, 13), sharex=False)
    fig.suptitle(f"FSEA Torque Excitation - {csv_path.stem}", fontsize=14, fontweight="bold")

    ax = axes[0, 0]
    ax.plot(time_s, df["command_torque_nm"], label="Generated Command", linewidth=1.3)
    ax.plot(time_s, df["action_nm"], label="Applied Command", linewidth=1.2, alpha=0.85)
    ax.set_title("Torque Command")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Torque (Nm)")
    ax.legend()
    ax.grid(True, alpha=0.3)

    ax = axes[0, 1]
    ax.plot(time_s, df["spring_torque_nm"], color="tab:red", label="Spring Torque", linewidth=1.2)
    if "torque_nm" in df.columns:
        ax.plot(time_s, df["torque_nm"], color="tab:gray", label="Drive Feedback", linewidth=1.0, alpha=0.7)
    ax.set_title("Torque Response")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Torque (Nm)")
    ax.legend()
    ax.grid(True, alpha=0.3)

    ax = axes[1, 0]
    ax.plot(time_s, df["spring_delta_theta_rad"], color="tab:cyan", label="Third Encoder", linewidth=1.2)
    ax.plot(time_s, aligned_motor - df["meas_theta_l_rad"], color="tab:purple", label="Encoder Diff (zero-shifted)", linewidth=1.0, alpha=0.7)
    ax.set_title("Spring Deflection")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Angle (rad)")
    ax.legend()
    ax.grid(True, alpha=0.3)

    ax = axes[1, 1]
    ax.plot(time_s, load_relative, label="Load Relative", linewidth=1.2)
    ax.plot(time_s, motor_relative, label="Motor Eq. Relative/Aligned", linewidth=1.0, alpha=0.8)
    ax.set_title("Joint Position Relative To Start")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Relative Position (rad)")
    ax.legend()
    ax.grid(True, alpha=0.3)

    ax = axes[2, 0]
    ax.plot(time_s, df["meas_dtheta_l_rad_s"], label="Load", linewidth=1.2)
    ax.plot(time_s, df["meas_dtheta_m_rad_s"], label="Motor Eq.", linewidth=1.0, alpha=0.8)
    ax.set_title("Joint Velocity")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Velocity (rad/s)")
    ax.legend()
    ax.grid(True, alpha=0.3)

    ax = axes[2, 1]
    if "torque_nm" in df.columns:
        ax.scatter(df["action_nm"], df["torque_nm"], s=8, alpha=0.45, label="Drive Feedback")
    ax.scatter(df["action_nm"], df["spring_torque_nm"], s=8, alpha=0.45, label="Spring Torque")
    ax.set_title("Torque Input/Response Scatter")
    ax.set_xlabel("Applied Torque (Nm)")
    ax.set_ylabel("Response Torque (Nm)")
    ax.legend()
    ax.grid(True, alpha=0.3)

    ax = axes[3, 0]
    if not dt.empty:
        ax.plot(time_s.iloc[1:], dt * 1000.0, linewidth=1.0)
    ax.set_title("Sample Interval")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("dt (ms)")
    ax.grid(True, alpha=0.3)

    ax = axes[3, 1]
    stats = [
        f"samples: {len(df)}",
        f"duration: {df['time_s'].iloc[-1] - df['time_s'].iloc[0]:.3f} s",
        f"max |action|: {df['action_nm'].abs().max():.3f} Nm",
        f"max |spring torque|: {df['spring_torque_nm'].abs().max():.3f} Nm",
        f"max |spring defl|: {df['spring_delta_theta_rad'].abs().max():.6f} rad",
        f"motor-load offset: {motor_load_offset:.6f} rad",
    ]
    if not dt.empty:
        stats.extend(
            [
                f"dt mean: {dt.mean() * 1000.0:.3f} ms",
                f"dt max: {dt.max() * 1000.0:.3f} ms",
            ]
        )
    if "third_encoder_crc_ok" in df.columns:
        stats.append(f"third encoder crc all ok: {bool(df['third_encoder_crc_ok'].astype(bool).all())}")
    if "wkc" in df.columns:
        wkc_values = sorted(pd.to_numeric(df["wkc"], errors="coerce").dropna().unique().tolist())
        stats.append(f"WKC values: {wkc_values}")
    ax.text(
        0.08,
        0.5,
        "\n".join(stats),
        transform=ax.transAxes,
        fontsize=10,
        verticalalignment="center",
        bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.5),
        family="monospace",
    )
    ax.axis("off")
    ax.set_title("Run Summary")

    plt.tight_layout()

    output_path = None
    if output_dir is not None:
        output_dir.mkdir(parents=True, exist_ok=True)
        output_path = output_dir / f"{csv_path.stem}.png"
        fig.savefig(output_path, dpi=300, bbox_inches="tight")
        print(f"Figure saved to: {output_path}")

    plt.close(fig)
    return output_path


def main() -> int:
    parser = argparse.ArgumentParser(description="Plot FSEA torque-excitation CSV data")
    parser.add_argument("csv_file", type=str, help="Path to CSV data file")
    parser.add_argument("--output-dir", type=str, default=None)
    args = parser.parse_args()

    csv_path = Path(args.csv_file)
    if not csv_path.exists():
        print(f"Error: CSV file not found: {csv_path}")
        return 1
    output_dir = Path(args.output_dir) if args.output_dir else Path(__file__).resolve().parent / "figures"
    plot_fsea_excitation_results(csv_path, output_dir)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
