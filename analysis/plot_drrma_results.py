#!/usr/bin/env python3
"""
DRRMA 控制结果可视化脚本
读取 CSV 数据并绘制位置跟踪、弹簧估计、动作抖动和力矩等图表。
"""
from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd

from plot_common import (
    find_latest_csv,
    print_summary,
    require_columns,
    summarize_timing,
    warn_if_flat_signal,
)


REQUIRED_COLUMNS = [
    "cycle_index",
    "time_s",
    "ref_theta_l_rad",
    "meas_theta_l_rad",
    "err_theta_l_rad",
    "spring_defl_rad",
    "spring_vel_rad_s",
    "action_norm",
    "action_nm",
    "J_hat",
    "J_true",
    "target_torque",
    "wkc",
]


def _add_derived_columns(df: pd.DataFrame) -> pd.DataFrame:
    df = df.copy()
    time_delta_s = df["time_s"].diff()
    action_delta_nm = df["action_nm"].diff().fillna(0.0)
    action_norm_delta = df["action_norm"].diff().fillna(0.0)
    df["action_delta_nm"] = action_delta_nm
    df["action_norm_delta"] = action_norm_delta
    df["action_slew_nm_s"] = action_delta_nm / time_delta_s.replace(0.0, pd.NA)
    df["target_torque_delta"] = df["target_torque"].diff().fillna(0.0)
    return df


def _plot_optional_j_hat(ax, df: pd.DataFrame) -> None:
    j_hat = pd.to_numeric(df["J_hat"], errors="coerce")
    j_true = pd.to_numeric(df["J_true"], errors="coerce")
    if j_hat.notna().any():
        ax.plot(df["time_s"], j_hat, "tab:purple", label="J_hat", linewidth=1.5)
    if j_true.notna().any() and j_true.abs().max() > 0.0:
        ax.plot(df["time_s"], j_true, "k--", label="J_true", linewidth=1.2)
    if not j_hat.notna().any() and not (j_true.notna().any() and j_true.abs().max() > 0.0):
        ax.text(
            0.5,
            0.5,
            "J_hat is not exported by the ONNX control-only model",
            transform=ax.transAxes,
            ha="center",
            va="center",
        )
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Inertia")
    ax.set_title("Inertia Estimate")
    lines, labels = ax.get_legend_handles_labels()
    if lines:
        ax.legend(lines, labels, loc="best")
    ax.grid(True, alpha=0.3)


def plot_drrma_results(csv_path: Path, output_dir: Path | None = None) -> None:
    df = pd.read_csv(csv_path)
    require_columns(df, REQUIRED_COLUMNS, csv_path)
    df = _add_derived_columns(df)

    warn_if_flat_signal(df, "meas_theta_l_rad", "Load-side measured position")
    warn_if_flat_signal(df, "spring_defl_rad", "Spring deflection")
    warn_if_flat_signal(df, "spring_vel_rad_s", "Spring velocity")
    warn_if_flat_signal(df, "action_nm", "DRRMA action")
    warn_if_flat_signal(df, "target_torque", "Target torque command")
    print_summary("Timing summary (ms):", summarize_timing(df, []))

    fig, axes = plt.subplots(4, 2, figsize=(14, 14))
    fig.suptitle(f"DRRMA Control Results - {csv_path.stem}", fontsize=14, fontweight="bold")

    ax = axes[0, 0]
    ax.plot(df["time_s"], df["ref_theta_l_rad"], "b--", label="Reference", linewidth=2)
    ax.plot(df["time_s"], df["meas_theta_l_rad"], "r-", label="Measured", linewidth=1.5)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Load Position (rad)")
    ax.set_title("Load Side Position Tracking")
    ax.legend()
    ax.grid(True, alpha=0.3)

    ax = axes[0, 1]
    ax.plot(df["time_s"], df["err_theta_l_rad"], "m-", label="Load Position Error", linewidth=1.5)
    ax.axhline(y=0, color="k", linestyle="--", linewidth=0.8)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Position Error (rad)")
    ax.set_title("Load Position Tracking Error")
    ax.legend()
    ax.grid(True, alpha=0.3)

    ax = axes[1, 0]
    ax.plot(df["time_s"], df["action_nm"], "tab:blue", label="Action", linewidth=1.5)
    ax.axhline(y=0, color="k", linestyle="--", linewidth=0.8)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Torque (Nm)")
    ax.set_title("DRRMA Command Torque")
    ax.legend()
    ax.grid(True, alpha=0.3)

    ax = axes[1, 1]
    ax.plot(df["time_s"], df["action_delta_nm"], "tab:red", label="Delta Action", linewidth=1.2)
    ax2 = ax.twinx()
    ax2.plot(df["time_s"], df["action_slew_nm_s"], "tab:orange", label="Action Slew", linewidth=1.0, alpha=0.75)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Delta Torque (Nm)")
    ax2.set_ylabel("Torque Slew (Nm/s)")
    ax.set_title("Action Jitter")
    lines_1, labels_1 = ax.get_legend_handles_labels()
    lines_2, labels_2 = ax2.get_legend_handles_labels()
    ax.legend(lines_1 + lines_2, labels_1 + labels_2, loc="best")
    ax.grid(True, alpha=0.3)

    ax = axes[2, 0]
    ax.plot(df["time_s"], df["action_norm"], "tab:green", label="Action Norm", linewidth=1.5)
    ax.plot(df["time_s"], df["action_norm_delta"], "tab:red", label="Delta Action Norm", linewidth=1.0, alpha=0.8)
    ax.axhline(y=0, color="k", linestyle="--", linewidth=0.8)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Normalized Action")
    ax.set_title("Policy Output")
    ax.legend()
    ax.grid(True, alpha=0.3)

    ax = axes[2, 1]
    ax.plot(df["time_s"], df["spring_defl_rad"], "c-", label="Spring Deflection", linewidth=1.5)
    ax2 = ax.twinx()
    ax2.plot(df["time_s"], df["spring_vel_rad_s"], "tab:orange", label="Spring Velocity", linewidth=1.0, alpha=0.8)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Spring Deflection (rad)")
    ax2.set_ylabel("Spring Velocity (rad/s)")
    ax.set_title("Spring State")
    lines_1, labels_1 = ax.get_legend_handles_labels()
    lines_2, labels_2 = ax2.get_legend_handles_labels()
    ax.legend(lines_1 + lines_2, labels_1 + labels_2, loc="best")
    ax.grid(True, alpha=0.3)

    ax = axes[3, 0]
    ax.plot(df["time_s"], df["target_torque"], "tab:blue", label="Target Torque", linewidth=1.2)
    ax.plot(df["time_s"], df["target_torque_delta"], "tab:red", label="Delta Target", linewidth=1.0, alpha=0.8)
    ax2 = ax.twinx()
    ax2.plot(df["time_s"], df["wkc"], "tab:gray", label="WKC", linewidth=1.0, alpha=0.7)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Target Torque (counts)")
    ax2.set_ylabel("WKC")
    ax.set_title("Target Torque And WKC")
    lines_1, labels_1 = ax.get_legend_handles_labels()
    lines_2, labels_2 = ax2.get_legend_handles_labels()
    ax.legend(lines_1 + lines_2, labels_1 + labels_2, loc="best")
    ax.grid(True, alpha=0.3)

    ax = axes[3, 1]
    _plot_optional_j_hat(ax, df)

    fig_metrics = plt.figure(figsize=(10, 5))
    fig_metrics.suptitle(f"DRRMA Metrics - {csv_path.stem}", fontsize=14, fontweight="bold")
    ax = fig_metrics.add_subplot(111)
    err_abs = df["err_theta_l_rad"].abs()
    action_abs = df["action_nm"].abs()
    action_delta_abs = df["action_delta_nm"].abs()
    slew_abs = pd.to_numeric(df["action_slew_nm_s"], errors="coerce").abs().dropna()
    stats_text = "DRRMA Metrics:\n"
    stats_text += f"Load Pos Error mean/std/max: {err_abs.mean():.5f} / {err_abs.std():.5f} / {err_abs.max():.5f} rad\n"
    stats_text += f"Action |mean|max: {action_abs.mean():.3f} / {action_abs.max():.3f} Nm\n"
    stats_text += f"Delta Action |mean|p95|max: {action_delta_abs.mean():.3f} / {action_delta_abs.quantile(0.95):.3f} / {action_delta_abs.max():.3f} Nm\n"
    stats_text += f"Action Slew |mean|p95|max: {slew_abs.mean():.1f} / {slew_abs.quantile(0.95):.1f} / {slew_abs.max():.1f} Nm/s\n"
    stats_text += f"Spring Defl max: {df['spring_defl_rad'].abs().max():.5f} rad\n"
    stats_text += f"Spring Vel max: {df['spring_vel_rad_s'].abs().max():.3f} rad/s\n"
    stats_text += f"WKC values: {sorted(df['wkc'].dropna().unique().tolist())}"
    ax.text(
        0.05,
        0.5,
        stats_text,
        transform=ax.transAxes,
        fontsize=11,
        verticalalignment="center",
        bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.5),
        family="monospace",
    )
    ax.axis("off")

    fig.tight_layout()
    fig_metrics.tight_layout()

    if output_dir is not None:
        output_dir.mkdir(parents=True, exist_ok=True)
        output_path = output_dir / f"{csv_path.stem}.png"
        metrics_path = output_dir / f"{csv_path.stem}_metrics.png"
        fig.savefig(output_path, dpi=300, bbox_inches="tight")
        fig_metrics.savefig(metrics_path, dpi=300, bbox_inches="tight")
        print(f"Figure saved to: {output_path}")
        print(f"Metrics saved to: {metrics_path}")

    plt.show()


def main() -> int:
    parser = argparse.ArgumentParser(description="Plot DRRMA control results from CSV data")
    parser.add_argument("csv_file", type=str, nargs="?", default=None, help="Path to CSV data file")
    parser.add_argument(
        "--output-dir",
        type=str,
        default=None,
        help="Directory to save figures (default: analysis/figures)",
    )
    args = parser.parse_args()

    if args.csv_file:
        csv_path = Path(args.csv_file)
    else:
        log_dir = Path(__file__).resolve().parents[1] / "logs"
        latest_csv = find_latest_csv(log_dir, "drrma_data_*.csv")
        if latest_csv is None:
            print(f"Error: No DRRMA CSV files found in {log_dir}")
            return 1
        csv_path = latest_csv

    if not csv_path.exists():
        print(f"Error: CSV file not found: {csv_path}")
        return 1

    print(f"Using CSV file: {csv_path}")
    output_dir = Path(args.output_dir) if args.output_dir else Path(__file__).resolve().parent / "figures"
    plot_drrma_results(csv_path, output_dir)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
