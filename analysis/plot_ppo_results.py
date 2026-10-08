#!/usr/bin/env python3
"""
PPO 控制结果可视化脚本
读取 CSV 数据并绘制位置跟踪、弹簧估计、力矩等图表
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
    summarize_schedule,
    summarize_timing,
    warn_if_flat_signal,
)


TIMING_COLUMNS = [
    ("read_before_dt_s", "Read Before"),
    ("actor_dt_s", "Actor"),
    ("measured_state_dt_s", "Measured State"),
    ("sample_dt_s", "Third Encoder"),
    ("send_dt_s", "Send"),
    ("spring_state_dt_s", "Spring State"),
    ("read_after_dt_s", "Read After"),
    ("diag_dt_s", "Diagnostics"),
]

REQUIRED_COLUMNS = [
    "time_s",
    "ref_theta_m_rad",
    "ref_theta_l_rad",
    "ref_dtheta_m_rad_s",
    "ref_dtheta_l_rad_s",
    "meas_theta_m_rad",
    "meas_theta_l_rad",
    "meas_dtheta_m_rad_s",
    "meas_dtheta_l_rad_s",
    "err_theta_m_rad",
    "err_theta_l_rad",
    "err_dtheta_m_rad_s",
    "err_dtheta_l_rad_s",
    "spring_defl_rad",
    "spring_torque_nm",
    "action_norm",
    "raw_action_nm",
    "action_nm",
    "target_torque",
    "raw_action_delta_nm",
    "wkc",
    "torque_nm",
    "third_encoder_crc_ok",
    "wake_lag_ms",
    "deadline_slip_ms",
    "sleep_target_ms",
    "sleep_actual_ms",
    "deadline_miss_delta",
    "deadline_miss_count",
    *(column for column, _ in TIMING_COLUMNS),
]


def plot_ppo_results(csv_path: Path, output_dir: Path | None = None) -> None:
    """
    绘制 PPO 控制结果图表

    Args:
        csv_path: CSV 数据文件路径
        output_dir: 图片保存目录，如果为 None 则只显示不保存
    """
    df = pd.read_csv(csv_path)
    require_columns(df, REQUIRED_COLUMNS, csv_path)

    crc_ok = bool(df["third_encoder_crc_ok"].astype(bool).all())
    print(f"Third encoder CRC validity: {crc_ok}")
    if not crc_ok:
        print("Warning: this CSV contains invalid third encoder frames.")

    warn_if_flat_signal(df, "meas_theta_m_rad", "Motor-side measured position")
    warn_if_flat_signal(df, "meas_theta_l_rad", "Load-side measured position")
    warn_if_flat_signal(df, "meas_dtheta_m_rad_s", "Motor-side measured velocity")
    warn_if_flat_signal(df, "meas_dtheta_l_rad_s", "Load-side measured velocity")
    warn_if_flat_signal(df, "spring_defl_rad", "Spring deflection")
    warn_if_flat_signal(df, "spring_torque_nm", "Estimated spring torque")
    warn_if_flat_signal(df, "torque_nm", "Drive torque feedback")
    warn_if_flat_signal(df, "target_torque", "Target torque command")
    warn_if_flat_signal(df, "raw_action_delta_nm", "Raw action delta")
    timing_summary = summarize_timing(df, TIMING_COLUMNS)
    schedule_summary = summarize_schedule(df)
    print_summary("Timing summary (ms):", timing_summary)
    print_summary("Schedule summary:", schedule_summary)

    fig, axes = plt.subplots(5, 2, figsize=(14, 17))
    fig.suptitle(f"PPO Control Results - {csv_path.stem}", fontsize=14, fontweight="bold")

    # 1. 负载侧位置跟踪
    ax = axes[0, 0]
    ax.plot(df["time_s"], df["ref_theta_l_rad"], "b--", label="Reference", linewidth=2)
    ax.plot(df["time_s"], df["meas_theta_l_rad"], "r-", label="Measured", linewidth=1.5)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Load Position (rad)")
    ax.set_title("Load Side Position Tracking")
    ax.legend()
    ax.grid(True, alpha=0.3)

    # 2. 电机侧位置跟踪
    ax = axes[0, 1]
    ax.plot(df["time_s"], df["ref_theta_m_rad"], "b--", label="Reference", linewidth=2)
    ax.plot(df["time_s"], df["meas_theta_m_rad"], "r-", label="Measured", linewidth=1.5)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Motor Position (rad)")
    ax.set_title("Motor Side Position Tracking")
    ax.legend()
    ax.grid(True, alpha=0.3)

    # 3. 位置跟踪误差
    ax = axes[1, 0]
    ax.plot(df["time_s"], df["err_theta_l_rad"], "m-", label="Load Position Error", linewidth=1.5)
    ax.plot(df["time_s"], df["err_theta_m_rad"], "g-", label="Motor Position Error", linewidth=1.5)
    ax.axhline(y=0, color="k", linestyle="--", linewidth=0.8)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Position Error (rad)")
    ax.set_title("Position Tracking Error")
    ax.legend()
    ax.grid(True, alpha=0.3)

    # 4. 速度跟踪
    ax = axes[1, 1]
    ax.plot(df["time_s"], df["ref_dtheta_l_rad_s"], "b--", label="Ref Velocity", linewidth=2)
    ax.plot(df["time_s"], df["meas_dtheta_l_rad_s"], "r-", label="Meas Velocity", linewidth=1.5)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Load Velocity (rad/s)")
    ax.set_title("Load Side Velocity Tracking")
    ax.legend()
    ax.grid(True, alpha=0.3)

    # 5. PPO 力矩相关量
    ax = axes[2, 0]
    ax.plot(df["time_s"], df["raw_action_nm"], "b-", label="Raw Action", linewidth=1.5)
    ax.plot(df["time_s"], df["action_nm"], "r-", label="Send Action", linewidth=1.5)
    ax.plot(df["time_s"], df["torque_nm"], "k-", label="Drive Feedback", linewidth=1.2, alpha=0.8)
    ax.axhline(y=0, color="k", linestyle="--", linewidth=0.8)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Torque (Nm)")
    ax.set_title("PPO Torque Terms")
    ax.legend()
    ax.grid(True, alpha=0.3)

    # 6. 弹簧形变
    ax = axes[2, 1]
    ax.plot(df["time_s"], df["spring_defl_rad"], "c-", label="Spring Deflection", linewidth=1.5)
    ax.axhline(y=0, color="k", linestyle="--", linewidth=0.8)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Angle (rad)")
    ax.set_title("Spring Deflection")
    ax.legend()
    ax.grid(True, alpha=0.3)

    # 7. 弹簧力矩
    ax = axes[3, 0]
    ax.plot(df["time_s"], df["spring_torque_nm"], "m-", label="Estimated Spring Torque", linewidth=1.5)
    ax.axhline(y=0, color="k", linestyle="--", linewidth=0.8)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Torque (Nm)")
    ax.set_title("Spring Torque Estimation")
    ax.legend()
    ax.grid(True, alpha=0.3)

    # 8. 目标值与 WKC
    ax = axes[3, 1]
    ax.plot(df["time_s"], df["target_torque"], "tab:orange", label="Target Torque", linewidth=1.5)
    ax2 = ax.twinx()
    ax2.plot(df["time_s"], df["wkc"], "tab:gray", label="WKC", linewidth=1.2, alpha=0.8)
    ax2.set_ylabel("WKC")
    lines_1, labels_1 = ax.get_legend_handles_labels()
    lines_2, labels_2 = ax2.get_legend_handles_labels()
    ax.legend(lines_1 + lines_2, labels_1 + labels_2, loc="best")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Target Torque (counts)")
    ax.set_title("Target Torque And WKC")
    ax.grid(True, alpha=0.3)

    # 9. 动作归一化
    ax = axes[4, 0]
    ax.plot(df["time_s"], df["action_norm"], "tab:green", label="Action Norm", linewidth=1.5)
    ax.axhline(y=0, color="k", linestyle="--", linewidth=0.8)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Normalized Action")
    ax.set_title("Policy Output")
    ax.legend()
    ax.grid(True, alpha=0.3)

    # 10. 误差统计
    ax = axes[4, 1]
    errors = {
        "Load Pos": df["err_theta_l_rad"].abs(),
        "Motor Pos": df["err_theta_m_rad"].abs(),
        "Load Vel": df["err_dtheta_l_rad_s"].abs(),
        "Motor Vel": df["err_dtheta_m_rad_s"].abs(),
    }
    stats_text = "PPO Error Statistics (Mean ± Std):\n"
    stats_text += f"Load Pos:  {errors['Load Pos'].mean():.4f} ± {errors['Load Pos'].std():.4f} rad\n"
    stats_text += f"Motor Pos: {errors['Motor Pos'].mean():.4f} ± {errors['Motor Pos'].std():.4f} rad\n"
    stats_text += f"Load Vel:  {errors['Load Vel'].mean():.4f} ± {errors['Load Vel'].std():.4f} rad/s\n"
    stats_text += f"Motor Vel: {errors['Motor Vel'].mean():.4f} ± {errors['Motor Vel'].std():.4f} rad/s\n"
    stats_text += f"\nMax |Raw Action|: {df['raw_action_nm'].abs().max():.2f} Nm"
    stats_text += f"\nMax |Spring Torque|: {df['spring_torque_nm'].abs().max():.2f} Nm"
    stats_text += f"\nMax |Send Action|: {df['action_nm'].abs().max():.2f} Nm"
    stats_text += f"\nMax |Raw Action Delta|: {df['raw_action_delta_nm'].abs().max():.2f} Nm"
    stats_text += f"\nMax |Drive Torque|: {df['torque_nm'].abs().max():.2f} Nm"
    stats_text += f"\nWKC values: {sorted(df['wkc'].dropna().unique().tolist())}"
    stats_text += "\n\nTiming (mean / p95 / p99 / max):"
    for row in timing_summary:
        stats_text += (
            f"\n{row['name']}: {row['mean_ms']:.3f} / {row['p95_ms']:.3f} / "
            f"{row['p99_ms']:.3f} / {row['max_ms']:.3f} ms"
        )

    ax.text(
        0.1,
        0.5,
        stats_text,
        transform=ax.transAxes,
        fontsize=10,
        verticalalignment="center",
        bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.5),
        family="monospace",
    )
    ax.axis("off")
    ax.set_title("Performance Metrics")

    plt.tight_layout()

    if output_dir is not None:
        output_dir.mkdir(parents=True, exist_ok=True)
        output_path = output_dir / f"{csv_path.stem}.png"
        plt.savefig(output_path, dpi=300, bbox_inches="tight")
        print(f"Figure saved to: {output_path}")

    plt.show()


def main() -> int:
    parser = argparse.ArgumentParser(description="Plot PPO control results from CSV data")
    parser.add_argument("csv_file", type=str, nargs="?", default=None, help="Path to CSV data file")
    parser.add_argument("--output-dir", type=str, default=None, help="Directory to save figures (default: show only)")
    args = parser.parse_args()

    if args.csv_file:
        csv_path = Path(args.csv_file)
    else:
        log_dir = Path(__file__).resolve().parents[1] / "logs"
        # csv_files = [log_dir / "ppo_data_20260423_151924.csv"]
        latest_csv = find_latest_csv(log_dir, "ppo_data_*.csv")
        if latest_csv is None:
            print(f"Error: No PPO CSV files found in {log_dir}")
            return 1
        csv_path = latest_csv

    if not csv_path.exists():
        print(f"Error: CSV file not found: {csv_path}")
        return 1

    print(f"Using CSV file: {csv_path}")

    output_dir = Path(args.output_dir) if args.output_dir else None
    plot_ppo_results(csv_path, output_dir)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
