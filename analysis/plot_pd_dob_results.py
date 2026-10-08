#!/usr/bin/env python3
"""
PD-DOB 控制结果可视化脚本
读取 CSV 数据并绘制位置跟踪、弹簧扭矩、DOB 和控制力矩等图表
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
    ("controller_dt_s", "Controller"),
    ("sample_dt_s", "Third Encoder"),
    ("send_dt_s", "Send"),
    ("read_after_dt_s", "Read After"),
    ("diag_dt_s", "Diagnostics"),
    ("buffer_append_dt_s", "Buffer Append"),
    ("cycle_total_dt_s", "Cycle Total"),
]

SCHEDULE_COLUMNS = [
    "wake_lag_ms",
    "deadline_slip_ms",
    "sleep_target_ms",
    "sleep_actual_ms",
    "deadline_miss_delta",
    "deadline_miss_count",
]

REQUIRED_COLUMNS = [
    "time_s",
    "ref_theta_l_rad",
    "ref_dtheta_l_rad_s",
    "meas_theta_l_rad",
    "meas_dtheta_l_rad_s",
    "err_theta_l_rad",
    "err_dtheta_l_rad_s",
    "torque_ref_nm",
    "disturbance_hat_nm",
    "raw_action_nm",
    "action_nm",
    "target_torque",
    "wkc",
    "spring_delta_theta_rad",
    "spring_torque_nm",
    "third_encoder_crc_ok",
]


def plot_pd_dob_results(csv_path: Path, output_dir: Path | None = None) -> None:
    """
    绘制 PD-DOB 控制结果图表

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

    warn_if_flat_signal(df, "meas_theta_l_rad", "Load-side measured position")
    warn_if_flat_signal(df, "meas_dtheta_l_rad_s", "Load-side measured velocity")
    warn_if_flat_signal(df, "spring_torque_nm", "Estimated spring torque")
    warn_if_flat_signal(df, "disturbance_hat_nm", "DOB disturbance estimate")

    available_timing_columns = [
        (column, label) for column, label in TIMING_COLUMNS if column in df.columns
    ]
    if len(available_timing_columns) != len(TIMING_COLUMNS):
        missing_timing_columns = [
            column for column, _ in TIMING_COLUMNS if column not in df.columns
        ]
        print(
            "Timing detail columns not found; printing cycle interval only. "
            f"Missing: {', '.join(missing_timing_columns)}"
        )
    print_summary("Timing summary (ms):", summarize_timing(df, available_timing_columns))

    missing_schedule_columns = [
        column for column in SCHEDULE_COLUMNS if column not in df.columns
    ]
    if missing_schedule_columns:
        print(
            "Schedule summary skipped; missing column(s): "
            f"{', '.join(missing_schedule_columns)}"
        )
    else:
        print_summary("Schedule summary:", summarize_schedule(df))

    fig, axes = plt.subplots(4, 2, figsize=(14, 14))
    fig.suptitle(f'PD-DOB Control Results - {csv_path.stem}', fontsize=14, fontweight='bold')

    # 1. 负载侧位置跟踪
    ax = axes[0, 0]
    ax.plot(df['time_s'], df['ref_theta_l_rad'], 'b--', label='Reference', linewidth=2)
    ax.plot(df['time_s'], df['meas_theta_l_rad'], 'r-', label='Measured', linewidth=1.5)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Load Position (rad)')
    ax.set_title('Load Side Position Tracking')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # 2. 负载侧速度跟踪
    ax = axes[0, 1]
    ax.plot(df['time_s'], df['ref_dtheta_l_rad_s'], 'b--', label='Reference', linewidth=2)
    ax.plot(df['time_s'], df['meas_dtheta_l_rad_s'], 'r-', label='Measured', linewidth=1.5)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Load Velocity (rad/s)')
    ax.set_title('Load Side Velocity Tracking')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # 3. 位置跟踪误差
    ax = axes[1, 0]
    ax.plot(df['time_s'], df['err_theta_l_rad'], 'm-', label='Load Position Error', linewidth=1.5)
    ax.axhline(y=0, color='k', linestyle='--', linewidth=0.8)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Position Error (rad)')
    ax.set_title('Load Position Tracking Error')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # 4. 力矩相关量
    ax = axes[1, 1]
    ax.plot(df['time_s'], df['torque_ref_nm'], 'b--', label='Torque Ref', linewidth=2)
    ax.plot(df['time_s'], df['disturbance_hat_nm'], 'g-', label='DOB Disturbance Hat', linewidth=1.5)
    ax.plot(df['time_s'], df['raw_action_nm'], 'r-', label='Raw Action', linewidth=1.5)
    ax.plot(df['time_s'], df['action_nm'], 'k-', label='Command Action', linewidth=1.2, alpha=0.8)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Torque (Nm)')
    ax.set_title('PD-DOB Torque Terms')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # 5. 弹簧估计
    ax = axes[2, 0]
    ax.plot(df['time_s'], df['spring_delta_theta_rad'], 'c-', label='Spring Delta Theta', linewidth=1.5)
    ax.axhline(y=0, color='k', linestyle='--', linewidth=0.8)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Angle (rad)')
    ax.set_title('Spring Angle Estimation')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # 6. 弹簧力矩
    ax = axes[2, 1]
    ax.plot(df['time_s'], df['spring_torque_nm'], 'm-', label='Estimated Spring Torque', linewidth=1.5)
    ax.axhline(y=0, color='k', linestyle='--', linewidth=0.8)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Torque (Nm)')
    ax.set_title('Spring Torque Estimation')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # 7. 误差统计
    ax = axes[3, 0]
    errors = {
        'Load Pos': df['err_theta_l_rad'].abs(),
        'Load Vel': df['err_dtheta_l_rad_s'].abs(),
    }
    stats_text = "PD-DOB Error Statistics (Mean ± Std):\n"
    stats_text += f"Load Pos:  {errors['Load Pos'].mean():.4f} ± {errors['Load Pos'].std():.4f} rad\n"
    stats_text += f"Load Vel:  {errors['Load Vel'].mean():.4f} ± {errors['Load Vel'].std():.4f} rad/s\n"
    stats_text += f"\nMax |Action|: {df['action_nm'].abs().max():.2f} Nm"
    stats_text += f"\nMax |Spring Torque|: {df['spring_torque_nm'].abs().max():.2f} Nm"

    ax.text(
        0.1,
        0.5,
        stats_text,
        transform=ax.transAxes,
        fontsize=10,
        verticalalignment='center',
        bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5),
        family='monospace',
    )
    ax.axis('off')
    ax.set_title('Performance Metrics')

    # 8. 保留一个空白子图，避免 4x2 布局里剩余面板显示多余坐标轴
    axes[3, 1].axis('off')

    plt.tight_layout()

    if output_dir is not None:
        output_dir.mkdir(parents=True, exist_ok=True)
        output_path = output_dir / f"{csv_path.stem}.png"
        plt.savefig(output_path, dpi=300, bbox_inches='tight')
        print(f"Figure saved to: {output_path}")

    plt.show()


def main() -> int:
    parser = argparse.ArgumentParser(description="Plot PD-DOB control results from CSV data")
    parser.add_argument("csv_file", type=str, nargs='?', default=None, help="Path to CSV data file")
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
        # csv_files = [log_dir / "pd_dob_data_20260423_151924.csv"]
        latest_csv = find_latest_csv(log_dir, "pd_dob_data_*.csv")
        if latest_csv is None:
            print(f"Error: No PD-DOB CSV files found in {log_dir}")
            return 1
        csv_path = latest_csv

    if not csv_path.exists():
        print(f"Error: CSV file not found: {csv_path}")
        return 1

    print(f"Using CSV file: {csv_path}")

    output_dir = Path(args.output_dir) if args.output_dir else Path(__file__).resolve().parent / "figures"
    plot_pd_dob_results(csv_path, output_dir)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
