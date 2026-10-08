#!/usr/bin/env python3
"""
LQR 控制结果可视化脚本
读取 CSV 数据并绘制轨迹跟踪、力矩等图表
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
    ("send_dt_s", "Send"),
    ("read_after_dt_s", "Read After"),
    ("diag_dt_s", "Diagnostics"),
    ("buffer_append_dt_s", "Buffer Append"),
    ("cycle_total_dt_s", "Cycle Total"),
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
    "action_nm",
    "torque_nm",
    "encoder1_adjusted_valid",
    "encoder2_adjusted_valid",
    "wake_lag_ms",
    "deadline_slip_ms",
    "sleep_target_ms",
    "sleep_actual_ms",
    "deadline_miss_delta",
    "deadline_miss_count",
    *(column for column, _ in TIMING_COLUMNS),
]


def plot_lqr_results(csv_path: Path, output_dir: Path | None = None) -> None:
    """
    绘制 LQR 控制结果图表

    Args:
        csv_path: CSV 数据文件路径
        output_dir: 图片保存目录，如果为 None 则只显示不保存
    """
    df = pd.read_csv(csv_path)
    require_columns(df, REQUIRED_COLUMNS, csv_path)

    enc1_valid = bool(df['encoder1_adjusted_valid'].astype(bool).all())
    enc2_valid = bool(df['encoder2_adjusted_valid'].astype(bool).all())
    print(f"Encoder adjusted validity: encoder1={enc1_valid} encoder2={enc2_valid}")
    if not enc1_valid or not enc2_valid:
        print("Warning: this CSV contains encoder adjusted position marked as invalid.")
    warn_if_flat_signal(df, 'meas_theta_m_rad', 'Motor-side measured position')
    warn_if_flat_signal(df, 'meas_theta_l_rad', 'Load-side measured position')
    warn_if_flat_signal(df, 'meas_dtheta_m_rad_s', 'Motor-side measured velocity')
    warn_if_flat_signal(df, 'meas_dtheta_l_rad_s', 'Load-side measured velocity')
    print_summary("Timing summary (ms):", summarize_timing(df, TIMING_COLUMNS))
    print_summary("Schedule summary:", summarize_schedule(df))

    # 创建图表
    fig, axes = plt.subplots(3, 2, figsize=(14, 10))
    fig.suptitle(f'LQR Control Results - {csv_path.stem}', fontsize=14, fontweight='bold')

    # 1. 电机侧位置跟踪
    ax = axes[0, 0]
    ax.plot(df['time_s'], df['ref_theta_m_rad'], 'b--', label='Reference', linewidth=2)
    ax.plot(df['time_s'], df['meas_theta_m_rad'], 'r-', label='Measured', linewidth=1.5)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Motor Position (rad)')
    ax.set_title('Motor Side Position Tracking')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # 2. 负载侧位置跟踪
    ax = axes[0, 1]
    ax.plot(df['time_s'], df['ref_theta_l_rad'], 'b--', label='Reference', linewidth=2)
    ax.plot(df['time_s'], df['meas_theta_l_rad'], 'r-', label='Measured', linewidth=1.5)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Load Position (rad)')
    ax.set_title('Load Side Position Tracking')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # 3. 位置跟踪误差
    ax = axes[1, 0]
    ax.plot(df['time_s'], df['err_theta_m_rad'], 'g-', label='Motor Error', linewidth=1.5)
    ax.plot(df['time_s'], df['err_theta_l_rad'], 'm-', label='Load Error', linewidth=1.5)
    ax.axhline(y=0, color='k', linestyle='--', linewidth=0.8)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Position Error (rad)')
    ax.set_title('Position Tracking Error')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # 4. 速度跟踪
    ax = axes[1, 1]
    ax.plot(df['time_s'], df['ref_dtheta_l_rad_s'], 'b--', label='Ref Velocity', linewidth=2)
    ax.plot(df['time_s'], df['meas_dtheta_l_rad_s'], 'r-', label='Meas Velocity', linewidth=1.5)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Load Velocity (rad/s)')
    ax.set_title('Load Side Velocity Tracking')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # 5. 控制力矩
    ax = axes[2, 0]
    ax.plot(df['time_s'], df['action_nm'], 'b-', label='Command Torque', linewidth=1.5)
    ax.plot(df['time_s'], df['torque_nm'], 'r-', label='Measured Torque', linewidth=1, alpha=0.7)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Torque (Nm)')
    ax.set_title('Control Torque')
    ax.legend()
    ax.grid(True, alpha=0.3)

    # 6. 误差统计
    ax = axes[2, 1]
    errors = {
        'Motor Pos': df['err_theta_m_rad'].abs(),
        'Load Pos': df['err_theta_l_rad'].abs(),
        'Motor Vel': df['err_dtheta_m_rad_s'].abs(),
        'Load Vel': df['err_dtheta_l_rad_s'].abs(),
    }

    # 计算统计量
    stats_text = "Tracking Error Statistics (Mean ± Std):\n"
    stats_text += f"Motor Pos:  {errors['Motor Pos'].mean():.4f} ± {errors['Motor Pos'].std():.4f} rad\n"
    stats_text += f"Load Pos:   {errors['Load Pos'].mean():.4f} ± {errors['Load Pos'].std():.4f} rad\n"
    stats_text += f"Motor Vel:  {errors['Motor Vel'].mean():.4f} ± {errors['Motor Vel'].std():.4f} rad/s\n"
    stats_text += f"Load Vel:   {errors['Load Vel'].mean():.4f} ± {errors['Load Vel'].std():.4f} rad/s\n"
    stats_text += f"\nMax Torque: {df['action_nm'].abs().max():.2f} Nm"

    ax.text(0.1, 0.5, stats_text, transform=ax.transAxes,
            fontsize=10, verticalalignment='center',
            bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5),
            family='monospace')
    ax.axis('off')
    ax.set_title('Performance Metrics')

    plt.tight_layout()

    # 保存或显示
    if output_dir is not None:
        output_dir.mkdir(parents=True, exist_ok=True)
        output_path = output_dir / f"{csv_path.stem}.png"
        plt.savefig(output_path, dpi=300, bbox_inches='tight')
        print(f"Figure saved to: {output_path}")

    plt.show()


def main() -> int:
    parser = argparse.ArgumentParser(description="Plot LQR control results from CSV data")
    parser.add_argument("csv_file", type=str, nargs='?', default=None, help="Path to CSV data file")
    parser.add_argument("--output-dir", type=str, default=None,
                        help="Directory to save figures (default: show only)")
    args = parser.parse_args()

    # 如果没有提供命令行参数，就自动选择 logs/ 目录下最新的一份 CSV。
    # 这样在 IDE 里直接运行脚本时，默认画的就是最近一次实验结果。
    if args.csv_file:
        csv_path = Path(args.csv_file)
    else:
        log_dir = Path(__file__).resolve().parents[1] / "logs"
        # csv_files = [log_dir / "lqr_data_20260423_151924.csv"]
        latest_csv = find_latest_csv(log_dir, "lqr_data_*.csv")
        if latest_csv is None:
            print(f"Error: No LQR CSV files found in {log_dir}")
            return 1
        csv_path = latest_csv

    if not csv_path.exists():
        print(f"Error: CSV file not found: {csv_path}")
        return 1

    print(f"Using CSV file: {csv_path}")

    output_dir = Path(args.output_dir) if args.output_dir else None
    plot_lqr_results(csv_path, output_dir)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
