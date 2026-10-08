#!/usr/bin/env python3
"""
实物实验数据导入和绘图脚本
Hardware Experiment Data Import and Plotting Script

使用方法 / Usage:
1. 将实验数据保存为 CSV 格式，包含以下列：
   - time: 时间戳 (s)
   - theta_l_ref: 参考位置 (rad)
   - theta_l: 实际位置 (rad)
   - theta_m: 电机位置 (rad)
   - torque: 电机力矩 (N*m)
   - inertia: 负载惯量 (kg*m^2)

2. 运行脚本：
   python hardware_experiment_plot.py --data your_data.csv --output results/

3. 生成的图表将保存在指定的输出目录
"""

import argparse
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path

def load_experiment_data(csv_path):
    """加载实验数据"""
    df = pd.read_csv(csv_path)

    required_columns = ['time', 'theta_l_ref', 'theta_l', 'theta_m', 'torque', 'inertia']
    missing = [col for col in required_columns if col not in df.columns]
    if missing:
        raise ValueError(f"Missing required columns: {missing}")

    return df

def calculate_metrics(df):
    """计算性能指标"""
    error = df['theta_l'] - df['theta_l_ref']

    metrics = {
        'rmse': np.sqrt(np.mean(error**2)),
        'mae': np.mean(np.abs(error)),
        'max_error': np.max(np.abs(error)),
        'torque_rms': np.sqrt(np.mean(df['torque']**2)),
        'saturation_rate': 100 * np.sum(np.abs(df['torque']) > 60.0) / len(df)
    }

    return metrics

def plot_tracking_performance(df, output_dir, title="Hardware Experiment"):
    """绘制跟踪性能图"""
    fig, axes = plt.subplots(4, 1, figsize=(12, 10), sharex=True)

    # 位置跟踪
    axes[0].plot(df['time'], df['theta_l_ref'], 'k--', label='Reference', linewidth=2)
    axes[0].plot(df['time'], df['theta_l'], 'b-', label='Actual', linewidth=1.5)
    axes[0].set_ylabel('Position (rad)', fontsize=11)
    axes[0].legend(loc='upper right')
    axes[0].grid(True, alpha=0.3)
    axes[0].set_title(title, fontsize=13, fontweight='bold')

    # 跟踪误差
    error = df['theta_l'] - df['theta_l_ref']
    axes[1].plot(df['time'], error, 'r-', linewidth=1.5)
    axes[1].axhline(y=0, color='k', linestyle='--', alpha=0.5)
    axes[1].set_ylabel('Tracking Error (rad)', fontsize=11)
    axes[1].grid(True, alpha=0.3)

    # 力矩
    axes[2].plot(df['time'], df['torque'], 'g-', linewidth=1.5)
    axes[2].axhline(y=61, color='r', linestyle='--', alpha=0.5, label='Limit')
    axes[2].axhline(y=-61, color='r', linestyle='--', alpha=0.5)
    axes[2].set_ylabel('Torque (N·m)', fontsize=11)
    axes[2].legend(loc='upper right')
    axes[2].grid(True, alpha=0.3)

    # 惯量
    axes[3].plot(df['time'], df['inertia'], 'm-', linewidth=2)
    axes[3].set_ylabel('Inertia (kg·m²)', fontsize=11)
    axes[3].set_xlabel('Time (s)', fontsize=11)
    axes[3].grid(True, alpha=0.3)

    plt.tight_layout()
    output_path = Path(output_dir) / 'hardware_tracking.png'
    plt.savefig(output_path, dpi=300, bbox_inches='tight')
    print(f"Saved: {output_path}")
    plt.close()

def plot_comparison(data_dict, output_dir):
    """对比多个算法的实验结果"""
    fig, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)

    colors = {
        'DRRMA': '#1f77b4',
        'PPO': '#ff7f0e',
        'RMA': '#2ca02c',
        'PD-DOB': '#d62728',
        'LQR': '#9467bd'
    }

    for name, df in data_dict.items():
        color = colors.get(name, 'gray')

        # 位置跟踪
        if name == list(data_dict.keys())[0]:
            axes[0].plot(df['time'], df['theta_l_ref'], 'k--',
                        label='Reference', linewidth=2, alpha=0.7)
        axes[0].plot(df['time'], df['theta_l'], color=color,
                    label=name, linewidth=1.5, alpha=0.8)

        # 跟踪误差
        error = df['theta_l'] - df['theta_l_ref']
        axes[1].plot(df['time'], error, color=color,
                    label=name, linewidth=1.5, alpha=0.8)

        # 力矩
        axes[2].plot(df['time'], df['torque'], color=color,
                    label=name, linewidth=1.5, alpha=0.8)

    axes[0].set_ylabel('Position (rad)', fontsize=11)
    axes[0].legend(loc='upper right', ncol=3)
    axes[0].grid(True, alpha=0.3)
    axes[0].set_title('Hardware Experiment: Algorithm Comparison',
                     fontsize=13, fontweight='bold')

    axes[1].axhline(y=0, color='k', linestyle='--', alpha=0.5)
    axes[1].set_ylabel('Tracking Error (rad)', fontsize=11)
    axes[1].grid(True, alpha=0.3)

    axes[2].axhline(y=61, color='r', linestyle='--', alpha=0.3)
    axes[2].axhline(y=-61, color='r', linestyle='--', alpha=0.3)
    axes[2].set_ylabel('Torque (N·m)', fontsize=11)
    axes[2].set_xlabel('Time (s)', fontsize=11)
    axes[2].grid(True, alpha=0.3)

    plt.tight_layout()
    output_path = Path(output_dir) / 'hardware_comparison.png'
    plt.savefig(output_path, dpi=300, bbox_inches='tight')
    print(f"Saved: {output_path}")
    plt.close()

def generate_metrics_table(metrics_dict, output_dir):
    """生成性能指标表格"""
    df = pd.DataFrame(metrics_dict).T
    df = df.round(4)

    # 保存为 CSV
    csv_path = Path(output_dir) / 'hardware_metrics.csv'
    df.to_csv(csv_path)
    print(f"Saved: {csv_path}")

    # 打印表格
    print("\n" + "="*60)
    print("Hardware Experiment Metrics")
    print("="*60)
    print(df.to_string())
    print("="*60)

    return df

def main():
    parser = argparse.ArgumentParser(description='Hardware experiment data plotting')
    parser.add_argument('--data', type=str, required=True,
                       help='Path to experiment data CSV file')
    parser.add_argument('--output', type=str, default='hardware_results',
                       help='Output directory for plots')
    parser.add_argument('--compare', type=str, nargs='+',
                       help='Additional CSV files for comparison')
    parser.add_argument('--title', type=str, default='Hardware Experiment',
                       help='Plot title')

    args = parser.parse_args()

    # 创建输出目录
    output_dir = Path(args.output)
    output_dir.mkdir(parents=True, exist_ok=True)

    # 加载主数据
    print(f"Loading data from: {args.data}")
    df = load_experiment_data(args.data)

    # 计算指标
    metrics = calculate_metrics(df)
    print(f"\nMetrics for {args.data}:")
    for key, value in metrics.items():
        print(f"  {key}: {value:.4f}")

    # 绘制单个实验结果
    plot_tracking_performance(df, output_dir, args.title)

    # 如果有对比数据
    if args.compare:
        data_dict = {Path(args.data).stem: df}
        metrics_dict = {Path(args.data).stem: metrics}

        for compare_path in args.compare:
            print(f"\nLoading comparison data: {compare_path}")
            df_compare = load_experiment_data(compare_path)
            metrics_compare = calculate_metrics(df_compare)

            name = Path(compare_path).stem
            data_dict[name] = df_compare
            metrics_dict[name] = metrics_compare

            print(f"Metrics for {name}:")
            for key, value in metrics_compare.items():
                print(f"  {key}: {value:.4f}")

        # 绘制对比图
        plot_comparison(data_dict, output_dir)

        # 生成指标表格
        generate_metrics_table(metrics_dict, output_dir)

    print(f"\nAll plots saved to: {output_dir}")

if __name__ == '__main__':
    main()
