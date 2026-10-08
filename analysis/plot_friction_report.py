#!/usr/bin/env python3
"""
Generate report-ready torque-velocity figures for SEA joint friction tests.
"""
from __future__ import annotations

import argparse
import math
from dataclasses import dataclass
from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd

from plot_common import require_columns


ROOT_DIR = Path(__file__).resolve().parents[1]
LOG_DIR = ROOT_DIR / "logs"
DEFAULT_QUASI_STATIC_CSV = LOG_DIR / "friction_test_20260514_173920.csv"
DEFAULT_FAST_CSV = LOG_DIR / "friction_test_20260514_175739.csv"
DEFAULT_OUTPUT_STEM = LOG_DIR / "friction_report_173920_175739"

REQUIRED_COLUMNS = [
    "stage",
    "direction",
    "time_s",
    "command_torque_nm",
    "feedback_torque_nm",
    "velocity_rad_s",
]


@dataclass(frozen=True)
class DatasetSummary:
    ramp_rate_nm_s: float
    max_abs_velocity_rad_s: float
    max_abs_command_torque_nm: float
    max_abs_measured_torque_nm: float


def load_sweep_data(csv_path: Path) -> pd.DataFrame:
    df = pd.read_csv(csv_path)
    require_columns(df, REQUIRED_COLUMNS, csv_path)
    sweep = df[df["stage"] == "sweep"].copy()
    if sweep.empty:
        raise ValueError(f"{csv_path} contains no sweep samples.")
    for column in ("direction", "time_s", "command_torque_nm", "feedback_torque_nm", "velocity_rad_s"):
        sweep[column] = pd.to_numeric(sweep[column], errors="coerce")
    sweep = sweep.dropna(subset=["direction", "time_s", "command_torque_nm", "feedback_torque_nm", "velocity_rad_s"])
    if sweep.empty:
        raise ValueError(f"{csv_path} contains no valid numeric sweep samples.")
    return sweep


def estimate_ramp_rate_nm_s(sweep: pd.DataFrame) -> float:
    rates: list[float] = []
    for direction in (1, -1):
        segment = sweep[sweep["direction"].astype(int) == direction].sort_values("time_s")
        if len(segment) < 2:
            continue
        dt = float(segment["time_s"].iloc[-1] - segment["time_s"].iloc[0])
        if dt <= 0.0:
            continue
        d_tau = float(segment["command_torque_nm"].iloc[-1] - segment["command_torque_nm"].iloc[0])
        rates.append(abs(d_tau / dt))
    if not rates:
        return float("nan")
    return float(sum(rates) / len(rates))


def summarize_dataset(sweep: pd.DataFrame) -> DatasetSummary:
    return DatasetSummary(
        ramp_rate_nm_s=estimate_ramp_rate_nm_s(sweep),
        max_abs_velocity_rad_s=float(sweep["velocity_rad_s"].abs().max()),
        max_abs_command_torque_nm=float(sweep["command_torque_nm"].abs().max()),
        max_abs_measured_torque_nm=float(sweep["feedback_torque_nm"].abs().max()),
    )


def fit_coulomb_viscous(sweep: pd.DataFrame, min_abs_velocity_rad_s: float = 0.05) -> tuple[float, float, float] | None:
    fit_df = sweep[sweep["velocity_rad_s"].abs() >= min_abs_velocity_rad_s].copy()
    if len(fit_df) < 6:
        return None
    velocities = fit_df["velocity_rad_s"].astype(float).tolist()
    torques = fit_df["feedback_torque_nm"].astype(float).tolist()
    if not any(v > 0.0 for v in velocities) or not any(v < 0.0 for v in velocities):
        return None

    normal = [[0.0 for _ in range(3)] for _ in range(3)]
    rhs = [0.0, 0.0, 0.0]
    for velocity, torque in zip(velocities, torques):
        features = [1.0 if velocity >= 0.0 else -1.0, velocity, 1.0]
        for row_index in range(3):
            rhs[row_index] += features[row_index] * torque
            for col_index in range(3):
                normal[row_index][col_index] += features[row_index] * features[col_index]

    return solve_3x3(normal, rhs)


def solve_3x3(matrix: list[list[float]], vector: list[float]) -> tuple[float, float, float] | None:
    augmented = [row[:] + [value] for row, value in zip(matrix, vector)]
    size = 3
    for column in range(size):
        pivot = max(range(column, size), key=lambda row: abs(augmented[row][column]))
        if abs(augmented[pivot][column]) < 1e-12:
            return None
        augmented[column], augmented[pivot] = augmented[pivot], augmented[column]
        pivot_value = augmented[column][column]
        for item in range(column, size + 1):
            augmented[column][item] /= pivot_value
        for row in range(size):
            if row == column:
                continue
            factor = augmented[row][column]
            for item in range(column, size + 1):
                augmented[row][item] -= factor * augmented[column][item]
    return (augmented[0][size], augmented[1][size], augmented[2][size])


def format_number(value: float, digits: int = 2) -> str:
    if not math.isfinite(value):
        return "n/a"
    return f"{value:.{digits}f}"


def plot_dataset(
    ax,
    sweep: pd.DataFrame,
    title: str,
    summary: DatasetSummary,
    *,
    highlight_near_zero: bool = False,
) -> None:
    colors = {
        1: "#1f77b4",
        -1: "#d62728",
    }
    labels = {
        1: "Positive direction",
        -1: "Negative direction",
    }
    for direction in (1, -1):
        segment = sweep[sweep["direction"].astype(int) == direction]
        ax.scatter(
            segment["velocity_rad_s"],
            segment["feedback_torque_nm"],
            s=8,
            alpha=0.58,
            linewidths=0,
            color=colors[direction],
            label=labels[direction],
        )

    fit = fit_coulomb_viscous(sweep)
    if fit is not None:
        tau_c, viscous, offset = fit
        x_min = float(sweep["velocity_rad_s"].min())
        x_max = float(sweep["velocity_rad_s"].max())
        xs = [x_min + (x_max - x_min) * index / 240.0 for index in range(241)]
        ys = [tau_c * (1.0 if x >= 0.0 else -1.0) + viscous * x + offset for x in xs]
        ax.plot(xs, ys, color="#111111", linewidth=1.8, label="Fitted friction curve")

    if highlight_near_zero:
        ax.axvspan(-0.1, 0.15, color="#777777", alpha=0.13)
        y_top = ax.get_ylim()[1]
        ax.text(
            0.02,
            y_top * 0.45,
            "stick-slip region",
            ha="left",
            va="top",
            fontsize=8.5,
            color="#444444",
        )

    ax.axhline(0.0, color="#333333", linewidth=0.8)
    ax.axvline(0.0, color="#333333", linewidth=0.8)
    ax.set_title(title, fontsize=12.0, fontweight="bold", pad=8)
    ax.set_xlabel("Joint velocity (rad/s)", fontsize=11.0)
    ax.set_ylabel("Joint torque (N·m)", fontsize=11.0)
    ax.grid(True, color="#d9d9d9", linewidth=0.7, alpha=0.8)
    ax.tick_params(axis="both", labelsize=9.5)
    # ax.text(
    #     0.03,
    #     0.96,
    #     (
    #         f"Ramp rate: {format_number(summary.ramp_rate_nm_s)} N·m/s\n"
    #         f"Max |velocity|: {format_number(summary.max_abs_velocity_rad_s)} rad/s\n"
    #         f"Max |command|: {format_number(summary.max_abs_command_torque_nm)} N·m"
    #     ),
    #     transform=ax.transAxes,
    #     ha="left",
    #     va="top",
    #     fontsize=8.7,
    #     bbox={
    #         "boxstyle": "round,pad=0.28",
    #         "facecolor": "white",
    #         "edgecolor": "#bdbdbd",
    #         "alpha": 0.92,
    #     },
    # )
    ax.legend(loc="lower right", frameon=True, framealpha=0.92, edgecolor="#bdbdbd", fontsize=8.8)


def build_report_figure(
    quasi_static_csv: Path,
    fast_csv: Path,
    output_stem: Path,
    *,
    show: bool = False,
) -> None:
    quasi_static = load_sweep_data(quasi_static_csv)
    fast = load_sweep_data(fast_csv)
    quasi_summary = summarize_dataset(quasi_static)
    fast_summary = summarize_dataset(fast)

    plt.rcParams.update(
        {
            "font.family": ["Noto Sans CJK JP", "DejaVu Sans"],
            "axes.unicode_minus": False,
            "axes.spines.top": False,
            "axes.spines.right": False,
            "figure.facecolor": "white",
            "axes.facecolor": "white",
            "savefig.facecolor": "white",
        }
    )
    fig, axes = plt.subplots(1, 2, figsize=(11.2, 4.4), constrained_layout=True)
    plot_dataset(
        axes[0],
        quasi_static,
        "(a) 准静态力矩斜坡",
        quasi_summary,
        highlight_near_zero=True,
    )
    plot_dataset(
        axes[1],
        fast,
        "(b) 快速力矩斜坡",
        fast_summary,
        highlight_near_zero=False,
    )

    output_stem.parent.mkdir(parents=True, exist_ok=True)
    png_path = output_stem.with_suffix(".png")
    pdf_path = output_stem.with_suffix(".pdf")
    svg_path = output_stem.with_suffix(".svg")
    fig.savefig(png_path, dpi=300, bbox_inches="tight")
    fig.savefig(pdf_path, bbox_inches="tight")
    fig.savefig(svg_path, bbox_inches="tight")
    print(f"Saved PNG: {png_path}")
    print(f"Saved PDF: {pdf_path}")
    print(f"Saved SVG: {svg_path}")
    print(
        "Suggested caption:\n"
        "Fig. X. Measured torque-velocity characteristics of the SEA joint under "
        "bidirectional torque-ramp excitation. (a) Quasi-static ramp test showing "
        "the low-velocity stick-slip behavior and the transition from static to "
        "kinetic friction. (b) Fast ramp test with a wider velocity range, where "
        "the measured torque includes both friction and acceleration-dependent "
        "inertial components."
    )
    if show:
        plt.show()
    else:
        plt.close(fig)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Generate report-ready SEA joint friction figures.")
    parser.add_argument("--quasi-static-csv", type=Path, default=DEFAULT_QUASI_STATIC_CSV)
    parser.add_argument("--fast-csv", type=Path, default=DEFAULT_FAST_CSV)
    parser.add_argument("--output-stem", type=Path, default=DEFAULT_OUTPUT_STEM)
    parser.add_argument("--show", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    build_report_figure(
        args.quasi_static_csv,
        args.fast_csv,
        args.output_stem,
        show=args.show,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
