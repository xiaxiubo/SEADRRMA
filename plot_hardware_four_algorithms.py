from __future__ import annotations

from pathlib import Path

import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


ROOT = Path(__file__).resolve().parent
BASELINE_DIR = ROOT / "hardware_baseline_results_20260729"
LQR_DIR = ROOT / "hardware_lqr_tuning_20260729"
OUTPUT_DIR = ROOT / "paper_latex" / "figures"

METHODS = ("PPO", "PD+DOB", "RMA", "LQR")
COLORS = {
    "PPO": "#4C78A8",
    "PD+DOB": "#F58518",
    "RMA": "#54A24B",
    "LQR": "#B279A2",
}
FILES = {
    "low": {
        "PPO": BASELINE_DIR / "J0p05_ppo.csv",
        "PD+DOB": BASELINE_DIR / "J0p05_pd_dob.csv",
        "RMA": BASELINE_DIR / "J0p05_rma_onnx.csv",
        "LQR": LQR_DIR / "C4_J0p05_A0p5_f0p3_10s_200Hz.csv",
    },
    "high": {
        "PPO": BASELINE_DIR / "J0p30_ppo.csv",
        "PD+DOB": BASELINE_DIR / "J0p30_pd_dob.csv",
        "RMA": BASELINE_DIR / "J0p30_rma_onnx.csv",
        "LQR": LQR_DIR / "C4_J0p30_A0p5_f0p3_10s_200Hz.csv",
    },
}
INERTIA_LABELS = {
    "low": r"Low inertia ($J\approx0.05$ kg m$^2$)",
    "high": r"High inertia ($J\approx0.30$ kg m$^2$)",
}


def configure_style() -> None:
    mpl.rcParams.update(
        {
            "font.family": "sans-serif",
            "font.sans-serif": ["Arial", "Helvetica", "DejaVu Sans"],
            "font.size": 8,
            "axes.labelsize": 8,
            "axes.titlesize": 9,
            "xtick.labelsize": 7,
            "ytick.labelsize": 7,
            "legend.fontsize": 7,
            "axes.spines.top": False,
            "axes.spines.right": False,
            "axes.linewidth": 0.8,
            "lines.linewidth": 1.0,
            "legend.frameon": False,
            "svg.fonttype": "none",
            "pdf.fonttype": 42,
        }
    )


def load_data() -> dict[str, dict[str, pd.DataFrame]]:
    data: dict[str, dict[str, pd.DataFrame]] = {}
    for inertia, method_files in FILES.items():
        data[inertia] = {}
        for method, path in method_files.items():
            frame = pd.read_csv(path)
            required = {
                "time_s",
                "ref_theta_l_rad",
                "meas_theta_l_rad",
                "err_theta_l_rad",
                "action_nm",
            }
            missing = required.difference(frame.columns)
            if missing:
                raise ValueError(f"{path} is missing columns: {sorted(missing)}")
            if not np.all(np.diff(frame["time_s"].to_numpy()) > 0.0):
                raise ValueError(f"{path} has non-monotonic timestamps")
            data[inertia][method] = frame
    return data


def calculate_metrics(
    data: dict[str, dict[str, pd.DataFrame]],
) -> pd.DataFrame:
    rows = []
    for inertia, method_data in data.items():
        for method, frame in method_data.items():
            error = frame["err_theta_l_rad"].to_numpy()
            action = frame["action_nm"].to_numpy()
            rows.append(
                {
                    "inertia": inertia,
                    "method": method,
                    "samples": len(frame),
                    "duration_s": float(frame["time_s"].iloc[-1]),
                    "position_rmse_rad": float(np.sqrt(np.mean(error**2))),
                    "position_mae_rad": float(np.mean(np.abs(error))),
                    "peak_abs_error_rad": float(np.max(np.abs(error))),
                    "action_rms_nm": float(np.sqrt(np.mean(action**2))),
                    "peak_abs_action_nm": float(np.max(np.abs(action))),
                }
            )
    return pd.DataFrame(rows)


def save_figure(fig: plt.Figure, stem: str) -> None:
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    common = {"bbox_inches": "tight", "facecolor": "white"}
    fig.savefig(OUTPUT_DIR / f"{stem}.png", dpi=400, **common)
    fig.savefig(OUTPUT_DIR / f"{stem}.tiff", dpi=600, **common)
    fig.savefig(OUTPUT_DIR / f"{stem}.pdf", **common)
    fig.savefig(OUTPUT_DIR / f"{stem}.svg", **common)


def plot_time_series(data: dict[str, dict[str, pd.DataFrame]]) -> None:
    fig, axes = plt.subplots(
        2,
        3,
        figsize=(7.16, 5.15),
        sharex=True,
        gridspec_kw={"wspace": 0.30, "hspace": 0.30},
    )
    panel_labels = ("a", "b", "c", "d", "e", "f")

    for row, inertia in enumerate(("low", "high")):
        reference_frame = data[inertia]["PPO"]
        reference_zero = float(reference_frame["ref_theta_l_rad"].iloc[0])
        axes[row, 0].plot(
            reference_frame["time_s"],
            reference_frame["ref_theta_l_rad"] - reference_zero,
            color="#202020",
            linestyle="--",
            linewidth=1.3,
            label="Reference",
            zorder=5,
        )

        for method in METHODS:
            frame = data[inertia][method]
            method_reference_zero = float(frame["ref_theta_l_rad"].iloc[0])
            relative_position = (
                frame["meas_theta_l_rad"].to_numpy() - method_reference_zero
            )
            axes[row, 0].plot(
                frame["time_s"],
                relative_position,
                color=COLORS[method],
                label=method,
                alpha=0.92,
            )
            axes[row, 1].plot(
                frame["time_s"],
                frame["err_theta_l_rad"],
                color=COLORS[method],
                label=method,
                alpha=0.92,
            )
            axes[row, 2].plot(
                frame["time_s"],
                frame["action_nm"],
                color=COLORS[method],
                label=method,
                alpha=0.85,
                linewidth=0.85,
            )

        axes[row, 0].set_ylabel("Relative position (rad)")
        axes[row, 1].set_ylabel("Position error (rad)")
        axes[row, 2].set_ylabel("Command torque (Nm)")
        axes[row, 0].text(
            0.03,
            0.06,
            INERTIA_LABELS[inertia],
            transform=axes[row, 0].transAxes,
            fontsize=7.5,
            fontweight="bold",
            bbox={
                "facecolor": "white",
                "edgecolor": "#B8B8B8",
                "boxstyle": "round,pad=0.22",
                "linewidth": 0.5,
                "alpha": 0.9,
            },
        )
        axes[row, 0].set_ylim(-0.66, 0.66)
        axes[row, 1].set_ylim(-0.42, 0.42)
        axes[row, 2].set_ylim(-22.0, 22.0)

    titles = ("Position tracking", "Tracking error", "Control action")
    for col, title in enumerate(titles):
        axes[0, col].set_title(title, pad=5)
        axes[1, col].set_xlabel("Time (s)")

    for label, ax in zip(panel_labels, axes.flat):
        ax.text(
            -0.19,
            1.04,
            label,
            transform=ax.transAxes,
            fontsize=9,
            fontweight="bold",
            va="bottom",
        )
        ax.axhline(0.0, color="#B8B8B8", linewidth=0.55, zorder=0)
        ax.grid(axis="both", color="#E8E8E8", linewidth=0.5)
        ax.set_xlim(0.0, 10.0)
        ax.set_xticks(np.arange(0.0, 10.1, 2.0))

    handles, labels = axes[0, 0].get_legend_handles_labels()
    fig.legend(
        handles,
        labels,
        loc="upper center",
        bbox_to_anchor=(0.5, 1.02),
        ncol=5,
        handlelength=2.2,
        columnspacing=1.4,
    )
    save_figure(fig, "hardware_four_algorithms_timeseries")
    plt.close(fig)


def plot_metrics(metrics: pd.DataFrame) -> None:
    fig, axes = plt.subplots(
        1,
        2,
        figsize=(7.16, 2.55),
        gridspec_kw={"wspace": 0.30},
    )
    x = np.arange(len(METHODS), dtype=float)
    width = 0.36

    for offset, inertia, hatch in (
        (-width / 2, "low", ""),
        (width / 2, "high", "///"),
    ):
        subset = metrics.set_index(["inertia", "method"]).loc[inertia]
        rmse = [subset.loc[method, "position_rmse_rad"] for method in METHODS]
        action_rms = [subset.loc[method, "action_rms_nm"] for method in METHODS]
        axes[0].bar(
            x + offset,
            rmse,
            width,
            color=[COLORS[method] for method in METHODS],
            hatch=hatch,
            edgecolor="#404040",
            linewidth=0.45,
            alpha=0.9,
            label=INERTIA_LABELS[inertia],
        )
        axes[1].bar(
            x + offset,
            action_rms,
            width,
            color=[COLORS[method] for method in METHODS],
            hatch=hatch,
            edgecolor="#404040",
            linewidth=0.45,
            alpha=0.9,
            label=INERTIA_LABELS[inertia],
        )

    for label, ax, ylabel in (
        ("a", axes[0], "Position RMSE (rad)"),
        ("b", axes[1], "Command torque RMS (Nm)"),
    ):
        ax.text(
            -0.13,
            1.04,
            label,
            transform=ax.transAxes,
            fontsize=9,
            fontweight="bold",
        )
        ax.set_ylabel(ylabel)
        ax.set_xticks(x, METHODS)
        ax.grid(axis="y", color="#E8E8E8", linewidth=0.5)
        ax.set_axisbelow(True)

    handles = [
        mpl.patches.Patch(
            facecolor="#D0D0D0",
            edgecolor="#404040",
            label=INERTIA_LABELS["low"],
        ),
        mpl.patches.Patch(
            facecolor="#D0D0D0",
            edgecolor="#404040",
            hatch="///",
            label=INERTIA_LABELS["high"],
        ),
    ]
    fig.legend(
        handles=handles,
        loc="upper center",
        bbox_to_anchor=(0.5, 1.04),
        ncol=2,
    )
    save_figure(fig, "hardware_four_algorithms_metrics")
    plt.close(fig)


def main() -> None:
    configure_style()
    data = load_data()
    metrics = calculate_metrics(data)
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    metrics.to_csv(
        OUTPUT_DIR / "hardware_four_algorithms_metrics.csv",
        index=False,
        float_format="%.8f",
    )
    plot_time_series(data)
    plot_metrics(metrics)
    print(metrics.to_string(index=False))


if __name__ == "__main__":
    main()
