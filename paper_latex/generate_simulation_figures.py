"""Generate the simulation figures used by the TRACE manuscript."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np
import torch


ROOT = Path(__file__).resolve().parents[1]
OUT_DIR = Path(__file__).resolve().parent / "figures"
COMPARISON_DIR = ROOT / "algorithm_comparison" / "results" / "paper_zero_phase_final_20260716"
MECHANISM_DATA = (
    ROOT
    / "figures"
    / "attention_online"
    / "paper_zero_phase_0p5sin0p3hz_fc0p2_seed2"
    / "trace_mechanism.npz"
)
GENERALIZATION_DATA = (
    ROOT
    / "figures"
    / "attention_online"
    / "final_dagger_raw_full_20260713_summary.json"
)
FAST_CHECKPOINT = ROOT / "checkpoints_attention_online" / "final_v1" / "fast_attn_v2.pth"

METHODS = [
    ("drrma", "TRACE", "#0072B2"),
    ("rma", "RMA", "#009E73"),
    ("ppo", "PPO", "#D55E00"),
    ("pd-dob", "PD+DOB", "#E69F00"),
    ("lqr", "LQR", "#CC79A7"),
]
EVENTS = (3.0, 7.0)
ATTENTION_CMAP = mpl.colors.LinearSegmentedColormap.from_list(
    "trace_attention",
    ["#ffffff", "#fff1ec", "#ffc9bd", "#ff8a75", "#e53935"],
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=OUT_DIR)
    parser.add_argument("--comparison-dir", type=Path, default=COMPARISON_DIR)
    return parser.parse_args()


def configure_style() -> None:
    mpl.rcParams.update(
        {
            "font.family": "serif",
            "font.serif": ["Times New Roman", "Times", "DejaVu Serif"],
            "font.size": 8.0,
            "axes.labelsize": 8.0,
            "axes.titlesize": 8.5,
            "legend.fontsize": 7.0,
            "xtick.labelsize": 7.0,
            "ytick.labelsize": 7.0,
            "axes.linewidth": 0.7,
            "lines.linewidth": 1.15,
            "savefig.dpi": 400,
            "pdf.fonttype": 42,
            "ps.fonttype": 42,
        }
    )


def panel_label(ax: plt.Axes, label: str) -> None:
    ax.text(
        -0.075,
        1.04,
        label,
        transform=ax.transAxes,
        fontsize=9,
        fontweight="bold",
        va="bottom",
    )


def add_event_lines(ax: plt.Axes) -> None:
    for event in EVENTS:
        ax.axvline(event, color="0.45", linestyle="--", linewidth=0.8, zorder=0)


def save_figure(fig: plt.Figure, stem: str) -> None:
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    fig.savefig(OUT_DIR / f"{stem}.pdf", bbox_inches="tight")
    fig.savefig(OUT_DIR / f"{stem}.png", bbox_inches="tight")
    plt.close(fig)


def generate_main_comparison() -> None:
    logs = {
        key: np.load(COMPARISON_DIR / f"logs_{key}.npz")
        for key, _, _ in METHODS
    }
    trace = logs["drrma"]
    fig = plt.figure(figsize=(7.16, 7.2))
    grid = fig.add_gridspec(
        6,
        1,
        height_ratios=[1.15, 0.9, 0.9, 0.72, 0.12, 0.92],
        hspace=0.12,
    )
    axes = [fig.add_subplot(grid[0])]
    axes.extend(fig.add_subplot(grid[index], sharex=axes[0]) for index in range(1, 4))
    zoom_grid = grid[5].subgridspec(1, 2, wspace=0.25)
    event_axes = [fig.add_subplot(zoom_grid[0]), fig.add_subplot(zoom_grid[1])]

    axes[0].plot(trace["t"], trace["ref"], "k--", linewidth=1.35, label="Reference")
    for key, label, color in METHODS:
        axes[0].plot(logs[key]["t"], logs[key]["load"], color=color, label=label)
    axes[0].set_ylabel("Position (rad)")
    axes[0].legend(
        ncol=6,
        loc="lower center",
        bbox_to_anchor=(0.5, 1.01),
        frameon=False,
        columnspacing=0.9,
    )

    for key, label, color in METHODS:
        axes[1].plot(logs[key]["t"], logs[key]["err"], color=color, label=label)
    axes[1].axhline(0.0, color="0.35", linewidth=0.6)
    axes[1].set_ylabel("Error (rad)")

    for key, label, color in METHODS:
        axes[2].plot(logs[key]["t"], logs[key]["torque"], color=color, label=label)
    axes[2].axhline(0.0, color="0.35", linewidth=0.6)
    axes[2].set_ylabel("Torque (N m)")

    axes[3].step(
        trace["t"], trace["J_true"], where="post", color="0.15", linestyle="--", label="True"
    )
    axes[3].plot(trace["t"], trace["J_hat"], color="#0072B2", label="TRACE estimate")
    axes[3].set_ylabel(r"Inertia (kg m$^2$)")
    axes[3].legend(loc="upper left", frameon=False, ncol=2)
    for ax in axes[:-1]:
        ax.tick_params(axis="x", labelbottom=False)

    for ax, label in zip(axes, ("(a)", "(b)", "(c)", "(d)")):
        add_event_lines(ax)
        ax.axvspan(3.0, 3.5, color="#56B4E9", alpha=0.08, linewidth=0)
        ax.axvspan(7.0, 7.5, color="#E69F00", alpha=0.08, linewidth=0)
        panel_label(ax, label)
        ax.grid(True, color="0.88", linewidth=0.5)
        ax.set_xlim(0, 10)

    event_specs = [
        (3.0, r"Payload release: $0.30\rightarrow0.05$"),
        (7.0, r"Payload attachment: $0.05\rightarrow0.75$"),
    ]
    event_errors = []
    for event, _ in event_specs:
        for key, _, _ in METHODS:
            mask = (logs[key]["t"] >= event) & (logs[key]["t"] <= event + 0.5)
            event_errors.append(logs[key]["err"][mask])
    event_min = min(float(values.min()) for values in event_errors)
    event_max = max(float(values.max()) for values in event_errors)
    event_margin = 0.06 * (event_max - event_min)

    for ax, (event, title), label in zip(
        event_axes, event_specs, ("(e)", "(f)")
    ):
        for key, method_label, color in METHODS:
            mask = (logs[key]["t"] >= event) & (logs[key]["t"] <= event + 0.5)
            ax.plot(
                logs[key]["t"][mask] - event,
                logs[key]["err"][mask],
                color=color,
                label=method_label,
            )
        ax.axhline(0.0, color="0.35", linewidth=0.6)
        ax.set_xlim(0.0, 0.5)
        ax.set_ylim(event_min - event_margin, event_max + event_margin)
        ax.set_title(title, pad=3)
        ax.set_xlabel("Time after switch (s)")
        ax.grid(True, color="0.88", linewidth=0.5)
        trace_mask = (trace["t"] >= event) & (trace["t"] < event + 0.5)
        trace_rmse = float(np.sqrt(np.mean(trace["err"][trace_mask] ** 2)))
        ax.text(
            0.97,
            0.08,
            f"TRACE RMSE = {trace_rmse:.4f} rad",
            transform=ax.transAxes,
            ha="right",
            va="bottom",
            color="#0072B2",
            fontsize=6.5,
        )
        panel_label(ax, label)
    event_axes[0].set_ylabel("Error (rad)")
    save_figure(fig, "fig_sim_main_comparison")


def _plot_attention_window(
    ax: plt.Axes,
    t: np.ndarray,
    attention: np.ndarray,
    start: float,
    stop: float,
    event: float,
    title: str,
    vmax: float,
) -> mpl.image.AxesImage:
    mask = (t >= start) & (t <= stop)
    image = ax.imshow(
        attention[mask],
        origin="lower",
        aspect="auto",
        interpolation="nearest",
        extent=(-0.5, 0.0, float(t[mask][0]), float(t[mask][-1])),
        cmap=ATTENTION_CMAP,
        vmin=0.0,
        vmax=vmax,
    )
    ax.axhline(event, color="#8b1a1a", linestyle="--", linewidth=0.9)
    ax.set_title(title)
    ax.set_xlabel("History lag (s)")
    ax.set_ylabel("Time (s)")
    return image


def compute_branch_estimates(observations: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    if str(ROOT) not in sys.path:
        sys.path.insert(0, str(ROOT))
    from attention_fast_v2 import ImprovedAttentionFastBranch

    histories = np.empty((len(observations), 100, observations.shape[1]), dtype=np.float32)
    buffer = np.repeat(observations[:1].astype(np.float32), 100, axis=0)
    for index, observation in enumerate(observations.astype(np.float32)):
        buffer[:-1] = buffer[1:]
        buffer[-1] = observation
        histories[index] = buffer

    # Freeze the paper's high-friction simulation plant independently of the
    # deployment-oriented hardware profile (which uses Ks=2400 N m/rad).
    model = ImprovedAttentionFastBranch(
        hist_len=100,
        control_dt=0.005,
        spring_stiffness=3800.0,
    )
    model.load_state_dict(torch.load(FAST_CHECKPOINT, map_location="cpu"))
    model.eval()
    short_values: list[np.ndarray] = []
    long_values: list[np.ndarray] = []
    with torch.no_grad():
        for start in range(0, len(histories), 256):
            history = torch.from_numpy(histories[start : start + 256])
            _, aux = model.forward_with_aux(history, need_weights=False)
            short_values.append(aux["short_estimate"].cpu().numpy().reshape(-1))
            long_values.append(aux["long_estimate"].cpu().numpy().reshape(-1))
    return np.concatenate(short_values), np.concatenate(long_values)


def generate_adaptation_figures() -> None:
    data = np.load(MECHANISM_DATA)
    t = data["t"]
    attention = data["mixture"]
    vmax = float(np.quantile(attention, 0.995))
    short_estimate, long_estimate = compute_branch_estimates(data["obs"])

    fig, (ax_inertia, ax_gate) = plt.subplots(
        2,
        1,
        figsize=(3.45, 3.55),
        sharex=True,
        gridspec_kw={"hspace": 0.22},
    )

    ax_inertia.step(t, data["J_true"], where="post", color="0.15", linestyle="--", label="True")
    ax_inertia.plot(t, short_estimate, color="#D55E00", linestyle=":", linewidth=0.9, label="Short")
    ax_inertia.plot(t, long_estimate, color="#009E73", linestyle="-.", linewidth=0.9, label="Long")
    ax_inertia.plot(t, data["J_hat"], color="#0072B2", linewidth=1.2, label="Fused")
    ax_inertia.set_ylabel(r"Inertia (kg m$^2$)")
    ax_inertia.legend(frameon=False, loc="upper left", ncol=2, columnspacing=0.8)
    ax_inertia.grid(True, color="0.88", linewidth=0.5)
    add_event_lines(ax_inertia)

    ax_gate.plot(t, data["fast_jump_gate"], color="#D55E00", label="Jump gate")
    ax_gate.plot(t, data["fast_confidence"], color="#009E73", label="Confidence")
    ax_gate.set_xlabel("Time (s)")
    ax_gate.set_ylabel("Activation")
    ax_gate.set_ylim(-0.03, 1.03)
    ax_gate.legend(frameon=False, loc="lower right", ncol=2)
    ax_gate.grid(True, color="0.88", linewidth=0.5)
    add_event_lines(ax_gate)

    for ax, label in zip((ax_inertia, ax_gate), ("(a)", "(b)")):
        panel_label(ax, label)
    save_figure(fig, "fig_sim_adaptation_estimates")

    fig, (ax_release, ax_attach) = plt.subplots(
        2,
        1,
        figsize=(3.45, 3.45),
        gridspec_kw={"hspace": 0.48},
    )
    image = _plot_attention_window(
        ax_release, t, attention, 2.6, 3.6, 3.0, "Payload release", vmax
    )
    _plot_attention_window(
        ax_attach, t, attention, 6.6, 7.6, 7.0, "Payload attachment", vmax
    )
    colorbar = fig.colorbar(image, ax=[ax_release, ax_attach], fraction=0.04, pad=0.035)
    colorbar.set_label("Attention weight")
    for ax, label in zip((ax_release, ax_attach), ("(a)", "(b)")):
        panel_label(ax, label)
    save_figure(fig, "fig_sim_attention_windows")


def _group_matrix(rows: list[dict], key: str, cases: list[str], fc_values: list[float]) -> np.ndarray:
    matrix = np.full((len(cases), len(fc_values)), np.nan, dtype=float)
    for i, case in enumerate(cases):
        for j, fc_load in enumerate(fc_values):
            values = [
                float(row[key])
                for row in rows
                if row["case"] == case and abs(float(row["fc_load"]) - fc_load) < 1e-9
            ]
            matrix[i, j] = float(np.mean(values))
    return matrix


def _annotated_heatmap(
    ax: plt.Axes,
    matrix: np.ndarray,
    xlabels: list[str],
    ylabels: list[str],
    title: str,
    vmax: float,
) -> mpl.image.AxesImage:
    image = ax.imshow(matrix, aspect="auto", cmap="YlGnBu", vmin=0.0, vmax=vmax)
    ax.set_xticks(range(len(xlabels)), xlabels)
    ax.set_yticks(range(len(ylabels)), ylabels)
    ax.set_xlabel(r"$F_{c,l}$ (N m)")
    ax.set_title(title)
    threshold = 0.58 * vmax
    for i in range(matrix.shape[0]):
        for j in range(matrix.shape[1]):
            color = "white" if matrix[i, j] > threshold else "black"
            ax.text(j, i, f"{matrix[i, j]:.3f}", ha="center", va="center", fontsize=5.3, color=color)
    return image


def generate_generalization_figures() -> None:
    payload = json.loads(GENERALIZATION_DATA.read_text(encoding="utf-8"))
    rows = payload["rows"]
    cases = ["main", "low1", "low2", "nearby", "heldout_multi", "heldout_step"]
    case_labels = ["Main", "Low amp.", "Low freq.", "Nearby", "Multi-sine", "Step"]
    fc_values = [0.0, 0.1, 0.2, 0.3, 0.4, 0.5]
    fc_labels = [f"{value:.1f}" for value in fc_values]
    inertia = _group_matrix(rows, "stable_mean_rmse", cases, fc_values)
    tracking = _group_matrix(rows, "tracking_rmse", cases, fc_values)

    fig, ax = plt.subplots(figsize=(3.45, 2.35))
    im1 = _annotated_heatmap(
        ax,
        inertia,
        fc_labels,
        case_labels,
        r"Stable-segment inertia RMSE (kg m$^2$)",
        0.07,
    )
    colorbar = fig.colorbar(im1, ax=ax, fraction=0.046, pad=0.03)
    colorbar.set_label(r"RMSE (kg m$^2$)")
    save_figure(fig, "fig_sim_generalization_inertia")

    fig, ax = plt.subplots(figsize=(3.45, 2.35))
    im2 = _annotated_heatmap(
        ax, tracking, fc_labels, case_labels, "Closed-loop tracking RMSE (rad)", 0.08
    )
    colorbar = fig.colorbar(im2, ax=ax, fraction=0.046, pad=0.03)
    colorbar.set_label("RMSE (rad)")
    save_figure(fig, "fig_sim_generalization_tracking")


def main() -> int:
    global OUT_DIR, COMPARISON_DIR
    args = parse_args()
    OUT_DIR = args.output_dir.resolve()
    COMPARISON_DIR = args.comparison_dir.resolve()
    configure_style()
    generate_main_comparison()
    generate_adaptation_figures()
    generate_generalization_figures()
    print(f"Saved manuscript simulation figures to {OUT_DIR}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
