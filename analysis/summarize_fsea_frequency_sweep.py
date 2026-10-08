#!/usr/bin/env python3
"""
Summarize single-sine FSEA torque-excitation sweep logs.

The script reads files such as load_sine_15Hz_amp30.csv, estimates the complex
FRF at each named sine frequency, and writes a sweep-level CSV, Markdown report,
and aligned Bode-style figure.
"""
from __future__ import annotations

import argparse
import re
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from matplotlib.gridspec import GridSpec
from scipy import signal

from plot_common import require_columns


REQUIRED_COLUMNS = [
    "time_s",
    "action_nm",
    "spring_torque_nm",
]


FREQUENCY_RE = re.compile(r"_sine_([0-9]+(?:[._][0-9]+)?)Hz", re.IGNORECASE)


def parse_frequency_from_name(path: Path) -> float:
    match = FREQUENCY_RE.search(path.stem)
    if not match:
        raise ValueError(
            f"Cannot parse sine frequency from {path.name}; expected name like load_sine_15Hz_amp30.csv"
        )
    return float(match.group(1).replace("_", "."))


def complex_sine_fit(t: np.ndarray, z: np.ndarray, freq_hz: float) -> tuple[complex, float, float]:
    omega = 2.0 * np.pi * freq_hz
    design = np.column_stack([np.cos(omega * t), np.sin(omega * t), np.ones_like(t)])
    coeff, *_ = np.linalg.lstsq(design, z, rcond=None)
    cos_coeff, sin_coeff, bias = coeff
    complex_peak = cos_coeff - 1j * sin_coeff
    residual = z - design @ coeff
    residual_rms = float(np.sqrt(np.mean(residual**2)))
    return complex_peak, float(bias), residual_rms


def theory_transmissibility(freq_hz: np.ndarray, Jm: float, Jl: float, Bm: float, Bl: float, Ks: float) -> np.ndarray:
    s = 1j * 2.0 * np.pi * np.asarray(freq_hz, dtype=float)
    den = (
        Jl * Jm * s**3
        + (Bl * Jm + Bm * Jl) * s**2
        + (Jl * Ks + Jm * Ks + Bl * Bm) * s
        + Ks * (Bl + Bm)
    )
    num = Ks * Jl * s + Ks * Bl
    return num / den


def effective_load_inertia(base_jl: float, load_count: int, load_mass_each_kg: float, load_radius_m: float) -> float:
    return base_jl + load_count * load_mass_each_kg * load_radius_m**2


def analyze_file(
    csv_path: Path,
    freq_hz: float,
    input_col: str,
    output_col: str,
    invert_output: bool,
    skip_seconds: float,
) -> dict[str, object]:
    df = pd.read_csv(csv_path)
    require_columns(df, [*REQUIRED_COLUMNS, input_col, output_col], csv_path)

    t = df["time_s"].to_numpy(dtype=float)
    keep = t >= (t[0] + skip_seconds)
    if keep.sum() < 16:
        raise ValueError(f"{csv_path} has too few samples after --skip-seconds={skip_seconds}")
    t_fit = t[keep] - t[keep][0]
    input_data = df[input_col].to_numpy(dtype=float)[keep]
    output_data = df[output_col].to_numpy(dtype=float)[keep]
    if invert_output:
        output_data = -output_data

    input_peak, input_bias, input_residual_rms = complex_sine_fit(t_fit, input_data, freq_hz)
    output_peak, output_bias, output_residual_rms = complex_sine_fit(t_fit, output_data, freq_hz)
    frf = output_peak / input_peak if abs(input_peak) > 1e-12 else np.nan + 1j * np.nan

    dt = float(np.median(np.diff(t)))
    fs = 1.0 / dt
    nperseg = min(
        len(input_data),
        max(256, int(round(fs * min(10.0, max(4.0 / max(freq_hz, 1e-9), 2.0))))),
    )
    coh_freq, coh = signal.coherence(
        signal.detrend(input_data),
        signal.detrend(output_data),
        fs=fs,
        nperseg=nperseg,
    )
    coh_idx = int(np.argmin(np.abs(coh_freq - freq_hz)))

    result: dict[str, object] = {
        "csv": csv_path.name,
        "frequency_hz": freq_hz,
        "samples": int(len(df)),
        "duration_s": float(t[-1] - t[0]),
        "input_col": input_col,
        "output_col": output_col,
        "invert_output": bool(invert_output),
        "input_peak_fit_nm": float(abs(input_peak)),
        "input_bias_nm": input_bias,
        "input_residual_rms_nm": input_residual_rms,
        "output_peak_fit_nm": float(abs(output_peak)),
        "output_bias_nm": output_bias,
        "output_residual_rms_nm": output_residual_rms,
        "output_rms_nm": float(np.sqrt(np.mean(output_data**2))),
        "output_max_abs_nm": float(np.max(np.abs(output_data))),
        "frf_mag": float(abs(frf)),
        "frf_mag_db": float(20.0 * np.log10(max(abs(frf), 1e-300))),
        "frf_phase_wrapped_deg": float(np.angle(frf, deg=True)),
        "coherence": float(coh[coh_idx]),
        "coherence_bin_hz": float(coh_freq[coh_idx]),
    }
    if "torque_nm" in df.columns:
        drive_data = df["torque_nm"].to_numpy(dtype=float)[keep]
        drive_peak, _, _ = complex_sine_fit(t_fit, drive_data, freq_hz)
        drive_frf = drive_peak / input_peak if abs(input_peak) > 1e-12 else np.nan + 1j * np.nan
        result["drive_follow_mag"] = float(abs(drive_frf))
        result["drive_follow_phase_deg"] = float(np.angle(drive_frf, deg=True))
    if "spring_delta_theta_rad" in df.columns:
        spring_defl = df["spring_delta_theta_rad"].to_numpy(dtype=float)[keep]
        if invert_output:
            spring_defl = -spring_defl
        result["spring_defl_max_abs_rad"] = float(np.max(np.abs(spring_defl)))
    if {"wkc", "expected_wkc"}.issubset(df.columns):
        result["wkc_all_ok"] = bool((df["wkc"] == df["expected_wkc"]).all())
    if "third_encoder_crc_ok" in df.columns:
        result["third_crc_all_ok"] = bool(df["third_encoder_crc_ok"].astype(bool).all())
    if {"third_encoder_enc_err", "third_encoder_comm_alarm"}.issubset(df.columns):
        result["third_error_any"] = bool(
            df["third_encoder_enc_err"].astype(bool).any()
            or df["third_encoder_comm_alarm"].astype(bool).any()
        )
    return result


def phase_unwrap_deg(phase_deg: np.ndarray) -> np.ndarray:
    unwrapped = np.rad2deg(np.unwrap(np.deg2rad(phase_deg)))
    if len(unwrapped):
        unwrapped += 360.0 * round((phase_deg[0] - unwrapped[0]) / 360.0)
    return unwrapped


def write_outputs(
    result: pd.DataFrame,
    out_dir: Path,
    args: argparse.Namespace,
) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)

    Jl_eff = effective_load_inertia(args.Jl, args.load_count, args.load_mass_each_kg, args.load_radius_m)
    result = result.sort_values("frequency_hz").reset_index(drop=True)
    result["frf_phase_unwrapped_deg"] = phase_unwrap_deg(result["frf_phase_wrapped_deg"].to_numpy(dtype=float))

    theory_at_points = theory_transmissibility(
        result["frequency_hz"].to_numpy(dtype=float),
        args.Jm,
        Jl_eff,
        args.Bm,
        args.Bl,
        args.Ks,
    )
    result["theory_T_mag"] = np.abs(theory_at_points)
    result["theory_T_mag_db"] = 20.0 * np.log10(np.maximum(result["theory_T_mag"].to_numpy(dtype=float), 1e-300))
    result["theory_T_phase_deg"] = np.angle(theory_at_points, deg=True)

    response_csv = out_dir / "frequency_sweep_response.csv"
    result.to_csv(response_csv, index=False)

    freq_min = max(0.05, float(result["frequency_hz"].min()) * 0.7)
    freq_max = max(float(result["frequency_hz"].max()) * 1.8, 60.0)
    fgrid = np.logspace(np.log10(freq_min), np.log10(freq_max), 1400)
    theory = theory_transmissibility(fgrid, args.Jm, Jl_eff, args.Bm, args.Bl, args.Ks)
    anti_hz = np.sqrt(args.Ks / args.Jm) / (2.0 * np.pi)
    res_hz = np.sqrt(args.Ks * (args.Jm + Jl_eff) / (args.Jm * Jl_eff)) / (2.0 * np.pi)

    fig = plt.figure(figsize=(10.5, 10.5), constrained_layout=True)
    grid = GridSpec(3, 2, figure=fig, width_ratios=[1.0, 0.035], height_ratios=[1, 1, 1])
    ax_mag = fig.add_subplot(grid[0, 0])
    ax_phase = fig.add_subplot(grid[1, 0], sharex=ax_mag)
    ax_aux = fig.add_subplot(grid[2, 0], sharex=ax_mag)
    cax = fig.add_subplot(grid[0, 1])

    scatter = ax_mag.scatter(
        result["frequency_hz"],
        result["frf_mag_db"],
        c=result["coherence"],
        cmap="viridis",
        vmin=0.0,
        vmax=1.0,
        s=62,
        edgecolor="black",
        label="Measured sine sweep",
        zorder=3,
    )
    ax_mag.semilogx(
        fgrid,
        20.0 * np.log10(np.maximum(np.abs(theory), 1e-300)),
        label="Theory T loaded",
        color="tab:blue",
    )
    ax_mag.axvline(anti_hz, color="tab:orange", linestyle="--", linewidth=1.2, label=f"anti-res {anti_hz:.2f} Hz")
    ax_mag.axvline(res_hz, color="tab:red", linestyle="--", linewidth=1.2, label=f"res {res_hz:.2f} Hz")
    ax_mag.set_ylabel(f"|{args.output_col}/{args.input_col}| [dB]")
    ax_mag.grid(True, which="both", alpha=0.35)
    ax_mag.legend(loc="best")
    colorbar = fig.colorbar(scatter, cax=cax)
    colorbar.set_label("coherence")

    ax_phase.semilogx(fgrid, np.rad2deg(np.unwrap(np.angle(theory))), color="tab:blue", label="Theory phase")
    ax_phase.scatter(
        result["frequency_hz"],
        result["frf_phase_unwrapped_deg"],
        c=result["coherence"],
        cmap="viridis",
        vmin=0.0,
        vmax=1.0,
        s=62,
        edgecolor="black",
        label="Measured unwrapped",
        zorder=3,
    )
    ax_phase.set_ylabel("phase [deg]")
    ax_phase.grid(True, which="both", alpha=0.35)
    ax_phase.legend(loc="best")

    ax_aux.semilogx(result["frequency_hz"], result["coherence"], marker="o", color="tab:green", label="coherence")
    ax_aux.axhline(args.coherence_threshold, color="tab:gray", linestyle="--", linewidth=1, label=f"{args.coherence_threshold:g} threshold")
    ax_aux2 = ax_aux.twinx()
    ax_aux2.semilogx(
        result["frequency_hz"],
        result["input_peak_fit_nm"],
        marker="s",
        color="tab:purple",
        label="input peak fit",
    )
    if args.static_friction_nm is not None:
        ax_aux2.axhline(args.static_friction_nm, color="tab:red", linestyle=":", linewidth=1.2, label=f"static friction {args.static_friction_nm:g} Nm")
    ax_aux.set_xlabel("frequency [Hz]")
    ax_aux.set_ylabel("coherence")
    ax_aux2.set_ylabel("input peak [Nm]")
    ax_aux.set_ylim(0.0, 1.05)
    ax_aux.grid(True, which="both", alpha=0.35)
    lines, labels = ax_aux.get_legend_handles_labels()
    lines2, labels2 = ax_aux2.get_legend_handles_labels()
    ax_aux.legend(lines + lines2, labels + labels2, loc="best")

    for axis in (ax_mag, ax_phase, ax_aux):
        axis.set_xlim(freq_min, freq_max)
        axis.label_outer()

    sign_note = "output inverted" if args.invert_output else "output sign as recorded"
    fig.suptitle(f"FSEA loaded joint sine-sweep response ({sign_note})")
    figure_path = out_dir / "frequency_sweep_response.png"
    fig.savefig(figure_path, dpi=220, bbox_inches="tight")
    plt.close(fig)

    lines = [
        "# FSEA loaded joint sine-sweep summary",
        "",
        f"- CSV files analyzed: {len(result)}",
        f"- Input/output: {args.input_col} -> {args.output_col}",
        f"- Output sign correction: {'enabled' if args.invert_output else 'disabled'}",
        f"- Effective load inertia: Jl_eff = {Jl_eff:.6g} kg*m^2 (base {args.Jl:.6g} + added {Jl_eff - args.Jl:.6g})",
        f"- Anti-resonance estimate: {anti_hz:.3f} Hz",
        f"- Resonance estimate: {res_hz:.3f} Hz",
    ]
    if args.static_friction_nm is not None:
        lines.append(f"- Static friction assumption: {args.static_friction_nm:.3g} Nm")
    lines.extend(
        [
            "",
            "## Measured Points",
            "",
            "| f [Hz] | |T| [dB] | phase unwrapped [deg] | coherence | output peak [Nm] | input peak [Nm] |",
            "|---:|---:|---:|---:|---:|---:|",
        ]
    )
    for _, row in result.iterrows():
        lines.append(
            f"| {row['frequency_hz']:.3f} | {row['frf_mag_db']:.2f} | "
            f"{row['frf_phase_unwrapped_deg']:.1f} | {row['coherence']:.3f} | "
            f"{row['output_peak_fit_nm']:.3f} | {row['input_peak_fit_nm']:.3f} |"
        )
    lines.extend(
        [
            "",
            "## Interpretation Notes",
            "- Interpret only points with sufficient input amplitude and high coherence.",
            "- This sweep estimates tau_out/tau_m for the selected input/output columns; it does not measure S(s) or C(s) unless tau_ext is recorded and selected.",
            "- If these CSVs were recorded before the sweep-script spring sign correction, use --invert-output.",
        ]
    )
    summary_path = out_dir / "frequency_sweep_summary.md"
    summary_path.write_text("\n".join(lines), encoding="utf-8")

    print(f"Wrote {response_csv}")
    print(f"Wrote {figure_path}")
    print(f"Wrote {summary_path}")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Summarize FSEA single-sine sweep CSV files.")
    parser.add_argument(
        "--csv-dir",
        type=Path,
        default=Path("ETHERCAT_SEABOX/temp/fsea_experiments/test/plant_excitation"),
    )
    parser.add_argument("--pattern", default="load_sine_*Hz_amp*.csv")
    parser.add_argument("--out-dir", type=Path, default=None)
    parser.add_argument("--input-col", default="action_nm")
    parser.add_argument("--output-col", default="spring_torque_nm")
    parser.add_argument("--invert-output", action="store_true")
    parser.add_argument("--skip-seconds", type=float, default=2.0)
    parser.add_argument("--coherence-threshold", type=float, default=0.8)
    parser.add_argument("--Jm", type=float, default=0.417)
    parser.add_argument("--Jl", type=float, default=0.021)
    parser.add_argument("--Bm", type=float, default=0.0955)
    parser.add_argument("--Bl", type=float, default=0.0)
    parser.add_argument("--Ks", type=float, default=3800.0)
    parser.add_argument("--load-mass-each-kg", type=float, default=1.0)
    parser.add_argument("--load-count", type=int, default=2)
    parser.add_argument("--load-radius-m", type=float, default=0.13)
    parser.add_argument("--static-friction-nm", type=float, default=8.0)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    csv_files = sorted(args.csv_dir.glob(args.pattern), key=parse_frequency_from_name)
    if not csv_files:
        raise FileNotFoundError(f"No CSV files matched {args.csv_dir / args.pattern}")
    out_dir = args.out_dir or (args.csv_dir / "frequency_sweep_summary")
    rows = [
        analyze_file(
            csv_path=csv_path,
            freq_hz=parse_frequency_from_name(csv_path),
            input_col=args.input_col,
            output_col=args.output_col,
            invert_output=args.invert_output,
            skip_seconds=args.skip_seconds,
        )
        for csv_path in csv_files
    ]
    write_outputs(pd.DataFrame(rows), out_dir, args)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
