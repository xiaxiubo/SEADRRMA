#!/usr/bin/env python3
from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
from datetime import datetime
from pathlib import Path


REPO_DIR = Path(__file__).resolve().parents[2]
PROJECT_DIR = Path(__file__).resolve().parents[1]
LOG_DIR = PROJECT_DIR / "logs"
TEMP_DIR = PROJECT_DIR / "temp"
ANALYSIS_DIR = PROJECT_DIR / "analysis"


def latest_csv(pattern: str) -> Path | None:
    csv_files = sorted(LOG_DIR.glob(pattern), key=lambda path: path.stat().st_mtime, reverse=True)
    return csv_files[0] if csv_files else None


def run_command(command: list[str], *, cwd: Path = REPO_DIR) -> None:
    print("+ " + " ".join(str(part) for part in command), flush=True)
    env = os.environ.copy()
    env.setdefault("MPLBACKEND", "Agg")
    subprocess.run(command, cwd=cwd, check=True, env=env)


def copy_csv_to_dir(csv_path: Path, out_dir: Path) -> Path:
    out_dir.mkdir(parents=True, exist_ok=True)
    target = out_dir / csv_path.name
    if csv_path.resolve() != target.resolve():
        shutil.copy2(csv_path, target)
    return target


def analyze_frf(csv_path: Path, out_dir: Path, input_col: str, output_col: str) -> None:
    run_command(
        [
            sys.executable,
            str(TEMP_DIR / "fsea_joint_frequency_analysis.py"),
            "--csv",
            str(csv_path),
            "--input-col",
            input_col,
            "--output-col",
            output_col,
            "--out-dir",
            str(out_dir),
        ]
    )


def plot_control_result(kind: str, csv_path: Path, out_dir: Path) -> None:
    script_by_kind = {
        "pd_dob": ANALYSIS_DIR / "plot_pd_dob_results.py",
        "lqr": ANALYSIS_DIR / "plot_lqr_results.py",
        "ppo": ANALYSIS_DIR / "plot_ppo_results.py",
    }
    run_command(
        [
            sys.executable,
            str(script_by_kind[kind]),
            str(csv_path),
            "--output-dir",
            str(out_dir),
        ]
    )


def plot_plant_excitation(csv_path: Path, out_dir: Path) -> None:
    run_command(
        [
            sys.executable,
            str(ANALYSIS_DIR / "plot_fsea_excitation_results.py"),
            str(csv_path),
            "--output-dir",
            str(out_dir),
        ]
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Create a timestamped FSEA experiment batch directory.")
    parser.add_argument(
        "--batch-dir",
        type=Path,
        default=None,
        help="Output batch directory. Default: temp/fsea_experiments/YYYYMMDD_HHMMSS",
    )
    parser.add_argument("--run-plant", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--analyze-latest-controllers", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--ifname", default="eno1")
    parser.add_argument("--duration", type=float, default=20.0)
    parser.add_argument("--cycle-time", type=float, default=0.005)
    parser.add_argument("--mode", choices=("multisine", "chirp"), default="multisine")
    parser.add_argument("--amplitude-nm", type=float, default=5.0)
    parser.add_argument("--bias-nm", type=float, default=0.0)
    parser.add_argument("--max-command-nm", type=float, default=61.0)
    parser.add_argument("--f-min", type=float, default=0.2)
    parser.add_argument("--f-max", type=float, default=14.0)
    parser.add_argument("--components", default="0.4,0.8,1.5,3,6,10,14")
    parser.add_argument("--execute", action=argparse.BooleanOptionalAction, default=False)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    batch_dir = args.batch_dir or (TEMP_DIR / "fsea_experiments" / timestamp)
    batch_dir.mkdir(parents=True, exist_ok=True)
    print(f"Batch directory: {batch_dir}", flush=True)

    if args.run_plant:
        plant_dir = batch_dir / "plant_excitation"
        plant_csv = plant_dir / f"fsea_excitation_{timestamp}.csv"
        plant_dir.mkdir(parents=True, exist_ok=True)
        run_command(
            [
                sys.executable,
                str(PROJECT_DIR / "scripts" / "fsea_torque_excitation.py"),
                "--ifname",
                args.ifname,
                "--duration",
                str(args.duration),
                "--cycle-time",
                str(args.cycle_time),
                "--mode",
                args.mode,
                "--amplitude-nm",
                str(args.amplitude_nm),
                "--bias-nm",
                str(args.bias_nm),
                "--max-command-nm",
                str(args.max_command_nm),
                "--f-min",
                str(args.f_min),
                "--f-max",
                str(args.f_max),
                "--components",
                args.components,
                "--csv-path",
                str(plant_csv),
                "--execute" if args.execute else "--no-execute",
            ]
        )
        plot_plant_excitation(plant_csv, plant_dir / "time_domain")
        analyze_frf(plant_csv, plant_dir / "frequency", "action_nm", "spring_torque_nm")

    if args.analyze_latest_controllers:
        controller_specs = [
            ("pd_dob", "pd_dob_data_*.csv", "action_nm", "spring_torque_nm"),
            ("lqr", "lqr_data_*.csv", "action_nm", "spring_torque_nm"),
            ("ppo", "ppo_data_*.csv", "action_nm", "spring_torque_nm"),
        ]
        for kind, pattern, input_col, output_col in controller_specs:
            source = latest_csv(pattern)
            out_dir = batch_dir / kind
            if source is None:
                print(f"Skipping {kind}: no CSV matching {pattern} in {LOG_DIR}", flush=True)
                continue
            copied = copy_csv_to_dir(source, out_dir)
            plot_control_result(kind, copied, out_dir / "time_domain")
            analyze_frf(copied, out_dir / "closed_loop_frequency", input_col, output_col)

    print(f"Batch complete: {batch_dir}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
