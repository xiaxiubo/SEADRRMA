#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import math
import os
import statistics
import sys
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

try:
    import project_bootstrap  # noqa: F401
except ModuleNotFoundError:
    ROOT_DIR = Path(__file__).resolve().parents[1]
    ROOT_STR = str(ROOT_DIR)
    if ROOT_STR not in sys.path:
        sys.path.insert(0, ROOT_STR)


DEFAULT_IFNAME = "eno1"
DEFAULT_CYCLE_TIME_S = 0.002
DEFAULT_SWEEP_TORQUE_NM = 50.0
DEFAULT_TORQUE_RAMP_RATE_NM_S = 20.0
DEFAULT_REPEATS = 1
DEFAULT_MAX_ABS_VELOCITY_RAD_S = 4.5
DEFAULT_MAX_POSITION_DELTA_RAD = 1.5
DEFAULT_BREAKAWAY_VELOCITY_RAD_S = 0.03
DEFAULT_DYNAMIC_MIN_VELOCITY_RAD_S = 0.05
DEFAULT_MAX_TORQUE_NM = 61.0
DEFAULT_SETTLE_DURATION_S = 0.5
DEFAULT_DIRECTION_DURATION_S = 3.0
CST_MODE = 10
ENABLE_OPERATION = 0x000F
SHUTDOWN = 0x0006
DriveCommand = None
SEARealtimeComm = None
summarize_wkc = None


CSV_FIELDNAMES = [
    "cycle_index",
    "time_s",
    "stage",
    "direction",
    "command_torque_nm",
    "target_torque",
    "feedback_torque_nm",
    "velocity_rad_s",
    "position_rad",
    "encoder1_rad",
    "encoder2_rad",
    "accel_rad_s2",
    "wkc",
    "statusword",
    "mode_display",
    "stop_reason",
]


@dataclass(frozen=True)
class FrictionAnalysis:
    positive_breakaway_nm: float | None
    negative_breakaway_nm: float | None
    positive_dynamic_median_nm: float | None
    negative_dynamic_median_nm: float | None
    dynamic_asymmetry_nm: float | None
    dynamic_asymmetry_percent: float | None
    fit_tau_c_nm: float | None
    fit_viscous_nm_per_rad_s: float | None
    fit_offset_nm: float | None
    fit_sample_count: int


def torque_nm_to_target_units(torque_nm: float, max_torque_nm: float) -> int:
    if max_torque_nm <= 0.0:
        raise ValueError("max_torque_nm must be positive.")
    target = int(round(torque_nm / max_torque_nm * 1000.0))
    return max(min(target, 1000), -1000)


def resolve_default_csv_path() -> Path:
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return Path(__file__).resolve().parents[1] / "logs" / f"friction_test_{timestamp}.csv"


def resolve_output_path(path_text: str | None, default_path: Path) -> Path:
    if path_text is None:
        return default_path
    path = Path(path_text).expanduser()
    if not path.is_absolute():
        path = Path.cwd() / path
    return path


def print_slave_inventory(comm: SEARealtimeComm) -> None:
    print(f"EtherCAT network on {comm.ifname}: expected_wkc={comm.expected_wkc}", flush=True)
    for item in comm.describe_slaves():
        print(
            "  slave[{idx}] name={name!r} state={state} in={inp} out={out}".format(
                idx=item.index,
                name=item.name,
                state=item.state_label,
                inp=item.input_size,
                out=item.output_size,
            ),
            flush=True,
        )
    print("  Runtime RxPDO command objects:", flush=True)
    for row in comm.describe_required_rxpdo():
        print(f"    {row}", flush=True)


def print_preheat_summary(label: str, result) -> None:
    diag = result.last_diagnostics
    wkc_summary = summarize_wkc(result.observed_wkc) if summarize_wkc is not None else "n/a"
    print(
        (
            f"{label}: success={result.success} expected_wkc={result.expected_wkc} "
            f"observed_wkc={wkc_summary} "
            f"stable={result.consecutive_stable_cycles}/{result.stable_cycles_required} "
            f"statusword=0x{diag.statusword:04X} mode={diag.mode_display} cia402={diag.cia402_state}"
        ),
        flush=True,
    )
    if not result.success and result.samples:
        print(result.sample_summary(), flush=True)


def enable_drive(comm: SEARealtimeComm) -> None:
    print("Stage 1: PDO preheat", flush=True)
    preheat = comm.preheat_pdo(
        DriveCommand(controlword=SHUTDOWN, mode_of_operation=CST_MODE, target_torque=0),
        cycles=400,
        stable_cycles=10,
        require_statusword_nonzero=True,
    )
    print_preheat_summary("PDO preheat", preheat)
    if not preheat.success:
        raise RuntimeError("PDO preheat failed before CiA 402 enable")

    print("Stage 2: CiA 402 enable", flush=True)
    transitions = comm.enable_cia402(
        mode_of_operation=CST_MODE,
        target_torque=0,
        timeout_cycles_per_step=300,
    )
    for result in transitions:
        diag = result.last_diagnostics
        wkc_summary = summarize_wkc(result.observed_wkc) if summarize_wkc is not None else "n/a"
        print(
            (
                f"  step={result.step_name} success={result.success} expected_state={result.expected_state} "
                f"wkc={wkc_summary} statusword=0x{diag.statusword:04X} "
                f"mode={diag.mode_display} cia402={diag.cia402_state}"
            ),
            flush=True,
        )
        if not result.success:
            if result.samples:
                print(result.sample_summary(), flush=True)
            raise RuntimeError(f"CiA 402 transition failed at {result.step_name}")

    print("Stage 3: enabled-state stabilization", flush=True)
    stabilized = comm.stabilize_enabled_state(
        mode_of_operation=CST_MODE,
        target_torque=0,
        cycles=20,
        stable_cycles=5,
    )
    print_preheat_summary("Enabled-state stabilization", stabilized)
    if not stabilized.success:
        raise RuntimeError("Drive failed to remain in Operation enabled state")


def zero_torque_cycles(comm: SEARealtimeComm, cycles: int = 20) -> None:
    command = DriveCommand(
        controlword=ENABLE_OPERATION,
        mode_of_operation=CST_MODE,
        target_torque=0,
    )
    for _ in range(max(cycles, 1)):
        comm.cycle(command)


def should_stop_for_safety(
    velocity_rad_s: float,
    position_rad: float,
    initial_position_rad: float,
    max_abs_velocity_rad_s: float,
    max_position_delta_rad: float,
) -> str | None:
    if abs(velocity_rad_s) >= max_abs_velocity_rad_s:
        return "velocity_limit"
    if abs(position_rad - initial_position_rad) >= max_position_delta_rad:
        return "position_delta_limit"
    return None


def append_sample(
    rows: list[dict[str, object]],
    *,
    cycle_index: int,
    start_time_s: float,
    stage: str,
    direction: int,
    command_torque_nm: float,
    target_torque: int,
    wkc: int,
    state,
    previous_velocity_rad_s: float | None,
    previous_time_s: float | None,
    stop_reason: str = "",
) -> tuple[float, float]:
    now_s = time.monotonic()
    elapsed_s = now_s - start_time_s
    if previous_velocity_rad_s is None or previous_time_s is None or now_s <= previous_time_s:
        accel_rad_s2 = 0.0
    else:
        accel_rad_s2 = (state.velocity_rad_s - previous_velocity_rad_s) / (now_s - previous_time_s)
    rows.append(
        {
            "cycle_index": cycle_index,
            "time_s": f"{elapsed_s:.9f}",
            "stage": stage,
            "direction": direction,
            "command_torque_nm": f"{command_torque_nm:.9f}",
            "target_torque": target_torque,
            "feedback_torque_nm": f"{state.torque_nm:.9f}",
            "velocity_rad_s": f"{state.velocity_rad_s:.9f}",
            "position_rad": f"{state.position_rad:.9f}",
            "encoder1_rad": f"{state.encoder1_rad:.9f}",
            "encoder2_rad": f"{state.encoder2_rad:.9f}",
            "accel_rad_s2": f"{accel_rad_s2:.9f}",
            "wkc": wkc,
            "statusword": f"0x{state.statusword:04X}",
            "mode_display": state.mode_display,
            "stop_reason": stop_reason,
        }
    )
    return state.velocity_rad_s, now_s


def run_settle_phase(
    comm: SEARealtimeComm,
    rows: list[dict[str, object]],
    *,
    cycle_index: int,
    start_time_s: float,
    duration_s: float,
    max_torque_nm: float,
) -> int:
    stop_time = time.monotonic() + max(duration_s, 0.0)
    prev_velocity: float | None = None
    prev_time: float | None = None
    target_torque = torque_nm_to_target_units(0.0, max_torque_nm)
    command = DriveCommand(
        controlword=ENABLE_OPERATION,
        mode_of_operation=CST_MODE,
        target_torque=target_torque,
    )
    while time.monotonic() < stop_time:
        wkc = comm.cycle(command)
        state = comm.read_state_si()
        prev_velocity, prev_time = append_sample(
            rows,
            cycle_index=cycle_index,
            start_time_s=start_time_s,
            stage="settle",
            direction=0,
            command_torque_nm=0.0,
            target_torque=target_torque,
            wkc=wkc,
            state=state,
            previous_velocity_rad_s=prev_velocity,
            previous_time_s=prev_time,
        )
        cycle_index += 1
    return cycle_index


def run_direction_sweep(
    comm: SEARealtimeComm,
    rows: list[dict[str, object]],
    *,
    cycle_index: int,
    start_time_s: float,
    direction: int,
    execute: bool,
    max_torque_nm: float,
    sweep_torque_nm: float,
    torque_ramp_rate_nm_s: float,
    duration_s: float,
    max_abs_velocity_rad_s: float,
    max_position_delta_rad: float,
    print_every: int,
) -> tuple[int, str]:
    initial_position_rad = comm.read_state_si().position_rad
    direction_start_s = time.monotonic()
    prev_velocity: float | None = None
    prev_time: float | None = None
    stop_reason = "duration_complete"
    local_cycle = 0
    max_abs_velocity_seen = 0.0
    max_abs_position_delta_seen = 0.0
    last_command_torque_nm = 0.0

    while True:
        elapsed_direction_s = time.monotonic() - direction_start_s
        if elapsed_direction_s >= duration_s:
            break

        requested_torque_nm = min(
            abs(sweep_torque_nm),
            max(torque_ramp_rate_nm_s, 1e-9) * elapsed_direction_s,
        )
        command_torque_nm = float(direction) * requested_torque_nm if execute else 0.0
        last_command_torque_nm = command_torque_nm
        target_torque = torque_nm_to_target_units(command_torque_nm, max_torque_nm)
        command = DriveCommand(
            controlword=ENABLE_OPERATION,
            mode_of_operation=CST_MODE,
            target_torque=target_torque,
        )
        wkc = comm.cycle(command)
        state = comm.read_state_si()
        position_delta_rad = state.position_rad - initial_position_rad
        max_abs_velocity_seen = max(max_abs_velocity_seen, abs(state.velocity_rad_s))
        max_abs_position_delta_seen = max(max_abs_position_delta_seen, abs(position_delta_rad))

        safety_reason = should_stop_for_safety(
            state.velocity_rad_s,
            state.position_rad,
            initial_position_rad,
            max_abs_velocity_rad_s,
            max_position_delta_rad,
        )
        if safety_reason is not None:
            stop_reason = safety_reason

        prev_velocity, prev_time = append_sample(
            rows,
            cycle_index=cycle_index,
            start_time_s=start_time_s,
            stage="sweep",
            direction=direction,
            command_torque_nm=command_torque_nm,
            target_torque=target_torque,
            wkc=wkc,
            state=state,
            previous_velocity_rad_s=prev_velocity,
            previous_time_s=prev_time,
            stop_reason=stop_reason if safety_reason is not None else "",
        )

        if local_cycle % max(print_every, 1) == 0:
            print(
                (
                    f"sweep dir={direction:+d} cycle={local_cycle} wkc={wkc} "
                    f"cmd={command_torque_nm:+.3f}Nm target={target_torque:+d} "
                    f"vel={state.velocity_rad_s:+.4f}rad/s pos_delta={position_delta_rad:+.4f}rad "
                    f"fb_torque={state.torque_nm:+.3f}Nm"
                ),
                flush=True,
            )

        cycle_index += 1
        local_cycle += 1
        if safety_reason is not None:
            break

    print(
        (
            f"sweep dir={direction:+d} summary stop_reason={stop_reason} "
            f"last_cmd={last_command_torque_nm:+.3f}Nm "
            f"max_abs_vel={max_abs_velocity_seen:.4f}rad/s "
            f"max_abs_pos_delta={max_abs_position_delta_seen:.4f}rad"
        ),
        flush=True,
    )
    return cycle_index, stop_reason


def float_from_row(row: dict[str, object], key: str) -> float:
    return float(str(row[key]))


def signed_rows(rows: list[dict[str, object]], direction: int) -> list[dict[str, object]]:
    return [row for row in rows if row["stage"] == "sweep" and int(row["direction"]) == direction]


def estimate_breakaway(rows: list[dict[str, object]], direction: int, threshold_rad_s: float) -> float | None:
    for row in signed_rows(rows, direction):
        velocity = float_from_row(row, "velocity_rad_s")
        if direction * velocity >= threshold_rad_s:
            return abs(float_from_row(row, "command_torque_nm"))
    return None


def estimate_dynamic_median(rows: list[dict[str, object]], direction: int, min_velocity_rad_s: float) -> float | None:
    samples = []
    for row in signed_rows(rows, direction):
        velocity = float_from_row(row, "velocity_rad_s")
        if direction * velocity >= min_velocity_rad_s:
            samples.append(abs(float_from_row(row, "feedback_torque_nm")))
    if not samples:
        return None
    return statistics.median(samples)


def solve_3x3(matrix: list[list[float]], vector: list[float]) -> list[float] | None:
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
    return [augmented[row][size] for row in range(size)]


def fit_friction_model(rows: list[dict[str, object]], min_velocity_rad_s: float) -> tuple[float | None, float | None, float | None, int]:
    samples: list[tuple[float, float]] = []
    for row in rows:
        if row["stage"] != "sweep":
            continue
        velocity = float_from_row(row, "velocity_rad_s")
        if abs(velocity) < min_velocity_rad_s:
            continue
        torque = float_from_row(row, "feedback_torque_nm")
        samples.append((velocity, torque))

    if len(samples) < 6 or not any(v > 0.0 for v, _ in samples) or not any(v < 0.0 for v, _ in samples):
        return None, None, None, len(samples)

    normal = [[0.0 for _ in range(3)] for _ in range(3)]
    rhs = [0.0, 0.0, 0.0]
    for velocity, torque in samples:
        features = [1.0 if velocity >= 0.0 else -1.0, velocity, 1.0]
        for row_index in range(3):
            rhs[row_index] += features[row_index] * torque
            for column_index in range(3):
                normal[row_index][column_index] += features[row_index] * features[column_index]

    solution = solve_3x3(normal, rhs)
    if solution is None:
        return None, None, None, len(samples)
    tau_c, viscous, offset = solution
    return tau_c, viscous, offset, len(samples)


def analyze_rows(
    rows: list[dict[str, object]],
    *,
    breakaway_velocity_rad_s: float,
    dynamic_min_velocity_rad_s: float,
) -> FrictionAnalysis:
    positive_breakaway = estimate_breakaway(rows, 1, breakaway_velocity_rad_s)
    negative_breakaway = estimate_breakaway(rows, -1, breakaway_velocity_rad_s)
    positive_dynamic = estimate_dynamic_median(rows, 1, dynamic_min_velocity_rad_s)
    negative_dynamic = estimate_dynamic_median(rows, -1, dynamic_min_velocity_rad_s)

    asymmetry = None
    asymmetry_percent = None
    if positive_dynamic is not None and negative_dynamic is not None:
        asymmetry = positive_dynamic - negative_dynamic
        mean_dynamic = (positive_dynamic + negative_dynamic) / 2.0
        if mean_dynamic > 1e-9:
            asymmetry_percent = asymmetry / mean_dynamic * 100.0

    fit_tau_c, fit_viscous, fit_offset, fit_count = fit_friction_model(rows, dynamic_min_velocity_rad_s)
    return FrictionAnalysis(
        positive_breakaway_nm=positive_breakaway,
        negative_breakaway_nm=negative_breakaway,
        positive_dynamic_median_nm=positive_dynamic,
        negative_dynamic_median_nm=negative_dynamic,
        dynamic_asymmetry_nm=asymmetry,
        dynamic_asymmetry_percent=asymmetry_percent,
        fit_tau_c_nm=fit_tau_c,
        fit_viscous_nm_per_rad_s=fit_viscous,
        fit_offset_nm=fit_offset,
        fit_sample_count=fit_count,
    )


def fmt_optional(value: float | None, suffix: str = "") -> str:
    if value is None or not math.isfinite(value):
        return "n/a"
    return f"{value:.4f}{suffix}"


def print_analysis(analysis: FrictionAnalysis) -> None:
    print("Friction analysis summary:", flush=True)
    print(f"  positive breakaway torque: {fmt_optional(analysis.positive_breakaway_nm, ' Nm')}", flush=True)
    print(f"  negative breakaway torque: {fmt_optional(analysis.negative_breakaway_nm, ' Nm')}", flush=True)
    print(f"  positive dynamic median:   {fmt_optional(analysis.positive_dynamic_median_nm, ' Nm')}", flush=True)
    print(f"  negative dynamic median:   {fmt_optional(analysis.negative_dynamic_median_nm, ' Nm')}", flush=True)
    print(
        (
            f"  dynamic asymmetry:         {fmt_optional(analysis.dynamic_asymmetry_nm, ' Nm')} "
            f"({fmt_optional(analysis.dynamic_asymmetry_percent, ' %')})"
        ),
        flush=True,
    )
    print(
        (
            "  fit torque = tau_c*sign(v) + b*v + offset: "
            f"tau_c={fmt_optional(analysis.fit_tau_c_nm, ' Nm')}, "
            f"b={fmt_optional(analysis.fit_viscous_nm_per_rad_s, ' Nm/(rad/s)')}, "
            f"offset={fmt_optional(analysis.fit_offset_nm, ' Nm')}, "
            f"samples={analysis.fit_sample_count}"
        ),
        flush=True,
    )


def write_csv(csv_path: Path, rows: list[dict[str, object]]) -> None:
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    try:
        os.chmod(csv_path.parent, 0o777)
    except OSError:
        pass
    with csv_path.open("w", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=CSV_FIELDNAMES)
        writer.writeheader()
        writer.writerows(rows)
    try:
        os.chmod(csv_path, 0o666)
    except OSError:
        pass


def plot_results(
    png_path: Path,
    rows: list[dict[str, object]],
    analysis: FrictionAnalysis,
    dynamic_min_velocity_rad_s: float,
) -> bool:
    try:
        import matplotlib.pyplot as plt
    except Exception as exc:
        print(f"Warning: matplotlib is unavailable; skip PNG generation: {exc}", flush=True)
        return False

    positive_points = [
        (float_from_row(row, "velocity_rad_s"), float_from_row(row, "feedback_torque_nm"))
        for row in signed_rows(rows, 1)
    ]
    negative_points = [
        (float_from_row(row, "velocity_rad_s"), float_from_row(row, "feedback_torque_nm"))
        for row in signed_rows(rows, -1)
    ]

    png_path.parent.mkdir(parents=True, exist_ok=True)
    fig, ax = plt.subplots(figsize=(7.0, 5.0))
    if positive_points:
        ax.scatter(
            [item[0] for item in positive_points],
            [item[1] for item in positive_points],
            s=8,
            alpha=0.65,
            label="positive sweep",
        )
    if negative_points:
        ax.scatter(
            [item[0] for item in negative_points],
            [item[1] for item in negative_points],
            s=8,
            alpha=0.65,
            label="negative sweep",
        )

    if (
        analysis.fit_tau_c_nm is not None
        and analysis.fit_viscous_nm_per_rad_s is not None
        and analysis.fit_offset_nm is not None
    ):
        velocities = [
            float_from_row(row, "velocity_rad_s")
            for row in rows
            if row["stage"] == "sweep" and abs(float_from_row(row, "velocity_rad_s")) >= dynamic_min_velocity_rad_s
        ]
        if velocities:
            v_min = min(velocities)
            v_max = max(velocities)
            fit_x = [v_min + (v_max - v_min) * index / 199.0 for index in range(200)]
            fit_y = [
                analysis.fit_tau_c_nm * (1.0 if velocity >= 0.0 else -1.0)
                + analysis.fit_viscous_nm_per_rad_s * velocity
                + analysis.fit_offset_nm
                for velocity in fit_x
            ]
            ax.plot(fit_x, fit_y, color="red", linewidth=2.0, label="fit")

    ax.axhline(0.0, color="black", linewidth=0.8)
    ax.axvline(0.0, color="black", linewidth=0.8)
    ax.set_xlabel("Velocity (rad/s)")
    ax.set_ylabel("Torque (Nm)")
    ax.set_title("SEA Joint Friction Test")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(png_path, dpi=300, bbox_inches="tight")
    plt.close(fig)
    try:
        os.chmod(png_path, 0o666)
    except OSError:
        pass
    return True


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Conservative SEA joint friction test in CST torque mode.")
    parser.add_argument("--ifname", default=DEFAULT_IFNAME)
    parser.add_argument("--cycle-time", type=float, default=DEFAULT_CYCLE_TIME_S)
    parser.add_argument("--sweep-torque-nm", type=float, default=DEFAULT_SWEEP_TORQUE_NM)
    parser.add_argument("--torque-ramp-rate", type=float, default=DEFAULT_TORQUE_RAMP_RATE_NM_S)
    parser.add_argument("--repeats", type=int, default=DEFAULT_REPEATS)
    parser.add_argument("--duration", type=float, default=DEFAULT_DIRECTION_DURATION_S, help="Max duration per direction.")
    parser.add_argument("--settle-duration", type=float, default=DEFAULT_SETTLE_DURATION_S)
    parser.add_argument("--max-abs-velocity", type=float, default=DEFAULT_MAX_ABS_VELOCITY_RAD_S)
    parser.add_argument("--max-position-delta", type=float, default=DEFAULT_MAX_POSITION_DELTA_RAD)
    parser.add_argument("--breakaway-velocity", type=float, default=DEFAULT_BREAKAWAY_VELOCITY_RAD_S)
    parser.add_argument("--dynamic-min-velocity", type=float, default=DEFAULT_DYNAMIC_MIN_VELOCITY_RAD_S)
    parser.add_argument("--max-torque-nm", type=float, default=DEFAULT_MAX_TORQUE_NM)
    parser.add_argument("--print-every", type=int, default=100)
    parser.add_argument("--csv-path", type=str, default=None)
    parser.add_argument("--png-path", type=str, default=None)
    parser.add_argument(
        "--no-execute",
        action="store_true",
        help="Keep commanded torque at zero while still exercising startup, logging, and analysis paths.",
    )
    return parser.parse_args()


def load_runtime_imports() -> None:
    global DriveCommand, SEARealtimeComm, summarize_wkc
    from communication.sea_motor_comm import (  # noqa: PLC0415
        DriveCommand as _DriveCommand,
        SEARealtimeComm as _SEARealtimeComm,
        summarize_wkc as _summarize_wkc,
    )

    DriveCommand = _DriveCommand
    SEARealtimeComm = _SEARealtimeComm
    summarize_wkc = _summarize_wkc


def main() -> int:
    args = parse_args()
    load_runtime_imports()
    csv_path = resolve_output_path(args.csv_path, resolve_default_csv_path())
    default_png_path = csv_path.with_suffix(".png")
    png_path = resolve_output_path(args.png_path, default_png_path)
    rows: list[dict[str, object]] = []
    comm = SEARealtimeComm(ifname=args.ifname, cycle_time_s=args.cycle_time)
    exit_code = 0

    print(
        (
            f"Friction test config: ifname={args.ifname} cycle={args.cycle_time*1000.0:.3f}ms "
            f"execute={not args.no_execute} sweep_torque={args.sweep_torque_nm:.3f}Nm "
            f"ramp={args.torque_ramp_rate:.3f}Nm/s repeats={args.repeats} "
            f"velocity_limit={args.max_abs_velocity:.3f}rad/s position_delta_limit={args.max_position_delta:.3f}rad"
        ),
        flush=True,
    )

    try:
        comm.connect()
        print_slave_inventory(comm)
        enable_drive(comm)
        print("Stage 4: friction sweep", flush=True)

        start_time_s = time.monotonic()
        cycle_index = 0
        for repeat_index in range(max(args.repeats, 1)):
            print(f"Repeat {repeat_index + 1}/{max(args.repeats, 1)}: positive direction", flush=True)
            cycle_index = run_settle_phase(
                comm,
                rows,
                cycle_index=cycle_index,
                start_time_s=start_time_s,
                duration_s=args.settle_duration,
                max_torque_nm=args.max_torque_nm,
            )
            cycle_index, positive_stop = run_direction_sweep(
                comm,
                rows,
                cycle_index=cycle_index,
                start_time_s=start_time_s,
                direction=1,
                execute=not args.no_execute,
                max_torque_nm=args.max_torque_nm,
                sweep_torque_nm=args.sweep_torque_nm,
                torque_ramp_rate_nm_s=args.torque_ramp_rate,
                duration_s=args.duration,
                max_abs_velocity_rad_s=args.max_abs_velocity,
                max_position_delta_rad=args.max_position_delta,
                print_every=args.print_every,
            )
            print(f"Positive sweep stop_reason={positive_stop}", flush=True)
            zero_torque_cycles(comm, cycles=20)

            print(f"Repeat {repeat_index + 1}/{max(args.repeats, 1)}: negative direction", flush=True)
            cycle_index = run_settle_phase(
                comm,
                rows,
                cycle_index=cycle_index,
                start_time_s=start_time_s,
                duration_s=args.settle_duration,
                max_torque_nm=args.max_torque_nm,
            )
            cycle_index, negative_stop = run_direction_sweep(
                comm,
                rows,
                cycle_index=cycle_index,
                start_time_s=start_time_s,
                direction=-1,
                execute=not args.no_execute,
                max_torque_nm=args.max_torque_nm,
                sweep_torque_nm=args.sweep_torque_nm,
                torque_ramp_rate_nm_s=args.torque_ramp_rate,
                duration_s=args.duration,
                max_abs_velocity_rad_s=args.max_abs_velocity,
                max_position_delta_rad=args.max_position_delta,
                print_every=args.print_every,
            )
            print(f"Negative sweep stop_reason={negative_stop}", flush=True)
            zero_torque_cycles(comm, cycles=20)

        print("Stage 5: zero torque shutdown frames", flush=True)
        zero_torque_cycles(comm, cycles=50)
    except KeyboardInterrupt:
        exit_code = 130
        print("Interrupted by user; saving collected samples after zero-torque cleanup.", flush=True)
    except Exception as exc:
        exit_code = 1
        print(f"Friction test failed: {type(exc).__name__}: {exc}", flush=True)
    finally:
        try:
            zero_torque_cycles(comm, cycles=10)
        except Exception:
            pass
        comm.close()

    if rows:
        print(f"Writing CSV to: {csv_path}", flush=True)
        write_csv(csv_path, rows)
        analysis = analyze_rows(
            rows,
            breakaway_velocity_rad_s=args.breakaway_velocity,
            dynamic_min_velocity_rad_s=args.dynamic_min_velocity,
        )
        print_analysis(analysis)

        if plot_results(png_path, rows, analysis, args.dynamic_min_velocity):
            print(f"Figure saved to: {png_path}", flush=True)
        print(f"Data saved to: {csv_path}", flush=True)
    else:
        print("No samples collected; CSV/PNG not written.", flush=True)
    print("Friction test done." if exit_code == 0 else "Friction test ended with errors.", flush=True)
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
