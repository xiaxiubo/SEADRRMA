#!/usr/bin/env python3
"""Collect SEA spring calibration data while the motor brake remains locked."""

from __future__ import annotations

import argparse
import csv
import statistics
import sys
import time
from pathlib import Path

if __package__ in {None, ""}:
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from SEA_friction_identification import (
    CYCLE_TIME_S,
    NSEC_PER_SEC,
    SPRING_STIFFNESS_NM_PER_RAD,
    THIRD_ENCODER_COUNTS_PER_REV,
    THIRD_ENCODER_SIGN,
    configure_realtime_runtime,
    monotonic_time_ns,
    normalize_cycle_time_ns,
    print_preheat_summary,
    print_slave_inventory,
    sleep_until_monotonic_ns,
)
from communication.sea_motor_comm import (
    CST_MODE,
    SHUTDOWN,
    DriveCommand,
    SEARealtimeComm,
    _ensure_root,
)
from controllers.spring_sensor import SpringSensorConfig, SpringSensorEstimator


DEFAULT_IFNAME = "eno1"
DEFAULT_DURATION_S = 30.0
DEFAULT_BASELINE_SAMPLES = 200


def write_status(path: Path, state: str, **fields: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    lines = [f"state={state}"]
    lines.extend(f"{key}={value}" for key, value in fields.items())
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def locked_command() -> DriveCommand:
    return DriveCommand(
        controlword=SHUTDOWN,
        mode_of_operation=CST_MODE,
        target_torque=0,
    )


def collect_locked_baseline(
    comm: SEARealtimeComm,
    sample_count: int,
) -> tuple[int, float, int]:
    samples: list[int] = []
    attempts = 0
    max_attempts = max(sample_count * 4, 400)
    while len(samples) < sample_count and attempts < max_attempts:
        attempts += 1
        try:
            sample = comm.sample_third_encoder_position_21bit(
                command=locked_command(),
                valid=attempts & 0xFF,
            )
        except Exception:
            continue
        response = sample.response
        if (
            response.crc_ok
            and not response.encoder_error
            and not response.communication_alarm
        ):
            samples.append(response.position_21bit)
        time.sleep(CYCLE_TIME_S)
    if len(samples) < sample_count:
        raise RuntimeError(
            f"not enough valid baseline samples: {len(samples)}/{sample_count}"
        )
    zero_counts = int(round(statistics.median(samples)))
    std_counts = statistics.stdev(samples) if len(samples) > 1 else 0.0
    return zero_counts, float(std_counts), attempts


CSV_FIELDS = [
    "cycle_index",
    "time_s",
    "wall_time_ns",
    "zero_counts",
    "baseline_std_counts",
    "third_encoder_raw",
    "third_encoder_diff_counts",
    "third_encoder_valid",
    "spring_deflection_raw_rad",
    "spring_torque_raw_nm",
    "spring_stiffness_nm_per_rad",
    "drive_position_raw",
    "drive_velocity_raw",
    "drive_torque_raw",
    "drive_following_error_raw",
    "encoder1_raw",
    "encoder2_raw",
    "load_position_rad",
    "load_velocity_rad_s",
    "feedback_torque_nm",
    "following_error_rad",
    "encoder1_rad",
    "motor_position_rad",
    "statusword",
    "cia402_state",
    "operation_enabled",
    "drive_warning",
    "drive_fault",
    "wkc",
    "sample_dt_ms",
    "clutch_state",
    "load_step_kg",
]


def run(args: argparse.Namespace) -> int:
    _ensure_root()
    configure_realtime_runtime()

    csv_path = Path(args.csv_path)
    status_path = Path(args.status_path)
    existing_paths = [path for path in (csv_path, status_path) if path.exists()]
    if existing_paths and not args.overwrite:
        existing_text = ", ".join(str(path) for path in existing_paths)
        raise FileExistsError(
            f"refusing to overwrite existing calibration output: {existing_text}"
        )
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    write_status(status_path, "INITIALIZING")

    comm = SEARealtimeComm(ifname=args.ifname, cycle_time_s=args.cycle_time)
    rows_written = 0
    shutdown_reason = "duration_complete"
    error: BaseException | None = None
    connected = False

    try:
        comm.connect()
        connected = True
        print_slave_inventory(comm)
        comm.configure_box_transparent_preop()

        preheat_result = comm.preheat_pdo(
            locked_command(),
            cycles=400,
            stable_cycles=10,
            require_statusword_nonzero=True,
        )
        print_preheat_summary("Locked-brake PDO preheat", preheat_result)
        if not preheat_result.success:
            raise RuntimeError("locked-brake PDO preheat failed")

        initial_raw = comm.read_state()
        initial_diag = comm.diagnostics_from_state(initial_raw)
        if initial_diag.operation_enabled:
            raise RuntimeError(
                "drive is unexpectedly Operation enabled; refusing locked-brake calibration"
            )
        if initial_diag.fault:
            raise RuntimeError("drive reports a fault before calibration")

        write_status(
            status_path,
            "BASELINE",
            statusword=f"0x{initial_diag.statusword:04X}",
            cia402=initial_diag.cia402_state,
            operation_enabled=int(initial_diag.operation_enabled),
        )
        zero_counts, baseline_std_counts, baseline_attempts = collect_locked_baseline(
            comm,
            args.baseline_samples,
        )

        estimator = SpringSensorEstimator(
            SpringSensorConfig(
                counts_per_rev=THIRD_ENCODER_COUNTS_PER_REV,
                zero_counts=zero_counts,
                sign=THIRD_ENCODER_SIGN,
                spring_stiffness_nm_per_rad=SPRING_STIFFNESS_NM_PER_RAD,
            )
        )

        write_status(
            status_path,
            "READY_FOR_LOADING",
            duration_s=args.duration,
            zero_counts=zero_counts,
            baseline_std_counts=f"{baseline_std_counts:.6f}",
            baseline_attempts=baseline_attempts,
            instruction="add_500g_steps_now",
            operation_enabled=0,
            target_torque_nm=0,
        )
        print(
            f"READY_FOR_LOADING zero_counts={zero_counts} "
            f"baseline_std_counts={baseline_std_counts:.3f}",
            flush=True,
        )

        period_ns = normalize_cycle_time_ns(args.cycle_time)
        start_ns = monotonic_time_ns()
        stop_ns = start_ns + int(round(args.duration * NSEC_PER_SEC))
        deadline_ns = start_ns
        previous_loop_ns: int | None = None
        expected_wkc = comm.expected_wkc
        wkc_errors = 0

        with csv_path.open("w", newline="", encoding="utf-8") as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=CSV_FIELDS)
            writer.writeheader()

            cycle = 0
            while monotonic_time_ns() < stop_ns:
                loop_ns = monotonic_time_ns()
                elapsed_s = (loop_ns - start_ns) / NSEC_PER_SEC
                sample_dt_ms = (
                    args.cycle_time * 1000.0
                    if previous_loop_ns is None
                    else (loop_ns - previous_loop_ns) / 1e6
                )
                previous_loop_ns = loop_ns

                sample = comm.sample_third_encoder_position_21bit(
                    command=locked_command(),
                    valid=cycle & 0xFF,
                )
                response = sample.response
                valid = (
                    response.crc_ok
                    and not response.encoder_error
                    and not response.communication_alarm
                )
                if not valid:
                    shutdown_reason = "invalid_third_encoder"
                    raise RuntimeError("invalid third encoder frame during calibration")

                state_raw = comm.read_state()
                state_si = comm.state_to_si(state_raw)
                diag = comm.diagnostics_from_state(state_raw)
                if diag.operation_enabled:
                    shutdown_reason = "unexpected_operation_enabled"
                    raise RuntimeError("drive became Operation enabled during calibration")
                if diag.fault:
                    shutdown_reason = "drive_fault"
                    raise RuntimeError("drive fault during calibration")

                if expected_wkc is not None and sample.wkc != expected_wkc:
                    wkc_errors += 1
                    if wkc_errors >= 3:
                        shutdown_reason = "wkc_error"
                        raise RuntimeError(
                            f"WKC remained {sample.wkc}, expected {expected_wkc}"
                        )
                else:
                    wkc_errors = 0

                estimate = estimator.estimate(response.position_21bit)
                diff_counts = estimator.counts_diff_wrapped(response.position_21bit)
                writer.writerow(
                    {
                        "cycle_index": cycle,
                        "time_s": elapsed_s,
                        "wall_time_ns": time.time_ns(),
                        "zero_counts": zero_counts,
                        "baseline_std_counts": baseline_std_counts,
                        "third_encoder_raw": response.position_21bit,
                        "third_encoder_diff_counts": diff_counts,
                        "third_encoder_valid": int(valid),
                        "spring_deflection_raw_rad": estimate.delta_theta_rad,
                        "spring_torque_raw_nm": estimate.spring_torque_nm,
                        "spring_stiffness_nm_per_rad": (
                            SPRING_STIFFNESS_NM_PER_RAD
                        ),
                        "drive_position_raw": state_raw.position,
                        "drive_velocity_raw": state_raw.velocity,
                        "drive_torque_raw": state_raw.torque,
                        "drive_following_error_raw": state_raw.following_error,
                        "encoder1_raw": state_raw.encoder1,
                        "encoder2_raw": state_raw.encoder2,
                        "load_position_rad": state_si.position_rad,
                        "load_velocity_rad_s": state_si.velocity_rad_s,
                        "feedback_torque_nm": state_si.torque_nm,
                        "following_error_rad": state_si.following_error_rad,
                        "encoder1_rad": state_si.encoder1_rad,
                        "motor_position_rad": state_si.encoder2_rad,
                        "statusword": state_raw.statusword,
                        "cia402_state": diag.cia402_state,
                        "operation_enabled": int(diag.operation_enabled),
                        "drive_warning": int(diag.warning),
                        "drive_fault": int(diag.fault),
                        "wkc": sample.wkc,
                        "sample_dt_ms": sample_dt_ms,
                        "clutch_state": "engaged",
                        "load_step_kg": "",
                    }
                )
                rows_written += 1
                if rows_written % 20 == 0:
                    csv_file.flush()
                if args.print_every > 0 and cycle % args.print_every == 0:
                    print(
                        f"cycle={cycle} t={elapsed_s:.3f}s "
                        f"raw={response.position_21bit} diff={diff_counts:+d} "
                        f"tau_raw={estimate.spring_torque_nm:+.4f}Nm "
                        f"op_enabled={int(diag.operation_enabled)} wkc={sample.wkc}",
                        flush=True,
                    )

                cycle += 1
                deadline_ns += period_ns
                if monotonic_time_ns() < deadline_ns:
                    sleep_until_monotonic_ns(deadline_ns)

    except BaseException as exc:
        error = exc
        if shutdown_reason == "duration_complete":
            shutdown_reason = f"exception_{type(exc).__name__}"
    finally:
        if connected:
            try:
                for index in range(20):
                    comm.exchange_cycle(command=locked_command(), sleep=True)
            except Exception as exc:
                print(f"WARNING: locked shutdown sequence failed: {exc}", flush=True)
        try:
            comm.close()
        except Exception as exc:
            print(f"WARNING: EtherCAT close failed: {exc}", flush=True)

        write_status(
            status_path,
            "COMPLETED" if error is None else "ERROR",
            reason=shutdown_reason,
            rows=rows_written,
            csv_path=csv_path,
            error="" if error is None else repr(error),
        )

    if error is not None:
        raise error
    print(f"Calibration collection complete: rows={rows_written} csv={csv_path}")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Collect spring calibration data with the motor brake locked."
    )
    parser.add_argument("--ifname", default=DEFAULT_IFNAME)
    parser.add_argument("--duration", type=float, default=DEFAULT_DURATION_S)
    parser.add_argument("--cycle-time", type=float, default=CYCLE_TIME_S)
    parser.add_argument("--baseline-samples", type=int, default=DEFAULT_BASELINE_SAMPLES)
    parser.add_argument("--print-every", type=int, default=200)
    parser.add_argument("--csv-path", required=True)
    parser.add_argument("--status-path", required=True)
    parser.add_argument(
        "--overwrite",
        action="store_true",
        help="allow replacing an existing CSV or status file",
    )
    args = parser.parse_args()

    if args.duration <= 0.0:
        parser.error("--duration must be positive")
    if args.cycle_time <= 0.0:
        parser.error("--cycle-time must be positive")
    if args.baseline_samples < 20:
        parser.error("--baseline-samples must be at least 20")
    return run(args)


if __name__ == "__main__":
    raise SystemExit(main())
