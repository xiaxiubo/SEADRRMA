#!/usr/bin/env python3
"""Safe data collection for SEA friction identification on the Orin testbench.

The script has four modes:

* self-test: no EtherCAT access; validates profiles, controller and CSV logging.
* monitor: enables the drive but always sends zero target torque.
* breakaway: ramps torque in one direction and stops when motion is detected.
* velocity: tracks smooth constant-velocity plateaus with a bounded PI loop.

Nonzero torque requires both --execute and
--confirm-live-hardware FRICTION_TEST_ARMED. The clutch is never controlled.
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import resource
import sys
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

if __package__ in {None, ""}:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from communication.sea_motor_comm import (
    CST_MODE,
    ENABLE_OPERATION,
    SHUTDOWN,
    DriveCommand,
    SEARealtimeComm,
    _ensure_root,
    summarize_wkc,
)
from controllers.spring_sensor import SpringSensorConfig, SpringSensorEstimator


ARM_PHRASE = "FRICTION_TEST_ARMED"
DEFAULT_IFNAME = "eno1"
DEFAULT_OUTPUT_DIR = Path(__file__).resolve().parents[1] / "logs"
CYCLE_TIME_S = 0.005
MAX_TORQUE_NM = 61.0
NSEC_PER_SEC = 1_000_000_000
THIRD_ENCODER_COUNTS_PER_REV = 1 << 19
THIRD_ENCODER_SIGN = 1
SPRING_STIFFNESS_NM_PER_RAD = 2400.0
CALIBRATION_SAMPLES = 8
REALTIME_CPUS = (8, 9, 10, 11)
REALTIME_RT_PRIORITY = 90


def monotonic_time_ns() -> int:
    return time.monotonic_ns()


def sleep_until_monotonic_ns(deadline_ns: int) -> None:
    while True:
        remaining_s = (deadline_ns - time.monotonic_ns()) / NSEC_PER_SEC
        if remaining_s <= 0.0:
            return
        time.sleep(remaining_s)


def normalize_cycle_time_ns(cycle_time_s: float) -> int:
    return max(int(round(cycle_time_s * NSEC_PER_SEC)), 1)


def configure_realtime_runtime() -> None:
    for env_name in (
        "OMP_NUM_THREADS",
        "MKL_NUM_THREADS",
        "OPENBLAS_NUM_THREADS",
        "NUMEXPR_NUM_THREADS",
    ):
        os.environ.setdefault(env_name, "1")
    try:
        os.sched_setaffinity(0, set(REALTIME_CPUS))
    except (AttributeError, OSError):
        pass
    try:
        os.nice(-20)
    except Exception:
        pass
    try:
        os.sched_setscheduler(
            0,
            os.SCHED_FIFO,
            os.sched_param(REALTIME_RT_PRIORITY),
        )
    except Exception:
        pass
    try:
        soft_limit, hard_limit = resource.getrlimit(resource.RLIMIT_MEMLOCK)
        if hard_limit > soft_limit:
            resource.setrlimit(resource.RLIMIT_MEMLOCK, (hard_limit, hard_limit))
    except Exception:
        pass


def torque_nm_to_target_units(torque_nm: float, max_torque_nm: float) -> int:
    if max_torque_nm <= 0.0:
        raise ValueError("max_torque_nm must be positive")
    target = round(torque_nm / max_torque_nm * 1000.0)
    return int(max(min(target, 1000), -1000))


def print_slave_inventory(comm: SEARealtimeComm) -> None:
    print(
        f"EtherCAT network on {comm.ifname}: expected_wkc={comm.expected_wkc}",
        flush=True,
    )
    for item in comm.describe_slaves():
        print(
            f"  slave[{item.index}] name={item.name!r} "
            f"state={item.state_label} in={item.input_size} out={item.output_size}",
            flush=True,
        )


def print_preheat_summary(label: str, result) -> None:
    diag = result.last_diagnostics
    print(
        f"{label}: success={result.success} expected_wkc={result.expected_wkc} "
        f"observed_wkc={summarize_wkc(result.observed_wkc)} "
        f"statusword=0x{diag.statusword:04X} mode={diag.mode_display} "
        f"cia402={diag.cia402_state} fault={int(diag.fault)}",
        flush=True,
    )


def warmup_third_encoder(
    comm: SEARealtimeComm,
    warmup_cycles: int = 100,
    required_valid: int = 5,
) -> None:
    valid_count = 0
    for attempt in range(warmup_cycles):
        try:
            sample = comm.sample_third_encoder_position_21bit(
                command=DriveCommand(
                    controlword=ENABLE_OPERATION,
                    mode_of_operation=CST_MODE,
                    target_torque=0,
                ),
                valid=attempt & 0xFF,
            )
        except Exception as exc:
            print(f"[spring warmup] attempt={attempt} failed: {exc}", flush=True)
            continue
        response = sample.response
        if (
            response.crc_ok
            and not response.encoder_error
            and not response.communication_alarm
        ):
            valid_count += 1
            if valid_count >= required_valid:
                return
    raise RuntimeError(
        f"third encoder warmup failed: {valid_count}/{required_valid} valid samples"
    )


def collect_zero_counts(comm: SEARealtimeComm, calibration_samples: int) -> int:
    samples: list[int] = []
    attempt_limit = max(calibration_samples * 20, 100)
    for attempt in range(attempt_limit):
        if len(samples) >= calibration_samples:
            break
        try:
            sample = comm.sample_third_encoder_position_21bit(
                command=DriveCommand(
                    controlword=ENABLE_OPERATION,
                    mode_of_operation=CST_MODE,
                    target_torque=0,
                ),
                valid=attempt & 0xFF,
            )
        except Exception as exc:
            print(f"[spring zero] attempt={attempt} failed: {exc}", flush=True)
            continue
        response = sample.response
        if (
            response.crc_ok
            and not response.encoder_error
            and not response.communication_alarm
        ):
            samples.append(response.position_21bit)
    if len(samples) < calibration_samples:
        raise RuntimeError(
            f"not enough spring zero samples: {len(samples)}/{calibration_samples}"
        )
    return int(round(sum(samples) / len(samples)))


@dataclass
class VelocityPI:
    kp: float
    ki: float
    torque_limit_nm: float
    integral_limit_nm: float
    feedforward_coulomb_nm: float
    smoothing_velocity_rad_s: float = 0.02
    integral_error: float = 0.0

    def reset(self) -> None:
        self.integral_error = 0.0

    def update(self, reference_rad_s: float, measured_rad_s: float, dt_s: float) -> float:
        error = reference_rad_s - measured_rad_s
        candidate_integral = self.integral_error + error * dt_s
        integral_nm = max(
            min(self.ki * candidate_integral, self.integral_limit_nm),
            -self.integral_limit_nm,
        )
        feedforward_nm = self.feedforward_coulomb_nm * math.tanh(
            reference_rad_s / max(self.smoothing_velocity_rad_s, 1e-6)
        )
        unsaturated_nm = self.kp * error + integral_nm + feedforward_nm
        command_nm = max(
            min(unsaturated_nm, self.torque_limit_nm),
            -self.torque_limit_nm,
        )

        # Conditional integration prevents accumulation while saturation pushes
        # farther away from the feasible command range.
        saturated = not math.isclose(command_nm, unsaturated_nm, abs_tol=1e-12)
        pushes_outward = (
            command_nm >= self.torque_limit_nm and error > 0.0
        ) or (
            command_nm <= -self.torque_limit_nm and error < 0.0
        )
        if not (saturated and pushes_outward):
            self.integral_error = candidate_integral
        return command_nm


@dataclass(frozen=True)
class VelocitySchedule:
    targets_rad_s: tuple[float, ...]
    ramp_s: float
    hold_s: float

    @property
    def duration_s(self) -> float:
        return len(self.targets_rad_s) * (self.ramp_s + self.hold_s) + self.ramp_s

    def reference(self, elapsed_s: float) -> tuple[float, int, str]:
        if elapsed_s <= 0.0:
            return 0.0, 0, "ramp"

        previous = 0.0
        phase_start = 0.0
        for index, target in enumerate(self.targets_rad_s):
            ramp_end = phase_start + self.ramp_s
            hold_end = ramp_end + self.hold_s
            if elapsed_s < ramp_end:
                fraction = (elapsed_s - phase_start) / max(self.ramp_s, 1e-9)
                return (
                    previous + (target - previous) * quintic_smoothstep(fraction),
                    index,
                    "ramp",
                )
            if elapsed_s < hold_end:
                return target, index, "hold"
            previous = target
            phase_start = hold_end

        fraction = (elapsed_s - phase_start) / max(self.ramp_s, 1e-9)
        return previous * (1.0 - quintic_smoothstep(fraction)), len(self.targets_rad_s), "stop"


def quintic_smoothstep(fraction: float) -> float:
    x = max(min(fraction, 1.0), 0.0)
    return 6.0 * x**5 - 15.0 * x**4 + 10.0 * x**3


def parse_velocity_sequence(text: str) -> tuple[float, ...]:
    values = tuple(float(item.strip()) for item in text.split(",") if item.strip())
    if not values:
        raise argparse.ArgumentTypeError("velocity sequence must not be empty")
    if any(not math.isfinite(value) for value in values):
        raise argparse.ArgumentTypeError("velocity sequence contains a non-finite value")
    return values


def resolve_output_path(args: argparse.Namespace) -> Path:
    if args.csv_path:
        path = Path(args.csv_path)
    else:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        label = f"{args.mode}_{args.clutch_state}_J{args.known_inertia_kgm2:g}"
        path = DEFAULT_OUTPUT_DIR / f"friction_{label}_{timestamp}.csv"
    if path.suffix.lower() != ".csv":
        path = path.with_suffix(".csv")
    return path


def require_live_confirmation(args: argparse.Namespace) -> None:
    live_mode = args.mode in {"breakaway", "velocity"}
    if live_mode and not args.execute:
        raise ValueError(f"{args.mode} mode requires --execute")
    if args.execute and args.confirm_live_hardware != ARM_PHRASE:
        raise ValueError(
            f"nonzero torque requires --confirm-live-hardware {ARM_PHRASE}"
        )
    if args.mode == "monitor" and args.execute:
        raise ValueError("monitor mode never accepts --execute")


def validate_args(args: argparse.Namespace) -> None:
    require_live_confirmation(args)
    if args.cycle_time <= 0.0:
        raise ValueError("--cycle-time must be positive")
    if not 0.0 < args.torque_limit_nm <= MAX_TORQUE_NM:
        raise ValueError(f"--torque-limit-nm must be in (0, {MAX_TORQUE_NM}]")
    if args.mode == "breakaway":
        if args.direction not in {-1, 1}:
            raise ValueError("--direction must be -1 or 1")
        if args.ramp_duration_s <= 0.0 or args.pre_delay_s < 0.0:
            raise ValueError("breakaway timing must be non-negative")
    if args.mode == "velocity":
        if args.velocity_ramp_s <= 0.0 or args.velocity_hold_s <= 0.0:
            raise ValueError("velocity ramp and hold times must be positive")
        if max(abs(value) for value in args.velocity_sequence) > args.max_velocity_rad_s:
            raise ValueError("velocity sequence exceeds --max-velocity-rad-s")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="SEA friction identification data collection with hard safety gates."
    )
    parser.add_argument(
        "--mode",
        choices=("self-test", "monitor", "breakaway", "velocity"),
        default="self-test",
    )
    parser.add_argument("--ifname", default=DEFAULT_IFNAME)
    parser.add_argument("--cycle-time", type=float, default=CYCLE_TIME_S)
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--csv-path", default=None)
    parser.add_argument("--print-every", type=int, default=40)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--confirm-live-hardware", default="")
    parser.add_argument(
        "--clutch-state",
        choices=("released", "engaged", "unknown"),
        default="unknown",
        help="Metadata only. This script never controls the clutch.",
    )
    parser.add_argument("--known-inertia-kgm2", type=float, default=0.0)
    parser.add_argument("--temperature-label", default="unknown")

    parser.add_argument("--direction", type=int, default=1)
    parser.add_argument("--pre-delay-s", type=float, default=1.0)
    parser.add_argument("--ramp-duration-s", type=float, default=8.0)
    parser.add_argument("--breakaway-velocity-rad-s", type=float, default=0.02)
    parser.add_argument("--breakaway-confirm-cycles", type=int, default=8)
    parser.add_argument("--post-detection-s", type=float, default=1.0)

    parser.add_argument(
        "--velocity-sequence",
        type=parse_velocity_sequence,
        default=parse_velocity_sequence("0.05,-0.05"),
    )
    parser.add_argument("--velocity-ramp-s", type=float, default=2.0)
    parser.add_argument("--velocity-hold-s", type=float, default=4.0)
    parser.add_argument("--velocity-kp", type=float, default=12.0)
    parser.add_argument("--velocity-ki", type=float, default=4.0)
    parser.add_argument("--velocity-integral-limit-nm", type=float, default=2.0)
    parser.add_argument("--friction-feedforward-nm", type=float, default=0.0)

    parser.add_argument("--torque-limit-nm", type=float, default=6.0)
    parser.add_argument("--max-position-travel-rad", type=float, default=0.35)
    parser.add_argument("--max-velocity-rad-s", type=float, default=0.8)
    parser.add_argument("--max-spring-deflection-rad", type=float, default=0.003)
    parser.add_argument("--max-feedback-torque-nm", type=float, default=12.0)
    parser.add_argument("--allow-drive-warning", action="store_true")
    parser.add_argument("--strict-wkc", action=argparse.BooleanOptionalAction, default=True)
    return parser


CSV_FIELDS = [
    "cycle_index",
    "time_s",
    "mode",
    "phase",
    "phase_index",
    "clutch_state",
    "known_inertia_kgm2",
    "temperature_label",
    "command_torque_nm",
    "target_torque_raw",
    "reference_velocity_rad_s",
    "breakaway_detected",
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
    "motor_velocity_rad_s_fd",
    "third_encoder_raw",
    "third_encoder_valid",
    "spring_deflection_rad",
    "spring_velocity_rad_s",
    "spring_torque_nm",
    "statusword",
    "drive_warning",
    "drive_fault",
    "wkc",
    "sample_dt_ms",
]


def run_self_test(args: argparse.Namespace) -> int:
    output_path = resolve_output_path(args)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    schedule = VelocitySchedule(
        targets_rad_s=args.velocity_sequence,
        ramp_s=args.velocity_ramp_s,
        hold_s=args.velocity_hold_s,
    )
    controller = VelocityPI(
        kp=args.velocity_kp,
        ki=args.velocity_ki,
        torque_limit_nm=args.torque_limit_nm,
        integral_limit_nm=args.velocity_integral_limit_nm,
        feedforward_coulomb_nm=args.friction_feedforward_nm,
    )

    dt_s = args.cycle_time
    simulated_velocity = 0.0
    simulated_position = 0.0
    simulated_inertia = max(args.known_inertia_kgm2, 0.3)
    simulated_coulomb = 0.35
    rows: list[dict[str, object]] = []
    cycles = int(math.ceil(schedule.duration_s / dt_s))
    for cycle in range(cycles):
        elapsed_s = cycle * dt_s
        reference_velocity, phase_index, phase = schedule.reference(elapsed_s)
        command_nm = controller.update(reference_velocity, simulated_velocity, dt_s)
        friction_nm = simulated_coulomb * math.tanh(simulated_velocity / 0.02)
        acceleration = (command_nm - friction_nm) / simulated_inertia
        simulated_velocity += acceleration * dt_s
        simulated_position += simulated_velocity * dt_s
        rows.append(
            {
                "cycle_index": cycle,
                "time_s": elapsed_s,
                "mode": "self-test",
                "phase": phase,
                "phase_index": phase_index,
                "clutch_state": args.clutch_state,
                "known_inertia_kgm2": args.known_inertia_kgm2,
                "temperature_label": args.temperature_label,
                "command_torque_nm": command_nm,
                "target_torque_raw": torque_nm_to_target_units(command_nm, MAX_TORQUE_NM),
                "reference_velocity_rad_s": reference_velocity,
                "breakaway_detected": 0,
                "load_position_rad": simulated_position,
                "load_velocity_rad_s": simulated_velocity,
                "feedback_torque_nm": command_nm,
                "spring_torque_nm": command_nm - simulated_inertia * acceleration,
                "wkc": 0,
                "sample_dt_ms": dt_s * 1000.0,
            }
        )

    max_command = max(abs(float(row["command_torque_nm"])) for row in rows)
    if max_command > args.torque_limit_nm + 1e-9:
        raise RuntimeError("self-test failed: torque limit violation")
    if not any(row["phase"] == "hold" for row in rows):
        raise RuntimeError("self-test failed: schedule never entered hold phase")

    with output_path.open("w", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=CSV_FIELDS)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)

    print("SEA friction identification self-test passed.", flush=True)
    print(f"  cycles={cycles} schedule_duration_s={schedule.duration_s:.3f}", flush=True)
    print(f"  max_abs_command_nm={max_command:.6f}", flush=True)
    print(f"  output={output_path}", flush=True)
    print("  EtherCAT_access=0 nonzero_hardware_torque=0 clutch_control=0", flush=True)
    return 0


def command_for_breakaway(
    elapsed_s: float,
    direction: int,
    pre_delay_s: float,
    ramp_duration_s: float,
    torque_limit_nm: float,
    detected: bool,
) -> tuple[float, str]:
    if detected:
        return 0.0, "detected_stop"
    if elapsed_s < pre_delay_s:
        return 0.0, "pre_delay"
    if elapsed_s >= pre_delay_s + ramp_duration_s:
        return 0.0, "ramp_complete"
    fraction = (elapsed_s - pre_delay_s) / max(ramp_duration_s, 1e-9)
    return direction * torque_limit_nm * max(min(fraction, 1.0), 0.0), "ramp"


def run_hardware(args: argparse.Namespace) -> int:
    _ensure_root()
    configure_realtime_runtime()
    output_path = resolve_output_path(args)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    comm = SEARealtimeComm(ifname=args.ifname, cycle_time_s=args.cycle_time)
    spring_estimator = SpringSensorEstimator(
        SpringSensorConfig(
            counts_per_rev=THIRD_ENCODER_COUNTS_PER_REV,
            zero_counts=0,
            sign=THIRD_ENCODER_SIGN,
            spring_stiffness_nm_per_rad=SPRING_STIFFNESS_NM_PER_RAD,
        )
    )
    velocity_controller = VelocityPI(
        kp=args.velocity_kp,
        ki=args.velocity_ki,
        torque_limit_nm=args.torque_limit_nm,
        integral_limit_nm=args.velocity_integral_limit_nm,
        feedforward_coulomb_nm=args.friction_feedforward_nm,
    )
    velocity_schedule = VelocitySchedule(
        targets_rad_s=args.velocity_sequence,
        ramp_s=args.velocity_ramp_s,
        hold_s=args.velocity_hold_s,
    )
    duration_s = (
        velocity_schedule.duration_s
        if args.mode == "velocity"
        else args.pre_delay_s + args.ramp_duration_s + args.post_detection_s
        if args.mode == "breakaway"
        else args.duration
    )

    rows: list[dict[str, object]] = []
    shutdown_reason = "duration_complete"
    detected = False
    detection_time_s: float | None = None
    detection_count = 0
    spring_invalid_count = 0
    breakaway_torque_nm: float | None = None
    pending_error: BaseException | None = None
    previous_motor_position_rad: float | None = None
    previous_spring_deflection_rad: float | None = None
    previous_loop_ns: int | None = None
    last_spring_raw = 0
    last_spring_valid = False
    last_spring_deflection_rad = 0.0
    last_spring_velocity_rad_s = 0.0
    last_spring_torque_nm = 0.0

    try:
        comm.connect()
        print_slave_inventory(comm)
        comm.configure_box_transparent_preop()

        preheat_result = comm.preheat_pdo(
            DriveCommand(controlword=SHUTDOWN, mode_of_operation=CST_MODE, target_torque=0),
            cycles=400,
            stable_cycles=10,
            require_statusword_nonzero=True,
        )
        print_preheat_summary("PDO preheat", preheat_result)
        if not preheat_result.success:
            raise RuntimeError("PDO preheat failed; drive power may be unavailable")

        for result in comm.enable_cia402(
            mode_of_operation=CST_MODE,
            target_torque=0,
            timeout_cycles_per_step=300,
        ):
            if not result.success:
                raise RuntimeError(f"CiA 402 transition failed: {result.step_name}")

        enabled_result = comm.stabilize_enabled_state(
            mode_of_operation=CST_MODE,
            target_torque=0,
            cycles=20,
            stable_cycles=5,
        )
        print_preheat_summary("Enabled-state stabilization", enabled_result)
        if not enabled_result.success:
            raise RuntimeError("drive did not remain operation enabled")

        warmup_third_encoder(comm, warmup_cycles=100, required_valid=5)
        zero_counts = collect_zero_counts(comm, CALIBRATION_SAMPLES)
        spring_estimator.tare_zero(zero_counts)

        initial_raw = comm.read_state()
        initial_si = comm.state_to_si(initial_raw)
        initial_diag = comm.diagnostics_from_state(initial_raw)
        if initial_diag.fault:
            raise RuntimeError("drive reports a fault before the experiment")
        if initial_diag.warning and not args.allow_drive_warning:
            raise RuntimeError("drive warning active; use --allow-drive-warning only after inspection")

        print(
            f"Starting mode={args.mode} execute={args.execute} duration={duration_s:.3f}s "
            f"clutch={args.clutch_state} known_J={args.known_inertia_kgm2:g}kgm2",
            flush=True,
        )
        print(
            f"Safety limits: torque={args.torque_limit_nm:g}Nm "
            f"travel={args.max_position_travel_rad:g}rad "
            f"velocity={args.max_velocity_rad_s:g}rad/s "
            f"spring={args.max_spring_deflection_rad:g}rad "
            f"feedback={args.max_feedback_torque_nm:g}Nm",
            flush=True,
        )

        period_ns = normalize_cycle_time_ns(args.cycle_time)
        start_ns = monotonic_time_ns()
        stop_ns = start_ns + int(round(duration_s * NSEC_PER_SEC))
        deadline_ns = start_ns
        cycle = 0
        expected_wkc = comm.expected_wkc
        wkc_error_count = 0

        comm.queue_third_encoder_request(
            command=DriveCommand(
                controlword=ENABLE_OPERATION,
                mode_of_operation=CST_MODE,
                target_torque=0,
            ),
            valid=0,
        )
        comm.send_processdata()

        while monotonic_time_ns() < stop_ns:
            loop_ns = monotonic_time_ns()
            elapsed_s = (loop_ns - start_ns) / NSEC_PER_SEC
            dt_s = (
                args.cycle_time
                if previous_loop_ns is None
                else max((loop_ns - previous_loop_ns) / NSEC_PER_SEC, 1e-6)
            )
            previous_loop_ns = loop_ns

            spring_sample = comm.poll_queued_third_encoder_sample(timeout_us=0)
            valid_spring = spring_sample is not None and (
                spring_sample.response.crc_ok
                and not spring_sample.response.encoder_error
                and not spring_sample.response.communication_alarm
            )
            if valid_spring:
                spring_invalid_count = 0
                estimate = spring_estimator.estimate(spring_sample.response.position_21bit)
                last_spring_raw = estimate.raw_counts
                last_spring_velocity_rad_s = (
                    0.0
                    if previous_spring_deflection_rad is None
                    else (estimate.delta_theta_rad - previous_spring_deflection_rad) / dt_s
                )
                previous_spring_deflection_rad = estimate.delta_theta_rad
                last_spring_deflection_rad = estimate.delta_theta_rad
                last_spring_torque_nm = estimate.spring_torque_nm
                last_spring_valid = True
            else:
                last_spring_valid = False
                spring_invalid_count += 1

            state_raw = comm.read_state()
            state_si = comm.state_to_si(state_raw)
            diag = comm.diagnostics_from_state(state_raw)
            motor_velocity_fd = (
                0.0
                if previous_motor_position_rad is None
                else (state_si.encoder2_rad - previous_motor_position_rad) / dt_s
            )
            previous_motor_position_rad = state_si.encoder2_rad

            reference_velocity_rad_s = 0.0
            phase_index = 0
            phase = "monitor"
            command_torque_nm = 0.0
            if args.mode == "breakaway":
                command_torque_nm, phase = command_for_breakaway(
                    elapsed_s,
                    args.direction,
                    args.pre_delay_s,
                    args.ramp_duration_s,
                    args.torque_limit_nm,
                    detected,
                )
                directional_velocity = args.direction * state_si.velocity_rad_s
                if not detected and directional_velocity >= args.breakaway_velocity_rad_s:
                    detection_count += 1
                else:
                    detection_count = 0
                if detection_count >= args.breakaway_confirm_cycles:
                    detected = True
                    detection_time_s = elapsed_s
                    breakaway_torque_nm = command_torque_nm
                    command_torque_nm = 0.0
                    phase = "detected_stop"
                    print(
                        f"Breakaway detected: t={elapsed_s:.3f}s "
                        f"torque={breakaway_torque_nm:.3f}Nm "
                        f"velocity={state_si.velocity_rad_s:.4f}rad/s",
                        flush=True,
                    )
            elif args.mode == "velocity":
                reference_velocity_rad_s, phase_index, phase = velocity_schedule.reference(
                    elapsed_s
                )
                command_torque_nm = velocity_controller.update(
                    reference_velocity_rad_s,
                    state_si.velocity_rad_s,
                    dt_s,
                )

            applied_torque_nm = command_torque_nm if args.execute else 0.0
            target_torque_raw = torque_nm_to_target_units(
                applied_torque_nm,
                MAX_TORQUE_NM,
            )

            safety_violation = None
            if diag.fault:
                safety_violation = "drive_fault"
            elif diag.warning and not args.allow_drive_warning:
                safety_violation = "drive_warning"
            elif abs(state_si.position_rad - initial_si.position_rad) > args.max_position_travel_rad:
                safety_violation = "position_travel"
            elif abs(state_si.velocity_rad_s) > args.max_velocity_rad_s:
                safety_violation = "load_velocity"
            elif abs(motor_velocity_fd) > args.max_velocity_rad_s:
                safety_violation = "motor_velocity"
            elif abs(last_spring_deflection_rad) > args.max_spring_deflection_rad:
                safety_violation = "spring_deflection"
            elif spring_invalid_count >= 5:
                safety_violation = "third_encoder_invalid"
            elif abs(state_si.torque_nm) > args.max_feedback_torque_nm:
                safety_violation = "feedback_torque"

            if safety_violation is not None:
                shutdown_reason = f"safety_{safety_violation}"
                print(f"SAFETY STOP: {safety_violation}", flush=True)
                break

            comm.queue_third_encoder_request(
                command=DriveCommand(
                    controlword=ENABLE_OPERATION,
                    mode_of_operation=CST_MODE,
                    target_torque=target_torque_raw,
                ),
                valid=cycle & 0xFF,
            )
            comm.send_processdata()
            wkc = (
                spring_sample.wkc
                if spring_sample is not None
                else comm.last_processdata_wkc
            )
            if expected_wkc is not None and wkc != expected_wkc:
                wkc_error_count += 1
                if args.strict_wkc and wkc_error_count >= 3:
                    shutdown_reason = f"wkc_{wkc}_expected_{expected_wkc}"
                    print(f"SAFETY STOP: {shutdown_reason}", flush=True)
                    break
            else:
                wkc_error_count = 0

            rows.append(
                {
                    "cycle_index": cycle,
                    "time_s": elapsed_s,
                    "mode": args.mode,
                    "phase": phase,
                    "phase_index": phase_index,
                    "clutch_state": args.clutch_state,
                    "known_inertia_kgm2": args.known_inertia_kgm2,
                    "temperature_label": args.temperature_label,
                    "command_torque_nm": applied_torque_nm,
                    "target_torque_raw": target_torque_raw,
                    "reference_velocity_rad_s": reference_velocity_rad_s,
                    "breakaway_detected": int(detected),
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
                    "motor_velocity_rad_s_fd": motor_velocity_fd,
                    "third_encoder_raw": last_spring_raw,
                    "third_encoder_valid": int(last_spring_valid),
                    "spring_deflection_rad": last_spring_deflection_rad,
                    "spring_velocity_rad_s": last_spring_velocity_rad_s,
                    "spring_torque_nm": last_spring_torque_nm,
                    "statusword": state_raw.statusword,
                    "drive_warning": int(diag.warning),
                    "drive_fault": int(diag.fault),
                    "wkc": wkc,
                    "sample_dt_ms": dt_s * 1000.0,
                }
            )

            if args.print_every > 0 and cycle % args.print_every == 0:
                print(
                    f"cycle={cycle} t={elapsed_s:.3f}s phase={phase} "
                    f"v_ref={reference_velocity_rad_s:+.4f} "
                    f"v={state_si.velocity_rad_s:+.4f}rad/s "
                    f"cmd={applied_torque_nm:+.3f}Nm "
                    f"tau_fb={state_si.torque_nm:+.3f}Nm "
                    f"spring={last_spring_torque_nm:+.3f}Nm wkc={wkc}",
                    flush=True,
                )

            if (
                args.mode == "breakaway"
                and detected
                and detection_time_s is not None
                and elapsed_s >= detection_time_s + args.post_detection_s
            ):
                shutdown_reason = "breakaway_detected"
                break
            if (
                args.mode == "breakaway"
                and not detected
                and elapsed_s >= args.pre_delay_s + args.ramp_duration_s
            ):
                shutdown_reason = "breakaway_not_detected"
                break

            cycle += 1
            deadline_ns += period_ns
            if monotonic_time_ns() < deadline_ns:
                sleep_until_monotonic_ns(deadline_ns)

    except BaseException as exc:
        pending_error = exc
        shutdown_reason = f"exception_{type(exc).__name__}"
    finally:
        try:
            for index in range(20):
                comm.queue_third_encoder_request(
                    command=DriveCommand(
                        controlword=ENABLE_OPERATION,
                        mode_of_operation=CST_MODE,
                        target_torque=0,
                    ),
                    valid=index & 0xFF,
                )
                comm.send_processdata()
                comm.receive_processdata(timeout_us=5_000)
                time.sleep(args.cycle_time)
            for index in range(5):
                comm.queue_third_encoder_request(
                    command=DriveCommand(
                        controlword=SHUTDOWN,
                        mode_of_operation=CST_MODE,
                        target_torque=0,
                    ),
                    valid=index & 0xFF,
                )
                comm.send_processdata()
                comm.receive_processdata(timeout_us=5_000)
                time.sleep(args.cycle_time)
        except Exception as exc:
            print(f"WARNING: zero-torque shutdown sequence failed: {exc}", flush=True)
        try:
            comm.close()
        except Exception as exc:
            print(f"WARNING: EtherCAT close failed: {exc}", flush=True)
        try:
            with output_path.open("w", newline="", encoding="utf-8") as csv_file:
                writer = csv.DictWriter(csv_file, fieldnames=CSV_FIELDS)
                writer.writeheader()
                writer.writerows(rows)
        except Exception as exc:
            print(f"ERROR: failed to save CSV: {exc}", flush=True)
            if pending_error is None:
                pending_error = exc

    print(f"Experiment finished: reason={shutdown_reason}", flush=True)
    if breakaway_torque_nm is not None:
        print(f"Breakaway torque estimate: {breakaway_torque_nm:.6f} Nm", flush=True)
    print(f"Rows={len(rows)} output={output_path}", flush=True)
    if pending_error is not None:
        raise pending_error
    return 0


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    try:
        validate_args(args)
    except ValueError as exc:
        parser.error(str(exc))
    if args.mode == "self-test":
        return run_self_test(args)
    return run_hardware(args)


if __name__ == "__main__":
    raise SystemExit(main())
