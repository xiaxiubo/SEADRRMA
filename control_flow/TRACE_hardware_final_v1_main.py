#!/usr/bin/env python3
from __future__ import annotations

import argparse
from collections import deque
import csv
import ctypes
import errno
import math
import os
import resource
import sys
import time
from datetime import datetime
from pathlib import Path

if __package__ in {None, ""}:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from communication.sea_motor_comm import (
    CST_MODE,
    ENABLE_OPERATION,
    DriveCommand,
    DriveStateSI,
    SEARealtimeComm,
    SHUTDOWN,
    summarize_wkc,
    _ensure_root,
)
from controllers.sea_model import SeaState
from controllers.spring_sensor import SpringSensorConfig, SpringSensorEstimator
from controllers.trace_inertia_calibrator import TRACEInertiaCalibrator
from controllers.trace_onnx_controller import TRACEOnnxController
from controllers.trace_physics_inertia_estimator import (
    TRACEPhysicsInertiaEstimator,
)


IFNAME = "eno1"
CYCLE_TIME_S = 0.005
DURATION_S = 3.0
PRINT_EVERY = 20
EXECUTE = False
MAX_TORQUE_NM = 61.0
WARMUP_CYCLES = 2
THIRD_ENCODER_COUNTS_PER_REV = 1 << 19
THIRD_ENCODER_SIGN = 1
SPRING_STIFFNESS_NM_PER_RAD = 2400.0
SPRING_OBSERVATION_SCALE = 1.0
ACTION_TORQUE_SCALE = 1.0
INERTIA_FRICTION_COMP_POSITIVE_NM = 0.0
INERTIA_FRICTION_COMP_NEGATIVE_NM = 0.0
INERTIA_FRICTION_COMP_VELOCITY_SCALE_RAD_S = 0.03
CALIBRATION_SAMPLES = 8
TRAJECTORY_MODE = "single"
TRAJECTORY_AMPLITUDE_RAD = 0.03
TRAJECTORY_FREQUENCY_HZ = 0.2
TRAJECTORY_PHASE_RAD = 0.0
TRAJECTORY_COMPOSITE_BIAS_RAD = 0.1
TRAJECTORY_COMPOSITE_COMPONENTS = (
    (0.04, 3.0, 0.0),
    (0.15, 1.0, 0.0),
    (0.2, 6.0, 0.0),
    (0.2, 0.5, 0.0),
)
REALTIME_CPUS = (8, 9, 10, 11)
REALTIME_RT_PRIORITY = 99
CLOCK_MONOTONIC = 1
TIMER_ABSTIME = 1
NSEC_PER_SEC = 1_000_000_000

# TRACE ONNX deployment package
DEPLOYMENT_PACKAGE_DIR = (
    Path(__file__).resolve().parents[1]
    / "deployment_package_trace_hardware_final_v1_20260726"
)
ONNX_MODEL_PATH = DEPLOYMENT_PACKAGE_DIR / "onnx" / "trace_hardware_control.onnx"
INERTIA_CALIBRATOR_PATH = (
    DEPLOYMENT_PACKAGE_DIR
    / "calibration"
    / "hardware_inertia_calibrator_v1.json"
)
STARTUP_OVERRIDE_SECONDS = 0.6


class Timespec(ctypes.Structure):
    _fields_ = [
        ("tv_sec", ctypes.c_long),
        ("tv_nsec", ctypes.c_long),
    ]


def _load_clock_lib() -> ctypes.CDLL:
    for lib_name in ("librt.so.1", "librt.so", "libc.so.6"):
        try:
            return ctypes.CDLL(lib_name, use_errno=True)
        except OSError:
            continue
    raise OSError("Unable to load a C library that provides clock_gettime/clock_nanosleep")


_CLOCK_LIB = _load_clock_lib()
_CLOCK_GETTIME = _CLOCK_LIB.clock_gettime
_CLOCK_GETTIME.argtypes = [ctypes.c_int, ctypes.POINTER(Timespec)]
_CLOCK_GETTIME.restype = ctypes.c_int
_CLOCK_NANOSLEEP = _CLOCK_LIB.clock_nanosleep
_CLOCK_NANOSLEEP.argtypes = [ctypes.c_int, ctypes.c_int, ctypes.POINTER(Timespec), ctypes.c_void_p]
_CLOCK_NANOSLEEP.restype = ctypes.c_int


def monotonic_time_ns() -> int:
    ts = Timespec()
    if _CLOCK_GETTIME(CLOCK_MONOTONIC, ctypes.byref(ts)) != 0:
        err = ctypes.get_errno()
        raise OSError(err, os.strerror(err))
    return int(ts.tv_sec) * NSEC_PER_SEC + int(ts.tv_nsec)


def sleep_until_monotonic_ns(deadline_ns: int) -> None:
    ts = Timespec(
        int(deadline_ns // NSEC_PER_SEC),
        int(deadline_ns % NSEC_PER_SEC),
    )
    while True:
        result = _CLOCK_NANOSLEEP(CLOCK_MONOTONIC, TIMER_ABSTIME, ctypes.byref(ts), None)
        if result == 0:
            return
        if result != errno.EINTR:
            raise OSError(result, os.strerror(result))


def configure_realtime_runtime(
    cpu_affinity: tuple[int, ...] = REALTIME_CPUS,
    rt_priority: int = REALTIME_RT_PRIORITY,
) -> None:
    for env_name in (
        "OMP_NUM_THREADS",
        "MKL_NUM_THREADS",
        "OPENBLAS_NUM_THREADS",
        "NUMEXPR_NUM_THREADS",
    ):
        os.environ.setdefault(env_name, "1")

    try:
        if cpu_affinity:
            os.sched_setaffinity(0, set(cpu_affinity))
    except (AttributeError, OSError):
        pass


    try:
        os.nice(-20)
    except Exception:
        pass

    try:
        soft_limit, hard_limit = resource.getrlimit(resource.RLIMIT_MEMLOCK)
        target_limit = hard_limit if hard_limit != resource.RLIM_INFINITY else soft_limit
        if target_limit > soft_limit:
            resource.setrlimit(resource.RLIMIT_MEMLOCK, (target_limit, target_limit))
    except Exception:
        pass

    try:
        libc = ctypes.CDLL("libc.so.6", use_errno=True)
        mcl_current = 1
        mcl_future = 2
        if libc.mlockall(mcl_current | mcl_future) != 0:
            err = ctypes.get_errno()
            raise OSError(err, os.strerror(err))
    except Exception:
        pass

    try:
        os.sched_setscheduler(0, os.SCHED_FIFO, os.sched_param(rt_priority))
    except Exception:
        pass


def normalize_cycle_time_ns(cycle_time_s: float) -> int:
    period_ns = int(round(cycle_time_s * NSEC_PER_SEC))
    return max(period_ns, 1)


def torque_nm_to_target_units(torque_nm: float, max_torque_nm: float) -> int:
    if max_torque_nm <= 0.0:
        raise ValueError("max_torque_nm must be positive.")
    target = round(torque_nm / max_torque_nm * 1000.0)
    return int(max(min(target, 1000), -1000))


def print_slave_inventory(comm: SEARealtimeComm) -> None:
    print(f"EtherCAT network on {comm.ifname}: expected_wkc={comm.expected_wkc}", flush=True)
    for item in comm.describe_slaves():
        print(
            "  slave[{idx}] name={name!r} man={man} id={prod} rev={rev} state={state} in={inp} out={out}".format(
                idx=item.index,
                name=item.name,
                man=item.vendor_id,
                prod=item.product_code,
                rev=item.revision,
                state=item.state_label,
                inp=item.input_size,
                out=item.output_size,
            ),
            flush=True,
        )
    for row in comm.describe_xml_targets():
        print(f"  {row}", flush=True)
    print("  Runtime RxPDO command objects:", flush=True)
    for row in comm.describe_required_rxpdo():
        print(f"    {row}", flush=True)


def print_preheat_summary(label: str, result) -> None:
    diag = result.last_diagnostics
    print(
        (
            f"{label}: success={result.success} expected_wkc={result.expected_wkc} "
            f"observed_wkc={summarize_wkc(result.observed_wkc)} "
            f"stable={result.consecutive_stable_cycles}/{result.stable_cycles_required} "
            f"statusword=0x{diag.statusword:04X} mode={diag.mode_display} "
            f"cia402={diag.cia402_state} remote={int(diag.remote)} "
            f"warning={int(diag.warning)} fault={int(diag.fault)}"
        ),
        flush=True,
    )
    if not result.success and result.samples:
        print(result.sample_summary(), flush=True)


def print_state(prefix: str, state: DriveStateSI) -> None:
    print(
        "{prefix} statusword=0x{sw:04X} mode={mode} pos={pos:.6f}rad vel={vel:.6f}rad/s "
        "torque={torque:.3f}Nm fe={fe:.6f}rad enc1={enc1:.6f}rad enc2={enc2:.6f}rad".format(
            prefix=prefix,
            sw=state.statusword,
            mode=state.mode_display,
            pos=state.position_rad,
            vel=state.velocity_rad_s,
            torque=state.torque_nm,
            fe=state.following_error_rad,
            enc1=state.encoder1_rad,
            enc2=state.encoder2_rad,
        ),
        flush=True,
    )


def print_third_encoder_sample(prefix: str, sample) -> None:
    response = sample.response
    print(
        (
            f"{prefix} wkc={sample.wkc} pos21={response.position_21bit} "
            f"crc_ok={int(response.crc_ok)} enc_err={int(response.encoder_error)} "
            f"comm_alarm={int(response.communication_alarm)}"
        ),
        flush=True,
    )


def print_spring_estimate(prefix: str, spring_estimate, spring_vel_rad_s: float) -> None:
    print(
        (
            f"{prefix} delta_theta={spring_estimate.delta_theta_rad:.6f}rad "
            f"spring_vel={spring_vel_rad_s:.6f}rad/s "
            f"spring_torque={spring_estimate.spring_torque_nm:.3f}Nm"
        ),
        flush=True,
    )


def print_drrma_terms(
    prefix: str,
    reference: SeaState,
    measured: SeaState,
    spring_defl_rad: float,
    spring_vel_rad_s: float,
    spring_torque_nm: float,
    action_norm: float,
    predicted_torque_nm: float,
    applied_torque_nm: float,
    target_torque: int,
    measured_torque_nm: float,
    J_hat: float,
) -> None:
    err_theta_m = reference.theta_m_rad - measured.theta_m_rad
    err_theta_l = reference.theta_l_rad - measured.theta_l_rad
    err_dtheta_m = reference.dtheta_m_rad_s - measured.dtheta_m_rad_s
    err_dtheta_l = reference.dtheta_l_rad_s - measured.dtheta_l_rad_s
    print(
        (
            f"{prefix} ref_m={reference.theta_m_rad:.6f} meas_m={measured.theta_m_rad:.6f} err_m={err_theta_m:+.6f} "
            f"ref_l={reference.theta_l_rad:.6f} meas_l={measured.theta_l_rad:.6f} err_l={err_theta_l:+.6f} "
            f"ref_dm={reference.dtheta_m_rad_s:.6f} meas_dm={measured.dtheta_m_rad_s:.6f} err_dm={err_dtheta_m:+.6f} "
            f"ref_dl={reference.dtheta_l_rad_s:.6f} meas_dl={measured.dtheta_l_rad_s:.6f} err_dl={err_dtheta_l:+.6f} "
            f"spring_defl={spring_defl_rad:.6f}rad spring_vel={spring_vel_rad_s:.6f}rad/s "
            f"spring_torque={spring_torque_nm:.3f}Nm action_norm={action_norm:.4f} raw_action={predicted_torque_nm:.3f}Nm "
            f"send_action={applied_torque_nm:.3f}Nm target={target_torque} feedback_torque={measured_torque_nm:.3f}Nm "
            f"J_hat={J_hat:.4f}kg·m²"
        ),
        flush=True,
    )



def resolve_csv_path(csv_path: str | Path | None) -> Path:
    if csv_path is not None:
        path = Path(csv_path)
    else:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        log_dir = Path(__file__).resolve().parents[1] / "logs"
        path = log_dir / f"drrma_data_{timestamp}.csv"
    if path.suffix.lower() != ".csv":
        path = path.with_suffix(".csv")
    return path


def quintic_smoothstep(fraction: float) -> tuple[float, float]:
    fraction = max(min(fraction, 1.0), 0.0)
    value = (
        6.0 * fraction**5
        - 15.0 * fraction**4
        + 10.0 * fraction**3
    )
    derivative = 30.0 * fraction**2 * (fraction - 1.0) ** 2
    return value, derivative


def reference_envelope(
    elapsed_s: float,
    duration_s: float,
    ramp_in_s: float,
    ramp_out_s: float,
) -> tuple[float, float]:
    ramp_in = 1.0
    ramp_in_rate = 0.0
    if ramp_in_s > 0.0 and elapsed_s < ramp_in_s:
        ramp_in, derivative = quintic_smoothstep(elapsed_s / ramp_in_s)
        ramp_in_rate = derivative / ramp_in_s

    ramp_out = 1.0
    ramp_out_rate = 0.0
    remaining_s = max(duration_s - elapsed_s, 0.0)
    if ramp_out_s > 0.0 and remaining_s < ramp_out_s:
        ramp_out, derivative = quintic_smoothstep(remaining_s / ramp_out_s)
        ramp_out_rate = -derivative / ramp_out_s

    return (
        ramp_in * ramp_out,
        ramp_in_rate * ramp_out + ramp_in * ramp_out_rate,
    )


def build_reference(
    elapsed_s: float,
    initial_position_rad: float,
    initial_motor_load_offset_rad: float,
    amplitude_rad: float,
    frequency_hz: float,
    duration_s: float,
    ramp_in_s: float = 0.0,
    ramp_out_s: float = 0.0,
) -> SeaState:
    if TRAJECTORY_MODE == "single":
        envelope, envelope_rate = reference_envelope(
            elapsed_s,
            duration_s=duration_s,
            ramp_in_s=ramp_in_s,
            ramp_out_s=ramp_out_s,
        )
        angle = 2.0 * math.pi * frequency_hz * elapsed_s + TRAJECTORY_PHASE_RAD
        sine = math.sin(angle)
        cosine = math.cos(angle)
        theta_l = initial_position_rad + amplitude_rad * envelope * sine
        dtheta_l = amplitude_rad * (
            envelope_rate * sine
            + envelope * 2.0 * math.pi * frequency_hz * cosine
        )
    elif TRAJECTORY_MODE == "composite":
        theta_l = initial_position_rad + TRAJECTORY_COMPOSITE_BIAS_RAD
        dtheta_l = 0.0
        for amplitude_rad, angular_frequency_rad_s, phase_rad in TRAJECTORY_COMPOSITE_COMPONENTS:
            angle = angular_frequency_rad_s * elapsed_s + phase_rad
            theta_l += amplitude_rad * math.sin(angle)
            dtheta_l += amplitude_rad * angular_frequency_rad_s * math.cos(angle)
    else:
        raise ValueError(f"Unsupported TRAJECTORY_MODE={TRAJECTORY_MODE!r}")
    theta_m = theta_l + initial_motor_load_offset_rad
    dtheta_m = dtheta_l
    return SeaState(
        theta_m_rad=theta_m,
        dtheta_m_rad_s=dtheta_m,
        theta_l_rad=theta_l,
        dtheta_l_rad_s=dtheta_l,
    )


def measured_to_sea_state(
    measured: DriveStateSI,
    prev_encoder2_rad: float | None = None,
    dt: float | None = CYCLE_TIME_S,
) -> SeaState:
    if prev_encoder2_rad is not None and dt is not None and dt > 0.0:
        dtheta_m_rad_s = (measured.encoder2_rad - prev_encoder2_rad) / dt
    else:
        dtheta_m_rad_s = measured.velocity_rad_s
    return SeaState(
        theta_m_rad=measured.encoder2_rad,
        dtheta_m_rad_s=dtheta_m_rad_s,
        theta_l_rad=measured.position_rad,
        dtheta_l_rad_s=measured.velocity_rad_s,
    )


def spring_velocity_rad_s(
    current_delta_theta_rad: float,
    previous_delta_theta_rad: float | None,
    dt: float | None,
) -> float:
    if previous_delta_theta_rad is None:
        return 0.0
    if dt is None or dt <= 0.0:
        raise ValueError("dt must be positive")
    return (current_delta_theta_rad - previous_delta_theta_rad) / dt


def directional_inertia_friction_compensation_nm(
    velocity_rad_s: float,
    positive_nm: float,
    negative_nm: float,
    velocity_scale_rad_s: float,
) -> float:
    if velocity_scale_rad_s <= 0.0:
        raise ValueError("velocity_scale_rad_s must be positive")
    direction = math.tanh(velocity_rad_s / velocity_scale_rad_s)
    return (
        positive_nm * max(direction, 0.0)
        + negative_nm * max(-direction, 0.0)
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
            print_third_encoder_sample(prefix=f"[warmup] attempt={attempt}", sample=sample)
            valid_count += 1
            if valid_count >= required_valid:
                return
        except Exception as exc:
            print(f"[warmup] attempt={attempt} failed: {exc}", flush=True)
    raise RuntimeError(
        f"Third encoder warm-up failed: only got {valid_count} valid frames in {warmup_cycles} attempts"
    )



def collect_zero_counts(comm: SEARealtimeComm, calibration_samples: int) -> int:
    zero_samples: list[int] = []
    attempt_limit = max(calibration_samples * 20, 100)
    last_error: Exception | None = None
    for attempt in range(attempt_limit):
        if len(zero_samples) >= calibration_samples:
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
            raw_counts = sample.response.position_21bit
            print_third_encoder_sample(f"[zero] attempt={attempt}", sample)
            zero_samples.append(raw_counts)
        except Exception as exc:
            last_error = exc
            print(f"[zero] attempt={attempt} failed: {exc}", flush=True)
            continue
    if len(zero_samples) < calibration_samples:
        raise RuntimeError(
            f"Failed to collect enough valid zero samples. got={len(zero_samples)} need={calibration_samples} "
            f"last_error={last_error}"
        )
    return int(round(sum(zero_samples) / len(zero_samples)))


def main() -> int:
    # Ensure root privileges BEFORE doing anything else
    # This avoids loading models twice (once before sudo, once after)
    _ensure_root()

    parser = argparse.ArgumentParser(description="SEA DRRMA control flow.")
    parser.add_argument("--ifname", default=IFNAME)
    parser.add_argument("--duration", type=float, default=DURATION_S)
    parser.add_argument("--print-every", type=int, default=PRINT_EVERY)
    parser.add_argument("--cycle-time", type=float, default=CYCLE_TIME_S)
    parser.add_argument("--warmup-cycles", type=int, default=WARMUP_CYCLES)
    parser.add_argument("--csv-path", type=str, default=None)
    parser.add_argument("--execute", action=argparse.BooleanOptionalAction, default=EXECUTE)
    parser.add_argument("--strict-wkc", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--onnx-path", type=str, default=str(ONNX_MODEL_PATH))
    parser.add_argument("--onnx-threads", type=int, default=4)
    parser.add_argument(
        "--inertia-calibrator",
        action=argparse.BooleanOptionalAction,
        default=False,
    )
    parser.add_argument(
        "--inertia-calibrator-path",
        type=str,
        default=str(INERTIA_CALIBRATOR_PATH),
    )
    parser.add_argument(
        "--physics-inertia-estimator",
        action=argparse.BooleanOptionalAction,
        default=False,
    )
    parser.add_argument(
        "--startup-override-s",
        type=float,
        default=STARTUP_OVERRIDE_SECONDS,
    )
    parser.add_argument(
        "--action-torque-scale",
        type=float,
        default=ACTION_TORQUE_SCALE,
    )
    parser.add_argument(
        "--inertia-friction-comp-positive-nm",
        type=float,
        default=INERTIA_FRICTION_COMP_POSITIVE_NM,
    )
    parser.add_argument(
        "--inertia-friction-comp-negative-nm",
        type=float,
        default=INERTIA_FRICTION_COMP_NEGATIVE_NM,
    )
    parser.add_argument(
        "--inertia-friction-comp-velocity-scale-rad-s",
        type=float,
        default=INERTIA_FRICTION_COMP_VELOCITY_SCALE_RAD_S,
    )
    parser.add_argument("--amplitude-rad", type=float, default=TRAJECTORY_AMPLITUDE_RAD)
    parser.add_argument("--frequency-hz", type=float, default=TRAJECTORY_FREQUENCY_HZ)
    parser.add_argument("--reference-ramp-s", type=float, default=0.0)
    parser.add_argument("--reference-ramp-out-s", type=float, default=0.0)
    parser.add_argument("--torque-limit-nm", type=float, default=5.0)
    parser.add_argument("--max-position-error-rad", type=float, default=0.5)
    parser.add_argument("--max-velocity-rad-s", type=float, default=3.0)
    parser.add_argument(
        "--max-motor-velocity-rad-s",
        type=float,
        default=None,
    )
    parser.add_argument(
        "--max-motor-raw-velocity-rad-s",
        type=float,
        default=12.0,
    )
    parser.add_argument("--max-spring-deflection-rad", type=float, default=0.005)
    parser.add_argument("--max-feedback-torque-nm", type=float, default=10.0)
    parser.add_argument("--allow-drive-warning", action="store_true")
    args = parser.parse_args()
    warmup_cycles = max(int(args.warmup_cycles), 0)
    if args.amplitude_rad < 0.0 or args.frequency_hz <= 0.0:
        raise ValueError("Trajectory amplitude must be non-negative and frequency must be positive.")
    if args.reference_ramp_s < 0.0:
        raise ValueError("--reference-ramp-s must be non-negative.")
    if args.reference_ramp_out_s < 0.0:
        raise ValueError("--reference-ramp-out-s must be non-negative.")
    if args.reference_ramp_s + args.reference_ramp_out_s > args.duration:
        raise ValueError("Reference ramp-in and ramp-out durations must not overlap.")
    if not 0.0 < args.torque_limit_nm <= MAX_TORQUE_NM:
        raise ValueError(f"--torque-limit-nm must be in (0, {MAX_TORQUE_NM}].")
    if not 0.0 < args.action_torque_scale <= 1.0:
        raise ValueError("--action-torque-scale must be in (0, 1].")
    if args.startup_override_s < 0.0:
        raise ValueError("--startup-override-s must be non-negative.")
    if abs(args.inertia_friction_comp_positive_nm) > 2.0:
        raise ValueError("--inertia-friction-comp-positive-nm must be within +/-2 Nm.")
    if abs(args.inertia_friction_comp_negative_nm) > 2.0:
        raise ValueError("--inertia-friction-comp-negative-nm must be within +/-2 Nm.")
    if args.inertia_friction_comp_velocity_scale_rad_s <= 0.0:
        raise ValueError(
            "--inertia-friction-comp-velocity-scale-rad-s must be positive."
        )
    max_motor_velocity_rad_s = (
        float(args.max_velocity_rad_s)
        if args.max_motor_velocity_rad_s is None
        else float(args.max_motor_velocity_rad_s)
    )
    if (
        args.max_velocity_rad_s <= 0.0
        or max_motor_velocity_rad_s <= 0.0
        or args.max_motor_raw_velocity_rad_s <= 0.0
    ):
        raise ValueError("Velocity safety limits must be positive.")
    if args.inertia_calibrator and args.physics_inertia_estimator:
        raise ValueError(
            "Enable either the statistical inertia calibrator or the physics "
            "inertia estimator, not both."
        )

    print(f"Loading DRRMA ONNX model: {args.onnx_path}", flush=True)
    drrma_controller = TRACEOnnxController(
        model_path=args.onnx_path,
        max_torque_nm=MAX_TORQUE_NM,
        num_threads=args.onnx_threads,
        startup_override_seconds=args.startup_override_s,
    )
    print("DRRMA ONNX model loaded successfully!", flush=True)
    inertia_calibrator = None
    if args.inertia_calibrator:
        inertia_calibrator = TRACEInertiaCalibrator(
            args.inertia_calibrator_path
        )
        print(
            "Hardware inertia calibrator loaded: "
            f"{args.inertia_calibrator_path}",
            flush=True,
        )
    physics_inertia_estimator = None
    if args.physics_inertia_estimator:
        physics_inertia_estimator = TRACEPhysicsInertiaEstimator()
        print("Physics inertia estimator enabled.", flush=True)

    # Configure realtime runtime AFTER model loading
    configure_realtime_runtime()
    try:
        current_affinity = sorted(os.sched_getaffinity(0))
    except Exception:
        current_affinity = []
    print(
        f"DRRMA control starting: cycle={args.cycle_time*1000.0:.3f}ms affinity={current_affinity}",
        flush=True,
    )
    print(f"Warm-up before CSV recording: {warmup_cycles} cycle(s)", flush=True)

    print("Creating SEARealtimeComm...", flush=True)
    comm = SEARealtimeComm(ifname=args.ifname, cycle_time_s=args.cycle_time)
    print("SEARealtimeComm created successfully!", flush=True)

    print("Initializing variables...", flush=True)
    csv_path = resolve_csv_path(args.csv_path)
    prev_encoder2_rad = None
    spring_estimator = SpringSensorEstimator(
        SpringSensorConfig(
            counts_per_rev=THIRD_ENCODER_COUNTS_PER_REV,
            zero_counts=0,
            sign=THIRD_ENCODER_SIGN,
            spring_stiffness_nm_per_rad=SPRING_STIFFNESS_NM_PER_RAD,
        )
    )
    prev_spring_delta_theta_rad: float | None = None
    execute = bool(args.execute)
    spring_observation_scale = SPRING_OBSERVATION_SCALE
    action_torque_scale = float(args.action_torque_scale)
    inertia_friction_comp_positive_nm = float(
        args.inertia_friction_comp_positive_nm
    )
    inertia_friction_comp_negative_nm = float(
        args.inertia_friction_comp_negative_nm
    )
    inertia_friction_comp_velocity_scale_rad_s = float(
        args.inertia_friction_comp_velocity_scale_rad_s
    )
    csv_rows: list[list[object]] = []
    csv_fieldnames = [
        "cycle_index", "time_s",
        "ref_theta_l_rad", "meas_theta_l_rad", "err_theta_l_rad",
        "ref_theta_m_rad", "meas_theta_m_rad", "err_theta_m_rad",
        "ref_vel_l_rad_s", "meas_vel_l_rad_s",
        "ref_vel_m_rad_s", "meas_vel_m_rad_s",
        "meas_vel_m_safety_rad_s",
        "reference_ramp_scale",
        "spring_defl_raw_rad", "spring_vel_raw_rad_s", "spring_torque_raw_nm",
        "spring_observation_scale",
        "spring_defl_rad", "spring_vel_rad_s", "spring_torque_nm",
        "inertia_friction_comp_nm", "spring_defl_estimator_rad",
        "third_encoder_valid",
        "action_norm", "raw_action_nm", "action_torque_scale",
        "scaled_action_nm", "action_nm", "startup_override",
        "J_hat", "J_hat_onnx", "J_hat_calibrated_unfiltered",
        "J_hat_calibrator_active", "inertia_calibration_ms",
        "J_hat_physics", "J_hat_physics_active",
        "j_raw", "j_short",
        "j_long", "jump_gate", "confidence", "J_true",
        "target_torque", "feedback_torque_nm", "statusword", "drive_warning",
        "wkc", "inference_ms", "sample_dt_ms",
    ]

    try:
        print("Connecting to EtherCAT...", flush=True)
        comm.connect()
        print("EtherCAT connected!", flush=True)
        print_slave_inventory(comm)
        print(
            f"Connected on {args.ifname}. DRRMA mode={CST_MODE}, execute={execute}, "
            f"duration={args.duration}s",
            flush=True,
        )

        print("Stage 0.5: box transparent PRE-OP configuration", flush=True)
        box_cfg = comm.configure_box_transparent_preop()
        print(
            "Third encoder transparent configured: "
            f"mode={box_cfg['mode']} interface={box_cfg['interface']} "
            f"frame={box_cfg['frame']} baud_selector={box_cfg['baud_selector']} "
            f"baud={box_cfg['explicit_baud']} polling_ms={box_cfg['polling_ms']}",
            flush=True,
        )

        print("Stage 1: PDO preheat", flush=True)
        preheat_result = comm.preheat_pdo(
            DriveCommand(controlword=SHUTDOWN, mode_of_operation=CST_MODE, target_torque=0),
            cycles=400,
            stable_cycles=10,
            require_statusword_nonzero=True,
        )
        print_preheat_summary("PDO preheat", preheat_result)
        if not preheat_result.success:
            raise RuntimeError("PDO preheat failed")

        print("Stage 2: CiA 402 enable", flush=True)
        transition_results = comm.enable_cia402(
            mode_of_operation=CST_MODE,
            target_torque=0,
            timeout_cycles_per_step=300,
        )
        for result in transition_results:
            diag = result.last_diagnostics
            print(
                (
                    f"{result.step_name}: success={result.success} expected_state={result.expected_state} "
                    f"observed_wkc={summarize_wkc(result.observed_wkc)} "
                    f"statusword=0x{diag.statusword:04X} mode={diag.mode_display} cia402={diag.cia402_state}"
                ),
                flush=True,
            )
            if not result.success:
                raise RuntimeError(f"CiA 402 transition {result.step_name} failed")


        print("Stage 3: enabled-state stabilization", flush=True)
        enabled_result = comm.stabilize_enabled_state(
            mode_of_operation=CST_MODE,
            target_torque=0,
            cycles=20,
            stable_cycles=5,
        )
        print_preheat_summary("Enabled-state stabilization", enabled_result)
        if not enabled_result.success:
            raise RuntimeError("Drive failed to remain in Operation enabled state")

        print("Stage 4: third encoder initialization", flush=True)
        print("Warming up third encoder RS485 channel...", flush=True)
        warmup_third_encoder(comm, warmup_cycles=100, required_valid=5)

        print("Stage 5: zero calibration", flush=True)
        zero_counts = collect_zero_counts(comm, CALIBRATION_SAMPLES)
        spring_estimator.tare_zero(zero_counts)
        print(
            f"Calibration done: zero_counts={zero_counts} "
            f"counts_per_rev={THIRD_ENCODER_COUNTS_PER_REV} "
            f"spring_stiffness={SPRING_STIFFNESS_NM_PER_RAD} "
            f"spring_observation_scale={spring_observation_scale} "
            f"action_torque_scale={action_torque_scale} "
            f"inertia_friction_comp_positive_nm={inertia_friction_comp_positive_nm} "
            f"inertia_friction_comp_negative_nm={inertia_friction_comp_negative_nm} "
            "inertia_friction_comp_velocity_scale_rad_s="
            f"{inertia_friction_comp_velocity_scale_rad_s}",
            flush=True,
        )

        initial_state_raw = comm.read_state()
        initial_state = comm.state_to_si(initial_state_raw)
        initial_diag = comm.diagnostics_from_state(initial_state_raw)
        print(
            "Drive ready for control: statusword=0x{sw:04X} mode={mode} cia402={cia402} "
            "remote={remote} warning={warning} fault={fault}".format(
                sw=initial_diag.statusword,
                mode=initial_diag.mode_display,
                cia402=initial_diag.cia402_state,
                remote=int(initial_diag.remote),
                warning=int(initial_diag.warning),
                fault=int(initial_diag.fault),
            ),
            flush=True,
        )
        if execute and initial_diag.warning and not args.allow_drive_warning:
            raise RuntimeError(
                "Drive warning bit is set. Refusing nonzero control without "
                "--allow-drive-warning."
            )
        initial_position_rad = initial_state.position_rad
        initial_motor_load_offset_rad = initial_state.encoder2_rad - initial_state.position_rad
        if TRAJECTORY_MODE == "composite":
            trajectory_description = (
                f"bias={TRAJECTORY_COMPOSITE_BIAS_RAD:.6f}rad "
                + "components=["
                + ", ".join(
                    (
                        f"A={amplitude:.6f}rad B={angular_frequency:.6f}rad/s "
                        f"C={phase:.6f}rad"
                    )
                    for amplitude, angular_frequency, phase in TRAJECTORY_COMPOSITE_COMPONENTS
                )
                + "]"
            )
        else:
            trajectory_description = (
                f"amplitude={args.amplitude_rad:.6f}rad "
                f"frequency={args.frequency_hz:.6f}Hz "
                f"phase={TRAJECTORY_PHASE_RAD:.6f}rad "
                f"ramp_in={args.reference_ramp_s:.3f}s "
                f"ramp_out={args.reference_ramp_out_s:.3f}s"
            )
        print(
            (
                "DRRMA sine reference initialized: "
                f"mode={TRAJECTORY_MODE} "
                f"initial_position={initial_position_rad:.6f}rad "
                f"initial_motor_load_offset={initial_motor_load_offset_rad:.6f}rad "
                f"{trajectory_description}"
            ),
            flush=True,
        )

        # Reset DRRMA controller
        drrma_controller.reset()
        print("DRRMA controller reset complete", flush=True)

        csv_path.parent.mkdir(parents=True, exist_ok=True)
        print(f"Will save CSV to: {csv_path}", flush=True)

        print("Stage 6: DRRMA control loop", flush=True)
        start_ns = monotonic_time_ns()
        stop_ns = start_ns + max(int(round(args.duration * NSEC_PER_SEC)), 0)
        period_ns = normalize_cycle_time_ns(args.cycle_time)
        cycle_deadline_ns = start_ns
        recording_started = warmup_cycles == 0
        cycle = 0
        record_cycle_index = 0
        expected_wkc = comm.expected_wkc
        wkc_error_count = 0
        max_wkc_errors = 3
        last_spring_estimate = spring_estimator.estimate(zero_counts)
        last_spring_vel_rad_s = 0.0
        prev_spring_sample_time: float | None = None
        prev_loop_start_ns: int | None = None
        motor_velocity_safety_history: deque[float] = deque(maxlen=5)


        priming_command = DriveCommand(
            controlword=ENABLE_OPERATION,
            mode_of_operation=CST_MODE,
            target_torque=0,
        )
        comm.queue_third_encoder_request(command=priming_command, valid=0)
        comm.send_processdata()

        while True:
            loop_start_ns = monotonic_time_ns()
            if loop_start_ns >= stop_ns:
                break
            sample_dt_ms = (
                args.cycle_time * 1000.0
                if prev_loop_start_ns is None
                else (loop_start_ns - prev_loop_start_ns) / 1e6
            )
            prev_loop_start_ns = loop_start_ns
            if not recording_started and cycle >= warmup_cycles:
                recording_started = True
                record_cycle_index = 0
                print(
                    f"Warm-up complete after {cycle} cycle(s); formal CSV recording begins now.",
                    flush=True,
                )

            elapsed_s = (loop_start_ns - start_ns) / NSEC_PER_SEC

            # Sample spring encoder
            spring_sample = comm.poll_queued_third_encoder_sample(timeout_us=0)
            spring_sample_time = time.monotonic()
            spring_state_dt_s = (
                spring_sample_time - prev_spring_sample_time
                if prev_spring_sample_time is not None
                else args.cycle_time
            )
            prev_spring_sample_time = spring_sample_time

            valid_third_encoder = spring_sample is not None and (
                spring_sample.response.crc_ok
                and not spring_sample.response.encoder_error
                and not spring_sample.response.communication_alarm
            )

            if spring_sample is None:
                spring_estimate = last_spring_estimate
                spring_vel_rad_s = last_spring_vel_rad_s
            elif not valid_third_encoder:
                print(
                    f"WARNING: invalid third encoder frame at cycle={cycle}, "
                    f"crc_ok={int(spring_sample.response.crc_ok)} "
                    f"enc_err={int(spring_sample.response.encoder_error)} "
                    f"comm_alarm={int(spring_sample.response.communication_alarm)} "
                    f"status=0x{spring_sample.response.status:02X}",
                    flush=True,
                )
                spring_estimate = last_spring_estimate
                spring_vel_rad_s = last_spring_vel_rad_s
            else:
                spring_estimate = spring_estimator.estimate(spring_sample.response.position_21bit)
                spring_vel_rad_s = spring_velocity_rad_s(
                    spring_estimate.delta_theta_rad,
                    prev_spring_delta_theta_rad,
                    spring_state_dt_s,
                )
                prev_spring_delta_theta_rad = spring_estimate.delta_theta_rad
                last_spring_estimate = spring_estimate
                last_spring_vel_rad_s = spring_vel_rad_s

            spring_defl_for_actor = last_spring_estimate.delta_theta_rad
            spring_vel_for_model = last_spring_vel_rad_s
            spring_torque_calibrated_nm = last_spring_estimate.spring_torque_nm

            # Read drive state
            measured_before_raw = comm.read_state()
            measured_before = comm.state_to_si(measured_before_raw)


            # Build reference trajectory
            reference = build_reference(
                elapsed_s,
                initial_position_rad=initial_position_rad,
                initial_motor_load_offset_rad=initial_motor_load_offset_rad,
                amplitude_rad=args.amplitude_rad,
                frequency_hz=args.frequency_hz,
                duration_s=args.duration,
                ramp_in_s=args.reference_ramp_s,
                ramp_out_s=args.reference_ramp_out_s,
            )
            reference_ramp_scale, _ = reference_envelope(
                elapsed_s,
                duration_s=args.duration,
                ramp_in_s=args.reference_ramp_s,
                ramp_out_s=args.reference_ramp_out_s,
            )

            # Build measured state
            measured_state = measured_to_sea_state(
                measured_before,
                prev_encoder2_rad=prev_encoder2_rad,
                dt=args.cycle_time,
            )

            inertia_friction_comp_nm = (
                directional_inertia_friction_compensation_nm(
                    measured_state.dtheta_l_rad_s,
                    inertia_friction_comp_positive_nm,
                    inertia_friction_comp_negative_nm,
                    inertia_friction_comp_velocity_scale_rad_s,
                )
            )
            spring_defl_for_estimator = (
                spring_defl_for_actor
                - inertia_friction_comp_nm / SPRING_STIFFNESS_NM_PER_RAD
            )

            # Keep PPO sensing physical and compensate only estimator history.
            actor_obs = drrma_controller.build_observation(
                pos_error_load=reference.theta_l_rad - measured_state.theta_l_rad,
                vel_error_load=reference.dtheta_l_rad_s - measured_state.dtheta_l_rad_s,
                pos_error_motor=reference.theta_m_rad - measured_state.theta_m_rad,
                vel_error_motor=reference.dtheta_m_rad_s - measured_state.dtheta_m_rad_s,
                spring_defl=spring_defl_for_actor,
                spring_vel=spring_vel_for_model,
                target_pos=reference.theta_l_rad - initial_position_rad,
                target_vel=reference.dtheta_l_rad_s,
            )
            estimator_obs = actor_obs.copy()
            estimator_obs[4] = spring_defl_for_estimator

            # DRRMA inference
            inference_start_ns = monotonic_time_ns()
            (
                action_norm_scalar,
                predicted_torque_nm,
                J_hat_onnx,
                j_raw,
                j_short,
                j_long,
                jump_gate,
                confidence,
            ) = drrma_controller.run_inference(
                actor_obs,
                elapsed_s,
                estimator_sensor_obs_8=estimator_obs,
            )
            inference_ms = (monotonic_time_ns() - inference_start_ns) / 1e6
            scaled_torque_nm = predicted_torque_nm * action_torque_scale
            limited_torque_nm = max(
                min(scaled_torque_nm, args.torque_limit_nm),
                -args.torque_limit_nm,
            )
            applied_torque_nm = limited_torque_nm if execute else 0.0
            target_torque = torque_nm_to_target_units(applied_torque_nm, MAX_TORQUE_NM)

            prev_encoder2_rad = measured_before.encoder2_rad

            # Send command
            comm.queue_third_encoder_request(
                command=DriveCommand(
                    controlword=ENABLE_OPERATION,
                    mode_of_operation=CST_MODE,
                    target_torque=target_torque if execute else 0,
                ),
                valid=cycle & 0xFF,
            )
            comm.send_processdata()

            wkc = spring_sample.wkc if spring_sample is not None else comm.last_processdata_wkc

            if expected_wkc is not None and wkc != expected_wkc:
                print(f"WARNING: wkc={wkc} (expected {expected_wkc})", flush=True)
                if execute and args.strict_wkc:
                    wkc_error_count += 1
                    print(f"  error_count={wkc_error_count}", flush=True)
                    if wkc_error_count >= max_wkc_errors:
                        print("ERROR: Too many wkc errors, stopping control!", flush=True)
                        break
            else:
                wkc_error_count = 0


            # Read state after command
            measured_after_raw = comm.read_state()
            measured_after = comm.state_to_si(measured_after_raw)
            measured_diag = comm.diagnostics_from_state(measured_after_raw)
            motor_velocity_safety_history.append(
                measured_state.dtheta_m_rad_s
            )
            motor_velocity_samples = sorted(motor_velocity_safety_history)
            motor_velocity_safety_rad_s = motor_velocity_samples[
                len(motor_velocity_samples) // 2
            ]
            J_hat_calibrated_unfiltered = J_hat_onnx
            J_hat = J_hat_onnx
            J_hat_calibrator_active = False
            inertia_calibration_ms = 0.0
            if inertia_calibrator is not None:
                calibration_start_ns = monotonic_time_ns()
                (
                    J_hat_calibrated_unfiltered,
                    J_hat,
                    J_hat_calibrator_active,
                ) = inertia_calibrator.update(
                    {
                        "J_hat": J_hat_onnx,
                        "j_long": j_long,
                        "j_short": j_short,
                        "jump_gate": jump_gate,
                        "confidence": confidence,
                        "spring_torque_nm": spring_torque_calibrated_nm,
                        "action_nm": applied_torque_nm,
                        "feedback_torque_nm": measured_after.torque_nm,
                        "meas_vel_l_rad_s": measured_state.dtheta_l_rad_s,
                    },
                    dt_s=args.cycle_time,
                )
                inertia_calibration_ms = (
                    monotonic_time_ns() - calibration_start_ns
                ) / 1e6
            J_hat_physics = J_hat_onnx
            J_hat_physics_active = False
            if physics_inertia_estimator is not None:
                reference_acceleration_rad_s2 = -(
                    2.0 * math.pi * args.frequency_hz
                ) ** 2 * (
                    reference.theta_l_rad - initial_position_rad
                )
                (
                    J_hat_physics,
                    J_hat_physics_active,
                ) = physics_inertia_estimator.update(
                    action_torque_nm=applied_torque_nm,
                    load_velocity_rad_s=measured_state.dtheta_l_rad_s,
                    reference_acceleration_rad_s2=(
                        reference_acceleration_rad_s2
                    ),
                    excitation_valid=reference_ramp_scale >= 0.999,
                )
                if J_hat_physics_active:
                    J_hat = J_hat_physics

            # Safety checks
            err_theta_l = measured_state.theta_l_rad - reference.theta_l_rad
            err_theta_m = measured_state.theta_m_rad - reference.theta_m_rad
            if (
                abs(err_theta_l) > args.max_position_error_rad
                or abs(err_theta_m) > args.max_position_error_rad
            ):
                print(
                    f"ERROR: Position error too large! err_m={err_theta_m:.3f}rad err_l={err_theta_l:.3f}rad",
                    flush=True,
                )
                print("Stopping control for safety!", flush=True)
                break
            if measured_diag.fault:
                print(
                    "ERROR: Drive entered fault during control! statusword=0x{sw:04X} cia402={cia402}".format(
                        sw=measured_diag.statusword,
                        cia402=measured_diag.cia402_state,
                    ),
                    flush=True,
                )
                break
            if abs(measured_state.dtheta_l_rad_s) > args.max_velocity_rad_s:
                print(
                    "ERROR: Load velocity exceeded the configured safety "
                    f"limit: {measured_state.dtheta_l_rad_s:.3f}rad/s.",
                    flush=True,
                )
                break
            if (
                abs(measured_state.dtheta_m_rad_s)
                > args.max_motor_raw_velocity_rad_s
            ):
                print(
                    "ERROR: Raw motor velocity exceeded the emergency safety "
                    f"limit: {measured_state.dtheta_m_rad_s:.3f}rad/s.",
                    flush=True,
                )
                break
            if (
                abs(motor_velocity_safety_rad_s)
                > max_motor_velocity_rad_s
            ):
                print(
                    "ERROR: Median motor velocity exceeded the configured "
                    f"safety limit: {motor_velocity_safety_rad_s:.3f}rad/s "
                    f"(raw={measured_state.dtheta_m_rad_s:.3f}rad/s).",
                    flush=True,
                )
                break
            if abs(spring_estimate.delta_theta_rad) > args.max_spring_deflection_rad:
                print("ERROR: Spring deflection exceeded the configured safety limit.", flush=True)
                break
            if abs(measured_after.torque_nm) > args.max_feedback_torque_nm:
                print("ERROR: Torque feedback exceeded the configured safety limit.", flush=True)
                break
            if execute and measured_diag.warning and not args.allow_drive_warning:
                print("ERROR: Drive warning bit became active during control.", flush=True)
                break

            # Print status
            if args.print_every > 0 and cycle % args.print_every == 0:
                print_state(
                    f"run cycle={cycle} t={elapsed_s:.3f}s wkc={wkc}",
                    measured_after,
                )
                print_drrma_terms(
                    "ctrl",
                    reference,
                    measured_state,
                    spring_defl_rad=spring_defl_for_actor,
                    spring_vel_rad_s=spring_vel_for_model,
                    spring_torque_nm=spring_torque_calibrated_nm,
                    action_norm=action_norm_scalar,
                    predicted_torque_nm=predicted_torque_nm,
                    applied_torque_nm=applied_torque_nm,
                    target_torque=target_torque if execute else 0,
                    measured_torque_nm=measured_after.torque_nm,
                    J_hat=J_hat,
                )
                if spring_sample is not None:
                    print_third_encoder_sample("spring", spring_sample)
                else:
                    print(
                        "spring poll: no fresh frame, using cached estimate "
                        f"delta_theta={last_spring_estimate.delta_theta_rad:.6f}rad "
                        f"spring_vel={last_spring_vel_rad_s:.6f}rad/s",
                        flush=True,
                    )
                print_spring_estimate("spring est", spring_estimate, spring_vel_rad_s)

            # Record data
            if recording_started:
                csv_row = [
                    record_cycle_index,
                    elapsed_s,
                    reference.theta_l_rad,
                    measured_state.theta_l_rad,
                    err_theta_l,
                    reference.theta_m_rad,
                    measured_state.theta_m_rad,
                    err_theta_m,
                    reference.dtheta_l_rad_s,
                    measured_state.dtheta_l_rad_s,
                    reference.dtheta_m_rad_s,
                    measured_state.dtheta_m_rad_s,
                    motor_velocity_safety_rad_s,
                    reference_ramp_scale,
                    spring_estimate.delta_theta_rad,
                    spring_vel_rad_s,
                    spring_estimate.spring_torque_nm,
                    spring_observation_scale,
                    spring_defl_for_actor,
                    spring_vel_for_model,
                    spring_torque_calibrated_nm,
                    inertia_friction_comp_nm,
                    spring_defl_for_estimator,
                    int(valid_third_encoder),
                    action_norm_scalar,
                    predicted_torque_nm,
                    action_torque_scale,
                    scaled_torque_nm,
                    applied_torque_nm,
                    drrma_controller.last_startup_override,
                    J_hat,
                    J_hat_onnx,
                    J_hat_calibrated_unfiltered,
                    int(J_hat_calibrator_active),
                    inertia_calibration_ms,
                    J_hat_physics,
                    int(J_hat_physics_active),
                    j_raw,
                    j_short,
                    j_long,
                    jump_gate,
                    confidence,
                    0.0,  # J_true placeholder (unknown in real system)
                    target_torque if execute else 0,
                    measured_after.torque_nm,
                    measured_diag.statusword,
                    int(measured_diag.warning),
                    wkc,
                    inference_ms,
                    sample_dt_ms,
                ]
                csv_rows.append(csv_row)
                record_cycle_index += 1

            cycle += 1

            # Sleep until next cycle
            cycle_end_ns = monotonic_time_ns()
            next_deadline_ns = cycle_deadline_ns + period_ns
            if cycle_end_ns < next_deadline_ns:
                sleep_until_monotonic_ns(next_deadline_ns)
            cycle_deadline_ns = next_deadline_ns


        # Stop motor
        for _ in range(10):
            command = DriveCommand(
                controlword=ENABLE_OPERATION,
                mode_of_operation=CST_MODE,
                target_torque=0,
            )
            comm.queue_third_encoder_request(command=command, valid=cycle & 0xFF)
            comm.send_processdata()
            wkc = comm.last_processdata_wkc
            state = comm.read_state_si()
        print_state(f"stop wkc={wkc}", state)

        # Save CSV
        if csv_rows:
            with csv_path.open("w", newline="", encoding="utf-8") as csv_file:
                csv_writer = csv.writer(csv_file)
                csv_writer.writerow(csv_fieldnames)
                csv_writer.writerows(csv_rows)
            try:
                os.chmod(csv_path, 0o666)
            except OSError:
                pass
        print("DRRMA control finished.", flush=True)
        print(f"Data saved to: {csv_path}", flush=True)
        return 0
    finally:
        comm.close()


if __name__ == "__main__":
    raise SystemExit(main())
