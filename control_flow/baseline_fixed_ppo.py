#!/usr/bin/env python3
from __future__ import annotations

import argparse
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

import numpy as np
import torch

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
)
from controllers.ppo.ppo_controller import PpoActorController
from controllers.sea_model import SeaState
from controllers.spring_sensor import SpringSensorConfig, SpringSensorEstimator


IFNAME = "eno1"
CYCLE_TIME_S = 0.005
DURATION_S = float(os.environ.get("BASELINE_DURATION_S", "10.0"))
PRINT_EVERY = int(os.environ.get("BASELINE_PRINT_EVERY", "20"))
EXECUTE = False
MAX_TORQUE_NM = 61.0
APPLIED_TORQUE_LIMIT_NM = float(os.environ.get("BASELINE_TORQUE_LIMIT_NM", "61.0"))
ACTION_SCALE = float(os.environ.get("BASELINE_ACTION_SCALE", "1.0"))
MAX_POSITION_ERROR_RAD = float(os.environ.get("BASELINE_MAX_POSITION_ERROR_RAD", "0.8"))
WARMUP_CYCLES = 2
THIRD_ENCODER_COUNTS_PER_REV = 1 << 19
THIRD_ENCODER_SIGN = 1
SPRING_STIFFNESS_NM_PER_RAD = 2400.0
CALIBRATION_SAMPLES = 8
TRAJECTORY_MODE = "single"  # "single" or "composite"
TRAJECTORY_AMPLITUDE_RAD = float(os.environ.get("BASELINE_AMPLITUDE_RAD", "0.2"))
TRAJECTORY_FREQUENCY_HZ = float(os.environ.get("BASELINE_FREQUENCY_HZ", "2.5"))
TRAJECTORY_PHASE_RAD = 0.0
TRAJECTORY_COMPOSITE_BIAS_RAD = 0.1
# TRAJECTORY_COMPOSITE_COMPONENTS = (
#     (0.30, 3.0, 0.0),
#     (0.15, 1.0, 0.0),
#     (0.45, 6.0, 0.0),
#     (0.20, 0.5, 0.0),
# )
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
        torch.set_num_threads(1)
    except Exception:
        pass

    try:
        torch.set_num_interop_threads(1)
    except Exception:
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


def resolve_csv_path(csv_path: str | Path | None) -> Path:
    if csv_path is not None:
        path = Path(csv_path)
    else:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        log_dir = Path(__file__).resolve().parents[1] / "logs"
        path = log_dir / f"ppo_data_{timestamp}.csv"
    if path.suffix.lower() != ".csv":
        path = path.with_suffix(".csv")
    return path


def build_reference(
    elapsed_s: float,
    initial_position_rad: float,
    initial_motor_load_offset_rad: float,
) -> SeaState:
    if TRAJECTORY_MODE == "single":
        angle = 2.0 * math.pi * TRAJECTORY_FREQUENCY_HZ * elapsed_s + TRAJECTORY_PHASE_RAD
        theta_l = initial_position_rad + TRAJECTORY_AMPLITUDE_RAD * math.sin(angle)
        dtheta_l = TRAJECTORY_AMPLITUDE_RAD * 2.0 * math.pi * TRAJECTORY_FREQUENCY_HZ * math.cos(angle)
    elif TRAJECTORY_MODE == "composite":
        theta_l = initial_position_rad + TRAJECTORY_COMPOSITE_BIAS_RAD
        dtheta_l = 0.0
        for amplitude_rad, angular_frequency_rad_s, phase_rad in TRAJECTORY_COMPOSITE_COMPONENTS:
            angle = angular_frequency_rad_s * elapsed_s + phase_rad
            theta_l += amplitude_rad * math.sin(angle)
            dtheta_l += amplitude_rad * angular_frequency_rad_s * math.cos(angle)
    else:
        raise ValueError(f"Unsupported TRAJECTORY_MODE={TRAJECTORY_MODE!r}; use 'single' or 'composite'.")
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


def build_observation(
    controller: PpoActorController,
    reference: SeaState,
    measured: SeaState,
    spring_defl_rad: float,
    spring_vel_rad_s: float,
    reference_origin_rad: float = 0.0,
    out: np.ndarray | None = None,
) -> np.ndarray:
    pos_error_load = reference.theta_l_rad - measured.theta_l_rad
    vel_error_load = reference.dtheta_l_rad_s - measured.dtheta_l_rad_s
    pos_error_motor = reference.theta_m_rad - measured.theta_m_rad
    vel_error_motor = reference.dtheta_m_rad_s - measured.dtheta_m_rad_s
    target_pos_for_actor = reference.theta_l_rad - reference_origin_rad
    return controller.build_observation(
        pos_error_load=pos_error_load,
        vel_error_load=vel_error_load,
        pos_error_motor=pos_error_motor,
        vel_error_motor=vel_error_motor,
        spring_defl=spring_defl_rad,
        spring_vel=spring_vel_rad_s,
        target_pos=target_pos_for_actor,
        target_vel=reference.dtheta_l_rad_s,
        out=out,
    )


def print_ppo_terms(
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
            f"send_action={applied_torque_nm:.3f}Nm target={target_torque} feedback_torque={measured_torque_nm:.3f}Nm"
        ),
        flush=True,
    )


def print_timing_terms(
    prefix: str,
    read_before_dt_s: float,
    actor_dt_s: float,
    sample_dt_s: float,
    send_dt_s: float,
    read_after_dt_s: float,
    diag_dt_s: float,
    csv_flush_dt_s: float,
    cycle_total_dt_s: float,
) -> None:
    print(
        (
            f"{prefix} read_before={read_before_dt_s*1000.0:.3f}ms "
            f"actor={actor_dt_s*1000.0:.3f}ms sample={sample_dt_s*1000.0:.3f}ms "
            f"send={send_dt_s*1000.0:.3f}ms "
            f"read_after={read_after_dt_s*1000.0:.3f}ms diag={diag_dt_s*1000.0:.3f}ms "
            f"csv_flush={csv_flush_dt_s*1000.0:.3f}ms total={cycle_total_dt_s*1000.0:.3f}ms"
        ),
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
        except Exception as exc:
            last_error = exc
            print(f"[zero] attempt={attempt} failed: {exc}", flush=True)
            continue
        zero_samples.append(raw_counts)
    if len(zero_samples) < calibration_samples:
        raise RuntimeError(
            f"Failed to collect enough valid zero samples. got={len(zero_samples)} need={calibration_samples} "
            f"last_error={last_error}"
        )
    return int(round(sum(zero_samples) / len(zero_samples)))


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


def main() -> int:
    parser = argparse.ArgumentParser(description="SEA PPO control flow.")
    parser.add_argument("--ifname", default=IFNAME)
    parser.add_argument("--duration", type=float, default=DURATION_S)
    parser.add_argument("--print-every", type=int, default=PRINT_EVERY)
    parser.add_argument("--cycle-time", type=float, default=CYCLE_TIME_S)
    parser.add_argument(
        "--warmup-cycles",
        type=int,
        default=WARMUP_CYCLES,
        help="Number of control cycles to run before starting formal CSV recording.",
    )
    parser.add_argument("--csv-path", type=str, default=None, help="Optional CSV output path.")
    parser.add_argument(
        "--execute",
        action=argparse.BooleanOptionalAction,
        default=EXECUTE,
        help="Send PPO torque commands; use --no-execute for zero-torque checks.",
    )
    parser.add_argument("--strict-wkc", action="store_true", help="Stop on repeated WKC mismatch, like PD-DOB.")
    args = parser.parse_args()
    warmup_cycles = max(int(args.warmup_cycles), 0)

    configure_realtime_runtime()
    try:
        current_affinity = sorted(os.sched_getaffinity(0))
    except Exception:
        current_affinity = []
    print(
        f"Deadline scheduling enabled: cycle={args.cycle_time*1000.0:.3f}ms affinity={current_affinity}",
        flush=True,
    )
    print(f"Warm-up before CSV recording: {warmup_cycles} cycle(s)", flush=True)
    controller = PpoActorController()
    comm = SEARealtimeComm(ifname=args.ifname, cycle_time_s=args.cycle_time)
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
    obs_raw = np.empty((9,), dtype=np.float32)
    csv_rows: list[list[object]] = []
    csv_fieldnames = [
        "cycle_index",
        "time_s",
        "wake_lag_ms",
        "deadline_slip_ms",
        "sleep_target_ms",
        "sleep_actual_ms",
        "deadline_miss_delta",
        "deadline_miss_count",
        "ref_theta_m_rad",
        "ref_theta_l_rad",
        "ref_dtheta_m_rad_s",
        "ref_dtheta_l_rad_s",
        "meas_theta_m_rad",
        "meas_theta_l_rad",
        "meas_dtheta_m_rad_s",
        "meas_dtheta_l_rad_s",
        "err_theta_m_rad",
        "err_theta_l_rad",
        "err_dtheta_m_rad_s",
        "err_dtheta_l_rad_s",
        "spring_defl_rad",
        "spring_vel_rad_s",
        "spring_torque_nm",
        "action_norm",
        "raw_action_nm",
        "action_nm",
        "target_torque",
        "raw_action_delta_nm",
        "wkc",
        "read_before_dt_s",
        "actor_dt_s",
        "measured_state_dt_s",
        "sample_dt_s",
        "send_dt_s",
        "spring_state_dt_s",
        "read_after_dt_s",
        "diag_dt_s",
        "third_encoder_pos21",
        "third_encoder_crc_ok",
        "third_encoder_enc_err",
        "third_encoder_comm_alarm",
        "statusword",
        "position_rad",
        "velocity_rad_s",
        "torque_nm",
        "encoder1_rad",
        "encoder2_rad",
    ]

    try:
        comm.connect()
        print_slave_inventory(comm)
        print(
            f"Connected on {args.ifname}. PPO mode={CST_MODE}, execute={execute}, "
            f"duration={args.duration}s actor={controller.actor_path}",
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
            raise RuntimeError("PDO preheat failed before CiA 402 enable")

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
            f"counts_per_rev={THIRD_ENCODER_COUNTS_PER_REV} spring_stiffness={SPRING_STIFFNESS_NM_PER_RAD}",
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
                f"amplitude={TRAJECTORY_AMPLITUDE_RAD:.6f}rad "
                f"frequency={TRAJECTORY_FREQUENCY_HZ:.6f}Hz "
                f"phase={TRAJECTORY_PHASE_RAD:.6f}rad"
            )
        print(
            (
                "PPO sine reference initialized: "
                f"mode={TRAJECTORY_MODE} "
                f"initial_position={initial_position_rad:.6f}rad "
                f"initial_motor_load_offset={initial_motor_load_offset_rad:.6f}rad "
                f"{trajectory_description}"
            ),
            flush=True,
        )

        csv_path.parent.mkdir(parents=True, exist_ok=True)
        try:
            os.chmod(csv_path.parent, 0o777)
        except OSError:
            pass
        print(f"Will save CSV to: {csv_path}", flush=True)

        print("Stage 6: PPO control loop", flush=True)
        start_ns = monotonic_time_ns()
        start = start_ns / NSEC_PER_SEC
        stop_ns = start_ns + max(int(round(args.duration * NSEC_PER_SEC)), 0)
        period_ns = normalize_cycle_time_ns(args.cycle_time)
        cycle_deadline_ns = start_ns
        recording_started = warmup_cycles == 0
        deadline_miss_count = 0
        cycle = 0
        record_cycle_index = 0
        last_state = None
        expected_wkc = comm.expected_wkc
        wkc_error_count = 0
        max_wkc_errors = 3
        prev_deadline_miss_count = 0
        last_spring_estimate = spring_estimator.estimate(zero_counts)
        last_spring_vel_rad_s = 0.0
        prev_measured_after_time: float | None = None
        prev_spring_sample_time: float | None = None
        prev_predicted_torque_nm = 0.0

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
            if not recording_started and cycle >= warmup_cycles:
                recording_started = True
                deadline_miss_count = 0
                prev_deadline_miss_count = 0
                record_cycle_index = 0
                print(
                    f"Warm-up complete after {cycle} cycle(s); formal CSV recording begins now.",
                    flush=True,
                )
            wake_lag_ms = max(0.0, (loop_start_ns - cycle_deadline_ns) / 1_000_000.0)
            deadline_slip_ms = wake_lag_ms
            cycle_total_start = time.monotonic()
            elapsed_s = (loop_start_ns - start_ns) / NSEC_PER_SEC
            sample_receive_start = time.monotonic()
            spring_sample = comm.poll_queued_third_encoder_sample(timeout_us=0)
            sample_dt_s = time.monotonic() - sample_receive_start
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

            read_before_start = time.monotonic()
            measured_before_raw = comm.read_state()
            measured_before = comm.state_to_si(measured_before_raw)
            read_before_dt_s = time.monotonic() - read_before_start
            measured_before_time = time.monotonic()
            measured_state_dt_s = (
                measured_before_time - prev_measured_after_time
                if prev_measured_after_time is not None
                else args.cycle_time
            )
            reference = build_reference(
                elapsed_s,
                initial_position_rad=initial_position_rad,
                initial_motor_load_offset_rad=initial_motor_load_offset_rad,
            )
            measured_state = measured_to_sea_state(
                measured_before,
                prev_encoder2_rad=prev_encoder2_rad,
                dt=measured_state_dt_s,
            )
            obs_raw = build_observation(
                controller,
                reference,
                measured_state,
                spring_defl_rad=last_spring_estimate.delta_theta_rad,
                spring_vel_rad_s=last_spring_vel_rad_s,
                reference_origin_rad=initial_position_rad,
                out=obs_raw,
            )
            actor_start = time.monotonic()
            action_norm, predicted_torque_nm = controller.run_actor(obs_raw)
            actor_dt_s = time.monotonic() - actor_start
            raw_action_delta_nm = predicted_torque_nm - prev_predicted_torque_nm
            prev_predicted_torque_nm = predicted_torque_nm
            applied_torque_nm = (
                float(np.clip(ACTION_SCALE * predicted_torque_nm, -APPLIED_TORQUE_LIMIT_NM, APPLIED_TORQUE_LIMIT_NM))
                if execute
                else 0.0
            )
            target_torque = torque_nm_to_target_units(applied_torque_nm, MAX_TORQUE_NM)

            prev_encoder2_rad = measured_before.encoder2_rad
            prev_measured_after_time = measured_before_time
            queue_start = time.monotonic()
            comm.queue_third_encoder_request(
                command=DriveCommand(
                    controlword=ENABLE_OPERATION,
                    mode_of_operation=CST_MODE,
                    target_torque=target_torque if execute else 0,
                ),
                valid=cycle & 0xFF,
            )
            comm.send_processdata()
            send_dt_s = time.monotonic() - queue_start

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

            read_after_start = time.monotonic()
            measured_after_raw = comm.read_state()
            measured_after = comm.state_to_si(measured_after_raw)
            read_after_dt_s = time.monotonic() - read_after_start
            diag_start = time.monotonic()
            measured_diag = comm.diagnostics_from_state(measured_after_raw)
            diag_dt_s = time.monotonic() - diag_start
            last_state = measured_after
            err_theta_l = measured_state.theta_l_rad - reference.theta_l_rad
            err_theta_m = measured_state.theta_m_rad - reference.theta_m_rad
            if abs(err_theta_l) > MAX_POSITION_ERROR_RAD or abs(err_theta_m) > MAX_POSITION_ERROR_RAD:
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

            if args.print_every > 0 and cycle % args.print_every == 0:
                print_state(
                    f"run cycle={cycle} t={elapsed_s:.3f}s wkc={wkc} sample_dt={sample_dt_s*1000.0:.3f}ms",
                    measured_after,
                )
                print_ppo_terms(
                    "ctrl",
                    reference,
                    measured_state,
                    spring_defl_rad=spring_estimate.delta_theta_rad,
                    spring_vel_rad_s=spring_vel_rad_s,
                    spring_torque_nm=spring_estimate.spring_torque_nm,
                    action_norm=action_norm,
                    predicted_torque_nm=predicted_torque_nm,
                    applied_torque_nm=applied_torque_nm,
                    target_torque=target_torque if execute else 0,
                    measured_torque_nm=measured_after.torque_nm,
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
                print(
                    (
                        f"motion timing measured_dt={measured_state_dt_s*1000.0:.3f}ms "
                        f"spring_dt={spring_state_dt_s*1000.0:.3f}ms "
                        f"send_dt={send_dt_s*1000.0:.3f}ms "
                        f"raw_action_delta={raw_action_delta_nm:.3f}Nm"
                    ),
                    flush=True,
                )

            csv_buffer_dt_s = 0.0
            csv_row: list[object] | None = None
            if recording_started:
                csv_write_start = time.monotonic()
                csv_row = [
                    record_cycle_index,
                    elapsed_s,
                    wake_lag_ms,
                    deadline_slip_ms,
                    0.0,
                    0.0,
                    0,
                    deadline_miss_count,
                    reference.theta_m_rad,
                    reference.theta_l_rad,
                    reference.dtheta_m_rad_s,
                    reference.dtheta_l_rad_s,
                    measured_after.encoder2_rad,
                    measured_after.position_rad,
                    measured_state.dtheta_m_rad_s,
                    measured_after.velocity_rad_s,
                    reference.theta_m_rad - measured_state.theta_m_rad,
                    reference.theta_l_rad - measured_state.theta_l_rad,
                    reference.dtheta_m_rad_s - measured_state.dtheta_m_rad_s,
                    reference.dtheta_l_rad_s - measured_state.dtheta_l_rad_s,
                    spring_estimate.delta_theta_rad,
                    spring_vel_rad_s,
                    spring_estimate.spring_torque_nm,
                    action_norm,
                    predicted_torque_nm,
                    applied_torque_nm,
                    target_torque if execute else 0,
                    raw_action_delta_nm,
                    wkc,
                    read_before_dt_s,
                    actor_dt_s,
                    measured_state_dt_s,
                    sample_dt_s,
                    send_dt_s,
                    spring_state_dt_s,
                    read_after_dt_s,
                    diag_dt_s,
                    spring_sample.response.position_21bit if spring_sample is not None else -1,
                    int(spring_sample.response.crc_ok) if spring_sample is not None else 0,
                    int(spring_sample.response.encoder_error) if spring_sample is not None else 1,
                    int(spring_sample.response.communication_alarm) if spring_sample is not None else 1,
                    f"0x{measured_after.statusword:04X}",
                    measured_after.position_rad,
                    measured_after.velocity_rad_s,
                    measured_after.torque_nm,
                    measured_after.encoder1_rad,
                    measured_after.encoder2_rad,
                ]
                csv_buffer_dt_s = time.monotonic() - csv_write_start
            cycle_total_dt_s = time.monotonic() - cycle_total_start

            if args.print_every > 0 and cycle % args.print_every == 0:
                print_timing_terms(
                    "timing",
                    read_before_dt_s=read_before_dt_s,
                    actor_dt_s=actor_dt_s,
                    sample_dt_s=sample_dt_s,
                    send_dt_s=send_dt_s,
                    read_after_dt_s=read_after_dt_s,
                    diag_dt_s=diag_dt_s,
                    csv_flush_dt_s=csv_buffer_dt_s,
                    cycle_total_dt_s=cycle_total_dt_s,
                )

            cycle += 1

            cycle_end_ns = monotonic_time_ns()
            next_deadline_ns = cycle_deadline_ns + period_ns
            sleep_target_ms = 0.0
            sleep_actual_ms = 0.0
            if cycle_end_ns > next_deadline_ns:
                missed_cycles = ((cycle_end_ns - next_deadline_ns) // period_ns) + 1
                deadline_miss_count += missed_cycles
                next_deadline_ns += missed_cycles * period_ns
            elif cycle_end_ns < next_deadline_ns:
                sleep_target_ms = (next_deadline_ns - cycle_end_ns) / 1_000_000.0
                sleep_start_ns = monotonic_time_ns()
                sleep_until_monotonic_ns(next_deadline_ns)
                sleep_end_ns = monotonic_time_ns()
                sleep_actual_ms = max(0.0, (sleep_end_ns - sleep_start_ns) / 1_000_000.0)
            cycle_deadline_ns = next_deadline_ns
            deadline_miss_delta = deadline_miss_count - prev_deadline_miss_count
            prev_deadline_miss_count = deadline_miss_count
            if csv_row is not None:
                csv_row[3] = deadline_slip_ms
                csv_row[4] = sleep_target_ms
                csv_row[5] = sleep_actual_ms
                csv_row[6] = deadline_miss_delta
                csv_row[7] = deadline_miss_count
                csv_rows.append(csv_row)
                record_cycle_index += 1

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
        if last_state is not None:
            print_state(f"stop wkc={wkc}", state)

        if csv_rows:
            with csv_path.open("w", newline="", encoding="utf-8") as csv_file:
                csv_writer = csv.writer(csv_file)
                csv_writer.writerow(csv_fieldnames)
                csv_writer.writerows(csv_rows)
            try:
                os.chmod(csv_path, 0o666)
            except OSError:
                pass
        if deadline_miss_count:
            print(f"Formal deadline misses (warm-up excluded): {deadline_miss_count}", flush=True)
        print("PPO control finished.", flush=True)
        print(f"Data saved to: {csv_path}", flush=True)
        return 0
    finally:
        comm.close()


if __name__ == "__main__":
    raise SystemExit(main())
