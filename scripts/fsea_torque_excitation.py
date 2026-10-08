#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import ctypes
import errno
import math
import os
import resource
import subprocess
import sys
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

if __package__ in {None, ""}:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


IFNAME = "eno1"
CYCLE_TIME_S = 0.005
DURATION_S = 20.0
MAX_TORQUE_NM = 61.0
SPRING_STIFFNESS_NM_PER_RAD = 3800.0
THIRD_ENCODER_COUNTS_PER_REV = 1 << 19
# Sweep/plant-identification convention: positive spring torque follows positive
# action_nm.  The existing control flows keep their own sign constants.
THIRD_ENCODER_SIGN = -1
CALIBRATION_SAMPLES = 16
PRINT_EVERY = 50
REALTIME_CPUS = (8, 9, 10, 11)
REALTIME_RT_PRIORITY = 99
CLOCK_MONOTONIC = 1
TIMER_ABSTIME = 1
NSEC_PER_SEC = 1_000_000_000
PROJECT_DIR = Path(__file__).resolve().parents[1]
CST_MODE = 10
SHUTDOWN = 0x0006
SWITCH_ON = 0x0007
ENABLE_OPERATION = 0x000F
DriveCommand = None
SEARealtimeComm = None
SpringSensorConfig = None
SpringSensorEstimator = None


def summarize_wkc(samples) -> str:
    if not samples:
        return "no samples"
    return f"min={min(samples)} max={max(samples)} avg={sum(samples) / len(samples):.2f}"


def load_hardware_runtime() -> None:
    global CST_MODE
    global ENABLE_OPERATION
    global DriveCommand
    global SEARealtimeComm
    global SHUTDOWN
    global SWITCH_ON
    global SpringSensorConfig
    global SpringSensorEstimator
    global summarize_wkc

    from communication.sea_motor_comm import (
        CST_MODE as comm_cst_mode,
        ENABLE_OPERATION as comm_enable_operation,
        DriveCommand as comm_drive_command,
        SEARealtimeComm as comm_realtime_comm,
        SHUTDOWN as comm_shutdown,
        SWITCH_ON as comm_switch_on,
        summarize_wkc as comm_summarize_wkc,
    )
    from controllers.spring_sensor import (
        SpringSensorConfig as spring_sensor_config,
        SpringSensorEstimator as spring_sensor_estimator,
    )

    CST_MODE = comm_cst_mode
    ENABLE_OPERATION = comm_enable_operation
    DriveCommand = comm_drive_command
    SEARealtimeComm = comm_realtime_comm
    SHUTDOWN = comm_shutdown
    SWITCH_ON = comm_switch_on
    SpringSensorConfig = spring_sensor_config
    SpringSensorEstimator = spring_sensor_estimator
    summarize_wkc = comm_summarize_wkc


@dataclass(frozen=True)
class ExcitationConfig:
    mode: str
    amplitude_nm: float
    bias_nm: float
    f_min_hz: float
    f_max_hz: float
    components: tuple[float, ...]


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
        if libc.mlockall(1 | 2) != 0:
            err = ctypes.get_errno()
            raise OSError(err, os.strerror(err))
    except Exception:
        pass
    try:
        os.sched_setscheduler(0, os.SCHED_FIFO, os.sched_param(rt_priority))
    except Exception:
        pass


def normalize_cycle_time_ns(cycle_time_s: float) -> int:
    return max(int(round(cycle_time_s * NSEC_PER_SEC)), 1)


def torque_nm_to_target_units(torque_nm: float, max_torque_nm: float) -> int:
    if max_torque_nm <= 0.0:
        raise ValueError("max_torque_nm must be positive")
    target = round(torque_nm / max_torque_nm * 1000.0)
    return int(max(min(target, 1000), -1000))


def clamp(value: float, limit: float) -> float:
    return max(min(value, limit), -limit)


def chirp_command(t: float, duration: float, cfg: ExcitationConfig) -> float:
    if duration <= 0.0:
        return cfg.bias_nm
    ratio = max(min(t / duration, 1.0), 0.0)
    if cfg.f_min_hz <= 0.0 or cfg.f_max_hz <= cfg.f_min_hz:
        freq = cfg.f_min_hz
        phase = 2.0 * math.pi * freq * t
    else:
        log_k = math.log(cfg.f_max_hz / cfg.f_min_hz)
        phase = 2.0 * math.pi * cfg.f_min_hz * duration * (math.exp(log_k * ratio) - 1.0) / log_k
    return cfg.bias_nm + cfg.amplitude_nm * math.sin(phase)


def multisine_command(t: float, cfg: ExcitationConfig) -> float:
    components = cfg.components
    if not components:
        components = (0.4, 0.8, 1.5, 3.0, 6.0, 10.0, 14.0)
    # Treat --amplitude-nm as the conservative peak bound of the summed command.
    # This keeps a nominal "5 Nm" multisine from producing much larger peaks.
    per_component_amp = cfg.amplitude_nm / len(components)
    value = cfg.bias_nm
    for idx, freq_hz in enumerate(components):
        phase = (idx * 2.399963229728653) % (2.0 * math.pi)
        value += per_component_amp * math.sin(2.0 * math.pi * freq_hz * t + phase)
    return value


def excitation_command(t: float, duration: float, cfg: ExcitationConfig) -> float:
    if cfg.mode == "chirp":
        return chirp_command(t, duration, cfg)
    if cfg.mode == "multisine":
        return multisine_command(t, cfg)
    raise ValueError(f"Unsupported mode={cfg.mode!r}")


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


def warmup_third_encoder(comm: SEARealtimeComm, warmup_cycles: int, required_valid: int) -> None:
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
            if attempt < 10 or valid_count + 1 >= required_valid:
                print_third_encoder_sample(prefix=f"[warmup] attempt={attempt}", sample=sample)
            valid_count += 1
            if valid_count >= required_valid:
                return
        except Exception as exc:
            print(f"[warmup] attempt={attempt} failed: {exc}", flush=True)
    raise RuntimeError(
        f"Third encoder warm-up failed: only got {valid_count} valid frames in {warmup_cycles} attempts"
    )


def collect_zero_counts(comm: SEARealtimeComm, sample_count: int) -> int:
    zero_samples: list[int] = []
    attempt_limit = max(sample_count * 20, 100)
    last_error: Exception | None = None
    for attempt in range(attempt_limit):
        if len(zero_samples) >= sample_count:
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
            print_third_encoder_sample(prefix=f"[zero] attempt={attempt}", sample=sample)
        except Exception as exc:
            last_error = exc
            print(f"[zero] attempt={attempt} failed: {exc}", flush=True)
            continue
        zero_samples.append(raw_counts)
    if len(zero_samples) < sample_count:
        raise RuntimeError(
            f"Failed to collect enough valid zero samples. got={len(zero_samples)} need={sample_count} "
            f"last_error={last_error}"
        )
    return round(sum(zero_samples) / len(zero_samples))


def spring_velocity_rad_s(
    current_delta_theta_rad: float,
    previous_delta_theta_rad: float | None,
    dt: float | None,
) -> float:
    if previous_delta_theta_rad is None or dt is None or dt <= 0.0:
        return 0.0
    return (current_delta_theta_rad - previous_delta_theta_rad) / dt


def enable_drive_simple(comm: SEARealtimeComm, cycle_time_s: float) -> None:
    del cycle_time_s
    for controlword in (SHUTDOWN, SWITCH_ON, ENABLE_OPERATION):
        for _ in range(10):
            wkc = comm.cycle(
                DriveCommand(
                    controlword=controlword,
                    mode_of_operation=CST_MODE,
                    target_torque=0,
                )
            )
            state = comm.read_state_si()
        print_state(f"enable cw=0x{controlword:04X} wkc={wkc}", state)


def enable_drive_full(comm: SEARealtimeComm) -> None:
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


def resolve_csv_path(csv_path: str | Path | None) -> Path:
    if csv_path is not None:
        path = Path(csv_path)
    else:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        path = Path(__file__).resolve().parents[1] / "logs" / f"fsea_excitation_{timestamp}.csv"
    if path.suffix.lower() != ".csv":
        path = path.with_suffix(".csv")
    return path


def run_post_analysis(
    csv_path: Path,
    load_mass_each_kg: float,
    load_count: int,
    load_radius_m: float,
    load_layout: str,
    static_friction_nm: float,
) -> None:
    env = os.environ.copy()
    env.setdefault("MPLBACKEND", "Agg")
    time_dir = csv_path.parent / "time_domain"
    freq_dir = csv_path.parent / "frequency"
    commands = [
        [
            sys.executable,
            str(PROJECT_DIR / "analysis" / "plot_fsea_excitation_results.py"),
            str(csv_path),
            "--output-dir",
            str(time_dir),
        ],
        [
            sys.executable,
            str(PROJECT_DIR / "temp" / "fsea_joint_frequency_analysis.py"),
            "--csv",
            str(csv_path),
            "--input-col",
            "action_nm",
            "--output-col",
            "spring_torque_nm",
            "--out-dir",
            str(freq_dir),
            "--load-mass-each-kg",
            str(load_mass_each_kg),
            "--load-count",
            str(load_count),
            "--load-radius-m",
            str(load_radius_m),
            "--load-layout",
            load_layout,
            "--static-friction-nm",
            str(static_friction_nm),
        ],
    ]
    for command in commands:
        print("+ " + " ".join(command), flush=True)
        subprocess.run(command, cwd=PROJECT_DIR.parent, check=True, env=env)


def parse_components(value: str) -> tuple[float, ...]:
    if not value.strip():
        return ()
    components = tuple(float(item.strip()) for item in value.split(",") if item.strip())
    if any(freq <= 0.0 for freq in components):
        raise argparse.ArgumentTypeError("all component frequencies must be positive")
    return components


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Collect FSEA plant torque-excitation data in CST mode.")
    parser.add_argument("--ifname", default=IFNAME)
    parser.add_argument("--duration", type=float, default=DURATION_S)
    parser.add_argument("--cycle-time", type=float, default=CYCLE_TIME_S)
    parser.add_argument("--mode", choices=("multisine", "chirp"), default="multisine")
    parser.add_argument("--amplitude-nm", type=float, default=5.0)
    parser.add_argument("--bias-nm", type=float, default=0.0)
    parser.add_argument("--max-command-nm", type=float, default=MAX_TORQUE_NM)
    parser.add_argument("--f-min", type=float, default=0.2)
    parser.add_argument("--f-max", type=float, default=14.0)
    parser.add_argument(
        "--components",
        type=parse_components,
        default=parse_components("0.4,0.8,1.5,3,6,10,14"),
        help="Comma-separated multisine component frequencies in Hz.",
    )
    parser.add_argument("--csv-path", type=str, default=None)
    parser.add_argument("--print-every", type=int, default=PRINT_EVERY)
    parser.add_argument("--calibration-samples", type=int, default=CALIBRATION_SAMPLES)
    parser.add_argument("--warmup-cycles", type=int, default=100)
    parser.add_argument("--warmup-valid", type=int, default=5)
    parser.add_argument(
        "--max-invalid-third-frames",
        type=int,
        default=20,
        help="Stop excitation after this many consecutive invalid/missing third-encoder frames.",
    )
    parser.add_argument(
        "--startup-mode",
        choices=("simple", "full"),
        default="simple",
        help="simple follows sea_motor_cst_demo.py; full uses preheat/CiA402 helpers.",
    )
    parser.add_argument("--strict-wkc", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--execute", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument(
        "--post-analyze",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Generate time-domain and frequency-domain plots after writing CSV.",
    )
    parser.add_argument("--load-mass-each-kg", type=float, default=1.0)
    parser.add_argument("--load-count", type=int, default=2)
    parser.add_argument("--load-radius-m", type=float, default=0.13)
    parser.add_argument(
        "--load-layout",
        choices=("none", "symmetric", "same_side", "unknown"),
        default="symmetric",
        help="Load arrangement metadata passed to the post-analysis report.",
    )
    parser.add_argument("--static-friction-nm", type=float, default=8.0)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    if args.duration <= 0.0:
        raise ValueError("--duration must be positive")
    if args.cycle_time <= 0.0:
        raise ValueError("--cycle-time must be positive")
    if args.max_command_nm <= 0.0:
        raise ValueError("--max-command-nm must be positive")
    if args.amplitude_nm < 0.0:
        raise ValueError("--amplitude-nm must be non-negative")
    if args.load_mass_each_kg < 0.0:
        raise ValueError("--load-mass-each-kg must be non-negative")
    if args.load_count < 0:
        raise ValueError("--load-count must be non-negative")
    if args.load_radius_m < 0.0:
        raise ValueError("--load-radius-m must be non-negative")
    if args.static_friction_nm < 0.0:
        raise ValueError("--static-friction-nm must be non-negative")
    if abs(args.bias_nm) + args.amplitude_nm > args.max_command_nm:
        raise ValueError(
            "abs(--bias-nm) + --amplitude-nm must not exceed --max-command-nm; "
            "reduce the excitation or increase the explicit limit"
        )

    load_hardware_runtime()
    configure_realtime_runtime()
    try:
        current_affinity = sorted(os.sched_getaffinity(0))
    except Exception:
        current_affinity = []
    csv_path = resolve_csv_path(args.csv_path)
    cfg = ExcitationConfig(
        mode=args.mode,
        amplitude_nm=args.amplitude_nm,
        bias_nm=args.bias_nm,
        f_min_hz=args.f_min,
        f_max_hz=args.f_max,
        components=args.components,
    )
    print(
        (
            f"FSEA torque excitation: ifname={args.ifname} execute={args.execute} "
            f"mode={cfg.mode} duration={args.duration:.3f}s cycle={args.cycle_time*1000.0:.3f}ms "
            f"amplitude={cfg.amplitude_nm:.3f}Nm bias={cfg.bias_nm:.3f}Nm "
            f"spring_sign={THIRD_ENCODER_SIGN:+d} "
            f"max_command={args.max_command_nm:.3f}Nm static_friction={args.static_friction_nm:.3f}Nm "
            f"load={args.load_count}x{args.load_mass_each_kg:.3f}kg@{args.load_radius_m:.3f}m "
            f"layout={args.load_layout} affinity={current_affinity}"
        ),
        flush=True,
    )
    if cfg.mode == "multisine":
        print(f"Multisine components Hz: {', '.join(f'{freq:g}' for freq in cfg.components)}", flush=True)
    else:
        print(f"Chirp range: {cfg.f_min_hz:g} Hz -> {cfg.f_max_hz:g} Hz", flush=True)

    comm = SEARealtimeComm(ifname=args.ifname, cycle_time_s=args.cycle_time)
    spring_estimator = SpringSensorEstimator(
        SpringSensorConfig(
            counts_per_rev=THIRD_ENCODER_COUNTS_PER_REV,
            zero_counts=0,
            sign=THIRD_ENCODER_SIGN,
            spring_stiffness_nm_per_rad=SPRING_STIFFNESS_NM_PER_RAD,
        )
    )

    rows: list[list[object]] = []
    fieldnames = [
        "cycle_index",
        "time_s",
        "command_torque_nm",
        "action_nm",
        "target_torque",
        "spring_delta_theta_rad",
        "spring_torque_nm",
        "spring_vel_rad_s",
        "meas_theta_m_rad",
        "meas_theta_l_rad",
        "meas_dtheta_m_rad_s",
        "meas_dtheta_l_rad_s",
        "position_rad",
        "velocity_rad_s",
        "torque_nm",
        "encoder1_rad",
        "encoder2_rad",
        "wkc",
        "expected_wkc",
        "statusword",
        "mode_display",
        "third_encoder_pos21",
        "third_encoder_crc_ok",
        "third_encoder_enc_err",
        "third_encoder_comm_alarm",
        "wake_lag_ms",
        "deadline_slip_ms",
        "sleep_target_ms",
        "sleep_actual_ms",
        "deadline_miss_delta",
        "deadline_miss_count",
        "sample_dt_s",
        "read_before_dt_s",
        "send_dt_s",
        "read_after_dt_s",
        "cycle_total_dt_s",
    ]

    try:
        comm.connect()
        print_slave_inventory(comm)

        print("Stage 0.5: box transparent PRE-OP configuration", flush=True)
        box_cfg = comm.configure_box_transparent_preop()
        print(
            "Third encoder transparent configured: "
            f"mode={box_cfg['mode']} interface={box_cfg['interface']} "
            f"frame={box_cfg['frame']} baud_selector={box_cfg['baud_selector']} "
            f"baud={box_cfg['explicit_baud']} polling_ms={box_cfg['polling_ms']}",
            flush=True,
        )

        print(f"Stage 1: drive enable ({args.startup_mode})", flush=True)
        if args.startup_mode == "full":
            enable_drive_full(comm)
        else:
            enable_drive_simple(comm, args.cycle_time)

        print("Stage 2: third encoder initialization", flush=True)
        warmup_third_encoder(comm, warmup_cycles=args.warmup_cycles, required_valid=args.warmup_valid)

        print("Stage 3: zero calibration", flush=True)
        zero_counts = collect_zero_counts(comm, args.calibration_samples)
        spring_estimator.tare_zero(zero_counts)
        print(
            f"Calibration done: zero_counts={zero_counts} "
            f"counts_per_rev={THIRD_ENCODER_COUNTS_PER_REV} spring_stiffness={SPRING_STIFFNESS_NM_PER_RAD}",
            flush=True,
        )

        state = comm.read_state_si()
        diag = comm.read_drive_diagnostics()
        print_state("ready", state)
        print(
            "Drive diagnostics: statusword=0x{sw:04X} mode={mode} cia402={cia402} "
            "remote={remote} warning={warning} fault={fault}".format(
                sw=diag.statusword,
                mode=diag.mode_display,
                cia402=diag.cia402_state,
                remote=int(diag.remote),
                warning=int(diag.warning),
                fault=int(diag.fault),
            ),
            flush=True,
        )

        csv_path.parent.mkdir(parents=True, exist_ok=True)
        try:
            os.chmod(csv_path.parent, 0o777)
        except OSError:
            pass
        print(f"Will save CSV to: {csv_path}", flush=True)

        start_ns = monotonic_time_ns()
        stop_ns = start_ns + max(int(round(args.duration * NSEC_PER_SEC)), 0)
        period_ns = normalize_cycle_time_ns(args.cycle_time)
        cycle_deadline_ns = start_ns
        deadline_miss_count = 0
        prev_deadline_miss_count = 0
        cycle = 0
        prev_encoder2_rad: float | None = None
        prev_spring_delta_theta_rad: float | None = None
        prev_spring_sample_time: float | None = None
        last_spring_delta_theta_rad = spring_estimator.estimate(zero_counts).delta_theta_rad
        last_spring_torque_nm = SPRING_STIFFNESS_NM_PER_RAD * last_spring_delta_theta_rad
        last_spring_vel_rad_s = 0.0
        expected_wkc = comm.expected_wkc
        wkc_error_count = 0
        invalid_third_encoder_count = 0
        max_wkc_errors = 3

        comm.queue_third_encoder_request(
            command=DriveCommand(controlword=ENABLE_OPERATION, mode_of_operation=CST_MODE, target_torque=0),
            valid=0,
        )
        comm.send_processdata()

        print("Stage 6: excitation loop", flush=True)
        while True:
            loop_start_ns = monotonic_time_ns()
            if loop_start_ns >= stop_ns:
                break
            cycle_total_start = time.monotonic()
            elapsed_s = (loop_start_ns - start_ns) / NSEC_PER_SEC
            wake_lag_ms = max(0.0, (loop_start_ns - cycle_deadline_ns) / 1_000_000.0)
            deadline_slip_ms = wake_lag_ms

            sample = None
            sample_dt_s = 0.0
            valid_third_encoder = False
            wkc = comm.last_processdata_wkc
            sample_start = time.monotonic()
            sample = comm.poll_queued_third_encoder_sample(timeout_us=0)
            sample_dt_s = time.monotonic() - sample_start
            spring_sample_time = time.monotonic()
            spring_state_dt_s = (
                spring_sample_time - prev_spring_sample_time
                if prev_spring_sample_time is not None
                else args.cycle_time
            )
            prev_spring_sample_time = spring_sample_time
            wkc = sample.wkc if sample is not None else comm.last_processdata_wkc

            valid_third_encoder = sample is not None and (
                sample.response.crc_ok
                and not sample.response.encoder_error
                and not sample.response.communication_alarm
            )
            if valid_third_encoder:
                invalid_third_encoder_count = 0
                spring_estimate = spring_estimator.estimate(sample.response.position_21bit)
                last_spring_delta_theta_rad = spring_estimate.delta_theta_rad
                last_spring_torque_nm = spring_estimate.spring_torque_nm
                last_spring_vel_rad_s = spring_velocity_rad_s(
                    last_spring_delta_theta_rad,
                    prev_spring_delta_theta_rad,
                    spring_state_dt_s,
                )
                prev_spring_delta_theta_rad = last_spring_delta_theta_rad
            else:
                invalid_third_encoder_count += 1
                if args.print_every > 0 and cycle % args.print_every == 0:
                    print(
                        "WARNING: invalid third encoder sample "
                        f"cycle={cycle} consecutive={invalid_third_encoder_count}",
                        flush=True,
                    )
                if invalid_third_encoder_count >= args.max_invalid_third_frames:
                    print(
                        "ERROR: Too many consecutive invalid third encoder frames, stopping excitation.",
                        flush=True,
                    )
                    break

            read_before_start = time.monotonic()
            measured_before = comm.read_state_si()
            read_before_dt_s = time.monotonic() - read_before_start

            command_torque_nm = clamp(excitation_command(elapsed_s, args.duration, cfg), args.max_command_nm)
            action_nm = command_torque_nm if args.execute else 0.0
            target_torque = torque_nm_to_target_units(action_nm, args.max_command_nm)

            send_start = time.monotonic()
            command = DriveCommand(
                controlword=ENABLE_OPERATION,
                mode_of_operation=CST_MODE,
                target_torque=target_torque,
            )
            comm.queue_third_encoder_request(command=command, valid=cycle & 0xFF)
            comm.send_processdata()
            send_dt_s = time.monotonic() - send_start

            if expected_wkc is not None and wkc != expected_wkc:
                wkc_error_count += 1
                print(f"WARNING: wkc={wkc} (expected {expected_wkc}) error_count={wkc_error_count}", flush=True)
                if args.strict_wkc and wkc_error_count >= max_wkc_errors:
                    print("ERROR: Too many wkc errors, stopping excitation.", flush=True)
                    break
            else:
                wkc_error_count = 0

            read_after_start = time.monotonic()
            measured_after = comm.read_state_si()
            read_after_dt_s = time.monotonic() - read_after_start
            diag = comm.read_drive_diagnostics()
            if diag.fault:
                print(f"ERROR: Drive entered fault. statusword=0x{diag.statusword:04X}", flush=True)
                break

            if prev_encoder2_rad is not None:
                meas_dtheta_m_rad_s = (measured_after.encoder2_rad - prev_encoder2_rad) / args.cycle_time
            else:
                meas_dtheta_m_rad_s = measured_after.velocity_rad_s
            prev_encoder2_rad = measured_after.encoder2_rad

            if args.print_every > 0 and cycle % args.print_every == 0:
                print_state(
                    (
                        f"run cycle={cycle} t={elapsed_s:.3f}s cmd={command_torque_nm:.3f}Nm "
                        f"send={action_nm:.3f}Nm target={target_torque} spring={last_spring_torque_nm:.3f}Nm "
                        "source=third_encoder"
                    ),
                    measured_after,
                )

            cycle_total_dt_s = time.monotonic() - cycle_total_start
            row = [
                cycle,
                elapsed_s,
                command_torque_nm,
                action_nm,
                target_torque,
                last_spring_delta_theta_rad,
                last_spring_torque_nm,
                last_spring_vel_rad_s,
                measured_after.encoder2_rad,
                measured_after.position_rad,
                meas_dtheta_m_rad_s,
                measured_after.velocity_rad_s,
                measured_after.position_rad,
                measured_after.velocity_rad_s,
                measured_after.torque_nm,
                measured_after.encoder1_rad,
                measured_after.encoder2_rad,
                wkc,
                expected_wkc if expected_wkc is not None else "",
                f"0x{measured_after.statusword:04X}",
                measured_after.mode_display,
                sample.response.position_21bit if sample is not None else -1,
                int(sample.response.crc_ok) if sample is not None else 0,
                int(sample.response.encoder_error) if sample is not None else 1,
                int(sample.response.communication_alarm) if sample is not None else 1,
                wake_lag_ms,
                deadline_slip_ms,
                0.0,
                0.0,
                0,
                deadline_miss_count,
                sample_dt_s,
                read_before_dt_s,
                send_dt_s,
                read_after_dt_s,
                cycle_total_dt_s,
            ]
            rows.append(row)

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
            row[27] = sleep_target_ms
            row[28] = sleep_actual_ms
            row[29] = deadline_miss_delta
            row[30] = deadline_miss_count

        print("Sending zero torque before exit...", flush=True)
        for _ in range(10):
            zero_command = DriveCommand(
                controlword=ENABLE_OPERATION,
                mode_of_operation=CST_MODE,
                target_torque=0,
            )
            comm.queue_third_encoder_request(command=zero_command, valid=cycle & 0xFF)
            comm.send_processdata()
            try:
                sample = comm.collect_queued_third_encoder_sample(timeout_us=5_000)
                wkc = sample.wkc
            except Exception:
                wkc = comm.last_processdata_wkc
            state = comm.read_state_si()
        print_state(f"stop wkc={wkc}", state)

        if rows:
            with csv_path.open("w", newline="", encoding="utf-8") as csv_file:
                writer = csv.writer(csv_file)
                writer.writerow(fieldnames)
                writer.writerows(rows)
            try:
                os.chmod(csv_path, 0o666)
            except OSError:
                pass
        if deadline_miss_count:
            print(f"Deadline misses: {deadline_miss_count}", flush=True)
        print(f"Data saved to: {csv_path}", flush=True)
        if rows and args.post_analyze:
            print("Generating time-domain and frequency-domain analysis outputs...", flush=True)
            run_post_analysis(
                csv_path,
                load_mass_each_kg=args.load_mass_each_kg,
                load_count=args.load_count,
                load_radius_m=args.load_radius_m,
                load_layout=args.load_layout,
                static_friction_nm=args.static_friction_nm,
            )
            print(f"Time-domain figures: {csv_path.parent / 'time_domain'}", flush=True)
            print(f"Frequency-domain outputs: {csv_path.parent / 'frequency'}", flush=True)
        return 0
    finally:
        comm.close()


if __name__ == "__main__":
    raise SystemExit(main())
