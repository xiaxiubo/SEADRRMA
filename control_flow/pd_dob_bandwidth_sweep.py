#!/usr/bin/env python3
"""PD-DOB swept-sine bandwidth test for one SEA joint.

Place this file in: ETHERCAT_SEABOX/control_flow/pd_dob_bandwidth_sweep.py
Run from the ETHERCAT_SEABOX project root, for example:

    sudo python control_flow/pd_dob_bandwidth_sweep.py run --execute \
        --ifname eno1

The script intentionally removes the relay disturbance logic from pd_dob_main.py and
keeps the same EtherCAT CST torque-control + third-encoder spring-torque feedback path.
It logs the chirp reference, measured load position, spring torque, PD-DOB terms and
then estimates the command-to-load-position Bode curve by local sine fitting.
"""
from __future__ import annotations

import argparse
import csv
import ctypes
import errno
import json
import math
import os
import resource
import sys
import time
from dataclasses import dataclass, asdict
from datetime import datetime
from pathlib import Path
from typing import Iterable

if __package__ in {None, ""}:
    # Expected layout: ETHERCAT_SEABOX/control_flow/this_script.py
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from communication.sea_motor_comm import (  # type: ignore
    CST_MODE,
    ENABLE_OPERATION,
    DriveCommand,
    DriveStateSI,
    SEARealtimeComm,
    SHUTDOWN,
    summarize_wkc,
)
from controllers.sea_model import SeaState  # type: ignore
from controllers.spring_sensor import SpringSensorConfig, SpringSensorEstimator  # type: ignore


# =========================
# Defaults copied from pd_dob_main.py unless overridden by CLI
# =========================
DEFAULT_IFNAME = "eno1"
DEFAULT_CYCLE_TIME_S = 0.005
DEFAULT_REALTIME_CPUS = (8, 9, 10, 11)
DEFAULT_REALTIME_RT_PRIORITY = 99

DEFAULT_MAX_TORQUE_NM = 61.0
DEFAULT_KP_POSITION = 800.0
DEFAULT_KD_POSITION = 20.0
DEFAULT_DOB_GAIN = 0.0
DEFAULT_MAX_DOB_HAT_NM = 0.0
DEFAULT_MAX_RAW_ACTION_NM = 61.0

DEFAULT_MAX_CYCLES_PER_OCTAVE = 2.0
DEFAULT_RAMP_CYCLES = 1.0

DEFAULT_THIRD_ENCODER_COUNTS_PER_REV = 1 << 19
DEFAULT_THIRD_ENCODER_SIGN = 1
DEFAULT_SPRING_STIFFNESS_NM_PER_RAD = 3800.0
DEFAULT_CALIBRATION_SAMPLES = 32

CLOCK_MONOTONIC = 1
TIMER_ABSTIME = 1
NSEC_PER_SEC = 1_000_000_000


class Timespec(ctypes.Structure):
    _fields_ = [("tv_sec", ctypes.c_long), ("tv_nsec", ctypes.c_long)]


def _load_clock_lib() -> ctypes.CDLL:
    for lib_name in ("librt.so.1", "librt.so", "libc.so.6"):
        try:
            return ctypes.CDLL(lib_name, use_errno=True)
        except OSError:
            continue
    raise OSError("Unable to load clock_gettime/clock_nanosleep provider")


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
    ts = Timespec(int(deadline_ns // NSEC_PER_SEC), int(deadline_ns % NSEC_PER_SEC))
    while True:
        result = _CLOCK_NANOSLEEP(CLOCK_MONOTONIC, TIMER_ABSTIME, ctypes.byref(ts), None)
        if result == 0:
            return
        if result != errno.EINTR:
            raise OSError(result, os.strerror(result))


def normalize_cycle_time_ns(cycle_time_s: float) -> int:
    return max(int(round(cycle_time_s * NSEC_PER_SEC)), 1)


def configure_realtime_runtime(cpu_affinity: Iterable[int], rt_priority: int) -> None:
    for env_name in (
        "OMP_NUM_THREADS",
        "MKL_NUM_THREADS",
        "OPENBLAS_NUM_THREADS",
        "NUMEXPR_NUM_THREADS",
    ):
        os.environ.setdefault(env_name, "1")

    try:
        cpus = tuple(cpu_affinity)
        if cpus:
            os.sched_setaffinity(0, set(cpus))
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
        if libc.mlockall(1 | 2) != 0:
            err = ctypes.get_errno()
            raise OSError(err, os.strerror(err))
    except Exception:
        pass

    try:
        os.sched_setscheduler(0, os.SCHED_FIFO, os.sched_param(rt_priority))
    except Exception:
        pass


def torque_nm_to_target_units(torque_nm: float, max_torque_nm: float) -> int:
    if max_torque_nm <= 0.0:
        raise ValueError("max_torque_nm must be positive")
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
        (
            "{prefix} statusword=0x{sw:04X} mode={mode} pos={pos:.6f}rad "
            "vel={vel:.6f}rad/s torque={torque:.3f}Nm fe={fe:.6f}rad "
            "enc1={enc1:.6f}rad enc2={enc2:.6f}rad"
        ).format(
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


def measured_to_sea_state(measured: DriveStateSI, prev_encoder2_rad: float | None, dt: float) -> SeaState:
    if prev_encoder2_rad is None:
        dtheta_m_rad_s = measured.velocity_rad_s
    else:
        dtheta_m_rad_s = (measured.encoder2_rad - prev_encoder2_rad) / max(dt, 1e-9)
    return SeaState(
        theta_m_rad=measured.encoder2_rad,
        dtheta_m_rad_s=dtheta_m_rad_s,
        theta_l_rad=measured.position_rad,
        dtheta_l_rad_s=measured.velocity_rad_s,
    )


class PdDobController:
    def __init__(
        self,
        kp_position: float,
        kd_position: float,
        dob_gain: float,
        max_dob_hat_nm: float,
        max_raw_action_nm: float,
    ) -> None:
        self.kp_position = kp_position
        self.kd_position = kd_position
        self.dob_gain = dob_gain
        self.max_dob_hat_nm = max_dob_hat_nm
        self.max_raw_action_nm = max_raw_action_nm
        self.prev_time_s: float | None = None
        self.disturbance_hat_nm = 0.0

    def reset(self) -> None:
        self.prev_time_s = None
        self.disturbance_hat_nm = 0.0

    def update(
        self,
        reference: SeaState,
        measured: DriveStateSI,
        spring_torque_nm: float,
        now_s: float,
    ) -> tuple[float, float, float, float]:
        pos_error = reference.theta_l_rad - measured.position_rad
        vel_error = reference.dtheta_l_rad_s - measured.velocity_rad_s
        torque_ref_nm = self.kp_position * pos_error + self.kd_position * vel_error

        if self.prev_time_s is None:
            dt = 0.0
        else:
            dt = max(now_s - self.prev_time_s, 1e-6)

        torque_error_nm = torque_ref_nm - spring_torque_nm
        self.disturbance_hat_nm += self.dob_gain * torque_error_nm * dt
        self.disturbance_hat_nm = max(
            min(self.disturbance_hat_nm, self.max_dob_hat_nm),
            -self.max_dob_hat_nm,
        )
        raw_action_nm = torque_ref_nm - self.disturbance_hat_nm
        raw_action_nm = max(min(raw_action_nm, self.max_raw_action_nm), -self.max_raw_action_nm)
        self.prev_time_s = now_s
        return torque_ref_nm, raw_action_nm, torque_error_nm, self.disturbance_hat_nm


@dataclass
class ChirpReference:
    initial_load_position_rad: float
    initial_motor_load_offset_rad: float
    amplitude_rad: float
    f_start_hz: float
    f_end_hz: float
    duration_s: float
    settle_s: float
    ramp_s: float
    ramp_cycles: float

    def _smoothstep(self, u: float) -> tuple[float, float]:
        u = max(0.0, min(1.0, u))
        value = 3.0 * u * u - 2.0 * u * u * u
        derivative = 6.0 * u - 6.0 * u * u
        return value, derivative

    def _envelope(self, sweep_t_s: float) -> tuple[float, float]:
        total_s = max(self.duration_s - self.settle_s, 1e-9)
        ramp_in_s = min(self.ramp_s, self.ramp_cycles / max(self.f_start_hz, 1e-6))
        ramp_out_s = min(self.ramp_s, self.ramp_cycles / max(self.f_end_hz, 1e-6))
        ramp_in_s = max(min(ramp_in_s, 0.45 * total_s), 0.0)
        ramp_out_s = max(min(ramp_out_s, 0.45 * total_s), 0.0)
        if ramp_in_s <= 0.0 and ramp_out_s <= 0.0:
            return 1.0, 0.0
        if ramp_in_s > 0.0 and sweep_t_s < ramp_in_s:
            value, du = self._smoothstep(sweep_t_s / ramp_in_s)
            return value, du / ramp_in_s
        if ramp_out_s > 0.0 and (total_s - sweep_t_s) < ramp_out_s:
            value, du = self._smoothstep((total_s - sweep_t_s) / ramp_out_s)
            return value, -du / ramp_out_s
        return 1.0, 0.0

    def _phase_and_frequency(self, sweep_t_s: float) -> tuple[float, float]:
        total_s = max(self.duration_s - self.settle_s, 1e-9)
        sweep_t_s = max(0.0, min(total_s, sweep_t_s))
        f0 = max(self.f_start_hz, 0.0)
        f1 = max(self.f_end_hz, f0 + 1e-6)

        # The paper-target experiment is a true 0 -> 6 Hz linear chirp. A
        # zero-frequency start cannot use a logarithmic/hyperbolic law, and a
        # linear sweep crosses each narrow resonance band quickly and evenly.
        if f0 <= 1e-12:
            sweep_rate_hz_s = (f1 - f0) / total_s
            freq_hz = f0 + sweep_rate_hz_s * sweep_t_s
            phase_rad = 2.0 * math.pi * (
                f0 * sweep_t_s + 0.5 * sweep_rate_hz_s * sweep_t_s * sweep_t_s
            )
            return phase_rad, freq_hz

        # Hyperbolic chirp: df/dt is proportional to f^2.  Therefore each
        # octave receives the same number of excitation cycles, instead of the
        # high-frequency octaves accumulating many more cycles near resonance.
        inverse_frequency_slope = (1.0 / f0 - 1.0 / f1) / total_s
        denominator = max(1.0 / f0 - inverse_frequency_slope * sweep_t_s, 1.0 / f1)
        freq_hz = 1.0 / denominator
        if inverse_frequency_slope <= 1e-12:
            phase_rad = 2.0 * math.pi * f0 * sweep_t_s
        else:
            phase_rad = 2.0 * math.pi * math.log(freq_hz / f0) / inverse_frequency_slope
        return phase_rad, freq_hz

    def at(self, elapsed_s: float) -> tuple[SeaState, float, float, float]:
        if elapsed_s < self.settle_s:
            theta_l = self.initial_load_position_rad
            dtheta_l = 0.0
            theta_m = theta_l + self.initial_motor_load_offset_rad
            return (
                SeaState(theta_m, 0.0, theta_l, 0.0),
                0.0,
                0.0,
                0.0,
            )

        sweep_t_s = elapsed_s - self.settle_s
        phase_rad, freq_hz = self._phase_and_frequency(sweep_t_s)
        env, env_dot = self._envelope(sweep_t_s)
        phase_dot = 2.0 * math.pi * freq_hz
        sin_p = math.sin(phase_rad)
        cos_p = math.cos(phase_rad)
        theta_l = self.initial_load_position_rad + self.amplitude_rad * env * sin_p
        dtheta_l = self.amplitude_rad * (env_dot * sin_p + env * phase_dot * cos_p)
        theta_m = theta_l + self.initial_motor_load_offset_rad
        dtheta_m = dtheta_l
        return (
            SeaState(theta_m, dtheta_m, theta_l, dtheta_l),
            phase_rad,
            freq_hz,
            env,
        )


def warmup_third_encoder(comm: SEARealtimeComm, warmup_cycles: int = 100, required_valid: int = 5) -> None:
    valid_count = 0
    for attempt in range(warmup_cycles):
        try:
            sample = comm.sample_third_encoder_position_21bit(
                command=DriveCommand(controlword=ENABLE_OPERATION, mode_of_operation=CST_MODE, target_torque=0),
                valid=attempt & 0xFF,
            )
            print_third_encoder_sample(prefix=f"[warmup] attempt={attempt}", sample=sample)
            if sample.response.crc_ok and not sample.response.encoder_error and not sample.response.communication_alarm:
                valid_count += 1
            if valid_count >= required_valid:
                return
        except Exception as exc:
            print(f"[warmup] attempt={attempt} failed: {exc}", flush=True)
    raise RuntimeError(f"Third encoder warm-up failed: valid={valid_count}/{required_valid}")


def collect_zero_counts(comm: SEARealtimeComm, sample_count: int) -> int:
    zero_samples: list[int] = []
    attempt_limit = max(sample_count * 20, 100)
    last_error: Exception | None = None
    for attempt in range(attempt_limit):
        if len(zero_samples) >= sample_count:
            break
        try:
            sample = comm.sample_third_encoder_position_21bit(
                command=DriveCommand(controlword=ENABLE_OPERATION, mode_of_operation=CST_MODE, target_torque=0),
                valid=attempt & 0xFF,
            )
            print_third_encoder_sample(prefix=f"[zero] attempt={attempt}", sample=sample)
            if sample.response.crc_ok and not sample.response.encoder_error and not sample.response.communication_alarm:
                zero_samples.append(sample.response.position_21bit)
        except Exception as exc:
            last_error = exc
            print(f"[zero] attempt={attempt} failed: {exc}", flush=True)
    if len(zero_samples) < sample_count:
        raise RuntimeError(
            f"Failed to collect enough valid zero samples: got={len(zero_samples)} need={sample_count} last_error={last_error}"
        )
    return round(sum(zero_samples) / len(zero_samples))


def zero_torque_shutdown(comm: SEARealtimeComm, cycles: int = 20, cycle_valid_start: int = 0) -> None:
    for idx in range(cycles):
        command = DriveCommand(controlword=ENABLE_OPERATION, mode_of_operation=CST_MODE, target_torque=0)
        try:
            comm.queue_third_encoder_request(command=command, valid=(cycle_valid_start + idx) & 0xFF)
            comm.send_processdata()
            try:
                comm.collect_queued_third_encoder_sample(timeout_us=5_000)
            except Exception:
                pass
        except Exception:
            break
        time.sleep(0.001)



def clamp_float(value: float, lo: float, hi: float) -> float:
    return max(min(value, hi), lo)


def compute_hold_torque_nm(
    measured: DriveStateSI,
    hold_position_rad: float,
    hold_kp: float,
    hold_kd: float,
    hold_torque_limit_nm: float,
) -> float:
    """Pure PD hold torque for catching load after brake release."""
    pos_error = hold_position_rad - measured.position_rad
    vel_error = -measured.velocity_rad_s
    torque_nm = hold_kp * pos_error + hold_kd * vel_error
    return clamp_float(torque_nm, -hold_torque_limit_nm, hold_torque_limit_nm)


def run_position_hold_stage(
    comm: SEARealtimeComm,
    cfg,
    hold_position_rad: float,
    duration_s: float,
    valid_start: int = 0,
    label: str = "hold",
) -> DriveStateSI:
    """Hold load-side position with direct CST torque before starting chirp.

    This is intended to catch the load immediately after brake release and to
    let the SEA settle before the chirp center is captured.
    """
    if duration_s <= 0.0:
        return comm.read_state_si()

    print(
        f"{label}: hold_s={duration_s:.3f}, center={hold_position_rad:.6f}rad, "
        f"kp={cfg.hold_kp}, kd={cfg.hold_kd}, "
        f"limit={cfg.hold_torque_limit_nm}Nm",
        flush=True,
    )

    period_ns = normalize_cycle_time_ns(cfg.cycle_time_s)
    start_ns = monotonic_time_ns()
    stop_ns = start_ns + int(round(duration_s * NSEC_PER_SEC))
    deadline_ns = start_ns
    cycle = 0
    last_state = comm.read_state_si()

    while True:
        now_ns = monotonic_time_ns()
        if now_ns >= stop_ns:
            break

        measured = comm.read_state_si()
        action_nm = compute_hold_torque_nm(
            measured=measured,
            hold_position_rad=hold_position_rad,
            hold_kp=cfg.hold_kp,
            hold_kd=cfg.hold_kd,
            hold_torque_limit_nm=cfg.hold_torque_limit_nm,
        )
        if not cfg.execute:
            action_nm = 0.0
        target_torque = torque_nm_to_target_units(action_nm, cfg.max_torque_nm)

        try:
            comm.queue_third_encoder_request(
                command=DriveCommand(
                    controlword=ENABLE_OPERATION,
                    mode_of_operation=CST_MODE,
                    target_torque=target_torque,
                ),
                valid=(valid_start + cycle) & 0xFF,
            )
            comm.send_processdata()
            try:
                comm.collect_queued_third_encoder_sample(timeout_us=5_000)
            except Exception:
                pass
        except Exception as exc:
            print(f"WARNING: {label} cycle={cycle} command failed: {exc}", flush=True)
            break

        last_state = measured
        if cycle % max(cfg.hold_print_every, 1) == 0:
            print_state(
                f"{label} cycle={cycle} cmd={action_nm:.2f}Nm "
                f"err={measured.position_rad - hold_position_rad:+.6f}rad",
                measured,
            )

        cycle += 1
        next_deadline_ns = deadline_ns + period_ns
        if monotonic_time_ns() < next_deadline_ns:
            sleep_until_monotonic_ns(next_deadline_ns)
        deadline_ns = next_deadline_ns

    final_state = comm.read_state_si()
    print_state(f"{label} final", final_state)
    print(
        f"{label}: final_err={final_state.position_rad - hold_position_rad:+.6f}rad "
        f"final_vel={final_state.velocity_rad_s:+.6f}rad/s",
        flush=True,
    )
    return final_state


def collect_zero_counts_with_hold(
    comm: SEARealtimeComm,
    cfg,
    sample_count: int,
    hold_position_rad: float,
    valid_start: int = 0,
) -> int:
    """Collect third-encoder zero samples while actively holding load position.

    Under payload, collecting zero samples with target_torque=0 can let the
    output side drop. This version keeps a PD hold torque active during zero
    sampling.
    """
    zero_samples: list[int] = []
    attempt_limit = max(sample_count * 30, 150)
    period_ns = normalize_cycle_time_ns(cfg.cycle_time_s)
    deadline_ns = monotonic_time_ns()
    last_error: Exception | None = None

    for attempt in range(attempt_limit):
        if len(zero_samples) >= sample_count:
            break

        measured = comm.read_state_si()
        action_nm = compute_hold_torque_nm(
            measured=measured,
            hold_position_rad=hold_position_rad,
            hold_kp=cfg.hold_kp,
            hold_kd=cfg.hold_kd,
            hold_torque_limit_nm=cfg.hold_torque_limit_nm,
        )
        if not cfg.execute:
            action_nm = 0.0

        try:
            sample = comm.sample_third_encoder_position_21bit(
                command=DriveCommand(
                    controlword=ENABLE_OPERATION,
                    mode_of_operation=CST_MODE,
                    target_torque=torque_nm_to_target_units(action_nm, cfg.max_torque_nm),
                ),
                valid=(valid_start + attempt) & 0xFF,
                sleep=False,
            )
            print_third_encoder_sample(prefix=f"[zero-hold] attempt={attempt}", sample=sample)
            if sample.response.crc_ok and not sample.response.encoder_error and not sample.response.communication_alarm:
                zero_samples.append(sample.response.position_21bit)
        except Exception as exc:
            last_error = exc
            print(f"[zero-hold] attempt={attempt} failed: {exc}", flush=True)

        if attempt % max(cfg.hold_print_every, 1) == 0:
            print(
                f"[zero-hold] cmd={action_nm:.2f}Nm "
                f"err={measured.position_rad - hold_position_rad:+.6f}rad "
                f"valid={len(zero_samples)}/{sample_count}",
                flush=True,
            )

        next_deadline_ns = deadline_ns + period_ns
        if monotonic_time_ns() < next_deadline_ns:
            sleep_until_monotonic_ns(next_deadline_ns)
        deadline_ns = next_deadline_ns

    if len(zero_samples) < sample_count:
        raise RuntimeError(
            f"Failed to collect enough valid zero-hold samples: "
            f"got={len(zero_samples)} need={sample_count} last_error={last_error}"
        )
    return round(sum(zero_samples) / len(zero_samples))


@dataclass
class RunConfig:
    ifname: str
    drive_index: int
    cycle_time_s: float
    duration_s: float
    settle_s: float
    ramp_s: float
    ramp_cycles: float
    max_cycles_per_octave: float
    amplitude_deg: float
    f_start_hz: float
    f_end_hz: float
    execute: bool
    kp: float
    kd: float
    dob_gain: float
    max_torque_nm: float
    max_dob_hat_nm: float
    max_raw_action_nm: float
    hold_s: float
    hold_kp: float
    hold_kd: float
    hold_torque_limit_nm: float
    hold_center_mode: str
    hold_print_every: int
    counts_per_rev: int
    third_encoder_sign: int
    spring_stiffness_nm_per_rad: float
    calibration_samples: int
    print_every: int
    max_pos_error_deg: float
    max_velocity_rad_s: float
    max_consecutive_wkc_errors: int
    cpu_affinity: tuple[int, ...]
    rt_priority: int
    out_dir: str
    analyze_after_run: bool


def run_bandwidth_test(args: argparse.Namespace) -> Path:
    cfg = RunConfig(
        ifname=args.ifname,
        drive_index=args.drive_index,
        cycle_time_s=args.cycle_time,
        duration_s=args.duration,
        settle_s=args.settle_s,
        ramp_s=args.ramp_s,
        ramp_cycles=args.ramp_cycles,
        max_cycles_per_octave=args.max_cycles_per_octave,
        amplitude_deg=args.amplitude_deg,
        f_start_hz=args.f_start,
        f_end_hz=args.f_end,
        execute=args.execute,
        kp=args.kp,
        kd=args.kd,
        dob_gain=args.dob_gain,
        max_torque_nm=args.max_torque_nm,
        max_dob_hat_nm=args.max_dob_hat_nm,
        max_raw_action_nm=args.max_raw_action_nm,
        hold_s=args.hold_s,
        hold_kp=args.hold_kp,
        hold_kd=args.hold_kd,
        hold_torque_limit_nm=args.hold_torque_limit_nm,
        hold_center_mode=args.hold_center_mode,
        hold_print_every=args.hold_print_every,
        counts_per_rev=args.counts_per_rev,
        third_encoder_sign=args.third_encoder_sign,
        spring_stiffness_nm_per_rad=args.spring_stiffness,
        calibration_samples=args.calibration_samples,
        print_every=args.print_every,
        max_pos_error_deg=args.max_pos_error_deg,
        max_velocity_rad_s=args.max_velocity_rad_s,
        max_consecutive_wkc_errors=args.max_consecutive_wkc_errors,
        cpu_affinity=tuple(args.cpu_affinity),
        rt_priority=args.rt_priority,
        out_dir=args.out_dir,
        analyze_after_run=not args.no_analyze,
    )

    if cfg.f_start_hz < 0.0:
        raise ValueError("--f-start must be >= 0")
    if cfg.f_end_hz <= cfg.f_start_hz:
        raise ValueError("--f-end must be larger than --f-start")
    if cfg.duration_s <= cfg.settle_s + 5.0:
        raise ValueError("--duration must be at least settle_s + 5 s")
    if cfg.max_cycles_per_octave <= 0.0:
        raise ValueError("--max-cycles-per-octave must be positive")

    requested_duration_s = cfg.duration_s
    requested_sweep_s = requested_duration_s - cfg.settle_s
    if cfg.f_start_hz == 0.0:
        effective_sweep_s = requested_sweep_s
        print(
            f"Linear paper-target chirp: 0 -> {cfg.f_end_hz:.3f}Hz in "
            f"{effective_sweep_s:.3f}s "
            f"({cfg.f_end_hz / effective_sweep_s:.3f}Hz/s).",
            flush=True,
        )
    else:
        max_sweep_s = (
            cfg.max_cycles_per_octave
            / math.log(2.0)
            * (1.0 / cfg.f_start_hz - 1.0 / cfg.f_end_hz)
        )
        effective_sweep_s = min(requested_sweep_s, max_sweep_s)
    cfg.duration_s = cfg.settle_s + effective_sweep_s
    if cfg.duration_s < requested_duration_s - 1e-9:
        print(
            f"Excitation shortened from {requested_duration_s:.3f}s to {cfg.duration_s:.3f}s: "
            f"sweep={effective_sweep_s:.3f}s, cap={cfg.max_cycles_per_octave:.2f} cycles/octave.",
            flush=True,
        )

    configure_realtime_runtime(cpu_affinity=cfg.cpu_affinity, rt_priority=cfg.rt_priority)
    try:
        current_affinity = sorted(os.sched_getaffinity(0))
    except Exception:
        current_affinity = []
    print(
        f"Deadline scheduling: cycle={cfg.cycle_time_s*1000.0:.3f}ms affinity={current_affinity} execute={cfg.execute}",
        flush=True,
    )
    if not cfg.execute:
        print("WARNING: --execute not set. The drive will be enabled but torque command remains zero.", flush=True)

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    out_dir = Path(cfg.out_dir).expanduser().resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    csv_path = out_dir / f"b_joint_pd_dob_bandwidth_{timestamp}.csv"
    json_path = out_dir / f"b_joint_pd_dob_bandwidth_{timestamp}_config.json"

    comm = SEARealtimeComm(ifname=cfg.ifname, cycle_time_s=cfg.cycle_time_s)
    spring_estimator = SpringSensorEstimator(
        SpringSensorConfig(
            counts_per_rev=cfg.counts_per_rev,
            zero_counts=0,
            sign=cfg.third_encoder_sign,
            spring_stiffness_nm_per_rad=cfg.spring_stiffness_nm_per_rad,
        )
    )
    controller = PdDobController(
        kp_position=cfg.kp,
        kd_position=cfg.kd,
        dob_gain=cfg.dob_gain,
        max_dob_hat_nm=cfg.max_dob_hat_nm,
        max_raw_action_nm=cfg.max_raw_action_nm,
    )

    prev_encoder2_rad: float | None = None
    cycle = 0
    deadline_miss_count = 0
    prev_deadline_miss_count = 0
    expected_wkc: int | None = None
    wkc_error_count = 0
    torque_saturation_count = 0
    zero_counts = 0

    fieldnames = [
        "cycle",
        "time_s",
        "freq_hz",
        "phase_rad",
        "envelope",
        "wake_lag_ms",
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
        "torque_ref_nm",
        "torque_error_nm",
        "disturbance_hat_nm",
        "raw_action_nm",
        "action_nm",
        "target_torque",
        "torque_saturated",
        "wkc",
        "third_encoder_pos21",
        "third_encoder_valid",
        "spring_delta_theta_rad",
        "spring_torque_nm",
        "statusword",
        "mode_display",
        "position_rad",
        "velocity_rad_s",
        "torque_nm",
        "following_error_rad",
        "encoder1_rad",
        "encoder2_rad",
        "sample_dt_s",
        "read_before_dt_s",
        "controller_dt_s",
        "send_dt_s",
        "read_after_dt_s",
        "diag_dt_s",
        "cycle_total_dt_s",
    ]

    try:
        comm.connect()
        print_slave_inventory(comm)
        print(
            f"Connected on {cfg.ifname}. CST mode={CST_MODE}, PD-DOB kp={cfg.kp}, kd={cfg.kd}, dob={cfg.dob_gain}",
            flush=True,
        )

        print("Stage 0.5: box transparent PRE-OP configuration", flush=True)
        third_encoder_cfg = comm.configure_box_transparent_preop()
        print(f"Third encoder transparent configured: {third_encoder_cfg}", flush=True)

        # Select the single target drive only after the EtherCAT box transparent
        # channel has been configured. Selecting drive_index too early can leave
        # slaves in SAFE-OP + ERROR and make the box reject SDO writes.
        if comm.master is None:
            raise RuntimeError("EtherCAT master is not initialized after comm.connect().")
        if comm.roles is None:
            raise RuntimeError("Slave roles are not initialized after comm.connect().")
        if cfg.drive_index <= 0 or cfg.drive_index >= len(comm.master.slaves):
            raise ValueError(
                f"--drive-index must select one SOMANET slave in [1, {len(comm.master.slaves)-1}], "
                f"got {cfg.drive_index}"
            )

        old_drive_index = comm.roles.drive_index
        comm.roles = type(comm.roles)(
            box_index=comm.roles.box_index,
            drive_index=cfg.drive_index,
        )
        comm.drive = comm.master.slaves[cfg.drive_index]

        # Do NOT refresh PDO layout here.
        # In this setup, slave[4] has 59-byte TxPDO and matches the XML target,
        # while slave[1] has 47-byte TxPDO. Reading 0x1C13 from slave[1] can raise
        # WkcError. The RxPDO output size is still 35 bytes, so we reuse the already
        # loaded command layout and only redirect the DriveCommand output to slave[1].
        print(
            f"Selected single target drive: old_drive_index={old_drive_index} -> drive_index={cfg.drive_index}, "
            f"name={getattr(comm.drive, 'name', None)!r}, "
            f"in={len(bytes(getattr(comm.drive, 'input', b'')))} "
            f"out={len(bytes(getattr(comm.drive, 'output', b'')))}. "
            f"PDO layout is reused; no SDO refresh on target drive.",
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
            raise RuntimeError("PDO preheat failed before CiA402 enable")

        pre_enable_state = comm.read_state_si()
        pre_enable_position_rad = pre_enable_state.position_rad
        print_state("Pre-enable hold reference", pre_enable_state)

        print("Stage 2: CiA402 enable", flush=True)
        transition_results = comm.enable_cia402(
            mode_of_operation=CST_MODE,
            target_torque=0,
            timeout_cycles_per_step=300,
        )
        for result in transition_results:
            diag = result.last_diagnostics
            print(
                (
                    f"  step={result.step_name} success={result.success} expected_state={result.expected_state} "
                    f"wkc={summarize_wkc(result.observed_wkc)} statusword=0x{diag.statusword:04X} "
                    f"mode={diag.mode_display} cia402={diag.cia402_state}"
                ),
                flush=True,
            )
            if not result.success:
                raise RuntimeError(f"CiA402 transition failed at step {result.step_name}")

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

        post_enable_state = comm.read_state_si()
        if cfg.hold_center_mode == "pre_enable":
            hold_position_rad = pre_enable_position_rad
        else:
            hold_position_rad = post_enable_state.position_rad
        print_state("Post-enable before hold", post_enable_state)
        print(
            f"Brake-release displacement before hold: "
            f"{post_enable_state.position_rad - pre_enable_position_rad:+.6f}rad "
            f"({math.degrees(post_enable_state.position_rad - pre_enable_position_rad):+.3f}deg), "
            f"hold_center_mode={cfg.hold_center_mode}",
            flush=True,
        )

        if cfg.hold_s > 0.0:
            print("Stage 3.5: anti-drop / preload hold before encoder calibration", flush=True)
            run_position_hold_stage(
                comm=comm,
                cfg=cfg,
                hold_position_rad=hold_position_rad,
                duration_s=cfg.hold_s,
                valid_start=0,
                label="preload-hold",
            )

        print("Stage 4: third encoder initialization", flush=True)
        if cfg.hold_s > 0.0:
            print("Third encoder warmup will be merged into zero calibration with active hold.", flush=True)
        else:
            warmup_third_encoder(comm, warmup_cycles=100, required_valid=5)

        print("Stage 5: zero calibration", flush=True)
        if cfg.hold_s > 0.0:
            zero_counts = collect_zero_counts_with_hold(
                comm=comm,
                cfg=cfg,
                sample_count=cfg.calibration_samples,
                hold_position_rad=hold_position_rad,
                valid_start=100,
            )
        else:
            zero_counts = collect_zero_counts(comm, cfg.calibration_samples)
        spring_estimator.tare_zero(zero_counts)
        print(
            f"Calibration done: zero_counts={zero_counts}, counts_per_rev={cfg.counts_per_rev}, "
            f"spring_stiffness={cfg.spring_stiffness_nm_per_rad}",
            flush=True,
        )

        if cfg.hold_s > 0.0:
            print("Stage 5.5: post-calibration hold before chirp center capture", flush=True)
            run_position_hold_stage(
                comm=comm,
                cfg=cfg,
                hold_position_rad=hold_position_rad,
                duration_s=cfg.hold_s,
                valid_start=200,
                label="post-cal-hold",
            )

        initial_state = comm.read_state_si()
        initial_diag = comm.read_drive_diagnostics()
        print_state("Initial", initial_state)
        print(
            "Drive ready: statusword=0x{sw:04X} mode={mode} cia402={cia402} remote={remote} fault={fault}".format(
                sw=initial_diag.statusword,
                mode=initial_diag.mode_display,
                cia402=initial_diag.cia402_state,
                remote=int(initial_diag.remote),
                fault=int(initial_diag.fault),
            ),
            flush=True,
        )

        initial_position_rad = initial_state.position_rad
        initial_motor_load_offset_rad = initial_state.encoder2_rad - initial_state.position_rad
        reference_gen = ChirpReference(
            initial_load_position_rad=initial_position_rad,
            initial_motor_load_offset_rad=initial_motor_load_offset_rad,
            amplitude_rad=math.radians(cfg.amplitude_deg),
            f_start_hz=cfg.f_start_hz,
            f_end_hz=cfg.f_end_hz,
            duration_s=cfg.duration_s,
            settle_s=cfg.settle_s,
            ramp_s=cfg.ramp_s,
            ramp_cycles=cfg.ramp_cycles,
        )

        metadata = asdict(cfg)
        metadata.update(
            {
                "created_at": timestamp,
                "requested_duration_s": requested_duration_s,
                "script": str(Path(__file__).resolve()),
                "initial_position_rad": initial_position_rad,
                "initial_motor_load_offset_rad": initial_motor_load_offset_rad,
                "zero_counts": zero_counts,
                "csv_path": str(csv_path),
            }
        )
        with json_path.open("w", encoding="utf-8") as f:
            json.dump(metadata, f, ensure_ascii=False, indent=2)
        print(f"Will save CSV to: {csv_path}", flush=True)
        print(f"Saved config to: {json_path}", flush=True)

        start_ns = monotonic_time_ns()
        stop_ns = start_ns + int(round(cfg.duration_s * NSEC_PER_SEC))
        period_ns = normalize_cycle_time_ns(cfg.cycle_time_s)
        cycle_deadline_ns = start_ns
        expected_wkc = comm.expected_wkc
        max_pos_error_rad = math.radians(cfg.max_pos_error_deg)
        last_spring_est = spring_estimator.estimate(zero_counts)

        priming_command = DriveCommand(controlword=ENABLE_OPERATION, mode_of_operation=CST_MODE, target_torque=0)
        comm.queue_third_encoder_request(command=priming_command, valid=0)
        comm.send_processdata()

        with csv_path.open("w", newline="", encoding="utf-8") as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
            writer.writeheader()

            while True:
                cycle_total_start = time.monotonic()
                loop_start_ns = monotonic_time_ns()
                if loop_start_ns >= stop_ns:
                    break
                elapsed_s = (loop_start_ns - start_ns) / NSEC_PER_SEC
                wake_lag_ms = max(0.0, (loop_start_ns - cycle_deadline_ns) / 1_000_000.0)

                sample_start = time.monotonic()
                sample = comm.poll_queued_third_encoder_sample(timeout_us=0)
                sample_dt_s = time.monotonic() - sample_start
                wkc = sample.wkc if sample is not None else comm.last_processdata_wkc

                if expected_wkc is not None and wkc != expected_wkc:
                    wkc_error_count += 1
                    print(f"WARNING: wkc={wkc} expected={expected_wkc} count={wkc_error_count}", flush=True)
                    if wkc_error_count >= cfg.max_consecutive_wkc_errors:
                        print("ERROR: too many consecutive WKC errors; stopping.", flush=True)
                        break
                else:
                    wkc_error_count = 0

                valid_third_encoder = sample is not None and (
                    sample.response.crc_ok
                    and not sample.response.encoder_error
                    and not sample.response.communication_alarm
                )
                if valid_third_encoder:
                    last_spring_est = spring_estimator.estimate(sample.response.position_21bit)
                elif sample is not None:
                    print(
                        f"WARNING: invalid third encoder frame cycle={cycle} "
                        f"crc={int(sample.response.crc_ok)} enc_err={int(sample.response.encoder_error)} "
                        f"comm_alarm={int(sample.response.communication_alarm)}",
                        flush=True,
                    )

                read_before_start = time.monotonic()
                measured_before = comm.read_state_si()
                read_before_dt_s = time.monotonic() - read_before_start

                controller_start = time.monotonic()
                reference, phase_rad, freq_hz, envelope = reference_gen.at(elapsed_s)
                measured_state = measured_to_sea_state(
                    measured_before,
                    prev_encoder2_rad=prev_encoder2_rad,
                    dt=cfg.cycle_time_s,
                )
                torque_ref_nm, raw_action_nm, torque_error_nm, disturbance_hat_nm = controller.update(
                    reference=reference,
                    measured=measured_before,
                    spring_torque_nm=last_spring_est.spring_torque_nm,
                    now_s=elapsed_s,
                )
                controller_dt_s = time.monotonic() - controller_start

                action_nm = raw_action_nm if cfg.execute else 0.0
                target_torque = torque_nm_to_target_units(action_nm, cfg.max_torque_nm)
                torque_saturated = int(abs(target_torque) >= 1000)
                torque_saturation_count += torque_saturated

                send_start = time.monotonic()
                comm.queue_third_encoder_request(
                    command=DriveCommand(
                        controlword=ENABLE_OPERATION,
                        mode_of_operation=CST_MODE,
                        target_torque=target_torque,
                    ),
                    valid=cycle & 0xFF,
                )
                comm.send_processdata()
                send_dt_s = time.monotonic() - send_start

                prev_encoder2_rad = measured_before.encoder2_rad
                read_after_start = time.monotonic()
                measured_after = comm.read_state_si()
                read_after_dt_s = time.monotonic() - read_after_start
                diag_start = time.monotonic()
                measured_diag = comm.read_drive_diagnostics()
                diag_dt_s = time.monotonic() - diag_start

                err_theta_l = measured_state.theta_l_rad - reference.theta_l_rad
                err_theta_m = measured_state.theta_m_rad - reference.theta_m_rad
                err_dtheta_l = measured_state.dtheta_l_rad_s - reference.dtheta_l_rad_s
                err_dtheta_m = measured_state.dtheta_m_rad_s - reference.dtheta_m_rad_s

                stop_reason: str | None = None
                if abs(err_theta_l) > max_pos_error_rad or abs(err_theta_m) > max_pos_error_rad:
                    stop_reason = f"position error too large: err_l={err_theta_l:.3f}, err_m={err_theta_m:.3f} rad"
                elif abs(measured_after.velocity_rad_s) > cfg.max_velocity_rad_s:
                    stop_reason = f"velocity too large: {measured_after.velocity_rad_s:.3f} rad/s"
                elif measured_diag.fault:
                    stop_reason = f"drive fault: statusword=0x{measured_diag.statusword:04X}"

                cycle_total_dt_s = time.monotonic() - cycle_total_start
                row = {
                    "cycle": cycle,
                    "time_s": elapsed_s,
                    "freq_hz": freq_hz,
                    "phase_rad": phase_rad,
                    "envelope": envelope,
                    "wake_lag_ms": wake_lag_ms,
                    "sleep_target_ms": 0.0,
                    "sleep_actual_ms": 0.0,
                    "deadline_miss_delta": 0,
                    "deadline_miss_count": deadline_miss_count,
                    "ref_theta_m_rad": reference.theta_m_rad,
                    "ref_theta_l_rad": reference.theta_l_rad,
                    "ref_dtheta_m_rad_s": reference.dtheta_m_rad_s,
                    "ref_dtheta_l_rad_s": reference.dtheta_l_rad_s,
                    "meas_theta_m_rad": measured_state.theta_m_rad,
                    "meas_theta_l_rad": measured_state.theta_l_rad,
                    "meas_dtheta_m_rad_s": measured_state.dtheta_m_rad_s,
                    "meas_dtheta_l_rad_s": measured_state.dtheta_l_rad_s,
                    "err_theta_m_rad": err_theta_m,
                    "err_theta_l_rad": err_theta_l,
                    "err_dtheta_m_rad_s": err_dtheta_m,
                    "err_dtheta_l_rad_s": err_dtheta_l,
                    "torque_ref_nm": torque_ref_nm,
                    "torque_error_nm": torque_error_nm,
                    "disturbance_hat_nm": disturbance_hat_nm,
                    "raw_action_nm": raw_action_nm,
                    "action_nm": action_nm,
                    "target_torque": target_torque,
                    "torque_saturated": torque_saturated,
                    "wkc": wkc,
                    "third_encoder_pos21": sample.response.position_21bit if sample is not None else -1,
                    "third_encoder_valid": int(valid_third_encoder),
                    "spring_delta_theta_rad": last_spring_est.delta_theta_rad,
                    "spring_torque_nm": last_spring_est.spring_torque_nm,
                    "statusword": measured_after.statusword,
                    "mode_display": measured_after.mode_display,
                    "position_rad": measured_after.position_rad,
                    "velocity_rad_s": measured_after.velocity_rad_s,
                    "torque_nm": measured_after.torque_nm,
                    "following_error_rad": measured_after.following_error_rad,
                    "encoder1_rad": measured_after.encoder1_rad,
                    "encoder2_rad": measured_after.encoder2_rad,
                    "sample_dt_s": sample_dt_s,
                    "read_before_dt_s": read_before_dt_s,
                    "controller_dt_s": controller_dt_s,
                    "send_dt_s": send_dt_s,
                    "read_after_dt_s": read_after_dt_s,
                    "diag_dt_s": diag_dt_s,
                    "cycle_total_dt_s": cycle_total_dt_s,
                }

                cycle += 1
                cycle_end_ns = monotonic_time_ns()
                next_deadline_ns = cycle_deadline_ns + period_ns
                sleep_target_ms = 0.0
                sleep_actual_ms = 0.0
                if cycle_end_ns > next_deadline_ns:
                    missed_cycles = ((cycle_end_ns - next_deadline_ns) // period_ns) + 1
                    deadline_miss_count += int(missed_cycles)
                    next_deadline_ns += int(missed_cycles) * period_ns
                elif cycle_end_ns < next_deadline_ns:
                    sleep_target_ms = (next_deadline_ns - cycle_end_ns) / 1_000_000.0
                    sleep_start_ns = monotonic_time_ns()
                    sleep_until_monotonic_ns(next_deadline_ns)
                    sleep_end_ns = monotonic_time_ns()
                    sleep_actual_ms = max(0.0, (sleep_end_ns - sleep_start_ns) / 1_000_000.0)
                cycle_deadline_ns = next_deadline_ns
                deadline_miss_delta = deadline_miss_count - prev_deadline_miss_count
                prev_deadline_miss_count = deadline_miss_count
                row["sleep_target_ms"] = sleep_target_ms
                row["sleep_actual_ms"] = sleep_actual_ms
                row["deadline_miss_delta"] = deadline_miss_delta
                row["deadline_miss_count"] = deadline_miss_count
                writer.writerow(row)

                if cycle % max(cfg.print_every, 1) == 0:
                    print_state(
                        f"run cycle={cycle} t={elapsed_s:.3f}s f={freq_hz:.3f}Hz wkc={wkc} cmd={action_nm:.2f}Nm",
                        measured_after,
                    )
                    print(
                        f"  ref={reference.theta_l_rad:.6f} meas={measured_after.position_rad:.6f} "
                        f"err={measured_after.position_rad - reference.theta_l_rad:+.6f} "
                        f"spring={last_spring_est.spring_torque_nm:.2f}Nm dob={disturbance_hat_nm:.2f}Nm "
                        f"sat_count={torque_saturation_count} deadline_miss={deadline_miss_count}",
                        flush=True,
                    )

                if stop_reason is not None:
                    print(f"ERROR: {stop_reason}. Sending zero torque and stopping.", flush=True)
                    break

        print("Sending zero torque before exit...", flush=True)
        zero_torque_shutdown(comm, cycles=20, cycle_valid_start=cycle)
        try:
            print_state("stop", comm.read_state_si())
        except Exception:
            pass
        print(f"CSV saved to: {csv_path}", flush=True)
        print(f"Total deadline misses: {deadline_miss_count}, torque saturation samples: {torque_saturation_count}", flush=True)

    finally:
        try:
            zero_torque_shutdown(comm, cycles=10, cycle_valid_start=cycle)
        except Exception:
            pass
        comm.close()

    if cfg.analyze_after_run:
        try:
            analyze_csv(csv_path, args)
        except Exception as exc:
            print(f"WARNING: analysis failed: {exc}", flush=True)
            print("You can run analysis later with: python control_flow/pd_dob_bandwidth_sweep.py analyze <csv>", flush=True)
    return csv_path


def _interp_crossing(x: list[float], y: list[float], threshold: float) -> float | None:
    if len(x) < 2:
        return None
    prev_x, prev_y = x[0], y[0]
    for cur_x, cur_y in zip(x[1:], y[1:]):
        if (prev_y - threshold) == 0:
            return prev_x
        if (prev_y - threshold) * (cur_y - threshold) <= 0 and cur_y != prev_y:
            ratio = (threshold - prev_y) / (cur_y - prev_y)
            return prev_x + ratio * (cur_x - prev_x)
        prev_x, prev_y = cur_x, cur_y
    return None


def analyze_csv(csv_path: Path, args: argparse.Namespace | None = None) -> None:
    import numpy as np

    try:
        import matplotlib.pyplot as plt
    except Exception as exc:  # pragma: no cover - only depends on user environment
        raise RuntimeError("matplotlib is required for plotting. Install it or use the Bode CSV only.") from exc

    csv_path = Path(csv_path).expanduser().resolve()
    rows: list[dict[str, str]] = []
    with csv_path.open("r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append(row)
    if not rows:
        raise RuntimeError(f"No rows found in {csv_path}")

    def arr(name: str) -> np.ndarray:
        return np.asarray([float(r[name]) for r in rows], dtype=np.float64)

    t = arr("time_s")
    freq = arr("freq_hz")
    phase = arr("phase_rad")
    ref = arr("ref_theta_l_rad")
    meas = arr("meas_theta_l_rad")
    env = arr("envelope") if "envelope" in rows[0] else np.ones_like(t)

    finite = np.isfinite(t) & np.isfinite(freq) & np.isfinite(phase) & np.isfinite(ref) & np.isfinite(meas)
    valid = finite & (freq > 0.0) & (env > 0.7)
    if int(np.sum(valid)) < 200:
        raise RuntimeError("Not enough valid chirp samples for analysis")

    f_min = max(float(np.nanmin(freq[valid])), 1e-6)
    f_max = float(np.nanmax(freq[valid]))
    n_points = int(getattr(args, "analysis_points", 90) if args is not None else 90)
    cycles_per_window = float(getattr(args, "cycles_per_window", 4.0) if args is not None else 4.0)
    min_points = int(getattr(args, "min_fit_points", 80) if args is not None else 80)

    target_freqs = np.geomspace(f_min, f_max, n_points)
    bode_rows: list[dict[str, float]] = []

    for fc in target_freqs:
        idx_center_candidates = np.where(valid)[0]
        if idx_center_candidates.size == 0:
            continue
        idx_center = int(idx_center_candidates[np.argmin(np.abs(freq[idx_center_candidates] - fc))])
        tc = t[idx_center]
        window_s = cycles_per_window / max(fc, 1e-6)
        local = valid & (t >= tc - 0.5 * window_s) & (t <= tc + 0.5 * window_s)
        if int(np.sum(local)) < min_points:
            continue

        phi = phase[local]
        y_ref = ref[local]
        y_meas = meas[local]
        # Hann-like weights centered at tc.
        tau = (t[local] - tc) / max(0.5 * window_s, 1e-9)
        weights = 0.5 + 0.5 * np.cos(np.clip(tau, -1.0, 1.0) * math.pi)
        weights = np.sqrt(np.maximum(weights, 1e-6))
        design = np.column_stack([np.ones_like(phi), np.sin(phi), np.cos(phi)])
        design_w = design * weights[:, None]
        beta_ref = np.linalg.lstsq(design_w, y_ref * weights, rcond=None)[0]
        beta_meas = np.linalg.lstsq(design_w, y_meas * weights, rcond=None)[0]

        amp_ref = float(math.hypot(beta_ref[1], beta_ref[2]))
        amp_meas = float(math.hypot(beta_meas[1], beta_meas[2]))
        if amp_ref < 1e-7:
            continue
        phase_ref = math.atan2(float(beta_ref[2]), float(beta_ref[1]))
        phase_meas = math.atan2(float(beta_meas[2]), float(beta_meas[1]))
        phase_diff = phase_meas - phase_ref
        mag_db = 20.0 * math.log10(max(amp_meas / amp_ref, 1e-12))
        bode_rows.append(
            {
                "frequency_hz": float(fc),
                "magnitude_db_raw": float(mag_db),
                "phase_deg_raw": float(math.degrees(phase_diff)),
                "amp_ref_rad": amp_ref,
                "amp_meas_rad": amp_meas,
                "center_time_s": float(tc),
                "fit_window_s": float(window_s),
                "fit_points": float(np.sum(local)),
            }
        )

    if len(bode_rows) < 5:
        raise RuntimeError("Too few Bode points produced; increase duration or reduce cycles_per_window")

    phase_raw = np.asarray([r["phase_deg_raw"] for r in bode_rows], dtype=np.float64)
    phase_unwrapped = np.rad2deg(np.unwrap(np.deg2rad(phase_raw)))
    mag_raw = np.asarray([r["magnitude_db_raw"] for r in bode_rows], dtype=np.float64)
    freqs = np.asarray([r["frequency_hz"] for r in bode_rows], dtype=np.float64)

    low_mask = freqs <= max(f_min * 1.8, min(0.35, f_max))
    if int(np.sum(low_mask)) < 2:
        low_mask = np.arange(len(freqs)) < max(2, min(5, len(freqs)))
    mag_baseline = float(np.median(mag_raw[low_mask]))
    phase_baseline = float(np.median(phase_unwrapped[low_mask]))
    mag_norm = mag_raw - mag_baseline
    phase_norm = phase_unwrapped - phase_baseline

    for row, mag_n, ph_n in zip(bode_rows, mag_norm, phase_norm):
        row["magnitude_db"] = float(mag_n)
        row["phase_deg"] = float(ph_n)

    bandwidth_3db = _interp_crossing(freqs.tolist(), mag_norm.tolist(), -3.0)
    phase_90 = _interp_crossing(freqs.tolist(), phase_norm.tolist(), -90.0)

    out_prefix = csv_path.with_suffix("")
    bode_csv = out_prefix.with_name(out_prefix.name + "_bode.csv")
    summary_json = out_prefix.with_name(out_prefix.name + "_summary.json")
    plot_png = out_prefix.with_name(out_prefix.name + "_bode.png")

    with bode_csv.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=[
                "frequency_hz",
                "magnitude_db",
                "phase_deg",
                "magnitude_db_raw",
                "phase_deg_raw",
                "amp_ref_rad",
                "amp_meas_rad",
                "center_time_s",
                "fit_window_s",
                "fit_points",
            ],
        )
        writer.writeheader()
        writer.writerows(bode_rows)

    summary = {
        "source_csv": str(csv_path),
        "bode_csv": str(bode_csv),
        "plot_png": str(plot_png),
        "mag_baseline_db": mag_baseline,
        "phase_baseline_deg": phase_baseline,
        "bandwidth_minus_3db_hz": bandwidth_3db,
        "phase_minus_90deg_hz": phase_90,
        "frequency_range_hz": [float(freqs[0]), float(freqs[-1])],
        "points": len(bode_rows),
        "method": "local sine fit using logged chirp phase; magnitude and phase normalized by low-frequency median",
    }
    with summary_json.open("w", encoding="utf-8") as f:
        json.dump(summary, f, ensure_ascii=False, indent=2)

    fig, ax_mag = plt.subplots(figsize=(9.0, 4.5))
    ax_phase = ax_mag.twinx()
    ax_mag.semilogx(freqs, mag_norm, linewidth=2.0, label="Magnitude")
    ax_phase.semilogx(freqs, phase_norm, linewidth=2.0, label="Phase")
    ax_mag.axhline(-3.0, linestyle="--", linewidth=1.0)
    ax_phase.axhline(-90.0, linestyle="--", linewidth=1.0)
    if bandwidth_3db is not None:
        ax_mag.axvline(bandwidth_3db, linestyle="--", linewidth=1.0)
        ax_mag.text(bandwidth_3db, -3.0, f"  -3 dB {bandwidth_3db:.2f} Hz", va="bottom")
    if phase_90 is not None:
        ax_phase.axvline(phase_90, linestyle="--", linewidth=1.0)
        ax_phase.text(phase_90, -90.0, f"  -90° {phase_90:.2f} Hz", va="top")
    ax_mag.set_xlabel("Frequency (Hz)")
    ax_mag.set_ylabel("Magnitude (dB)")
    ax_phase.set_ylabel("Phase (deg)")
    ax_mag.grid(True, which="both", linestyle=":", linewidth=0.8)
    lines1, labels1 = ax_mag.get_legend_handles_labels()
    lines2, labels2 = ax_phase.get_legend_handles_labels()
    ax_mag.legend(lines1 + lines2, labels1 + labels2, loc="best")
    fig.tight_layout()
    fig.savefig(plot_png, dpi=200)
    plt.close(fig)

    print(f"Bode CSV saved to: {bode_csv}", flush=True)
    print(f"Summary saved to: {summary_json}", flush=True)
    print(f"Bode plot saved to: {plot_png}", flush=True)
    print(
        f"Estimated -3 dB bandwidth: {bandwidth_3db if bandwidth_3db is not None else 'not crossed'} Hz; "
        f"-90 deg phase frequency: {phase_90 if phase_90 is not None else 'not crossed'} Hz",
        flush=True,
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="PD-DOB swept-sine bandwidth test for B-type SEA joint")
    subparsers = parser.add_subparsers(dest="command", required=True)

    run = subparsers.add_parser("run", help="Run EtherCAT PD-DOB chirp test and log CSV")
    run.add_argument("--ifname", default=DEFAULT_IFNAME)
    run.add_argument("--drive-index", type=int, default=1, help="EtherCAT slave index of the single SOMANET joint to control. The B joint closest to the master is usually slave[1].")
    run.add_argument("--cycle-time", type=float, default=DEFAULT_CYCLE_TIME_S)
    run.add_argument("--duration", type=float, default=15.0, help="Total duration including 3 s settle; default gives a 12 s linear sweep")
    run.add_argument("--settle-s", type=float, default=3.0)
    run.add_argument("--ramp-s", type=float, default=0.5)
    run.add_argument("--ramp-cycles", type=float, default=DEFAULT_RAMP_CYCLES, help="Cap each endpoint fade to this many local cycles")
    run.add_argument("--max-cycles-per-octave", type=float, default=DEFAULT_MAX_CYCLES_PER_OCTAVE, help="Safety cap on excitation cycles in every octave")
    run.add_argument("--amplitude-deg", type=float, default=3.0, help="Joint excitation amplitude: +/-3 deg")
    run.add_argument("--f-start", type=float, default=0.0, help="Paper-target linear chirp start frequency")
    run.add_argument("--f-end", type=float, default=6.0)
    run.add_argument("--execute", action="store_true", help="Actually send nonzero torque commands")
    run.add_argument("--kp", type=float, default=DEFAULT_KP_POSITION)
    run.add_argument("--kd", type=float, default=DEFAULT_KD_POSITION)
    run.add_argument("--dob-gain", type=float, default=DEFAULT_DOB_GAIN)
    run.add_argument("--max-torque-nm", type=float, default=DEFAULT_MAX_TORQUE_NM)
    run.add_argument("--max-dob-hat-nm", type=float, default=DEFAULT_MAX_DOB_HAT_NM)
    run.add_argument("--max-raw-action-nm", type=float, default=DEFAULT_MAX_RAW_ACTION_NM)
    run.add_argument("--hold-s", type=float, default=2.0, help="Active settling duration before and after loaded encoder calibration")
    run.add_argument("--hold-kp", type=float, default=800.0, help="PD hold Kp used during anti-drop/preload stage.")
    run.add_argument("--hold-kd", type=float, default=25.0, help="PD hold Kd used during anti-drop/preload stage.")
    run.add_argument("--hold-torque-limit-nm", type=float, default=45.0, help="Torque clamp for hold/preload stage.")
    run.add_argument("--hold-center-mode", choices=["pre_enable", "post_enable"], default="pre_enable", help="pre_enable tries to hold the position before brake release; post_enable holds the position after enable.")
    run.add_argument("--hold-print-every", type=int, default=50)
    run.add_argument("--counts-per-rev", type=int, default=DEFAULT_THIRD_ENCODER_COUNTS_PER_REV)
    run.add_argument("--third-encoder-sign", type=int, default=DEFAULT_THIRD_ENCODER_SIGN)
    run.add_argument("--spring-stiffness", type=float, default=DEFAULT_SPRING_STIFFNESS_NM_PER_RAD)
    run.add_argument("--calibration-samples", type=int, default=DEFAULT_CALIBRATION_SAMPLES)
    run.add_argument("--print-every", type=int, default=100)
    run.add_argument("--max-pos-error-deg", type=float, default=25.0)
    run.add_argument("--max-velocity-rad-s", type=float, default=12.0)
    run.add_argument("--max-consecutive-wkc-errors", type=int, default=3)
    run.add_argument("--cpu-affinity", type=int, nargs="*", default=list(DEFAULT_REALTIME_CPUS))
    run.add_argument("--rt-priority", type=int, default=DEFAULT_REALTIME_RT_PRIORITY)
    run.add_argument("--out-dir", default="logs")
    run.add_argument("--no-analyze", action="store_true", help="Do not analyze CSV after run")
    run.add_argument("--analysis-points", type=int, default=24)
    run.add_argument("--cycles-per-window", type=float, default=1.5)
    run.add_argument("--min-fit-points", type=int, default=50)
    run.set_defaults(func=run_bandwidth_test)

    analyze = subparsers.add_parser("analyze", help="Analyze a saved bandwidth CSV")
    analyze.add_argument("csv", type=Path)
    analyze.add_argument("--analysis-points", type=int, default=24)
    analyze.add_argument("--cycles-per-window", type=float, default=1.5)
    analyze.add_argument("--min-fit-points", type=int, default=50)
    analyze.set_defaults(func=lambda ns: analyze_csv(ns.csv, ns))
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    args.func(args)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
