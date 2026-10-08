#!/usr/bin/env python3
from __future__ import annotations

import csv
import ctypes
import errno
import math
import os
import resource
import serial
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
)
from controllers.sea_model import SeaState
from controllers.spring_sensor import SpringSensorConfig, SpringSensorEstimator


# =========================
# Runtime parameters
# =========================
IFNAME = "eno1"
CYCLE_TIME_S = 0.005
DURATION_S = 15.0
PRINT_EVERY = 20
EXECUTE = False

MAX_TORQUE_NM = 61.0
REALTIME_CPUS = (8, 9, 10, 11)
REALTIME_RT_PRIORITY = 99
CLOCK_MONOTONIC = 1
TIMER_ABSTIME = 1
NSEC_PER_SEC = 1_000_000_000

# 第三编码器 / 弹簧估计参数
THIRD_ENCODER_COUNTS_PER_REV = 1 << 19
THIRD_ENCODER_SIGN = 1
SPRING_STIFFNESS_NM_PER_RAD = 2400.0
CALIBRATION_SAMPLES = 32

# PD-DOB 参数
KP_POSITION = 85.0
KD_POSITION = 60.0
DOB_GAIN = -6.5
MAX_DOB_HAT_NM = 61.0
MAX_RAW_ACTION_NM = 61.0

# 位置轨迹参数
TRAJECTORY_MODE = "single"  # "single" or "composite"
TRAJECTORY_AMPLITUDE_RAD = 0.2
TRAJECTORY_FREQUENCY_HZ = 2.5
TRAJECTORY_PHASE_RAD = 0.0
TRAJECTORY_COMPOSITE_BIAS_RAD = 0.1
TRAJECTORY_COMPOSITE_COMPONENTS = (
    (0.30, 0.8*2*math.pi, -2*math.pi/3),
    (0.15, 0.5*2*math.pi, math.pi/4),
    (0.10, 0.3*2*math.pi, math.pi/2),
    (0.05, 1.3*2*math.pi, math.pi/3),
    # (0.30, 3.0, 0.0),
    # (0.15, 1.0, 0.0),
    # (0.45, 6.0, 0.0),
    # (0.20, 0.5, 0.0),
)


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


def describe_trajectory() -> str:
    if TRAJECTORY_MODE == "composite":
        return (
            f"mode={TRAJECTORY_MODE} bias={TRAJECTORY_COMPOSITE_BIAS_RAD:.6f}rad "
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
    return (
        f"mode={TRAJECTORY_MODE} amplitude={TRAJECTORY_AMPLITUDE_RAD:.6f}rad "
        f"frequency={TRAJECTORY_FREQUENCY_HZ:.6f}Hz phase={TRAJECTORY_PHASE_RAD:.6f}rad"
    )


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


def print_reference_terms(
    prefix: str,
    reference: SeaState,
    measured: DriveStateSI,
    spring_torque_nm: float,
    torque_ref_nm: float,
    raw_action_nm: float,
    action_nm: float,
    target_torque: int,
    disturbance_hat: float,
) -> None:
    err_theta_l = measured.position_rad - reference.theta_l_rad
    err_dtheta_l = measured.velocity_rad_s - reference.dtheta_l_rad_s
    print(
        (
            f"{prefix} ref_l={reference.theta_l_rad:.6f} meas_l={measured.position_rad:.6f} err_l={err_theta_l:+.6f} "
            f"ref_dl={reference.dtheta_l_rad_s:.6f} meas_dl={measured.velocity_rad_s:.6f} err_dl={err_dtheta_l:+.6f} "
            f"spring={spring_torque_nm:.3f}Nm torque_ref={torque_ref_nm:.3f}Nm "
            f"dob_hat={disturbance_hat:.3f}Nm u_raw={raw_action_nm:.3f}Nm cmd={action_nm:.3f}Nm target={target_torque}"
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


def print_timing_terms(
    prefix: str,
    read_before_dt_s: float,
    controller_dt_s: float,
    sample_dt_s: float,
    send_dt_s: float,
    read_after_dt_s: float,
    diag_dt_s: float,
    buffer_append_dt_s: float,
    cycle_total_dt_s: float,
) -> None:
    print(
        (
            f"{prefix} read_before={read_before_dt_s*1000.0:.3f}ms "
            f"controller={controller_dt_s*1000.0:.3f}ms sample={sample_dt_s*1000.0:.3f}ms "
            f"send={send_dt_s*1000.0:.3f}ms "
            f"read_after={read_after_dt_s*1000.0:.3f}ms diag={diag_dt_s*1000.0:.3f}ms "
            f"buffer_append={buffer_append_dt_s*1000.0:.3f}ms total={cycle_total_dt_s*1000.0:.3f}ms"
        ),
        flush=True,
    )


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
    dt: float = CYCLE_TIME_S,
) -> SeaState:
    if prev_encoder2_rad is not None:
        dtheta_m_rad_s = (measured.encoder2_rad - prev_encoder2_rad) / dt
    else:
        dtheta_m_rad_s = measured.velocity_rad_s
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
        max_dob_hat_nm: float = MAX_DOB_HAT_NM,
        max_raw_action_nm: float = MAX_RAW_ACTION_NM,
    ) -> None:
        self.kp_position = kp_position
        self.kd_position = kd_position
        self.dob_gain = dob_gain
        self.max_dob_hat_nm = max_dob_hat_nm
        self.max_raw_action_nm = max_raw_action_nm
        self.prev_time: float | None = None
        self.disturbance_hat = 0.0

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

        if self.prev_time is None:
            dt = 0.0
        else:
            dt = max(now_s - self.prev_time, 1e-6)

        torque_error_nm = torque_ref_nm - spring_torque_nm
        self.disturbance_hat += self.dob_gain * torque_error_nm * dt
        self.disturbance_hat = max(min(self.disturbance_hat, self.max_dob_hat_nm), -self.max_dob_hat_nm)
        raw_action_nm = torque_ref_nm - self.disturbance_hat
        raw_action_nm = max(min(raw_action_nm, self.max_raw_action_nm), -self.max_raw_action_nm)
        self.prev_time = now_s
        return torque_ref_nm, raw_action_nm, torque_error_nm, self.disturbance_hat


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


def main() -> int:
    configure_realtime_runtime()
    try:
        current_affinity = sorted(os.sched_getaffinity(0))
    except Exception:
        current_affinity = []
    print(
        f"Deadline scheduling enabled: cycle={CYCLE_TIME_S*1000.0:.3f}ms affinity={current_affinity}",
        flush=True,
    )

    # 初始化继电器控制
    relay_ser = None
    relay_opened = False
    relay_closed_at_4s = False
    relay_on_cmd = bytes([0x01, 0x05, 0x00, 0x00, 0xFF, 0x00, 0x8C, 0x3A])
    relay_off_cmd = bytes([0x01, 0x05, 0x00, 0x00, 0x00, 0x00, 0xCD, 0xCA])
    try:
        relay_ser = serial.Serial(
            port='/dev/ttyUSB0',
            baudrate=115200,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=1
        )
        print(f"Relay control initialized on {relay_ser.port}", flush=True)
    except Exception as e:
        print(f"Warning: Failed to initialize relay control: {e}", flush=True)
        relay_ser = None

    comm = SEARealtimeComm(ifname=IFNAME, cycle_time_s=CYCLE_TIME_S)
    spring_estimator = SpringSensorEstimator(
        SpringSensorConfig(
            counts_per_rev=THIRD_ENCODER_COUNTS_PER_REV,
            zero_counts=0,
            sign=THIRD_ENCODER_SIGN,
            spring_stiffness_nm_per_rad=SPRING_STIFFNESS_NM_PER_RAD,
        )
    )
    controller = PdDobController(
        kp_position=KP_POSITION,
        kd_position=KD_POSITION,
        dob_gain=DOB_GAIN,
    )

    prev_encoder2_rad = None

    try:
        # =========================
        # Stage 0: connect EtherCAT
        # =========================
        comm.connect()
        print_slave_inventory(comm)
        print(
            f"Connected on {IFNAME}. PD-DOB mode={CST_MODE}, execute={EXECUTE}, "
            f"duration={DURATION_S}s controller=kp={KP_POSITION} kd={KD_POSITION} dob={DOB_GAIN} "
            f"trajectory={describe_trajectory()}",
            flush=True,
        )

        print("Stage 0.5: box transparent PRE-OP configuration", flush=True)
        third_encoder_cfg = comm.configure_box_transparent_preop()
        print(
            "Third encoder transparent configured: "
            f"mode={third_encoder_cfg['mode']} interface={third_encoder_cfg['interface']} "
            f"frame={third_encoder_cfg['frame']} baud_selector={third_encoder_cfg['baud_selector']} "
            f"baud={third_encoder_cfg['explicit_baud']} polling_ms={third_encoder_cfg['polling_ms']}",
            flush=True,
        )

        # =========================
        # Stage 1: PDO preheat
        # =========================
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

        # =========================
        # Stage 2: CiA402 enable
        # =========================
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
                    f"  step={result.step_name} success={result.success} "
                    f"expected_state={result.expected_state} "
                    f"wkc={summarize_wkc(result.observed_wkc)} "
                    f"statusword=0x{diag.statusword:04X} mode={diag.mode_display} "
                    f"cia402={diag.cia402_state}"
                ),
                flush=True,
            )
            if not result.success:
                raise RuntimeError(f"CiA402 transition failed at step {result.step_name}")

        # =========================
        # Stage 3: enabled-state stabilization
        # =========================
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

        # =========================
        # Stage 4: third encoder init
        # =========================
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

        # 使能完成后，重新读取当前真实位置作为初始状态。
        initial_state = comm.read_state_si()
        initial_diag = comm.read_drive_diagnostics()
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
        print(
            "PD-DOB reference updated after enable: initial_position={pos:.6f}rad "
            "initial_motor_load_offset={offset:.6f}rad".format(
                pos=initial_position_rad,
                offset=initial_motor_load_offset_rad,
            ),
            flush=True,
        )

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        log_dir = Path(__file__).resolve().parents[1] / "logs"
        log_dir.mkdir(exist_ok=True)
        try:
            os.chmod(log_dir, 0o777)
        except OSError:
            pass
        csv_path = log_dir / f"pd_dob_data_{timestamp}.csv"
        data_buffer: list[list[object]] = []
        print(f"Will save data to: {csv_path}", flush=True)

        start_ns = monotonic_time_ns()
        stop_ns = start_ns + max(int(round(DURATION_S * NSEC_PER_SEC)), 0)
        period_ns = normalize_cycle_time_ns(CYCLE_TIME_S)
        cycle_deadline_ns = start_ns
        deadline_miss_count = 0
        prev_deadline_miss_count = 0
        cycle = 0
        last_spring_est = spring_estimator.estimate(zero_counts)
        expected_wkc = comm.expected_wkc
        wkc_error_count = 0
        max_wkc_errors = 3

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
            wake_lag_ms = max(0.0, (loop_start_ns - cycle_deadline_ns) / 1_000_000.0)
            deadline_slip_ms = wake_lag_ms
            cycle_total_start = time.monotonic()
            elapsed_s = (loop_start_ns - start_ns) / NSEC_PER_SEC

            # 在第10秒打开继电器
            if relay_ser is not None and not relay_opened and elapsed_s >= 5.0:
                try:
                    relay_ser.write(relay_on_cmd)
                    relay_opened = True
                    print(f"[{elapsed_s:.3f}s] Relay opened", flush=True)
                except Exception as e:
                    print(f"Warning: Failed to open relay: {e}", flush=True)

            # 在第20秒关闭继电器
            if relay_ser is not None and relay_opened and not relay_closed_at_4s and elapsed_s >= 10.0:
                try:
                    relay_ser.write(relay_off_cmd)
                    relay_closed_at_4s = True
                    print(f"[{elapsed_s:.3f}s] Relay closed", flush=True)
                except Exception as e:
                    print(f"Warning: Failed to close relay: {e}", flush=True)

            sample_start = time.monotonic()
            sample = comm.poll_queued_third_encoder_sample(timeout_us=0)
            sample_dt_s = time.monotonic() - sample_start
            wkc = sample.wkc if sample is not None else comm.last_processdata_wkc

            if expected_wkc is not None and wkc != expected_wkc:
                wkc_error_count += 1
                print(
                    f"WARNING: wkc={wkc} (expected {expected_wkc}), error_count={wkc_error_count}",
                    flush=True,
                )
                if wkc_error_count >= max_wkc_errors:
                    print("ERROR: Too many wkc errors, stopping control!", flush=True)
                    break
            else:
                wkc_error_count = 0

            valid_third_encoder = sample is not None and (
                sample.response.crc_ok
                and not sample.response.encoder_error
                and not sample.response.communication_alarm
            )

            if sample is None:
                pass
            elif not valid_third_encoder:
                print(
                    f"WARNING: invalid third encoder frame at cycle={cycle}, "
                    f"crc_ok={int(sample.response.crc_ok)} enc_err={int(sample.response.encoder_error)} "
                    f"comm_alarm={int(sample.response.communication_alarm)} status=0x{sample.response.status:02X}",
                    flush=True,
                )
            else:
                last_spring_est = spring_estimator.estimate(sample.response.position_21bit)

            read_before_start = time.monotonic()
            measured_before = comm.read_state_si()
            read_before_dt_s = time.monotonic() - read_before_start

            controller_start = time.monotonic()
            reference = build_reference(elapsed_s, initial_position_rad, initial_motor_load_offset_rad)
            measured_state = measured_to_sea_state(
                measured_before,
                prev_encoder2_rad=prev_encoder2_rad,
                dt=CYCLE_TIME_S,
            )

            torque_ref_nm, raw_action_nm, torque_error_nm, disturbance_hat = controller.update(
                reference=reference,
                measured=measured_before,
                spring_torque_nm=last_spring_est.spring_torque_nm,
                now_s=elapsed_s,
            )
            controller_dt_s = time.monotonic() - controller_start
            action_nm = raw_action_nm if EXECUTE else 0.0
            target_torque = torque_nm_to_target_units(action_nm, MAX_TORQUE_NM)

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
            if abs(err_theta_l) > 5.0 or abs(err_theta_m) > 5.0:
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

            err_dtheta_l = measured_state.dtheta_l_rad_s - reference.dtheta_l_rad_s
            err_dtheta_m = measured_state.dtheta_m_rad_s - reference.dtheta_m_rad_s

            buffer_append_start = time.monotonic()
            row = [
                cycle,
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
                measured_state.theta_m_rad,
                measured_state.theta_l_rad,
                measured_state.dtheta_m_rad_s,
                measured_state.dtheta_l_rad_s,
                err_theta_m,
                err_theta_l,
                err_dtheta_m,
                err_dtheta_l,
                torque_ref_nm,
                controller.disturbance_hat,
                raw_action_nm,
                action_nm,
                target_torque,
                wkc,
                sample.response.position_21bit if sample is not None else -1,
                last_spring_est.delta_theta_rad,
                last_spring_est.spring_torque_nm,
                measured_after.statusword,
                measured_after.position_rad,
                measured_after.velocity_rad_s,
                measured_after.torque_nm,
                measured_after.encoder1_rad,
                measured_after.encoder2_rad,
                int(valid_third_encoder),
                read_before_dt_s,
                controller_dt_s,
                sample_dt_s,
                send_dt_s,
                read_after_dt_s,
                diag_dt_s,
                0.0,
                time.monotonic() - cycle_total_start,
            ]
            data_buffer.append(row)
            buffer_append_dt_s = time.monotonic() - buffer_append_start
            row[-2] = buffer_append_dt_s

            if cycle % max(PRINT_EVERY, 1) == 0:
                print_state(
                    f"run cycle={cycle} t={elapsed_s:.3f}s wkc={wkc} sample_dt={sample_dt_s*1000.0:.3f}ms",
                    measured_after,
                )
                print_reference_terms(
                    "ctrl",
                    reference,
                    measured_after,
                    spring_torque_nm=last_spring_est.spring_torque_nm,
                    torque_ref_nm=torque_ref_nm,
                    raw_action_nm=raw_action_nm,
                    action_nm=action_nm,
                    target_torque=target_torque,
                    disturbance_hat=controller.disturbance_hat,
                )
                if sample is not None:
                    print_third_encoder_sample(f"spring cycle={cycle}", sample)
                else:
                    print(
                        "spring poll: no fresh frame, using cached estimate "
                        f"delta_theta={last_spring_est.delta_theta_rad:.6f}rad "
                        f"spring_torque={last_spring_est.spring_torque_nm:.3f}Nm",
                        flush=True,
                    )
                print_timing_terms(
                    "timing",
                    read_before_dt_s=read_before_dt_s,
                    controller_dt_s=controller_dt_s,
                    sample_dt_s=sample_dt_s,
                    send_dt_s=send_dt_s,
                    read_after_dt_s=read_after_dt_s,
                    diag_dt_s=diag_dt_s,
                    buffer_append_dt_s=buffer_append_dt_s,
                    cycle_total_dt_s=time.monotonic() - cycle_total_start,
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
            row[3] = deadline_slip_ms
            row[4] = sleep_target_ms
            row[5] = sleep_actual_ms
            row[6] = deadline_miss_delta
            row[7] = deadline_miss_count

        # 退出前把力矩指令清零，避免最后一帧还保持受力。
        for _ in range(10):
            command = DriveCommand(
                controlword=ENABLE_OPERATION,
                mode_of_operation=CST_MODE,
                target_torque=0,
            )
            comm.queue_third_encoder_request(
                command=command,
                valid=cycle & 0xFF,
            )
            comm.send_processdata()
            try:
                sample = comm.collect_queued_third_encoder_sample(timeout_us=5_000)
                wkc = sample.wkc
            except Exception:
                wkc = comm.last_processdata_wkc
            state = comm.read_state_si()
        print_state(f"stop wkc={wkc}", state)
        if deadline_miss_count:
            print(f"Formal deadline misses: {deadline_miss_count}", flush=True)

        print("Writing data to CSV...", flush=True)
        with open(csv_path, "w", newline="") as csv_file:
            csv_writer = csv.writer(csv_file)
            csv_writer.writerow(
                [
                    "cycle",
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
                    "torque_ref_nm",
                    "disturbance_hat_nm",
                    "raw_action_nm",
                    "action_nm",
                    "target_torque",
                    "wkc",
                    "third_encoder_pos21",
                    "spring_delta_theta_rad",
                    "spring_torque_nm",
                    "statusword",
                    "position_rad",
                    "velocity_rad_s",
                    "torque_nm",
                    "encoder1_rad",
                    "encoder2_rad",
                    "third_encoder_crc_ok",
                    "read_before_dt_s",
                    "controller_dt_s",
                    "sample_dt_s",
                    "send_dt_s",
                    "read_after_dt_s",
                    "diag_dt_s",
                    "buffer_append_dt_s",
                    "cycle_total_dt_s",
                ]
            )
            csv_writer.writerows(data_buffer)

        try:
            os.chmod(csv_path, 0o666)
        except OSError:
            pass

        print(f"Data saved to: {csv_path}", flush=True)
        print("PD-DOB control flow done.", flush=True)

        # 关闭继电器
        if relay_ser is not None:
            try:
                relay_ser.write(relay_off_cmd)
                print("Relay closed", flush=True)
                relay_ser.close()
            except Exception as e:
                print(f"Warning: Failed to close relay: {e}", flush=True)

        return 0
    finally:
        comm.close()
        # 确保继电器在异常退出时也能关闭
        if relay_ser is not None and relay_ser.is_open:
            try:
                relay_ser.write(relay_off_cmd)
                relay_ser.close()
                print("Relay closed (cleanup)", flush=True)
            except Exception:
                pass


if __name__ == "__main__":
    raise SystemExit(main())
