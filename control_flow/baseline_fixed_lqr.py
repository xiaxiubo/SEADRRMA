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
from types import SimpleNamespace

# 直接运行本文件时，把项目根目录加入搜索路径，保证能导入正式包。
if __package__ in {None, ""}:
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from communication.sea_motor_comm import (
    CST_MODE,
    ENABLE_OPERATION,
    DriveCommand,
    DriveStateSI,
    EncoderAssessment,
    PdoPreheatResult,
    SEARealtimeComm,
    SHUTDOWN,
    summarize_wkc,
)
from controllers.lqr import LqrController
from controllers.sea_model import SeaState


# =========================
# 这里是最常改的运行参数
# 你后面只需要改这个块里的常量
# =========================
IFNAME = "eno1"
CYCLE_TIME_S = float(os.environ.get("BASELINE_CYCLE_TIME_S", "0.005"))
DURATION_S = float(os.environ.get("BASELINE_DURATION_S", "30.0"))
PRINT_EVERY = int(os.environ.get("BASELINE_PRINT_EVERY", "20"))
EXECUTE = False

# 力矩换算相关参数
MAX_TORQUE_NM = 61.0
APPLIED_TORQUE_LIMIT_NM = float(os.environ.get("BASELINE_TORQUE_LIMIT_NM", "61.0"))
MAX_POSITION_ERROR_RAD = float(os.environ.get("BASELINE_MAX_POSITION_ERROR_RAD", "0.8"))
RELAY_CONTROL_ENABLED = os.environ.get("BASELINE_RELAY_CONTROL", "0") == "1"
MOTOR_VELOCITY_FILTER_ALPHA = float(
    os.environ.get("BASELINE_MOTOR_VELOCITY_FILTER_ALPHA", "1.0")
)
REALTIME_CPUS = (8, 9, 10, 11)
REALTIME_RT_PRIORITY = 99
CLOCK_MONOTONIC = 1
TIMER_ABSTIME = 1
NSEC_PER_SEC = 1_000_000_000

# LQR 控制器参数文件。调参时可选择独立 YAML，避免覆盖基线参数。
CONTROLLER_YAML = Path(
    os.environ.get(
        "BASELINE_LQR_YAML",
        str(Path(__file__).resolve().parents[1] / "controllers" / "lqr_params.yaml"),
    )
).expanduser()

# 正弦轨迹参数
# 注意：这里的 amplitude 是峰值，不是峰峰值。
TRAJECTORY_MODE = os.environ.get("BASELINE_TRAJECTORY_MODE", "composite")
TRAJECTORY_AMPLITUDE_RAD = float(os.environ.get("BASELINE_AMPLITUDE_RAD", "0.3"))
TRAJECTORY_FREQUENCY_HZ = float(os.environ.get("BASELINE_FREQUENCY_HZ", "1.0"))
TRAJECTORY_PHASE_RAD = 0  # math.pi / 2.0
TRAJECTORY_COMPOSITE_BIAS_RAD = 0.1
TRAJECTORY_COMPOSITE_COMPONENTS = (
    (0.30, 1.5*2*math.pi, -2*math.pi/3),
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
    # 0x6071 目标力矩是按额定最大力矩的 1/1000 来表示的。
    # 这里把物理单位 Nm 换算成驱动器需要的归一化数值。
    if max_torque_nm <= 0.0:
        raise ValueError("max_torque_nm must be positive.")
    target = round(torque_nm / max_torque_nm * 1000.0)
    return int(max(min(target, 1000), -1000))


def print_state(prefix: str, state) -> None:
    # 输出尽量保持和 CST demo 一致，只多保留调参最关键的几个量。
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


def print_lqr_terms(prefix: str, reference: SeaState, measured: SeaState, raw_action_nm: float, action_nm: float, target_torque: int) -> None:
    # 把 LQR 计算里的状态、误差和输出展开打印出来。
    # 这里的误差定义与控制器保持一致：measured - reference。
    err_theta_m = measured.theta_m_rad - reference.theta_m_rad
    err_dtheta_m = measured.dtheta_m_rad_s - reference.dtheta_m_rad_s
    err_theta_l = measured.theta_l_rad - reference.theta_l_rad
    err_dtheta_l = measured.dtheta_l_rad_s - reference.dtheta_l_rad_s
    print(
        (
            f"{prefix} ref_m={reference.theta_m_rad:.6f} meas_m={measured.theta_m_rad:.6f} err_m={err_theta_m:+.6f} "
            f"ref_l={reference.theta_l_rad:.6f} meas_l={measured.theta_l_rad:.6f} err_l={err_theta_l:+.6f} "
            f"ref_dm={reference.dtheta_m_rad_s:.6f} meas_dm={measured.dtheta_m_rad_s:.6f} err_dm={err_dtheta_m:+.6f} "
            f"ref_dl={reference.dtheta_l_rad_s:.6f} meas_dl={measured.dtheta_l_rad_s:.6f} err_dl={err_dtheta_l:+.6f} "
            f"u_raw={raw_action_nm:.3f}Nm cmd={action_nm:.3f}Nm target={target_torque}"
        ),
        flush=True,
    )


def print_timing_terms(
    prefix: str,
    read_before_dt_s: float,
    controller_dt_s: float,
    send_dt_s: float,
    read_after_dt_s: float,
    diag_dt_s: float,
    buffer_append_dt_s: float,
    cycle_total_dt_s: float,
) -> None:
    print(
        (
            f"{prefix} read_before={read_before_dt_s*1000.0:.3f}ms "
            f"controller={controller_dt_s*1000.0:.3f}ms send={send_dt_s*1000.0:.3f}ms "
            f"read_after={read_after_dt_s*1000.0:.3f}ms diag={diag_dt_s*1000.0:.3f}ms "
            f"buffer_append={buffer_append_dt_s*1000.0:.3f}ms total={cycle_total_dt_s*1000.0:.3f}ms"
        ),
        flush=True,
    )


def print_encoder_assessment(assessment: EncoderAssessment) -> None:
    config = assessment.config
    feedback = assessment.last_feedback
    print(
        (
            f"Encoder {assessment.channel} assessment: type={config.encoder_type_name} port={config.sensor_port} "
            f"resolution={config.resolution} polarity={int(config.polarity)} offset={config.singleturn_offset} "
            f"index={config.index_availability} raw={feedback.raw_position} adjusted={feedback.adjusted_position} "
            f"pdo_last={assessment.pdo_encoder_samples[-1] if assessment.pdo_encoder_samples else 'n/a'} "
            f"velocity_rpm={feedback.velocity_rpm} adjusted_valid={int(assessment.adjusted_valid_assumed)} "
            f"status={assessment.status_label}"
        ),
        flush=True,
    )
    print(f"  detail={assessment.detail}", flush=True)


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


def print_preheat_summary(label: str, result: PdoPreheatResult) -> None:
    diag = result.last_diagnostics
    print(
        (
            f"{label}: success={result.success} expected_wkc={result.expected_wkc} "
            f"observed_wkc={summarize_wkc(result.observed_wkc)} stable={result.consecutive_stable_cycles}/"
            f"{result.stable_cycles_required} statusword=0x{diag.statusword:04X} mode={diag.mode_display} "
            f"cia402={diag.cia402_state} remote={int(diag.remote)} warning={int(diag.warning)} fault={int(diag.fault)}"
        ),
        flush=True,
    )
    if not result.success and result.samples:
        print(result.sample_summary(), flush=True)


def build_reference(
    elapsed_s: float,
    initial_position_rad: float,
    initial_motor_load_offset_rad: float,
) -> SeaState:
    # 先生成末端侧正弦，再把初始电机-末端偏置加回去，得到完整 4 状态参考。
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
    gear_ratio: float,
    prev_encoder2_rad: float | None = None,
    dt: float = CYCLE_TIME_S,
    motor_encoder_adjusted_valid: bool = True,
) -> SeaState:
    """
    把驱动器读到的 SI 反馈转换成控制器需要的 4 维状态。

    Args:
        measured: 驱动器状态
        gear_ratio: 减速比（未使用，保留接口兼容性）
        prev_encoder2_rad: 上一次的电机侧编码器角度，用于计算电机侧速度
        dt: 采样周期
        motor_encoder_adjusted_valid: 电机侧 adjusted position 是否已确认有效

    Returns:
        SeaState: 包含电机侧和负载侧位置、速度的状态
    """
    if not motor_encoder_adjusted_valid:
        raise ValueError("encoder2 adjusted position is not valid enough for LQR state estimation")

    # encoder2_rad 是电机侧角度（已经换算到关节侧）
    # velocity_rad_s 是负载侧速度（从输出编码器计算）

    # 计算电机侧速度：通过对 encoder2_rad 求导
    if prev_encoder2_rad is not None:
        dtheta_m_rad_s = (measured.encoder2_rad - prev_encoder2_rad) / dt
    else:
        # 第一帧没有历史数据，假设电机侧速度等于负载侧速度
        dtheta_m_rad_s = measured.velocity_rad_s

    return SeaState(
        theta_m_rad=measured.encoder2_rad,
        dtheta_m_rad_s=dtheta_m_rad_s,
        theta_l_rad=measured.position_rad,
        dtheta_l_rad_s=measured.velocity_rad_s,
    )


def main() -> int:
    configure_realtime_runtime()
    if not 0.0 < MOTOR_VELOCITY_FILTER_ALPHA <= 1.0:
        raise ValueError("BASELINE_MOTOR_VELOCITY_FILTER_ALPHA must be in (0, 1].")
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
        if not RELAY_CONTROL_ENABLED:
            raise RuntimeError("relay control disabled for fixed-inertia test")
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

    # 控制器从 YAML 里读 K，轨迹参数直接写在本文件顶部。
    controller = LqrController(param_path=CONTROLLER_YAML)
    comm = SEARealtimeComm(ifname=IFNAME, cycle_time_s=CYCLE_TIME_S)

    # 用于计算电机侧速度的历史数据
    prev_encoder2_rad = None
    prev_time = None
    filtered_motor_velocity_rad_s = None

    try:
        # 先完成 EtherCAT 上电、扫描、PDO 映射和 OP 切换。
        comm.connect()
        print_slave_inventory(comm)
        print(
            f"Connected on {IFNAME}. LQR mode={CST_MODE}, execute={EXECUTE}, controller_yaml={CONTROLLER_YAML}, "
            f"trajectory={describe_trajectory()} duration={DURATION_S}s",
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
        max_enable_attempts = 2
        enable_ok = False
        for attempt in range(max_enable_attempts):
            print(f"  enable attempt {attempt + 1}/{max_enable_attempts}", flush=True)
            transition_results = comm.enable_cia402(
                mode_of_operation=CST_MODE,
                target_torque=0,
                timeout_cycles_per_step=300,
            )
            failed_step = None
            for result in transition_results:
                diag = result.last_diagnostics
                print(
                    (
                        f"  step={result.step_name} success={result.success} expected_state={result.expected_state} "
                        f"wkc={summarize_wkc(result.observed_wkc)} statusword=0x{diag.statusword:04X} "
                        f"mode={diag.mode_display} cia402={diag.cia402_state} remote={int(diag.remote)} "
                        f"warning={int(diag.warning)} fault={int(diag.fault)}"
                    ),
                    flush=True,
                )
                if not result.success:
                    if result.samples:
                        print(result.sample_summary(), flush=True)
                    failed_step = result.step_name
                    break
            if failed_step is None:
                enable_ok = True
                break
            print(
                f"  enable failed at {failed_step}, sending extra SHUTDOWN wake-up cycles...",
                flush=True,
            )
            for _ in range(100):
                comm.cycle(
                    DriveCommand(
                        controlword=SHUTDOWN,
                        mode_of_operation=CST_MODE,
                        target_torque=0,
                    )
                )
        if not enable_ok:
            raise RuntimeError("CiA 402 transition failed after retry")

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

        print("Stage 4: encoder feedback assessment", flush=True)
        encoder_command = DriveCommand(
            controlword=ENABLE_OPERATION,
            mode_of_operation=CST_MODE,
            target_torque=0,
        )
        try:
            encoder1_assessment = comm.sample_encoder_assessment(channel=1, cycles=20, command=encoder_command)
            encoder2_assessment = comm.sample_encoder_assessment(channel=2, cycles=20, command=encoder_command)
            print_encoder_assessment(encoder1_assessment)
            print_encoder_assessment(encoder2_assessment)
        except Exception as exc:
            if os.environ.get("BASELINE_ALLOW_CACHED_ENCODER_CONFIG", "0") != "1":
                raise
            print(
                "Warning: encoder SDO assessment failed; using the encoder validity "
                f"confirmed by the earlier hardware smoke test: {exc}",
                flush=True,
            )
            encoder1_assessment = SimpleNamespace(
                adjusted_valid_assumed=True,
                status_label="cached_valid_from_smoke",
                detail="Encoder validity confirmed in the preceding LQR smoke test.",
            )
            encoder2_assessment = SimpleNamespace(
                adjusted_valid_assumed=True,
                status_label="cached_valid_from_smoke",
                detail="Encoder validity confirmed in the preceding LQR smoke test.",
            )
        if not encoder2_assessment.adjusted_valid_assumed:
            raise RuntimeError(
                "Encoder2 adjusted position is not valid for LQR control. "
                f"status={encoder2_assessment.status_label} detail={encoder2_assessment.detail}"
            )

        # 使能完成后，重新读取当前真实位置作为初始状态
        # 这一步非常重要！确保参考轨迹从当前位置开始，避免巨大的初始误差
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
            "LQR reference updated after enable: initial_position={pos:.6f}rad "
            "initial_motor_load_offset={offset:.6f}rad".format(
                pos=initial_position_rad,
                offset=initial_motor_load_offset_rad,
            ),
            flush=True,
        )

        # 准备 CSV 数据记录
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        log_dir = Path(os.environ.get("BASELINE_LOG_DIR", Path(__file__).resolve().parents[1] / "logs"))
        log_dir.mkdir(parents=True, exist_ok=True)
        try:
            os.chmod(log_dir, 0o777)
        except OSError:
            pass
        csv_path = log_dir / f"lqr_data_{timestamp}.csv"

        # 先不打开 CSV 文件，收集数据到内存，结束后再写入
        # 这样避免 I/O 操作影响实时性
        data_buffer = []
        print(f"Will save data to: {csv_path}", flush=True)

        # 先热身一次 LQR 矩阵计算，避免第一次 numpy/BLAS 调用污染正式 2ms 周期统计。
        warmup_reference = build_reference(0.0, initial_position_rad, initial_motor_load_offset_rad)
        warmup_state = measured_to_sea_state(
            initial_state,
            comm.calibration.gear_ratio,
            prev_encoder2_rad=None,
            dt=CYCLE_TIME_S,
            motor_encoder_adjusted_valid=encoder2_assessment.adjusted_valid_assumed,
        )
        _ = controller.control(warmup_state, warmup_reference)

        # 进入 LQR 正式控制循环。周期由绝对 deadline 调度控制。
        start_ns = monotonic_time_ns()
        stop_ns = start_ns + max(int(round(DURATION_S * NSEC_PER_SEC)), 0)
        period_ns = normalize_cycle_time_ns(CYCLE_TIME_S)
        cycle_deadline_ns = start_ns
        deadline_miss_count = 0
        prev_deadline_miss_count = 0
        cycle = 0
        last_state = None
        expected_wkc = comm.expected_wkc
        wkc_error_count = 0
        max_wkc_errors = 3

        while True:
            loop_start_ns = monotonic_time_ns()
            if loop_start_ns >= stop_ns:
                break
            wake_lag_ms = max(0.0, (loop_start_ns - cycle_deadline_ns) / 1_000_000.0)
            deadline_slip_ms = wake_lag_ms
            cycle_total_start = time.monotonic()
            elapsed_s = (loop_start_ns - start_ns) / NSEC_PER_SEC

            # 在第2秒打开继电器
            if relay_ser is not None and not relay_opened and elapsed_s >= 10.0:
                try:
                    relay_ser.write(relay_on_cmd)
                    relay_opened = True
                    print(f"[{elapsed_s:.3f}s] Relay opened", flush=True)
                except Exception as e:
                    print(f"Warning: Failed to open relay: {e}", flush=True)

            # 在第4秒关闭继电器
            if relay_ser is not None and relay_opened and not relay_closed_at_4s and elapsed_s >= 20.0:
                try:
                    relay_ser.write(relay_off_cmd)
                    relay_closed_at_4s = True
                    print(f"[{elapsed_s:.3f}s] Relay closed", flush=True)
                except Exception as e:
                    print(f"Warning: Failed to close relay: {e}", flush=True)

            # 先读反馈，再根据当前状态计算下一帧力矩。
            read_before_start = time.monotonic()
            measured_before = comm.read_state_si()
            read_before_dt_s = time.monotonic() - read_before_start

            controller_start = time.monotonic()
            reference = build_reference(elapsed_s, initial_position_rad, initial_motor_load_offset_rad)
            measured_state = measured_to_sea_state(
                measured_before,
                comm.calibration.gear_ratio,
                prev_encoder2_rad=prev_encoder2_rad,
                dt=CYCLE_TIME_S,
                motor_encoder_adjusted_valid=encoder2_assessment.adjusted_valid_assumed,
            )
            raw_motor_velocity_rad_s = measured_state.dtheta_m_rad_s
            if filtered_motor_velocity_rad_s is None:
                filtered_motor_velocity_rad_s = raw_motor_velocity_rad_s
            else:
                filtered_motor_velocity_rad_s += MOTOR_VELOCITY_FILTER_ALPHA * (
                    raw_motor_velocity_rad_s - filtered_motor_velocity_rad_s
                )
            measured_state = SeaState(
                theta_m_rad=measured_state.theta_m_rad,
                dtheta_m_rad_s=filtered_motor_velocity_rad_s,
                theta_l_rad=measured_state.theta_l_rad,
                dtheta_l_rad_s=measured_state.dtheta_l_rad_s,
            )
            raw_action_nm = float(controller.control(measured_state, reference))
            action_nm = (
                max(-APPLIED_TORQUE_LIMIT_NM, min(APPLIED_TORQUE_LIMIT_NM, raw_action_nm))
                if EXECUTE
                else 0.0
            )
            target_torque = torque_nm_to_target_units(action_nm, MAX_TORQUE_NM)
            controller_dt_s = time.monotonic() - controller_start

            # 更新历史数据用于下一次速度计算
            prev_encoder2_rad = measured_before.encoder2_rad
            prev_time = elapsed_s

            # 过程数据交换：把下一帧目标力矩发给驱动器。
            send_start = time.monotonic()
            wkc = comm.exchange_cycle(
                DriveCommand(
                    controlword=ENABLE_OPERATION,
                    mode_of_operation=CST_MODE,
                    target_torque=target_torque,
                ),
                sleep=False,
            )
            send_dt_s = time.monotonic() - send_start

            # 安全检查：监控 wkc
            if expected_wkc is not None and wkc != expected_wkc:
                wkc_error_count += 1
                print(f"WARNING: wkc={wkc} (expected {expected_wkc}), error_count={wkc_error_count}", flush=True)
                if wkc_error_count >= max_wkc_errors:
                    print("ERROR: Too many wkc errors, stopping control!", flush=True)
                    break
            else:
                wkc_error_count = 0  # 重置错误计数

            read_after_start = time.monotonic()
            measured_after = comm.read_state_si()
            read_after_dt_s = time.monotonic() - read_after_start
            diag_start = time.monotonic()
            measured_diag = comm.read_drive_diagnostics()
            diag_dt_s = time.monotonic() - diag_start
            last_state = measured_after

            # 安全检查：如果误差过大，立即停止
            err_theta_m = measured_state.theta_m_rad - reference.theta_m_rad
            err_theta_l = measured_state.theta_l_rad - reference.theta_l_rad
            if abs(err_theta_m) > MAX_POSITION_ERROR_RAD or abs(err_theta_l) > MAX_POSITION_ERROR_RAD:
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

            # 记录数据到内存缓冲区
            err_dtheta_m = measured_state.dtheta_m_rad_s - reference.dtheta_m_rad_s
            err_dtheta_l = measured_state.dtheta_l_rad_s - reference.dtheta_l_rad_s

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
                raw_action_nm,
                action_nm,
                target_torque,
                measured_after.statusword,
                measured_after.position_rad,
                measured_after.velocity_rad_s,
                measured_after.torque_nm,
                measured_after.encoder1_rad,
                measured_after.encoder2_rad,
                int(encoder1_assessment.adjusted_valid_assumed),
                int(encoder2_assessment.adjusted_valid_assumed),
                encoder1_assessment.status_label,
                encoder2_assessment.status_label,
                read_before_dt_s,
                controller_dt_s,
                send_dt_s,
                read_after_dt_s,
                diag_dt_s,
                0.0,
                time.monotonic() - cycle_total_start,
            ]
            data_buffer.append(row)
            buffer_append_dt_s = time.monotonic() - buffer_append_start
            row[-2] = buffer_append_dt_s
            cycle_total_dt_s = time.monotonic() - cycle_total_start

            if cycle % max(PRINT_EVERY, 1) == 0:
                print_state(f"run cycle={cycle} t={elapsed_s:.3f}s wkc={wkc}", measured_after)
                print_lqr_terms("ctrl", reference, measured_state, raw_action_nm, action_nm, target_torque)
                print_timing_terms(
                    "timing",
                    read_before_dt_s=read_before_dt_s,
                    controller_dt_s=controller_dt_s,
                    send_dt_s=send_dt_s,
                    read_after_dt_s=read_after_dt_s,
                    diag_dt_s=diag_dt_s,
                    buffer_append_dt_s=buffer_append_dt_s,
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
            row[3] = deadline_slip_ms
            row[4] = sleep_target_ms
            row[5] = sleep_actual_ms
            row[6] = deadline_miss_delta
            row[7] = deadline_miss_count

        # 退出前把力矩指令清零，避免最后一帧还保持受力。
        for _ in range(10):
            wkc = comm.exchange_cycle(
                DriveCommand(
                    controlword=ENABLE_OPERATION,
                    mode_of_operation=CST_MODE,
                    target_torque=0,
                ),
                sleep=False,
            )
            state = comm.read_state_si()
        if last_state is not None:
            print_state(f"stop wkc={wkc}", state)
        if deadline_miss_count:
            print(f"Formal deadline misses: {deadline_miss_count}", flush=True)

        # 写入 CSV 文件
        print(f"Writing data to CSV...", flush=True)
        with open(csv_path, "w", newline="") as csv_file:
            csv_writer = csv.writer(csv_file)
            # 写入表头
            csv_writer.writerow([
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
                "raw_action_nm",
                "action_nm",
                "target_torque",
                "statusword",
                "position_rad",
                "velocity_rad_s",
                "torque_nm",
                "encoder1_rad",
                "encoder2_rad",
                "encoder1_adjusted_valid",
                "encoder2_adjusted_valid",
                "encoder1_assessment",
                "encoder2_assessment",
                "read_before_dt_s",
                "controller_dt_s",
                "send_dt_s",
                "read_after_dt_s",
                "diag_dt_s",
                "buffer_append_dt_s",
                "cycle_total_dt_s",
            ])
            # 写入数据
            csv_writer.writerows(data_buffer)

        try:
            os.chmod(csv_path, 0o666)
        except OSError:
            pass

        print(f"Data saved to: {csv_path}", flush=True)
        print("LQR control flow done.", flush=True)

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
