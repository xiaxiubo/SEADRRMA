from __future__ import annotations

import os
import struct
import sys
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from xml.etree import ElementTree as ET

import pysoem


# 驱动器输入 PDO 的数据布局：
#  - 0x6041 状态字
#  - 0x6061 运行模式显示
#  - 0x6064 当前位置
#  - 0x606C 当前速度
#  - 0x6077 当前力矩
#  - 0x60F4 跟随误差
#  - 0x2111:02 编码器1修正位置
#  - 0x2113:02 编码器2修正位置
DRIVE_STATUS_SIZE = 25

# 驱动器输出 PDO 已切换为按运行时 RxPDO 映射写入。
# 当前 CST 主流程最小必需控制对象为 0x6040 / 0x6060 / 0x6071；
# 其余对象只有在运行时映射中存在时才写入。
REQUIRED_RXPDO_ENTRIES = (
    (0x6040, 0x00, "controlword"),
    (0x6060, 0x00, "mode_of_operation"),
    (0x6071, 0x00, "target_torque"),
)

OPTIONAL_RXPDO_ENTRIES = (
    (0x607A, 0x00, "target_position"),
    (0x60FF, 0x00, "target_velocity"),
    (0x60B2, 0x00, "torque_offset"),
    (0x60C7, 0x00, "tuning_command"),
)

PREHEAT_WKC_TOLERANCE = 1
PDO_ENCODER_MATCH_TOLERANCE_COUNTS = 8

# 两个编码器都按 20 位处理，单圈 1,048,576 个计数。
# 机械零位按中值 524,288 作为参考点。
ENCODER_COUNTS_PER_REV = 1_048_576
ENCODER_ZERO_COUNTS = 524_288
GEAR_RATIO = 100.0
MAX_TORQUE_NM = 61.0
VELOCITY_UNIT_MRPM = 1000.0
DEFAULT_POSITION_SIGN = 1
DEFAULT_VELOCITY_SIGN = 1
DEFAULT_TORQUE_SIGN = 1

BOX_VENDOR_ID = 2965
BOX_PRODUCT_CODE = 5120
BOX_TRANSPARENT_PDO_BYTES = 130
BOX_OBJECT_INDEX = 0x8000
BOX_MODE_SUBINDEX = 7
BOX_INTERFACE_SUBINDEX = 8
BOX_BAUD_SELECTOR_SUBINDEX = 1
BOX_FRAME_SUBINDEX = 2
BOX_EXPLICIT_BAUD_SUBINDEX = 3
BOX_POLLING_TIME_SUBINDEX = 4
BOX_MODE_TRANSPARENT = 1
BOX_INTERFACE_RS485 = 0
BOX_FRAME_8N1 = 3
BOX_BAUD_SELECTOR_CUSTOM = 0x10
ECODER_ID0_REQUEST = 0x02
ECODER_ID0_RESPONSE_SIZE = 6

STATUSWORD_MASK = 0x006F

CST_MODE = 10
SHUTDOWN = 0x0006
SWITCH_ON = 0x0007
ENABLE_OPERATION = 0x000F
DEFAULT_ENI_XML_PATH = Path(__file__).resolve().parents[2] / "SEA1JointBoxENI0414.xml"

ENCODER_TYPE_NAMES = {
    0: "None",
    1: "Hall sensor",
    2: "Incremental encoder",
    3: "Nikon A-format",
    4: "BiSS",
    5: "Internal encoder",
    7: "SSI",
    9: "SinCos module",
}


def _ensure_root() -> None:
    # EtherCAT 原始套接字需要 root 权限。
    # 这里通过 sudo -n 重新启动，方便直接在终端或 IDE 里运行。
    if os.geteuid() == 0:
        return
    os.execvp(
        "sudo",
        [
            "sudo",
            "-n",
            "env",
            "PYTHONDONTWRITEBYTECODE=1",
            "PYTHONUNBUFFERED=1",
            sys.executable,
            *sys.argv,
        ],
    )


def state_name(state: int) -> str:
    names = {
        pysoem.INIT_STATE: "INIT",
        pysoem.PREOP_STATE: "PRE-OP",
        pysoem.SAFEOP_STATE: "SAFE-OP",
        pysoem.OP_STATE: "OP",
    }
    base = state & 0x0F
    suffix = " + ERROR" if state & 0x10 else ""
    return f"{names.get(base, hex(base))}{suffix}"


def summarize_wkc(samples: list[int] | tuple[int, ...]) -> str:
    if not samples:
        return "no samples"
    return f"min={min(samples)} max={max(samples)} avg={sum(samples) / len(samples):.2f}"


def cia402_state_name(statusword: int) -> str:
    # CiA402: "Switch on disabled" 和 "Not ready to switch on" 的 bit5（Quick Stop）为 don't-care，
    # 用 0x4F 掩码（不含 bit5）。SOMANET 驱动在 Switch-on-disabled 时会置 bit5=1（sw≈0x1070），
    # 若用 0x6F 掩码则得到 0x0060 而非 0x0040，导致误识别为 Unknown。
    if statusword & 0x4F == 0x40:
        return "Switch on disabled"
    if statusword & 0x4F == 0x00:
        return "Not ready to switch on"
    masked = statusword & STATUSWORD_MASK  # 0x006F，用于其余状态（bit5 有确定意义）
    mapping = {
        0x0021: "Ready to switch on",
        0x0023: "Switched on",
        0x0027: "Operation enabled",
        0x0007: "Quick stop active",
        0x000F: "Fault reaction active",
        0x0008: "Fault",
    }
    return mapping.get(masked, f"Unknown(0x{masked:04X})")


def ecoder_crc_x8_plus_1(payload: bytes) -> int:
    crc = 0
    for byte in payload:
        crc ^= byte
    return crc & 0xFF


def parse_ecoder_id0_frame(frame: bytes, request: int = ECODER_ID0_REQUEST) -> ECoderId0Response:
    if len(frame) < ECODER_ID0_RESPONSE_SIZE:
        raise ValueError(f"Expected at least {ECODER_ID0_RESPONSE_SIZE} bytes, got {len(frame)}")

    control, status, as0, as1, as2, crc = frame[:6]
    if control != request:
        raise ValueError(f"Unexpected control echo. expected=0x{request:02X} got=0x{control:02X}")

    crc_ok = ecoder_crc_x8_plus_1(frame[:5]) == crc
    encoder_error = bool((status >> 4) & 0b11)
    communication_alarm = bool((status >> 6) & 0b11)
    position_21bit = (as0 | (as1 << 8) | (as2 << 16)) & 0x1F_FFFF

    return ECoderId0Response(
        control=control,
        status=status,
        as0=as0,
        as1=as1,
        as2=as2,
        crc=crc,
        position_21bit=position_21bit,
        encoder_error=encoder_error,
        communication_alarm=communication_alarm,
        crc_ok=crc_ok,
    )


@dataclass(frozen=True)
class DriveState:
    # 记录驱动器 TxPDO 中与控制和日志相关的数据快照。
    statusword: int
    mode_display: int
    position: int
    velocity: int
    torque: int
    following_error: int
    encoder1: int
    encoder2: int


@dataclass(frozen=True)
class DriveStateSI:
    # 把驱动器状态换算成标准国际单位，供控制器直接使用。
    statusword: int
    mode_display: int
    position_rad: float
    velocity_rad_s: float
    torque_nm: float
    following_error_rad: float
    encoder1_rad: float
    encoder2_rad: float


@dataclass(frozen=True)
class ECoderId0Response:
    control: int
    status: int
    as0: int
    as1: int
    as2: int
    crc: int
    position_21bit: int
    encoder_error: bool
    communication_alarm: bool
    crc_ok: bool


@dataclass(frozen=True)
class ThirdEncoderSample:
    cycle_index: int
    tx_valid: int
    wkc: int
    request: int
    response: ECoderId0Response


@dataclass(frozen=True)
class PdoEntry:
    pdo_index: int
    entry_subindex: int
    raw_mapping: int
    object_index: int
    object_subindex: int
    bit_length: int
    offset_bits: int

    @property
    def offset_bytes(self) -> int:
        if self.offset_bits % 8 != 0:
            raise ValueError("PDO entry is not byte aligned")
        return self.offset_bits // 8

    @property
    def size_bytes(self) -> int:
        if self.bit_length % 8 != 0:
            raise ValueError("PDO entry size is not byte aligned")
        return self.bit_length // 8

    @property
    def is_byte_aligned(self) -> bool:
        return self.offset_bits % 8 == 0

    @property
    def has_byte_sized_length(self) -> bool:
        return self.bit_length % 8 == 0

    @property
    def object_key(self) -> tuple[int, int]:
        return (self.object_index, self.object_subindex)


@dataclass(frozen=True)
class PdoLayout:
    assign_index: int
    assigned_pdos: tuple[int, ...]
    entries: tuple[PdoEntry, ...]

    @property
    def total_bits(self) -> int:
        return sum(entry.bit_length for entry in self.entries)

    @property
    def total_bytes(self) -> int:
        return (self.total_bits + 7) // 8

    @property
    def is_byte_aligned(self) -> bool:
        return self.total_bits % 8 == 0

    def find_entry(self, object_index: int, object_subindex: int) -> PdoEntry | None:
        for entry in self.entries:
            if entry.object_index == object_index and entry.object_subindex == object_subindex:
                return entry
        return None


@dataclass(frozen=True)
class ProcessDataTarget:
    input_bytes: int
    output_bytes: int


@dataclass(frozen=True)
class XmlCoEInitCmd:
    index: int
    subindex: int
    payload: bytes
    complete_access: bool
    comment: str
    transition: str


@dataclass(frozen=True)
class XmlSlaveProfile:
    name: str
    vendor_id: int
    product_code: int
    revision: int
    process_data_target: ProcessDataTarget
    coe_init_cmds: tuple[XmlCoEInitCmd, ...]


@dataclass(frozen=True)
class EncoderConfig:
    channel: int
    sensor_port: int
    encoder_type: int
    resolution: int
    polarity: bool
    singleturn_offset: int
    index_availability: bool | None

    @property
    def encoder_type_name(self) -> str:
        return ENCODER_TYPE_NAMES.get(self.encoder_type, f"Unknown({self.encoder_type})")


@dataclass(frozen=True)
class EncoderFeedback:
    channel: int
    raw_position: int
    adjusted_position: int
    velocity_rpm: int


@dataclass(frozen=True)
class EncoderAssessment:
    channel: int
    config: EncoderConfig
    last_feedback: EncoderFeedback
    raw_samples: tuple[int, ...]
    adjusted_samples: tuple[int, ...]
    velocity_samples: tuple[int, ...]
    drive_position_samples: tuple[int, ...]
    pdo_encoder_samples: tuple[int, ...]
    raw_updates: bool
    adjusted_updates: bool
    adjusted_stuck_zero: bool
    drive_position_updates: bool
    pdo_updates: bool
    pdo_stuck_zero: bool
    pdo_matches_adjusted: bool
    adjusted_valid_assumed: bool
    status_label: str
    detail: str

    def summary_line(self) -> str:
        return (
            f"encoder{self.channel} type={self.config.encoder_type_name} port={self.config.sensor_port} "
            f"index={self.config.index_availability} raw_updates={int(self.raw_updates)} "
            f"adjusted_updates={int(self.adjusted_updates)} adjusted_stuck_zero={int(self.adjusted_stuck_zero)} "
            f"drive_position_updates={int(self.drive_position_updates)} pdo_updates={int(self.pdo_updates)} "
            f"pdo_stuck_zero={int(self.pdo_stuck_zero)} pdo_matches_adjusted={int(self.pdo_matches_adjusted)} "
            f"adjusted_valid={int(self.adjusted_valid_assumed)} "
            f"status={self.status_label} detail={self.detail}"
        )


@dataclass(frozen=True)
class SlaveInfo:
    index: int
    name: str
    vendor_id: int
    product_code: int
    revision: int
    state: int
    input_size: int
    output_size: int

    @property
    def state_label(self) -> str:
        return state_name(self.state)


@dataclass(frozen=True)
class SlaveRoles:
    box_index: int | None
    drive_index: int


@dataclass(frozen=True)
class DriveDiagnostics:
    statusword: int
    mode_display: int
    cia402_state: str
    ready_to_switch_on: bool
    switched_on: bool
    operation_enabled: bool
    fault: bool
    voltage_enabled: bool
    quick_stop_active: bool
    switch_on_disabled: bool
    warning: bool
    remote: bool
    target_reached: bool
    internal_limit_active: bool
    input_pdo_size: int

    @property
    def pdo_valid(self) -> bool:
        return self.input_pdo_size >= DRIVE_STATUS_SIZE


@dataclass(frozen=True)
class PdoPreheatSample:
    cycle_index: int
    wkc: int
    statusword: int
    mode_display: int
    cia402_state: str
    slave_states: tuple[str, ...]


@dataclass(frozen=True)
class PdoPreheatResult:
    success: bool
    expected_wkc: int | None
    stable_cycles_required: int
    consecutive_stable_cycles: int
    observed_wkc: tuple[int, ...]
    samples: tuple[PdoPreheatSample, ...]
    last_diagnostics: DriveDiagnostics

    def sample_summary(self, limit: int = 8) -> str:
        rows = []
        for sample in self.samples[-limit:]:
            rows.append(
                "cycle={cycle} wkc={wkc} sw=0x{sw:04X} mode={mode} cia402={cia402} slaves={slaves}".format(
                    cycle=sample.cycle_index,
                    wkc=sample.wkc,
                    sw=sample.statusword,
                    mode=sample.mode_display,
                    cia402=sample.cia402_state,
                    slaves=", ".join(sample.slave_states),
                )
            )
        return "\n".join(rows)


@dataclass(frozen=True)
class Cia402TransitionResult:
    success: bool
    step_name: str
    controlword: int
    expected_state: str
    expected_wkc: int | None
    observed_wkc: tuple[int, ...]
    samples: tuple[PdoPreheatSample, ...]
    last_diagnostics: DriveDiagnostics

    def sample_summary(self, limit: int = 8) -> str:
        rows = []
        for sample in self.samples[-limit:]:
            rows.append(
                "cycle={cycle} wkc={wkc} sw=0x{sw:04X} mode={mode} cia402={cia402} slaves={slaves}".format(
                    cycle=sample.cycle_index,
                    wkc=sample.wkc,
                    sw=sample.statusword,
                    mode=sample.mode_display,
                    cia402=sample.cia402_state,
                    slaves=", ".join(sample.slave_states),
                )
            )
        return "\n".join(rows)


@dataclass(frozen=True)
class StartupDiagnostics:
    slave_snapshots: tuple[SlaveInfo, ...]
    expected_wkc: int | None
    observed_wkc: tuple[int, ...]
    last_statusword: int
    last_mode_display: int
    last_cia402_state: str


@dataclass(frozen=True)
class DriveCalibration:
    # 单位换算所需的标定参数。
    # 方向默认值已经按当前实测结果固化为 +1；后续如有新测试再调整。
    # encoder1 是输出端编码器，encoder2 是电机端编码器。
    encoder_zero_counts: int = ENCODER_ZERO_COUNTS
    encoder_counts_per_rev: int = ENCODER_COUNTS_PER_REV
    gear_ratio: float = GEAR_RATIO
    max_torque_nm: float = MAX_TORQUE_NM
    velocity_unit_mrpm: float = VELOCITY_UNIT_MRPM
    position_sign: int = DEFAULT_POSITION_SIGN
    velocity_sign: int = DEFAULT_VELOCITY_SIGN
    torque_sign: int = DEFAULT_TORQUE_SIGN

    def _counts_to_rad(self, counts: int, gear_ratio: float) -> float:
        turns = (counts - self.encoder_zero_counts) / float(self.encoder_counts_per_rev)
        return turns * 2.0 * 3.141592653589793 / gear_ratio

    def counts_to_joint_rad(self, counts: int) -> float:
        # 驱动器位置实际值 / 输出端编码器 -> 关节侧弧度
        return self.position_sign * self._counts_to_rad(counts, gear_ratio=1.0)

    def output_encoder_to_joint_rad(self, counts: int) -> float:
        # 编码器1在输出端，直接对应关节侧角度
        return self.position_sign * self._counts_to_rad(counts, gear_ratio=1.0)

    def motor_encoder_to_joint_rad(self, counts: int) -> float:
        # 编码器2在电机端，需要除以减速比得到关节侧角度
        return self.position_sign * self._counts_to_rad(counts, gear_ratio=self.gear_ratio)

    def mrpm_to_joint_rad_s(self, mrpm: int) -> float:
        # 驱动器回读的 0x606C 速度已经按负载侧口径使用，不再除以减速比。
        motor_rpm = mrpm / self.velocity_unit_mrpm
        motor_rad_s = motor_rpm * 2.0 * 3.141592653589793 / 60.0
        return self.velocity_sign * motor_rad_s

    def raw_torque_to_nm(self, torque_raw: int) -> float:
        # 0x6077 的原始值是最大力矩的 1/1000
        return self.torque_sign * (torque_raw / 1000.0) * self.max_torque_nm

    def state_to_si(self, state: DriveState) -> DriveStateSI:
        return DriveStateSI(
            statusword=state.statusword,
            mode_display=state.mode_display,
            position_rad=self.output_encoder_to_joint_rad(state.position),
            velocity_rad_s=self.mrpm_to_joint_rad_s(state.velocity),
            torque_nm=self.raw_torque_to_nm(state.torque),
            following_error_rad=self.output_encoder_to_joint_rad(state.following_error),
            encoder1_rad=self.output_encoder_to_joint_rad(state.encoder1),
            encoder2_rad=self.motor_encoder_to_joint_rad(state.encoder2),
        )


@dataclass(frozen=True)
class DriveCommand:
    # 控制循环里常用的 RxPDO 命令对象，字段尽量保持精简。
    controlword: int = 0
    mode_of_operation: int = 0
    target_torque: int = 0
    target_position: int = 0
    target_velocity: int = 0
    torque_offset: int = 0
    tuning_command: int = 0

    def pack(self) -> bytes:
        # 保留这个旧接口做兼容，但正式实时写入已改为按运行时 RxPDO 映射逐项写入。
        return struct.pack(
            "<HbhiihI",
            self.controlword,
            self.mode_of_operation,
            self.target_torque,
            self.target_position,
            self.target_velocity,
            self.torque_offset,
            self.tuning_command,
        )


class SEARealtimeComm:
    # 单个 SEA 驱动器的正式 EtherCAT 通信封装。
    # 这个类只保留正式控制会用到的接口：
    # - connect/open/close 管主站生命周期
    # - read_state/read_state_si 管反馈读取
    # - write_command/cycle 管实时控制循环
    # - read_sdo/write_sdo 管配置和诊断
    def __init__(
        self,
        ifname: str = "eno1",
        cycle_time_s: float = 0.001,
        calibration: DriveCalibration | None = None,
        eni_xml_path: Path | None = None,
    ):
        self.ifname = ifname
        self.cycle_time_s = cycle_time_s
        self.calibration = calibration or DriveCalibration()
        self.eni_xml_path = eni_xml_path or DEFAULT_ENI_XML_PATH
        self.master = pysoem.Master()
        self.box = None
        self.drive = None
        self.roles: SlaveRoles | None = None
        self._opened = False
        self.last_startup_diagnostics: StartupDiagnostics | None = None
        self.txpdo_layout: PdoLayout | None = None
        self.rxpdo_layout: PdoLayout | None = None
        self.xml_slave_profiles: tuple[XmlSlaveProfile, ...] = ()
        self._third_encoder_tx_valid = 0
        self._third_encoder_sample_index = 0
        self._third_encoder_pending_tx_valid: int | None = None
        self._last_processdata_wkc = 0

    def open(self) -> None:
        # 在指定网口上打开 EtherCAT 原始接口。
        _ensure_root()
        self.master = pysoem.Master()
        self.master.open(self.ifname)
        self.box = None
        self.drive = None
        self.roles = None
        self.last_startup_diagnostics = None
        self.txpdo_layout = None
        self.rxpdo_layout = None
        self.xml_slave_profiles = ()
        self._opened = True
        self._third_encoder_tx_valid = 0
        self._third_encoder_sample_index = 0
        self._third_encoder_pending_tx_valid = None
        self._last_processdata_wkc = 0

    def close(self) -> None:
        # 异常退出时尽量先把输出清零，再把网络切回 INIT 并关闭套接字。
        if not self._opened:
            return
        try:
            self._flush_zero_outputs()
        except Exception:
            pass
        try:
            self.master.state = pysoem.INIT_STATE
            self.master.write_state()
        except Exception:
            pass
        try:
            self.master.close()
        except Exception:
            pass
        self._opened = False

    def scan(self) -> int:
        # 扫描总线上的从站，并找到后面要控制的驱动器对象。
        count = self.master.config_init()
        if count <= 0:
            raise RuntimeError("No EtherCAT slaves found.")
        self.roles = self._identify_roles()
        if self.roles.box_index is not None:
            self.box = self.master.slaves[self.roles.box_index]
        self.drive = self.master.slaves[self.roles.drive_index]
        return count

    def configure(self) -> int:
        # 映射 PDO 之前关闭 complete access，避免某些从站不兼容。
        for slave in self.master.slaves:
            try:
                slave._disable_complete_access()
            except Exception:
                pass
        return self.master.config_map()

    def start_op(self) -> None:
        # 将总线从 PRE-OP/SAFE-OP 推到 OP。
        for slave in self.master.slaves:
            output_size = len(bytes(getattr(slave, "output", b"")))
            if output_size:
                slave.output = bytes(output_size)

        if self.master.state_check(pysoem.SAFEOP_STATE, 50_000) != pysoem.SAFEOP_STATE:
            self.master.read_state()
            raise RuntimeError(
                "Failed to reach SAFE-OP. Current slave states: "
                + ", ".join(f"{item.index}:{item.state_label}" for item in self.read_slave_states())
            )
        self.master.read_state()
        if not all((item.state & 0x0F) == pysoem.SAFEOP_STATE for item in self.read_slave_states()):
            raise RuntimeError(
                "Master reported SAFE-OP, but some slaves did not enter SAFE-OP: "
                + ", ".join(f"{item.index}:{item.state_label}" for item in self.read_slave_states())
            )

        self.master.state = pysoem.OP_STATE
        self.master.write_state()
        for slave in self.master.slaves:
            try:
                slave.state = pysoem.OP_STATE
            except Exception:
                pass
        for _ in range(200):
            self.cycle()
            self.master.read_state()
            if self.master.state_check(pysoem.OP_STATE, 5_000) == pysoem.OP_STATE and all(
                (item.state & 0x0F) == pysoem.OP_STATE for item in self.read_slave_states()
            ):
                return

        self.master.read_state()
        raise RuntimeError(
            "Failed to reach OP. Current slave states: "
            + ", ".join(f"{item.index}:{item.state_label}" for item in self.read_slave_states())
        )

    def start(self) -> None:
        # 向后兼容旧接口。
        self.start_op()

    def connect(self) -> None:
        # connect 只负责建立 EtherCAT 网络，不混入驱动 ready 语义。
        self.open()
        self.scan()
        self.load_xml_slave_profiles()
        self.ensure_runtime_matches_xml()
        self.configure()
        try:
            self.refresh_pdo_layouts()
        except Exception as exc:
            raise RuntimeError(
                "Failed to read runtime PDO layouts before OP. "
                "Runtime-mapped startup requires Tx/Rx PDO layouts to be readable before entering OP."
            ) from exc
        self.ensure_required_rxpdo_entries()
        self.start_op()

    def cycle(self, command: DriveCommand | None = None) -> int:
        # 一次实时循环：
        # 1) 需要的话先写入下一帧 RxPDO 命令
        # 2) 发送 EtherCAT 过程数据
        # 3) 接收最新的 TxPDO
        # 4) 按设定周期休眠
        return self.exchange_cycle(command=command, sleep=True)

    def exchange_cycle(
        self,
        command: DriveCommand | None = None,
        *,
        sleep: bool = False,
    ) -> int:
        # 一次过程数据交换，不强制附带周期休眠。
        if command is not None:
            self.write_command(command)
        self.master.send_processdata()
        wkc = self.master.receive_processdata(5_000)
        self._last_processdata_wkc = wkc
        if sleep:
            time.sleep(self.cycle_time_s)
        return wkc

    def send_processdata(self) -> None:
        self.master.send_processdata()

    def receive_processdata(self, timeout_us: int = 5_000) -> int:
        wkc = self.master.receive_processdata(timeout_us)
        self._last_processdata_wkc = wkc
        return wkc

    @property
    def last_processdata_wkc(self) -> int:
        return self._last_processdata_wkc

    def configure_box_transparent(
        self,
        *,
        box_mode: int = BOX_MODE_TRANSPARENT,
        box_interface: int = BOX_INTERFACE_RS485,
        box_frame: int = BOX_FRAME_8N1,
        box_baud_selector: int = BOX_BAUD_SELECTOR_CUSTOM,
        box_baud: int = 2_500_000,
        box_polling_time_ms: int = 50,
    ) -> dict[str, int]:
        if self.box is None:
            raise RuntimeError("Box not ready. Call connect() first.")
        self.box.sdo_write(BOX_OBJECT_INDEX, BOX_MODE_SUBINDEX, struct.pack("<B", box_mode))
        self.box.sdo_write(BOX_OBJECT_INDEX, BOX_INTERFACE_SUBINDEX, struct.pack("<B", box_interface))
        self.box.sdo_write(BOX_OBJECT_INDEX, BOX_FRAME_SUBINDEX, struct.pack("<B", box_frame))
        try:
            self.box.sdo_write(BOX_OBJECT_INDEX, BOX_BAUD_SELECTOR_SUBINDEX, struct.pack("<B", box_baud_selector))
        except Exception:
            fallback = 10 if box_baud_selector == 0x10 else 0x10
            self.box.sdo_write(BOX_OBJECT_INDEX, BOX_BAUD_SELECTOR_SUBINDEX, struct.pack("<B", fallback))
            box_baud_selector = fallback
        self.box.sdo_write(BOX_OBJECT_INDEX, BOX_EXPLICIT_BAUD_SUBINDEX, struct.pack("<I", box_baud))
        self.box.sdo_write(BOX_OBJECT_INDEX, BOX_POLLING_TIME_SUBINDEX, struct.pack("<H", box_polling_time_ms))
        return {
            "mode": box_mode,
            "interface": box_interface,
            "frame": box_frame,
            "baud_selector": box_baud_selector,
            "explicit_baud": box_baud,
            "polling_ms": box_polling_time_ms,
        }

    def configure_box_transparent_preop(
        self,
        *,
        box_mode: int = BOX_MODE_TRANSPARENT,
        box_interface: int = BOX_INTERFACE_RS485,
        box_frame: int = BOX_FRAME_8N1,
        box_baud_selector: int = BOX_BAUD_SELECTOR_CUSTOM,
        box_baud: int = 2_500_000,
        box_polling_time_ms: int = 50,
    ) -> dict[str, int]:
        if self.box is None:
            raise RuntimeError("Box not ready. Call connect() first.")
        self._set_preop_for_config()
        cfg = self.configure_box_transparent(
            box_mode=box_mode,
            box_interface=box_interface,
            box_frame=box_frame,
            box_baud_selector=box_baud_selector,
            box_baud=box_baud,
            box_polling_time_ms=box_polling_time_ms,
        )
        self.configure()
        # self.refresh_pdo_layouts()
        # self.ensure_required_rxpdo_entries()
        self.start_op()
        return cfg

    def write_box_transparent(self, payload: bytes, valid: int) -> bytes:
        if self.box is None:
            raise RuntimeError("Box not ready. Call connect() first.")
        output = bytearray(bytes(self.box.output))
        if len(output) < 2:
            raise RuntimeError("Box PDO too short for transparent mode")
        size = min(len(payload), max(len(output) - 2, 0))
        output[0] = valid & 0xFF
        output[1] = size & 0xFF
        output[2 : 2 + size] = payload[:size]
        self.box.output = bytes(output)
        return bytes(output[2 : 2 + size])

    def read_box_transparent(self) -> bytes:
        if self.box is None:
            raise RuntimeError("Box not ready. Call connect() first.")
        return bytes(self.box.input)

    def queue_third_encoder_request(
        self,
        command: DriveCommand | None = None,
        valid: int | None = None,
    ) -> int:
        tx_valid = self._third_encoder_tx_valid if valid is None else int(valid) & 0xFF
        if valid is None:
            self._third_encoder_tx_valid = (self._third_encoder_tx_valid + 1) & 0xFF
        self.write_box_transparent(bytes([ECODER_ID0_REQUEST]), tx_valid)
        if command is not None:
            self.write_command(command)
        self._third_encoder_pending_tx_valid = tx_valid
        return tx_valid

    def collect_queued_third_encoder_sample(self, timeout_us: int = 5_000) -> ThirdEncoderSample:
        sample = self.poll_queued_third_encoder_sample(timeout_us=timeout_us)
        if sample is None:
            raise RuntimeError("No fresh third encoder sample available yet.")
        return sample

    def poll_queued_third_encoder_sample(self, timeout_us: int = 0) -> ThirdEncoderSample | None:
        pending_tx_valid = self._third_encoder_pending_tx_valid
        if pending_tx_valid is None:
            return None

        wkc = self.receive_processdata(timeout_us)
        rx_frame = self.read_box_transparent()
        if len(rx_frame) < 2:
            return None
        rx_size = min(int(rx_frame[1]), max(len(rx_frame) - 2, 0))
        if rx_size < ECODER_ID0_RESPONSE_SIZE:
            return None

        try:
            response = parse_ecoder_id0_frame(rx_frame[2 : 2 + rx_size], request=ECODER_ID0_REQUEST)
        except Exception:
            return None

        sample_index = self._third_encoder_sample_index
        self._third_encoder_sample_index += 1
        self._third_encoder_pending_tx_valid = None
        return ThirdEncoderSample(
            cycle_index=sample_index,
            tx_valid=pending_tx_valid,
            wkc=wkc,
            request=ECODER_ID0_REQUEST,
            response=response,
        )

    def sample_third_encoder_position_21bit(
        self,
        command: DriveCommand | None = None,
        valid: int | None = None,
        *,
        sleep: bool = True,
    ) -> ThirdEncoderSample:
        tx_valid = self.queue_third_encoder_request(command=command, valid=valid)
        wkc = self.exchange_cycle(sleep=sleep)
        sample = self.collect_queued_third_encoder_sample()
        return ThirdEncoderSample(
            cycle_index=sample.cycle_index,
            tx_valid=tx_valid,
            wkc=wkc,
            request=sample.request,
            response=sample.response,
        )

    def read_third_encoder_position(
        self,
        command: DriveCommand | None = None,
        valid: int | None = None,
    ) -> int:
        return self.sample_third_encoder_position_21bit(command=command, valid=valid).response.position_21bit

    @property
    def expected_wkc(self) -> int | None:
        return getattr(self.master, "expected_wkc", None)

    def _wkc_is_acceptable(self, wkc: int) -> bool:
        expected = self.expected_wkc
        if expected is None:
            return True
        return wkc >= max(expected - PREHEAT_WKC_TOLERANCE, 0)

    def describe_slaves(self) -> list[SlaveInfo]:
        return self.read_slave_states()

    def describe_xml_targets(self) -> list[str]:
        if not self.xml_slave_profiles:
            return []
        rows = []
        for profile in self.xml_slave_profiles:
            rows.append(
                "xml slave name={name!r} man={man} id={prod} rev={rev} target_in={inp} target_out={out} coe_cmds={cmds}".format(
                    name=profile.name,
                    man=profile.vendor_id,
                    prod=profile.product_code,
                    rev=profile.revision,
                    inp=profile.process_data_target.input_bytes,
                    out=profile.process_data_target.output_bytes,
                    cmds=len(profile.coe_init_cmds),
                )
            )
        return rows

    def read_slave_states(self) -> list[SlaveInfo]:
        try:
            self.master.read_state()
        except Exception:
            pass
        return [
            SlaveInfo(
                index=index,
                name=getattr(slave, "name", "") or "",
                vendor_id=getattr(slave, "man", 0),
                product_code=getattr(slave, "id", 0),
                revision=getattr(slave, "rev", 0),
                state=getattr(slave, "state", 0),
                input_size=len(bytes(getattr(slave, "input", b""))),
                output_size=len(bytes(getattr(slave, "output", b""))),
            )
            for index, slave in enumerate(self.master.slaves)
        ]

    def read_state(self) -> DriveState:
        # 把驱动器 TxPDO 解析成一个简单、强类型的状态快照。
        raw = self._drive_input()
        if len(raw) < DRIVE_STATUS_SIZE:
            raise RuntimeError(f"Drive PDO too short: got {len(raw)}, need {DRIVE_STATUS_SIZE}")
        self._ensure_pdo_layouts()
        return DriveState(
            statusword=self._read_input_entry(raw, 0x6041, 0x00, "<H"),
            mode_display=self._read_input_entry(raw, 0x6061, 0x00, "<b"),
            position=self._read_input_entry(raw, 0x6064, 0x00, "<i"),
            velocity=self._read_input_entry(raw, 0x606C, 0x00, "<i"),
            torque=self._read_input_entry(raw, 0x6077, 0x00, "<h"),
            following_error=self._read_input_entry(raw, 0x60F4, 0x00, "<i"),
            encoder1=self._read_input_entry(raw, 0x2111, 0x02, "<i"),
            encoder2=self._read_input_entry(raw, 0x2113, 0x02, "<i"),
        )

    def read_state_si(self) -> DriveStateSI:
        # 直接读取并返回标准国际单位版本的状态。
        return self.calibration.state_to_si(self.read_state())

    def state_to_si(self, state: DriveState) -> DriveStateSI:
        # 如果外部已经拿到原始状态，也可以显式换算成 SI。
        return self.calibration.state_to_si(state)

    def read_drive_diagnostics(self) -> DriveDiagnostics:
        state = self.read_state()
        return self.diagnostics_from_state(state, input_pdo_size=len(self._drive_input()))

    def diagnostics_from_state(self, state: DriveState, input_pdo_size: int = DRIVE_STATUS_SIZE) -> DriveDiagnostics:
        statusword = state.statusword
        return DriveDiagnostics(
            statusword=statusword,
            mode_display=state.mode_display,
            cia402_state=cia402_state_name(statusword),
            ready_to_switch_on=bool(statusword & 0x0001),
            switched_on=bool(statusword & 0x0002),
            operation_enabled=bool(statusword & 0x0004),
            fault=bool(statusword & 0x0008),
            voltage_enabled=bool(statusword & 0x0010),
            quick_stop_active=not bool(statusword & 0x0020),
            switch_on_disabled=bool(statusword & 0x0040),
            warning=bool(statusword & 0x0080),
            remote=bool(statusword & 0x0200),
            target_reached=bool(statusword & 0x0400),
            internal_limit_active=bool(statusword & 0x0800),
            input_pdo_size=input_pdo_size,
        )

    def write_command(self, command: DriveCommand) -> None:
        # 重新构造一份干净的输出缓冲，只按运行时 RxPDO 映射写入存在的对象。
        # 必需控制对象缺失时直接报错；可选扩展对象缺失时跳过。
        output = bytearray(len(self._drive_output()))
        self._ensure_pdo_layouts()
        self.ensure_required_rxpdo_entries()
        self._write_output_entry_if_present(output, 0x6040, 0x00, command.controlword, signed=False, required=True)
        self._write_output_entry_if_present(output, 0x6060, 0x00, command.mode_of_operation, signed=True, required=True)
        self._write_output_entry_if_present(output, 0x6071, 0x00, command.target_torque, signed=True, required=True)
        self._write_output_entry_if_present(output, 0x607A, 0x00, command.target_position, signed=True)
        self._write_output_entry_if_present(output, 0x60FF, 0x00, command.target_velocity, signed=True)
        self._write_output_entry_if_present(output, 0x60B2, 0x00, command.torque_offset, signed=True)
        self._write_output_entry_if_present(output, 0x60C7, 0x00, command.tuning_command, signed=False)
        self.drive.output = bytes(output)

    def preheat_pdo(
        self,
        command: DriveCommand,
        cycles: int = 100,
        stable_cycles: int = 5,
        sample_limit: int = 20,
        require_statusword_nonzero: bool = False,
        recovery_attempts: int = 3,
        recovery_cycles: int = 20,
    ) -> PdoPreheatResult:
        last_result: PdoPreheatResult | None = None
        for attempt in range(max(recovery_attempts, 1)):
            observed_wkc: list[int] = []
            samples: deque[PdoPreheatSample] = deque(maxlen=sample_limit)
            consecutive_stable = 0
            last_diag = self.read_drive_diagnostics()

            for cycle_index in range(cycles):
                wkc = self.cycle(command)
                diag = self.read_drive_diagnostics()
                last_diag = diag
                observed_wkc.append(wkc)
                samples.append(
                    PdoPreheatSample(
                        cycle_index=cycle_index,
                        wkc=wkc,
                        statusword=diag.statusword,
                        mode_display=diag.mode_display,
                        cia402_state=diag.cia402_state,
                        slave_states=tuple(item.state_label for item in self.read_slave_states()),
                    )
                )

                stable = (
                    self._wkc_is_acceptable(wkc)
                    and diag.pdo_valid
                    and (not require_statusword_nonzero or diag.statusword != 0x0000)
                )
                consecutive_stable = consecutive_stable + 1 if stable else 0
                if consecutive_stable >= stable_cycles:
                    result = PdoPreheatResult(
                        success=True,
                        expected_wkc=self.expected_wkc,
                        stable_cycles_required=stable_cycles,
                        consecutive_stable_cycles=consecutive_stable,
                        observed_wkc=tuple(observed_wkc),
                        samples=tuple(samples),
                        last_diagnostics=last_diag,
                    )
                    self.last_startup_diagnostics = StartupDiagnostics(
                        slave_snapshots=tuple(self.read_slave_states()),
                        expected_wkc=self.expected_wkc,
                        observed_wkc=tuple(observed_wkc),
                        last_statusword=last_diag.statusword,
                        last_mode_display=last_diag.mode_display,
                        last_cia402_state=last_diag.cia402_state,
                    )
                    return result

            last_result = PdoPreheatResult(
                success=False,
                expected_wkc=self.expected_wkc,
                stable_cycles_required=stable_cycles,
                consecutive_stable_cycles=consecutive_stable,
                observed_wkc=tuple(observed_wkc),
                samples=tuple(samples),
                last_diagnostics=last_diag,
            )
            self.last_startup_diagnostics = StartupDiagnostics(
                slave_snapshots=tuple(self.read_slave_states()),
                expected_wkc=self.expected_wkc,
                observed_wkc=tuple(observed_wkc),
                last_statusword=last_diag.statusword,
                last_mode_display=last_diag.mode_display,
                last_cia402_state=last_diag.cia402_state,
            )
            if attempt + 1 < max(recovery_attempts, 1):
                self._recover_processdata_exchange(recovery_cycles=recovery_cycles)

        assert last_result is not None
        return last_result

    def transition_cia402(
        self,
        step_name: str,
        controlword: int,
        expected_state: str,
        mode_of_operation: int,
        target_torque: int = 0,
        timeout_cycles: int = 200,
        require_remote: bool = False,
        sample_limit: int = 20,
    ) -> Cia402TransitionResult:
        observed_wkc: list[int] = []
        samples: deque[PdoPreheatSample] = deque(maxlen=sample_limit)
        last_diag = self.read_drive_diagnostics()
        command = DriveCommand(
            controlword=controlword,
            mode_of_operation=mode_of_operation,
            target_torque=target_torque,
        )
        for cycle_index in range(timeout_cycles):
            wkc = self.cycle(command)
            diag = self.read_drive_diagnostics()
            last_diag = diag
            observed_wkc.append(wkc)
            samples.append(
                PdoPreheatSample(
                    cycle_index=cycle_index,
                    wkc=wkc,
                    statusword=diag.statusword,
                    mode_display=diag.mode_display,
                    cia402_state=diag.cia402_state,
                    slave_states=tuple(item.state_label for item in self.read_slave_states()),
                )
            )
            success = (
                self._wkc_is_acceptable(wkc)
                and diag.cia402_state == expected_state
                and (not require_remote or diag.remote)
            )
            if success:
                return Cia402TransitionResult(
                    success=True,
                    step_name=step_name,
                    controlword=controlword,
                    expected_state=expected_state,
                    expected_wkc=self.expected_wkc,
                    observed_wkc=tuple(observed_wkc),
                    samples=tuple(samples),
                    last_diagnostics=last_diag,
                )
            if diag.fault:
                break

        return Cia402TransitionResult(
            success=False,
            step_name=step_name,
            controlword=controlword,
            expected_state=expected_state,
            expected_wkc=self.expected_wkc,
            observed_wkc=tuple(observed_wkc),
            samples=tuple(samples),
            last_diagnostics=last_diag,
        )

    def enable_cia402(
        self,
        mode_of_operation: int,
        target_torque: int = 0,
        timeout_cycles_per_step: int = 300,
    ) -> list[Cia402TransitionResult]:
        results: list[Cia402TransitionResult] = []
        for step_name, controlword, expected_state in (
            ("SHUTDOWN", SHUTDOWN, "Ready to switch on"),
            ("SWITCH_ON", SWITCH_ON, "Switched on"),
            ("ENABLE_OPERATION", ENABLE_OPERATION, "Operation enabled"),
        ):
            result = self.transition_cia402(
                step_name=step_name,
                controlword=controlword,
                expected_state=expected_state,
                mode_of_operation=mode_of_operation,
                target_torque=target_torque,
                timeout_cycles=timeout_cycles_per_step,
            )
            results.append(result)
            if not result.success:
                break
        return results

    def stabilize_enabled_state(
        self,
        mode_of_operation: int,
        target_torque: int = 0,
        cycles: int = 20,
        stable_cycles: int = 5,
    ) -> PdoPreheatResult:
        return self.preheat_pdo(
            command=DriveCommand(
                controlword=ENABLE_OPERATION,
                mode_of_operation=mode_of_operation,
                target_torque=target_torque,
            ),
            cycles=cycles,
            stable_cycles=stable_cycles,
            require_statusword_nonzero=True,
        )

    def read_sdo_raw(self, index: int, subindex: int) -> bytes:
        # 通用 SDO 读取接口，适合低层配置或调试。
        return self._drive().sdo_read(index, subindex)

    def write_sdo_raw(self, index: int, subindex: int, payload: bytes) -> None:
        # 通用 SDO 写入接口，适合低层配置或调试。
        self._drive().sdo_write(index, subindex, payload)

    def read_sdo_u8(self, index: int, subindex: int) -> int:
        # 带类型的 SDO 辅助函数，让调用端更短更清晰。
        return struct.unpack("<B", self.read_sdo_raw(index, subindex))[0]

    def read_sdo_u16(self, index: int, subindex: int) -> int:
        return struct.unpack("<H", self.read_sdo_raw(index, subindex))[0]

    def read_sdo_u32(self, index: int, subindex: int) -> int:
        return struct.unpack("<I", self.read_sdo_raw(index, subindex))[0]

    def read_sdo_i32(self, index: int, subindex: int) -> int:
        return struct.unpack("<i", self.read_sdo_raw(index, subindex))[0]

    def write_sdo_u8(self, index: int, subindex: int, value: int) -> None:
        self.write_sdo_raw(index, subindex, struct.pack("<B", value))

    def write_sdo_u16(self, index: int, subindex: int, value: int) -> None:
        self.write_sdo_raw(index, subindex, struct.pack("<H", value))

    def write_sdo_u32(self, index: int, subindex: int, value: int) -> None:
        self.write_sdo_raw(index, subindex, struct.pack("<I", value))

    def load_xml_slave_profiles(self) -> tuple[XmlSlaveProfile, ...]:
        if self.xml_slave_profiles:
            return self.xml_slave_profiles
        tree = ET.parse(self.eni_xml_path)
        root = tree.getroot()
        profiles: list[XmlSlaveProfile] = []
        for slave_elem in root.findall("./Config/Slave"):
            info_elem = slave_elem.find("Info")
            process_data_elem = slave_elem.find("ProcessData")
            if info_elem is None or process_data_elem is None:
                continue
            name = (info_elem.findtext("Name") or "").strip()
            vendor_id = int(info_elem.findtext("VendorId") or "0")
            product_code = int(info_elem.findtext("ProductCode") or "0")
            revision = int(info_elem.findtext("RevisionNo") or "0")
            output_bits = int(process_data_elem.findtext("Send/BitLength") or "0")
            input_bits = int(process_data_elem.findtext("Recv/BitLength") or "0")
            coe_init_cmds = self._parse_xml_coe_init_cmds(slave_elem)
            profiles.append(
                XmlSlaveProfile(
                    name=name,
                    vendor_id=vendor_id,
                    product_code=product_code,
                    revision=revision,
                    process_data_target=ProcessDataTarget(
                        input_bytes=input_bits // 8,
                        output_bytes=output_bits // 8,
                    ),
                    coe_init_cmds=tuple(coe_init_cmds),
                )
            )
        self.xml_slave_profiles = tuple(profiles)
        return self.xml_slave_profiles

    def ensure_runtime_matches_xml(self, max_attempts: int = 2) -> None:
        if not self.xml_slave_profiles:
            self.load_xml_slave_profiles()
        if self._runtime_matches_xml_targets():
            return
        self._set_preop_for_config()
        for attempt in range(max(max_attempts, 1)):
            self._apply_xml_reconfiguration()
            self.configure()
            if self._runtime_matches_xml_targets():
                return
            if attempt + 1 < max(max_attempts, 1):
                self._set_preop_for_config()
        raise RuntimeError(self._xml_mismatch_summary())

    def refresh_pdo_layouts(self) -> None:
        self.txpdo_layout = self._read_pdo_layout(0x1C13)
        self.rxpdo_layout = self._read_pdo_layout(0x1C12)

    def describe_pdo_layout(self, direction: str = "tx") -> list[str]:
        self._ensure_pdo_layouts()
        layout = self.txpdo_layout if direction.lower() == "tx" else self.rxpdo_layout
        if layout is None:
            return []
        rows = [
            f"assign=0x{layout.assign_index:04X} bits={layout.total_bits} bytes={layout.total_bytes} "
            f"aligned={int(layout.is_byte_aligned)} pdos="
            + ",".join(f"0x{pdo:04X}" for pdo in layout.assigned_pdos)
        ]
        for entry in layout.entries:
            offset_text = (
                f"{entry.offset_bytes}B"
                if entry.is_byte_aligned
                else f"{entry.offset_bits}b"
            )
            size_text = (
                f"{entry.size_bytes}B"
                if entry.has_byte_sized_length
                else f"{entry.bit_length}b"
            )
            rows.append(
                "  pdo=0x{pdo:04X}:{pdo_sub} map=0x{mapping:08X} obj=0x{obj:04X}:{sub} bits={bits} size={size} offset={offset}".format(
                    pdo=entry.pdo_index,
                    pdo_sub=entry.entry_subindex,
                    mapping=entry.raw_mapping,
                    obj=entry.object_index,
                    sub=entry.object_subindex,
                    bits=entry.bit_length,
                    size=size_text,
                    offset=offset_text,
                )
            )
        return rows

    def describe_required_rxpdo(self) -> list[str]:
        self._ensure_pdo_layouts()
        if self.rxpdo_layout is None:
            return ["RxPDO layout unavailable"]
        rows: list[str] = []
        for object_index, object_subindex, label in REQUIRED_RXPDO_ENTRIES:
            present = self.rxpdo_layout.find_entry(object_index, object_subindex) is not None
            rows.append(
                "required obj=0x{obj:04X}:{sub} label={label} present={present}".format(
                    obj=object_index,
                    sub=object_subindex,
                    label=label,
                    present=int(present),
                )
            )
        for object_index, object_subindex, label in OPTIONAL_RXPDO_ENTRIES:
            present = self.rxpdo_layout.find_entry(object_index, object_subindex) is not None
            rows.append(
                "optional obj=0x{obj:04X}:{sub} label={label} present={present}".format(
                    obj=object_index,
                    sub=object_subindex,
                    label=label,
                    present=int(present),
                )
            )
        return rows

    def ensure_required_rxpdo_entries(self) -> None:
        self._ensure_pdo_layouts()
        if self.rxpdo_layout is None:
            raise RuntimeError("RxPDO layout is not available")
        missing = [
            f"0x{object_index:04X}:{object_subindex}"
            for object_index, object_subindex, _label in REQUIRED_RXPDO_ENTRIES
            if self.rxpdo_layout.find_entry(object_index, object_subindex) is None
        ]
        if missing:
            raise RuntimeError(
                "Runtime RxPDO is missing required control objects: " + ", ".join(missing)
            )

    def read_encoder_config(self, channel: int) -> EncoderConfig:
        config_index = self._encoder_config_index(channel)
        index_availability = None
        try:
            index_availability = bool(self.read_sdo_u8(config_index, 22))
        except Exception:
            index_availability = None
        return EncoderConfig(
            channel=channel,
            sensor_port=self.read_sdo_u8(config_index, 1),
            encoder_type=self.read_sdo_u8(config_index, 2),
            resolution=self.read_sdo_u32(config_index, 3),
            polarity=bool(self.read_sdo_u8(config_index, 5)),
            singleturn_offset=self.read_sdo_u32(config_index, 6),
            index_availability=index_availability,
        )

    def read_encoder_feedback(self, channel: int) -> EncoderFeedback:
        feedback_index = self._encoder_feedback_index(channel)
        return EncoderFeedback(
            channel=channel,
            raw_position=self.read_sdo_u32(feedback_index, 1),
            adjusted_position=self.read_sdo_i32(feedback_index, 2),
            velocity_rpm=self.read_sdo_i32(feedback_index, 3),
        )

    def assess_encoder_feedback_samples(
        self,
        channel: int,
        config: EncoderConfig,
        feedback_samples: list[EncoderFeedback] | tuple[EncoderFeedback, ...],
        drive_position_samples: list[int] | tuple[int, ...],
        pdo_encoder_samples: list[int] | tuple[int, ...],
    ) -> EncoderAssessment:
        if not feedback_samples:
            raise ValueError("feedback_samples must not be empty")

        raw_samples = tuple(sample.raw_position for sample in feedback_samples)
        adjusted_samples = tuple(sample.adjusted_position for sample in feedback_samples)
        velocity_samples = tuple(sample.velocity_rpm for sample in feedback_samples)
        drive_positions = tuple(drive_position_samples)
        pdo_samples = tuple(pdo_encoder_samples)

        raw_updates = len(set(raw_samples)) > 1
        adjusted_updates = len(set(adjusted_samples)) > 1
        adjusted_stuck_zero = all(value == 0 for value in adjusted_samples)
        drive_position_updates = len(set(drive_positions)) > 1 if drive_positions else False
        pdo_updates = len(set(pdo_samples)) > 1 if pdo_samples else False
        pdo_stuck_zero = all(value == 0 for value in pdo_samples) if pdo_samples else False
        pdo_abs_errors = (
            tuple(abs(pdo_value - adjusted_value) for pdo_value, adjusted_value in zip(pdo_samples, adjusted_samples))
            if pdo_samples and len(pdo_samples) == len(adjusted_samples)
            else ()
        )
        pdo_matches_adjusted = bool(pdo_abs_errors) and all(
            err <= PDO_ENCODER_MATCH_TOLERANCE_COUNTS for err in pdo_abs_errors
        )
        max_pdo_abs_error = max(pdo_abs_errors) if pdo_abs_errors else None

        adjusted_nonzero = any(value != 0 for value in adjusted_samples)

        if adjusted_updates and adjusted_nonzero:
            adjusted_valid_assumed = True
            if pdo_matches_adjusted:
                status_label = "adjusted_valid"
                detail = (
                    "Adjusted position is updating and matches PDO within tolerance "
                    f"(tol={PDO_ENCODER_MATCH_TOLERANCE_COUNTS} counts"
                    + (f", max_err={max_pdo_abs_error}" if max_pdo_abs_error is not None else "")
                    + ")."
                )
            else:
                status_label = "adjusted_valid_with_sampling_skew"
                detail = (
                    "Adjusted position is updating and non-zero; PDO differs slightly from SDO, "
                    "which is acceptable for asynchronous sampling."
                    + (f" max_err={max_pdo_abs_error} counts." if max_pdo_abs_error is not None else "")
                )
        elif config.encoder_type == 4 and adjusted_nonzero:
            adjusted_valid_assumed = True
            status_label = "absolute_encoder_valid"
            detail = "BiSS encoder adjusted position is non-zero; treat it as valid."
        elif raw_updates and adjusted_stuck_zero:
            adjusted_valid_assumed = False
            status_label = "waiting_for_index"
            detail = "Raw feedback is alive, but adjusted position is still zero. Likely waiting for index/reference."
        elif drive_position_updates and not raw_updates and adjusted_stuck_zero:
            adjusted_valid_assumed = False
            status_label = "feedback_dead"
            detail = "Drive position is moving, but encoder raw/adjusted feedback is not updating."
        elif config.encoder_type == 2 and config.index_availability:
            adjusted_valid_assumed = False
            status_label = "incremental_needs_reference"
            detail = "Incremental encoder with index cannot be assumed valid until adjusted position clearly updates."
        elif adjusted_stuck_zero:
            adjusted_valid_assumed = False
            status_label = "adjusted_zero"
            detail = "Adjusted position stayed at zero during the sample window."
        else:
            adjusted_valid_assumed = False
            status_label = "inconclusive"
            detail = "Feedback did not provide enough evidence to treat adjusted position as valid."

        return EncoderAssessment(
            channel=channel,
            config=config,
            last_feedback=feedback_samples[-1],
            raw_samples=raw_samples,
            adjusted_samples=adjusted_samples,
            velocity_samples=velocity_samples,
            drive_position_samples=drive_positions,
            pdo_encoder_samples=pdo_samples,
            raw_updates=raw_updates,
            adjusted_updates=adjusted_updates,
            adjusted_stuck_zero=adjusted_stuck_zero,
            drive_position_updates=drive_position_updates,
            pdo_updates=pdo_updates,
            pdo_stuck_zero=pdo_stuck_zero,
            pdo_matches_adjusted=pdo_matches_adjusted,
            adjusted_valid_assumed=adjusted_valid_assumed,
            status_label=status_label,
            detail=detail,
        )

    def sample_encoder_assessment(
        self,
        channel: int,
        cycles: int,
        command: DriveCommand | None = None,
    ) -> EncoderAssessment:
        config = self.read_encoder_config(channel)
        feedback_samples: list[EncoderFeedback] = []
        drive_position_samples: list[int] = []
        pdo_encoder_samples: list[int] = []
        for _ in range(max(cycles, 1)):
            if command is not None:
                self.cycle(command)
            feedback_samples.append(self.read_encoder_feedback(channel))
            state = self.read_state()
            drive_position_samples.append(state.position)
            pdo_encoder_samples.append(state.encoder1 if channel == 1 else state.encoder2)
        return self.assess_encoder_feedback_samples(
            channel,
            config,
            feedback_samples,
            drive_position_samples,
            pdo_encoder_samples,
        )

    def _drive(self):
        # 驱动器对象在 scan() 后才会赋值，统一从这里访问，避免到处分散判断。
        if self.drive is None:
            raise RuntimeError("Drive not ready. Call connect() first.")
        return self.drive

    def _drive_input(self) -> bytes:
        # 读取所选驱动器的原始输入 PDO 字节。
        return bytes(self._drive().input)

    def _drive_output(self) -> bytes:
        # 读取所选驱动器的原始输出 PDO 字节。
        return bytes(self._drive().output)

    def _ensure_pdo_layouts(self) -> None:
        if self.txpdo_layout is None or self.rxpdo_layout is None:
            self.refresh_pdo_layouts()

    def _parse_xml_coe_init_cmds(self, slave_elem: ET.Element) -> list[XmlCoEInitCmd]:
        commands: list[XmlCoEInitCmd] = []
        paths = (
            "./Mailbox/CoE/InitCmds/InitCmd",
            "./CoE/InitCmds/InitCmd",
        )
        for path in paths:
            for cmd_elem in slave_elem.findall(path):
                ccs = (cmd_elem.findtext("Ccs") or "").strip()
                if ccs and ccs != "1":
                    continue
                index_text = (cmd_elem.findtext("Index") or "").strip()
                subindex_text = (cmd_elem.findtext("SubIndex") or "").strip()
                data_hex = ((cmd_elem.findtext("Data") or "").strip()).replace(" ", "")
                if not index_text or not subindex_text or not data_hex:
                    continue
                commands.append(
                    XmlCoEInitCmd(
                        index=int(index_text),
                        subindex=int(subindex_text),
                        payload=bytes.fromhex(data_hex),
                        complete_access=str(cmd_elem.get("CompleteAccess", "")).lower() == "true",
                        comment=(cmd_elem.findtext("Comment") or "").strip(),
                        transition=(cmd_elem.findtext("Transition") or "").strip(),
                    )
                )
        return commands

    def _runtime_matches_xml_targets(self) -> bool:
        if not self.xml_slave_profiles:
            return True
        runtime_by_key = {
            (item.vendor_id, item.product_code): item
            for item in self.read_slave_states()
        }
        for profile in self.xml_slave_profiles:
            runtime = runtime_by_key.get((profile.vendor_id, profile.product_code))
            if runtime is None:
                continue
            if runtime.input_size != profile.process_data_target.input_bytes:
                return False
            if runtime.output_size != profile.process_data_target.output_bytes:
                return False
        return True

    def _xml_mismatch_summary(self) -> str:
        runtime_by_key = {
            (item.vendor_id, item.product_code): item
            for item in self.read_slave_states()
        }
        rows = ["Runtime process data does not match XML targets:"]
        for profile in self.xml_slave_profiles:
            runtime = runtime_by_key.get((profile.vendor_id, profile.product_code))
            if runtime is None:
                rows.append(
                    "  xml slave name={name!r} man={man} id={prod}: runtime slave not found".format(
                        name=profile.name,
                        man=profile.vendor_id,
                        prod=profile.product_code,
                    )
                )
                continue
            rows.append(
                "  slave[{idx}] name={name!r} in={inp}/{target_in} out={out}/{target_out}".format(
                    idx=runtime.index,
                    name=runtime.name,
                    inp=runtime.input_size,
                    target_in=profile.process_data_target.input_bytes,
                    out=runtime.output_size,
                    target_out=profile.process_data_target.output_bytes,
                )
            )
        return "\n".join(rows)

    def _apply_xml_reconfiguration(self) -> None:
        runtime_by_key = {
            (item.vendor_id, item.product_code): item
            for item in self.read_slave_states()
        }
        for profile in self.xml_slave_profiles:
            runtime = runtime_by_key.get((profile.vendor_id, profile.product_code))
            if runtime is None or not profile.coe_init_cmds:
                continue
            slave = self.master.slaves[runtime.index]
            for cmd in profile.coe_init_cmds:
                slave.sdo_write(cmd.index, cmd.subindex, cmd.payload, ca=cmd.complete_access)

    def _set_preop_for_config(self) -> None:
        self.master.state = pysoem.PREOP_STATE
        self.master.write_state()
        for slave in self.master.slaves:
            try:
                slave.state = pysoem.PREOP_STATE
            except Exception:
                pass
        self.master.state_check(pysoem.PREOP_STATE, 50_000)
        self.master.read_state()

    def _read_pdo_layout(self, assign_index: int) -> PdoLayout:
        assigned_count = self.read_sdo_u8(assign_index, 0)
        assigned_pdos = tuple(self.read_sdo_u16(assign_index, subindex) for subindex in range(1, assigned_count + 1))
        entries: list[PdoEntry] = []
        offset_bits = 0
        for pdo_index in assigned_pdos:
            entry_count = self.read_sdo_u8(pdo_index, 0)
            for entry_subindex in range(1, entry_count + 1):
                raw_mapping = self.read_sdo_u32(pdo_index, entry_subindex)
                # Standard PDO mapping entry layout:
                # [31:16] object index, [15:8] object subindex, [7:0] bit length.
                object_index = (raw_mapping >> 16) & 0xFFFF
                object_subindex = (raw_mapping >> 8) & 0xFF
                bit_length = raw_mapping & 0xFF
                entries.append(
                    PdoEntry(
                        pdo_index=pdo_index,
                        entry_subindex=entry_subindex,
                        raw_mapping=raw_mapping,
                        object_index=object_index,
                        object_subindex=object_subindex,
                        bit_length=bit_length,
                        offset_bits=offset_bits,
                    )
                )
                offset_bits += bit_length
        return PdoLayout(assign_index=assign_index, assigned_pdos=assigned_pdos, entries=tuple(entries))

    def _read_input_entry(self, raw: bytes, object_index: int, object_subindex: int, fmt: str):
        self._ensure_pdo_layouts()
        if self.txpdo_layout is None:
            raise RuntimeError("TxPDO layout is not available")
        entry = self.txpdo_layout.find_entry(object_index, object_subindex)
        if entry is None:
            raise RuntimeError(f"TxPDO entry 0x{object_index:04X}:{object_subindex} not found in runtime mapping")
        return struct.unpack_from(fmt, raw, entry.offset_bytes)[0]

    def _write_output_entry(
        self,
        output: bytearray,
        object_index: int,
        object_subindex: int,
        value: int,
        signed: bool,
    ) -> None:
        self._ensure_pdo_layouts()
        if self.rxpdo_layout is None:
            raise RuntimeError("RxPDO layout is not available")
        entry = self.rxpdo_layout.find_entry(object_index, object_subindex)
        if entry is None:
            raise RuntimeError(f"RxPDO entry 0x{object_index:04X}:{object_subindex} not found in runtime mapping")
        output[entry.offset_bytes : entry.offset_bytes + entry.size_bytes] = int(value).to_bytes(
            entry.size_bytes,
            byteorder="little",
            signed=signed,
        )

    def _write_output_entry_if_present(
        self,
        output: bytearray,
        object_index: int,
        object_subindex: int,
        value: int,
        signed: bool,
        required: bool = False,
    ) -> bool:
        self._ensure_pdo_layouts()
        if self.rxpdo_layout is None:
            raise RuntimeError("RxPDO layout is not available")
        entry = self.rxpdo_layout.find_entry(object_index, object_subindex)
        if entry is None:
            if required:
                raise RuntimeError(
                    f"Required RxPDO entry 0x{object_index:04X}:{object_subindex} not found in runtime mapping"
                )
            return False
        self._write_output_entry(output, object_index, object_subindex, value, signed=signed)
        return True

    def _encoder_config_index(self, channel: int) -> int:
        if channel == 1:
            return 0x2110
        if channel == 2:
            return 0x2112
        raise ValueError(f"Unsupported encoder channel: {channel}")

    def _encoder_feedback_index(self, channel: int) -> int:
        if channel == 1:
            return 0x2111
        if channel == 2:
            return 0x2113
        raise ValueError(f"Unsupported encoder channel: {channel}")

    def _identify_roles(self) -> SlaveRoles:
        box_index = None
        drive_index = None
        for index, slave in enumerate(self.master.slaves):
            name = (getattr(slave, "name", "") or "").lower()
            vendor_id = getattr(slave, "man", 0)
            product_code = getattr(slave, "id", 0)
            input_size = len(bytes(getattr(slave, "input", b"")))
            output_size = len(bytes(getattr(slave, "output", b"")))

            # Check the explicit drive identity first.  A SOMANET drive can also
            # have PDOs larger than the transparent-box payload, so using PDO
            # size before the slave name can misclassify every drive as a box on
            # a multi-joint bus.
            if "somanet" in name or "circulo" in name:
                drive_index = index
                continue

            if (
                "amsamotion_ec_mb" in name
                or (vendor_id == BOX_VENDOR_ID and product_code == BOX_PRODUCT_CODE)
                or (input_size >= BOX_TRANSPARENT_PDO_BYTES and output_size >= BOX_TRANSPARENT_PDO_BYTES)
            ):
                box_index = index
                continue

        if box_index is not None and drive_index is None and len(self.master.slaves) == 2:
            drive_index = 1 - box_index

        if drive_index is None:
            raise RuntimeError(
                f"Could not identify drive automatically. box_index={box_index}, drive_index={drive_index}"
            )
        return SlaveRoles(box_index=box_index, drive_index=drive_index)

    def _flush_zero_outputs(self, cycles: int = 3) -> None:
        if not getattr(self.master, "slaves", None):
            return
        for slave in self.master.slaves:
            output_size = len(bytes(getattr(slave, "output", b"")))
            if output_size:
                slave.output = bytes(output_size)
        for _ in range(cycles):
            self.master.send_processdata()
            self.master.receive_processdata(5_000)
            time.sleep(min(self.cycle_time_s, 0.01))

    def _recover_processdata_exchange(self, recovery_cycles: int = 20) -> None:
        for slave in self.master.slaves:
            output_size = len(bytes(getattr(slave, "output", b"")))
            if output_size:
                slave.output = bytes(output_size)
        try:
            self.master.state = pysoem.OP_STATE
            self.master.write_state()
        except Exception:
            pass
        for slave in self.master.slaves:
            try:
                slave.state = pysoem.OP_STATE
            except Exception:
                pass
        for _ in range(max(recovery_cycles, 1)):
            self.master.send_processdata()
            self.master.receive_processdata(5_000)
            time.sleep(min(self.cycle_time_s, 0.01))


__all__ = [
    "BOX_PRODUCT_CODE",
    "BOX_TRANSPARENT_PDO_BYTES",
    "BOX_VENDOR_ID",
    "CST_MODE",
    "Cia402TransitionResult",
    "DEFAULT_POSITION_SIGN",
    "DEFAULT_TORQUE_SIGN",
    "DEFAULT_VELOCITY_SIGN",
    "ENCODER_TYPE_NAMES",
    "EncoderAssessment",
    "DriveCalibration",
    "DriveCommand",
    "DriveDiagnostics",
    "EncoderConfig",
    "EncoderFeedback",
    "DriveState",
    "DriveStateSI",
    "ENABLE_OPERATION",
    "PdoPreheatResult",
    "SEARealtimeComm",
    "SHUTDOWN",
    "SWITCH_ON",
    "SlaveInfo",
    "SlaveRoles",
    "StartupDiagnostics",
    "cia402_state_name",
    "state_name",
    "summarize_wkc",
]
