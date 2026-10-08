from __future__ import annotations

import os
import struct
import sys
import time
from dataclasses import dataclass

import pysoem


BOX_VENDOR_ID = 2965
BOX_PRODUCT_CODE = 5120

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

DRIVE_STATUS_LAYOUT_BYTES = 25
BOX_TRANSPARENT_PDO_BYTES = 130
ECODER_ID0_REQUEST = 0x02
ECODER_ID0_RESPONSE_SIZE = 6


@dataclass(frozen=True)
class RuntimeConfig:
    ifname: str = "eno1"
    cycle_time_s: float = 0.010
    box_mode: int = BOX_MODE_TRANSPARENT
    box_interface: int = BOX_INTERFACE_RS485
    box_frame: int = BOX_FRAME_8N1
    box_baud_selector: int = BOX_BAUD_SELECTOR_CUSTOM
    box_baud: int = 2_500_000
    box_polling_time_ms: int = 50
    ecoder_request: int = ECODER_ID0_REQUEST


@dataclass(frozen=True)
class SlaveRoles:
    box_index: int
    drive_index: int


@dataclass(frozen=True)
class DriveStatus:
    statusword: int
    mode_display: int
    position: int
    velocity: int
    torque: int
    following_error: int
    encoder1: int
    encoder2: int


@dataclass(frozen=True)
class TransparentFrame:
    valid: int
    size: int
    data: bytes


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


def log(message: str) -> None:
    print(message, flush=True)


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


def ensure_root() -> None:
    if os.geteuid() == 0:
        return
    sudo_cmd = [
        "sudo",
        "-n",
        "env",
        "PYTHONDONTWRITEBYTECODE=1",
        "PYTHONUNBUFFERED=1",
        sys.executable,
        *sys.argv,
    ]
    try:
        os.execvp("sudo", sudo_cmd)
    except FileNotFoundError as exc:
        raise SystemExit("EtherCAT raw socket access requires root. Re-run this script with sudo.") from exc
    except OSError as exc:
        raise SystemExit("EtherCAT raw socket access requires root, and automatic sudo re-exec failed.") from exc


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


class EtherCATSlave:
    def __init__(self, slave: pysoem.CdefSlave):
        self.slave = slave

    def read_pdo(self) -> bytes:
        return bytes(self.slave.input)

    def write_pdo(self, payload: bytes) -> None:
        self.slave.output = payload

    def read_sdo_raw(self, index: int, subindex: int) -> bytes:
        return self.slave.sdo_read(index, subindex)

    def write_sdo_raw(self, index: int, subindex: int, payload: bytes) -> None:
        self.slave.sdo_write(index, subindex, payload)

    def read_sdo_u8(self, index: int, subindex: int) -> int:
        return struct.unpack("<B", self.read_sdo_raw(index, subindex))[0]

    def read_sdo_u16(self, index: int, subindex: int) -> int:
        return struct.unpack("<H", self.read_sdo_raw(index, subindex))[0]

    def read_sdo_u32(self, index: int, subindex: int) -> int:
        return struct.unpack("<I", self.read_sdo_raw(index, subindex))[0]

    def write_sdo_u8(self, index: int, subindex: int, value: int) -> None:
        self.write_sdo_raw(index, subindex, struct.pack("<B", value))

    def write_sdo_u16(self, index: int, subindex: int, value: int) -> None:
        self.write_sdo_raw(index, subindex, struct.pack("<H", value))

    def write_sdo_u32(self, index: int, subindex: int, value: int) -> None:
        self.write_sdo_raw(index, subindex, struct.pack("<I", value))

    def input_bytes(self) -> bytes:
        return self.read_pdo()

    def output_bytes(self) -> bytes:
        return bytes(self.slave.output)

    def write_output(self, payload: bytes) -> None:
        self.write_pdo(payload)


class RS485Box(EtherCATSlave):
    def configure_transparent(self, config: RuntimeConfig) -> dict[str, int]:
        self.write_sdo_u8(BOX_OBJECT_INDEX, BOX_MODE_SUBINDEX, config.box_mode)
        self.write_sdo_u8(BOX_OBJECT_INDEX, BOX_INTERFACE_SUBINDEX, config.box_interface)
        self.write_sdo_u8(BOX_OBJECT_INDEX, BOX_FRAME_SUBINDEX, config.box_frame)
        self._set_custom_baud_selector(config.box_baud_selector)
        self.write_sdo_u32(BOX_OBJECT_INDEX, BOX_EXPLICIT_BAUD_SUBINDEX, config.box_baud)
        self.write_sdo_u16(BOX_OBJECT_INDEX, BOX_POLLING_TIME_SUBINDEX, config.box_polling_time_ms)
        return {
            "mode": config.box_mode,
            "interface": config.box_interface,
            "frame": config.box_frame,
            "baud_selector": config.box_baud_selector,
            "explicit_baud": config.box_baud,
            "polling_ms": config.box_polling_time_ms,
        }

    def _set_custom_baud_selector(self, preferred: int) -> None:
        try:
            self.write_sdo_u8(BOX_OBJECT_INDEX, BOX_BAUD_SELECTOR_SUBINDEX, preferred)
            return
        except Exception:
            fallback = 10 if preferred == 0x10 else 0x10
            self.write_sdo_u8(BOX_OBJECT_INDEX, BOX_BAUD_SELECTOR_SUBINDEX, fallback)

    def write_transparent(self, payload: bytes, valid: int) -> TransparentFrame:
        output = bytearray(self.output_bytes())
        if not output:
            return TransparentFrame(valid=0, size=0, data=b"")
        size = min(len(payload), max(len(output) - 2, 0))
        output[0] = valid & 0xFF
        output[1] = size
        output[2 : 2 + size] = payload[:size]
        self.write_output(bytes(output))
        return TransparentFrame(valid=output[0], size=size, data=bytes(output[2 : 2 + size]))

    def read_transparent(self) -> TransparentFrame:
        raw = self.read_pdo()
        if len(raw) < 2:
            return TransparentFrame(valid=0, size=0, data=b"")
        size = min(raw[1], max(len(raw) - 2, 0))
        return TransparentFrame(valid=raw[0], size=size, data=raw[2 : 2 + size])


class SEADrive(EtherCATSlave):
    def read_status(self) -> DriveStatus:
        raw = self.read_pdo()
        if len(raw) < DRIVE_STATUS_LAYOUT_BYTES:
            raise RuntimeError(
                f"Drive input PDO is too short for TxPDO Mapping 1. got={len(raw)} need={DRIVE_STATUS_LAYOUT_BYTES}"
            )
        return DriveStatus(
            statusword=struct.unpack_from("<H", raw, 0)[0],
            mode_display=struct.unpack_from("<b", raw, 2)[0],
            position=struct.unpack_from("<i", raw, 3)[0],
            velocity=struct.unpack_from("<i", raw, 7)[0],
            torque=struct.unpack_from("<h", raw, 11)[0],
            following_error=struct.unpack_from("<i", raw, 13)[0],
            encoder1=struct.unpack_from("<i", raw, 17)[0],
            encoder2=struct.unpack_from("<i", raw, 21)[0],
        )


class EtherCATSystem:
    def __init__(self, config: RuntimeConfig):
        self.config = config
        self.master = pysoem.Master()
        self.roles: SlaveRoles | None = None
        self.box: RS485Box | None = None
        self.drive: SEADrive | None = None

    @property
    def slaves(self):
        return self.master.slaves

    @property
    def state(self) -> int:
        return self.master.state

    @state.setter
    def state(self, value: int) -> None:
        self.master.state = value

    @property
    def expected_wkc(self):
        return getattr(self.master, "expected_wkc", None)

    def read_state(self) -> None:
        self.master.read_state()

    def write_state(self) -> None:
        self.master.write_state()

    def open(self) -> None:
        ensure_root()
        self.master.open(self.config.ifname)

    def close(self) -> None:
        try:
            self.master.state = pysoem.INIT_STATE
            self.master.write_state()
        except Exception:
            pass
        try:
            self.master.close()
        except Exception:
            pass

    def scan(self) -> int:
        count = self.master.config_init()
        if count <= 0:
            raise RuntimeError("No EtherCAT slaves were found on the selected interface.")
        self.roles = self._identify_roles()
        self.box = RS485Box(self.master.slaves[self.roles.box_index])
        self.drive = SEADrive(self.master.slaves[self.roles.drive_index])
        return count

    def describe_slaves(self) -> None:
        for index, slave in enumerate(self.master.slaves):
            log(
                "Slave {idx}: name={name!r} man={man} id={prod} rev={rev} "
                "state={state} in={inp} out={out}".format(
                    idx=index,
                    name=getattr(slave, "name", ""),
                    man=getattr(slave, "man", 0),
                    prod=getattr(slave, "id", 0),
                    rev=getattr(slave, "rev", 0),
                    state=state_name(getattr(slave, "state", 0)),
                    inp=len(bytes(getattr(slave, "input", b""))),
                    out=len(bytes(getattr(slave, "output", b""))),
                )
            )

    def map_process_data(self) -> int:
        for slave in self.master.slaves:
            try:
                slave._disable_complete_access()
            except Exception:
                pass
        return self.master.config_map()

    def start_op(self) -> None:
        for slave in self.master.slaves:
            output_size = len(bytes(getattr(slave, "output", b"")))
            if output_size:
                slave.output = bytes(output_size)

        if self.master.state_check(pysoem.SAFEOP_STATE, 50_000) != pysoem.SAFEOP_STATE:
            self.read_state()
            raise RuntimeError(
                "Failed to reach SAFE-OP. Current slave states: "
                + ", ".join(f"{idx}:{state_name(slave.state)}" for idx, slave in enumerate(self.master.slaves))
            )

        self.state = pysoem.OP_STATE
        self.write_state()
        for _ in range(50):
            self.cycle()
            if self.master.state_check(pysoem.OP_STATE, 5_000) == pysoem.OP_STATE:
                return

        self.read_state()
        raise RuntimeError(
            "Failed to reach OP. Current slave states: "
            + ", ".join(f"{idx}:{state_name(slave.state)}" for idx, slave in enumerate(self.master.slaves))
        )

    def cycle(self) -> int:
        self.master.send_processdata()
        wkc = self.master.receive_processdata(5_000)
        time.sleep(self.config.cycle_time_s)
        return wkc

    def _identify_roles(self) -> SlaveRoles:
        box_index = None
        drive_index = None
        for index, slave in enumerate(self.master.slaves):
            name = (getattr(slave, "name", "") or "").lower()
            man = getattr(slave, "man", None)
            product = getattr(slave, "id", None)
            input_size = len(bytes(getattr(slave, "input", b"")))
            output_size = len(bytes(getattr(slave, "output", b"")))

            if (
                "amsamotion_ec_mb" in name
                or (man == BOX_VENDOR_ID and product == BOX_PRODUCT_CODE)
                or (input_size >= BOX_TRANSPARENT_PDO_BYTES and output_size >= BOX_TRANSPARENT_PDO_BYTES)
            ):
                box_index = index
                continue

            if "somanet" in name or "circulo" in name:
                drive_index = index

        if box_index is not None and drive_index is None and len(self.master.slaves) == 2:
            drive_index = 1 - box_index

        if box_index is None or drive_index is None:
            raise RuntimeError(
                f"Could not identify both box and drive automatically. box_index={box_index}, drive_index={drive_index}"
            )
        return SlaveRoles(box_index=box_index, drive_index=drive_index)


def print_exception_and_exit(prefix: str, exc: BaseException) -> None:
    log(f"{prefix}: {exc}")
    raise SystemExit(1) from exc
