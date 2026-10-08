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

BOX_TRANSPARENT_PDO_BYTES = 130
ECODER_ID0_REQUEST = 0x02
ECODER_ID0_RESPONSE_SIZE = 6


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
        raise SystemExit("EtherCAT raw socket access requires root. Re-run with sudo.") from exc
    except OSError as exc:
        raise SystemExit("EtherCAT raw socket access requires root, and automatic sudo re-exec failed.") from exc


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


@dataclass(frozen=True, slots=True)
class ThirdEncoderRuntimeConfig:
    ifname: str = "eno1"
    cycle_time_s: float = 0.010
    box_mode: int = BOX_MODE_TRANSPARENT
    box_interface: int = BOX_INTERFACE_RS485
    box_frame: int = BOX_FRAME_8N1
    box_baud_selector: int = BOX_BAUD_SELECTOR_CUSTOM
    box_baud: int = 2_500_000
    box_polling_time_ms: int = 50
    ecoder_request: int = ECODER_ID0_REQUEST


@dataclass(frozen=True, slots=True)
class SlaveRoles:
    box_index: int
    drive_index: int


@dataclass(frozen=True, slots=True)
class TransparentFrame:
    valid: int
    size: int
    data: bytes


@dataclass(frozen=True, slots=True)
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


@dataclass(frozen=True, slots=True)
class ThirdEncoderSample:
    cycle_index: int
    tx_valid: int
    wkc: int
    request: int
    response: ECoderId0Response


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
    def configure_transparent(self, config: ThirdEncoderRuntimeConfig) -> dict[str, int]:
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

    def read_transparent_config(self) -> dict[str, int]:
        return {
            "mode": self.read_sdo_u8(BOX_OBJECT_INDEX, BOX_MODE_SUBINDEX),
            "interface": self.read_sdo_u8(BOX_OBJECT_INDEX, BOX_INTERFACE_SUBINDEX),
            "frame": self.read_sdo_u8(BOX_OBJECT_INDEX, BOX_FRAME_SUBINDEX),
            "baud_selector": self.read_sdo_u8(BOX_OBJECT_INDEX, BOX_BAUD_SELECTOR_SUBINDEX),
            "explicit_baud": self.read_sdo_u32(BOX_OBJECT_INDEX, BOX_EXPLICIT_BAUD_SUBINDEX),
            "polling_ms": self.read_sdo_u16(BOX_OBJECT_INDEX, BOX_POLLING_TIME_SUBINDEX),
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


class ThirdEncoderSensor:
    def __init__(self, config: ThirdEncoderRuntimeConfig | None = None):
        self.config = config or ThirdEncoderRuntimeConfig()
        self.master = pysoem.Master()
        self.roles: SlaveRoles | None = None
        self.box: RS485Box | None = None
        self.drive: EtherCATSlave | None = None
        self._opened = False
        self._tx_valid = 0
        self._sample_index = 0

    @property
    def expected_wkc(self) -> int | None:
        return getattr(self.master, "expected_wkc", None)

    def open(self) -> None:
        ensure_root()
        self.master = pysoem.Master()
        self.master.open(self.config.ifname)
        self.roles = None
        self.box = None
        self.drive = None
        self._opened = True
        self._tx_valid = 0
        self._sample_index = 0

    def close(self) -> None:
        if not self._opened:
            return
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
        count = self.master.config_init()
        if count <= 0:
            raise RuntimeError("No EtherCAT slaves were found on the selected interface.")
        self.roles = self._identify_roles()
        self.box = RS485Box(self.master.slaves[self.roles.box_index])
        if self.roles.drive_index is not None:
            self.drive = EtherCATSlave(self.master.slaves[self.roles.drive_index])
        return count

    def describe_slaves(self) -> list[str]:
        self.master.read_state()
        rows = []
        for index, slave in enumerate(self.master.slaves):
            rows.append(
                "slave[{idx}] name={name!r} man={man} id={prod} rev={rev} state={state} in={inp} out={out}".format(
                    idx=index,
                    name=getattr(slave, "name", "") or "",
                    man=getattr(slave, "man", 0),
                    prod=getattr(slave, "id", 0),
                    rev=getattr(slave, "rev", 0),
                    state=state_name(getattr(slave, "state", 0)),
                    inp=len(bytes(getattr(slave, "input", b""))),
                    out=len(bytes(getattr(slave, "output", b""))),
                )
            )
        return rows

    def configure_transparent(self) -> dict[str, int]:
        if self.box is None:
            raise RuntimeError("Box not ready. Call open() and scan() first.")
        return self.box.configure_transparent(self.config)

    def connect(self) -> int:
        self.open()
        count = self.scan()
        self.configure_transparent()
        self.map_process_data()
        self.start_op()
        return count

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
            self.master.read_state()
            raise RuntimeError(
                "Failed to reach SAFE-OP. Current slave states: "
                + ", ".join(f"{idx}:{state_name(slave.state)}" for idx, slave in enumerate(self.master.slaves))
            )

        self.master.state = pysoem.OP_STATE
        self.master.write_state()
        for _ in range(50):
            self.cycle()
            if self.master.state_check(pysoem.OP_STATE, 5_000) == pysoem.OP_STATE:
                return

        self.master.read_state()
        raise RuntimeError(
            "Failed to reach OP. Current slave states: "
            + ", ".join(f"{idx}:{state_name(slave.state)}" for idx, slave in enumerate(self.master.slaves))
        )

    def cycle(self) -> int:
        self.master.send_processdata()
        wkc = self.master.receive_processdata(5_000)
        time.sleep(self.config.cycle_time_s)
        return wkc

    def sample_position_21bit(self, valid: int | None = None) -> ThirdEncoderSample:
        if self.box is None:
            raise RuntimeError("Box not ready. Call open() and scan() first.")
        tx_valid = self._tx_valid if valid is None else int(valid) & 0xFF
        if valid is None:
            self._tx_valid = (self._tx_valid + 1) & 0xFF
        frame = self.box.write_transparent(bytes([self.config.ecoder_request]), tx_valid)
        wkc = self.cycle()
        rx_frame = self.box.read_transparent()
        if rx_frame.size == 0:
            raise RuntimeError("No RS485 response yet.")
        response = parse_ecoder_id0_frame(rx_frame.data, request=self.config.ecoder_request)
        sample_index = self._sample_index
        self._sample_index += 1
        return ThirdEncoderSample(
            cycle_index=sample_index,
            tx_valid=frame.valid,
            wkc=wkc,
            request=self.config.ecoder_request,
            response=response,
        )

    def read_position_21bit(self, valid: int | None = None) -> int:
        return self.sample_position_21bit(valid=valid).response.position_21bit

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


__all__ = [
    "BOX_BAUD_SELECTOR_CUSTOM",
    "BOX_FRAME_8N1",
    "BOX_INTERFACE_RS485",
    "BOX_MODE_TRANSPARENT",
    "ECODER_ID0_REQUEST",
    "ECoderId0Response",
    "RS485Box",
    "ThirdEncoderRuntimeConfig",
    "ThirdEncoderSample",
    "ThirdEncoderSensor",
    "TransparentFrame",
    "parse_ecoder_id0_frame",
]
