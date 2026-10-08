#!/usr/bin/env python3
from __future__ import annotations

import argparse
import struct

import project_bootstrap  # noqa: F401

from communication.sea_motor_comm import SEARealtimeComm


# 这里是最小化的驱动器故障诊断脚本。
# 作用：
# - 连接 EtherCAT
# - 读取驱动器标准 CiA402 错误码 0x603F
# - 读取错误寄存器 0x1001
# - 读取错误历史 0x1003
# 这样设备亮红灯时，可以先快速确认是不是标准故障码或历史故障残留。
DEFAULT_IFNAME = "eno1"

ERROR_CODE_INDEX = 0x603F
ERROR_REGISTER_INDEX = 0x1001
ERROR_HISTORY_INDEX = 0x1003


def read_u8(comm: SEARealtimeComm, index: int, subindex: int) -> int:
    return struct.unpack("<B", comm.read_sdo_raw(index, subindex))[0]


def read_u16(comm: SEARealtimeComm, index: int, subindex: int) -> int:
    return struct.unpack("<H", comm.read_sdo_raw(index, subindex))[0]


def read_u32(comm: SEARealtimeComm, index: int, subindex: int) -> int:
    return struct.unpack("<I", comm.read_sdo_raw(index, subindex))[0]


def print_hex_or_none(label: str, value: int | None, width: int) -> None:
    if value is None:
        print(f"{label}: <unavailable>", flush=True)
        return
    print(f"{label}: 0x{value:0{width}X} ({value})", flush=True)


def main() -> int:
    parser = argparse.ArgumentParser(description="Read CiA402 fault information from one SOMANET drive")
    parser.add_argument("--ifname", default=DEFAULT_IFNAME)
    parser.add_argument("--drive-index", type=int, default=0)
    args = parser.parse_args()

    # 只做 SDO 诊断，不下发控制字和力矩。
    comm = SEARealtimeComm(ifname=args.ifname)
    try:
        comm.connect()
        if comm.roles is None:
            raise RuntimeError("EtherCAT slave roles are unavailable")
        if not 0 <= args.drive_index < len(comm.master.slaves):
            raise ValueError(
                f"--drive-index {args.drive_index} is outside [0, {len(comm.master.slaves) - 1}]"
            )
        slave = comm.master.slaves[args.drive_index]
        name = (getattr(slave, "name", "") or "").lower()
        if "somanet" not in name and "circulo" not in name:
            raise ValueError(f"slave[{args.drive_index}] is not a SOMANET/Circulo drive: {name!r}")
        comm.drive = slave
        comm.roles = type(comm.roles)(
            box_index=comm.roles.box_index,
            drive_index=args.drive_index,
        )
        print(
            f"Connected on {args.ifname}. Reading fault information from "
            f"slave[{args.drive_index}] {getattr(slave, 'name', '')!r}...",
            flush=True,
        )

        # 标准 CiA402 故障码。
        fault_code: int | None
        try:
            fault_code = read_u16(comm, ERROR_CODE_INDEX, 0)
        except Exception as exc:
            print(f"Failed to read 0x603F: {exc}", flush=True)
            fault_code = None
        print_hex_or_none("0x603F error code", fault_code, 4)

        # 标准错误寄存器。
        error_register: int | None
        try:
            error_register = read_u8(comm, ERROR_REGISTER_INDEX, 0)
        except Exception as exc:
            print(f"Failed to read 0x1001: {exc}", flush=True)
            error_register = None
        print_hex_or_none("0x1001 error register", error_register, 2)

        # 错误历史：0x1003:00 是条目数，后续子索引是历史错误码。
        try:
            history_count = read_u8(comm, ERROR_HISTORY_INDEX, 0)
            print(f"0x1003 history count: {history_count}", flush=True)
            if history_count > 0:
                for subindex in range(1, history_count + 1):
                    try:
                        history_value = read_u32(comm, ERROR_HISTORY_INDEX, subindex)
                        print_hex_or_none(f"0x1003:{subindex:02X}", history_value, 8)
                    except Exception as exc:
                        print(f"Failed to read 0x1003:{subindex:02X}: {exc}", flush=True)
        except Exception as exc:
            print(f"Failed to read 0x1003 history: {exc}", flush=True)

        print("Drive fault readout done.", flush=True)
        return 0
    finally:
        comm.close()


if __name__ == "__main__":
    raise SystemExit(main())
