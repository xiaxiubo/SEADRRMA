#!/usr/bin/env python3
"""Read the EtherCAT drive's standard CiA 402 error objects without enabling it."""

from __future__ import annotations

import argparse

import pysoem


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--ifname", default="eno1")
    parser.add_argument("--slave-index", type=int, default=1)
    args = parser.parse_args()

    master = pysoem.Master()
    try:
        master.open(args.ifname)
        slave_count = master.config_init()
        if slave_count <= args.slave_index:
            raise RuntimeError(
                f"Expected slave index {args.slave_index}, found {slave_count} slave(s)"
            )

        slave = master.slaves[args.slave_index]
        error_code = int.from_bytes(slave.sdo_read(0x603F, 0), "little")
        error_register = int.from_bytes(slave.sdo_read(0x1001, 0), "little")
        print(f"slave={args.slave_index} name={slave.name!r}")
        print(f"error_code_0x603F=0x{error_code:04X}")
        print(f"error_register_0x1001=0x{error_register:02X}")
    finally:
        master.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
