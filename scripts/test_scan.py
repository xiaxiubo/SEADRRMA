#!/usr/bin/env python3
from __future__ import annotations

import argparse

from ecat_testlib import (
    EtherCATSystem,
    RuntimeConfig,
    log,
    print_exception_and_exit,
    state_name,
    summarize_wkc,
)


def main() -> int:
    parser = argparse.ArgumentParser(description="Scan EtherCAT slaves and drive the network to OP.")
    parser.add_argument("--ifname", default=RuntimeConfig.ifname)
    parser.add_argument("--cycles", type=int, default=20, help="PDO cycles to sample after reaching OP.")
    args = parser.parse_args()

    config = RuntimeConfig(ifname=args.ifname)
    system = None
    try:
        system = EtherCATSystem(config)
        system.open()
        count = system.scan()
        log(f"Found {count} EtherCAT slaves on {config.ifname}.")
        system.describe_slaves()

        roles = system.roles
        assert roles is not None
        log(f"Identified roles: box_index={roles.box_index}, drive_index={roles.drive_index}")

        io_map_size = system.map_process_data()
        log(f"Configured process data mapping, io_map_size={io_map_size} bytes")

        system.start_op()
        log(f"Master reached {state_name(system.state)}")

        wkc_samples = [system.cycle() for _ in range(args.cycles)]
        expected = system.expected_wkc
        log(f"Expected WKC: {expected}")
        log(f"Observed WKC over {args.cycles} cycles: {summarize_wkc(wkc_samples)}")

        system.read_state()
        for idx, slave in enumerate(system.slaves):
            log(f"Slave {idx} final state: {state_name(slave.state)}")
        return 0
    except Exception as exc:
        print_exception_and_exit("test_scan failed", exc)
    finally:
        if system is not None:
            system.close()


if __name__ == "__main__":
    raise SystemExit(main())
