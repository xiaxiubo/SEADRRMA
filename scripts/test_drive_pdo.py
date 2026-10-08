#!/usr/bin/env python3
from __future__ import annotations

import argparse

from ecat_testlib import (
    EtherCATSystem,
    RuntimeConfig,
    log,
    print_exception_and_exit,
    summarize_wkc,
)


PREHEAT_CYCLES = 100
PREHEAT_STABLE_WKC = 6
DEFAULT_IFNAME = "eno1"
DEFAULT_CYCLES = 200
DEFAULT_PRINT_EVERY = 5


def main() -> int:
    parser = argparse.ArgumentParser(description="Pure read SEA drive PDOs without sending controlwords.")
    parser.add_argument("--ifname", default=DEFAULT_IFNAME)
    parser.add_argument("--cycles", type=int, default=DEFAULT_CYCLES)
    parser.add_argument("--print-every", type=int, default=DEFAULT_PRINT_EVERY)
    args = parser.parse_args()

    config = RuntimeConfig(ifname=args.ifname)
    system = None
    try:
        log(
            "test_drive_pdo defaults: ifname={ifname} cycles={cycles} print_every={print_every} preheat_cycles={preheat}".format(
                ifname=args.ifname,
                cycles=args.cycles,
                print_every=args.print_every,
                preheat=PREHEAT_CYCLES,
            )
        )
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
        log("Master reached OP")

        # 先等总线和从站稳定下来，避免把冷启动阶段的 0 值写进正式采样。
        preheat_last_status = None
        preheat_last_wkc = None
        for preheat_cycle in range(PREHEAT_CYCLES):
            preheat_last_wkc = system.cycle()
            preheat_last_status = system.drive.read_status() if system.drive is not None else None
            if (
                preheat_last_status is not None
                and preheat_last_wkc == PREHEAT_STABLE_WKC
                and preheat_last_status.statusword != 0
            ):
                log(
                    "Preheat ready at cycle={cycle}: wkc={wkc} statusword=0x{sw:04X} mode={mode}".format(
                        cycle=preheat_cycle,
                        wkc=preheat_last_wkc,
                        sw=preheat_last_status.statusword,
                        mode=preheat_last_status.mode_display,
                    )
                )
                break
        else:
            log(
                "Preheat finished without stable PDO. "
                "Continuing with whatever state is available."
            )

        wkc_samples: list[int] = []
        last_status = None
        for cycle in range(args.cycles):
            wkc = system.cycle()
            wkc_samples.append(wkc)
            last_status = system.drive.read_status() if system.drive is not None else None
            if last_status is None:
                raise RuntimeError("Drive is not ready after OP.")
            if cycle % max(args.print_every, 1) == 0 or cycle == args.cycles - 1:
                log(
                    "cycle={cycle} wkc={wkc} statusword=0x{sw:04X} mode={mode} "
                    "pos={pos} vel={vel} torque={torque} enc1={enc1} enc2={enc2}".format(
                        cycle=cycle,
                        wkc=wkc,
                        sw=last_status.statusword,
                        mode=last_status.mode_display,
                        pos=last_status.position,
                        vel=last_status.velocity,
                        torque=last_status.torque,
                        enc1=last_status.encoder1,
                        enc2=last_status.encoder2,
                    )
                )

        log(f"WKC summary: {summarize_wkc(wkc_samples)}")
        if last_status is not None:
            log(
                "Final drive sample: statusword=0x{sw:04X}, mode={mode}, position={pos}, velocity={vel}, torque={torque}".format(
                    sw=last_status.statusword,
                    mode=last_status.mode_display,
                    pos=last_status.position,
                    vel=last_status.velocity,
                    torque=last_status.torque,
                )
            )
        return 0
    except Exception as exc:
        print_exception_and_exit("test_drive_pdo failed", exc)
    finally:
        if system is not None:
            system.close()


if __name__ == "__main__":
    raise SystemExit(main())
