#!/usr/bin/env python3
from __future__ import annotations

import argparse

from ecat_testlib import EtherCATSystem, RuntimeConfig, log, print_exception_and_exit


def main() -> int:
    parser = argparse.ArgumentParser(description="Validate RS485/EtherCAT box transparent PDO.")
    parser.add_argument("--ifname", default=RuntimeConfig.ifname)
    parser.add_argument(
        "--payload",
        default="01 01 02",
        help="Hex byte string to push through the transparent PDO, e.g. '02' or '55 aa 02'.",
    )
    parser.add_argument("--cycles", type=int, default=50)
    args = parser.parse_args()

    config = RuntimeConfig(ifname=args.ifname)
    payload = bytes.fromhex(args.payload)
    system = None
    try:
        system = EtherCATSystem(config)
        system.open()
        system.scan()
        box = system.box
        assert box is not None

        system.map_process_data()
        system.start_op()

        tx_valid = 0
        seen_nonempty_rx = False
        for cycle in range(args.cycles):
            frame = box.write_transparent(payload, tx_valid)
            wkc = system.cycle()
            rx = box.read_transparent()
            log(
                "cycle={cycle} wkc={wkc} tx_valid={txv} tx_size={txs} "
                "rx_valid={rxv} rx_size={rxs} rx_data={rxd}".format(
                    cycle=cycle,
                    wkc=wkc,
                    txv=frame.valid,
                    txs=frame.size,
                    rxv=rx.valid,
                    rxs=rx.size,
                    rxd=rx.data.hex(" "),
                )
            )
            if rx.size > 0:
                seen_nonempty_rx = True
            tx_valid ^= 1

        if seen_nonempty_rx:
            log("Transparent PDO observed non-empty receive data.")
        else:
            log("Transparent PDO never produced non-empty receive data. This usually means no RS485-side response yet.")
        return 0
    except Exception as exc:
        print_exception_and_exit("test_box_passthrough failed", exc)
    finally:
        if system is not None:
            system.close()


if __name__ == "__main__":
    raise SystemExit(main())
