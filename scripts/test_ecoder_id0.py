#!/usr/bin/env python3
from __future__ import annotations

import argparse

from ecat_testlib import (
    EtherCATSystem,
    ECoderId0Response,
    RuntimeConfig,
    log,
    parse_ecoder_id0_frame,
    print_exception_and_exit,
)


def main() -> int:
    parser = argparse.ArgumentParser(description="Read eCoder35 Data ID 0 (0x02) through the transparent PDO.")
    parser.add_argument("--ifname", default=RuntimeConfig.ifname)
    parser.add_argument("--attempts", type=int, default=200)
    parser.add_argument("--required-valid", type=int, default=200)
    args = parser.parse_args()

    config = RuntimeConfig(ifname=args.ifname)
    system = None
    try:
        system = EtherCATSystem(config)
        system.open()
        system.scan()
        box = system.box
        assert box is not None

        system.map_process_data()
        system.start_op()

        valid_frames: list[ECoderId0Response] = []
        for attempt in range(args.attempts):
            tx_valid = attempt & 0xFF
            tx_frame = box.write_transparent(bytes([config.ecoder_request]), tx_valid)
            wkc = system.cycle()
            rx_frame = box.read_transparent()

            if rx_frame.size == 0:
                log(
                    "attempt={attempt} tx_valid={txv} wkc={wkc} tx_size={txs} no RS485 response yet".format(
                        attempt=attempt,
                        txv=tx_frame.valid,
                        wkc=wkc,
                        txs=tx_frame.size,
                    )
                )
                continue

            try:
                parsed = parse_ecoder_id0_frame(rx_frame.data, request=config.ecoder_request)
            except Exception as exc:
                log(
                    "attempt={attempt} tx_valid={txv} wkc={wkc} rx_valid={rxv} rx_size={rxs} "
                    "raw={raw} parse_error={err}".format(
                        attempt=attempt,
                        txv=tx_frame.valid,
                        wkc=wkc,
                        rxv=rx_frame.valid,
                        rxs=rx_frame.size,
                        raw=rx_frame.data.hex(" "),
                        err=exc,
                    )
                )
                continue

            log(
                "attempt={attempt} tx_valid={txv} wkc={wkc} raw={raw} pos21={pos} "
                "crc_ok={crc_ok} encoder_error={enc_err} comm_alarm={comm_alarm}".format(
                    attempt=attempt,
                    txv=tx_frame.valid,
                    wkc=wkc,
                    raw=rx_frame.data[:6].hex(" "),
                    pos=parsed.position_21bit,
                    crc_ok=parsed.crc_ok,
                    enc_err=parsed.encoder_error,
                    comm_alarm=parsed.communication_alarm,
                )
            )

            if parsed.crc_ok and not parsed.encoder_error and not parsed.communication_alarm:
                valid_frames.append(parsed)
                if len(valid_frames) >= args.required_valid:
                    break

        if len(valid_frames) < args.required_valid:
            raise RuntimeError(
                f"Expected {args.required_valid} valid eCoder ID0 responses, only got {len(valid_frames)}."
            )

        log(
            "Collected {count} valid eCoder ID0 frames. Last position_21bit={pos}".format(
                count=len(valid_frames),
                pos=valid_frames[-1].position_21bit,
            )
        )
        return 0
    except Exception as exc:
        print_exception_and_exit("test_ecoder_id0 failed", exc)
    finally:
        if system is not None:
            system.close()


if __name__ == "__main__":
    raise SystemExit(main())
