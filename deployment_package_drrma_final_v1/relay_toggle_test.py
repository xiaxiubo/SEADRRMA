"""Toggle relay coil 0 once per second and verify every requested state."""

from __future__ import annotations

import argparse
import csv
import time
from pathlib import Path

import serial

from relay_control import add_crc, exchange


def write_and_read(
    stream: serial.Serial,
    *,
    slave: int,
    enabled: bool,
) -> tuple[bytes, bytes, bool]:
    value = 0xFF00 if enabled else 0x0000
    write_request = add_crc(
        bytes(
            (
                slave,
                0x05,
                0x00,
                0x00,
                (value >> 8) & 0xFF,
                value & 0xFF,
            )
        )
    )
    write_response = exchange(stream, write_request, 8)
    if write_response != write_request:
        raise RuntimeError(
            "Relay write response does not echo the request: "
            f"{write_response.hex(' ')}"
        )

    read_request = add_crc(bytes((slave, 0x01, 0x00, 0x00, 0x00, 0x01)))
    time.sleep(0.05)
    read_response = exchange(stream, read_request, 6)
    if (
        read_response[0] != slave
        or read_response[1] != 0x01
        or read_response[2] != 0x01
    ):
        raise RuntimeError(f"Unexpected relay read response: {read_response.hex(' ')}")
    actual_enabled = bool(read_response[3] & 0x01)
    return write_response, read_response, actual_enabled


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--port",
        default="/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0",
    )
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--slave", type=int, default=1)
    parser.add_argument("--duration", type=int, default=10)
    parser.add_argument("--interval", type=float, default=1.0)
    parser.add_argument("--csv-path", type=Path)
    args = parser.parse_args()
    if args.duration <= 0:
        raise ValueError("--duration must be positive")
    if args.interval <= 0.0:
        raise ValueError("--interval must be positive")

    rows: list[tuple[int, float, str, str, int]] = []
    with serial.Serial(
        port=args.port,
        baudrate=args.baudrate,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=0.5,
    ) as stream:
        try:
            _, _, actual_enabled = write_and_read(
                stream,
                slave=args.slave,
                enabled=False,
            )
            if actual_enabled:
                raise RuntimeError("Failed to precondition relay to OFF")
            print("precondition requested=OFF readback=OFF", flush=True)
            time.sleep(0.5)

            start = time.monotonic()
            for index in range(args.duration):
                deadline = start + index * args.interval
                delay = deadline - time.monotonic()
                if delay > 0.0:
                    time.sleep(delay)

                requested = index % 2 == 0
                _, _, actual = write_and_read(
                    stream,
                    slave=args.slave,
                    enabled=requested,
                )
                elapsed = time.monotonic() - start
                passed = actual == requested
                rows.append(
                    (
                        index,
                        elapsed,
                        "ON" if requested else "OFF",
                        "ON" if actual else "OFF",
                        int(passed),
                    )
                )
                print(
                    f"toggle={index:02d} t={elapsed:.3f}s "
                    f"requested={'ON' if requested else 'OFF'} "
                    f"readback={'ON' if actual else 'OFF'} pass={int(passed)}",
                    flush=True,
                )
                if not passed:
                    raise RuntimeError(f"Relay verification failed at toggle {index}")

            final_deadline = start + args.duration * args.interval
            delay = final_deadline - time.monotonic()
            if delay > 0.0:
                time.sleep(delay)
        finally:
            try:
                _, _, final_enabled = write_and_read(
                    stream,
                    slave=args.slave,
                    enabled=False,
                )
                print(
                    f"final requested=OFF readback={'ON' if final_enabled else 'OFF'}",
                    flush=True,
                )
            except Exception as exc:
                print(f"final OFF command failed: {exc}", flush=True)

    if args.csv_path is not None:
        args.csv_path.parent.mkdir(parents=True, exist_ok=True)
        with args.csv_path.open("w", newline="", encoding="utf-8") as output:
            writer = csv.writer(output)
            writer.writerow(
                ("toggle_index", "elapsed_s", "requested", "readback", "passed")
            )
            writer.writerows(rows)
        print(f"CSV saved to: {args.csv_path}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
