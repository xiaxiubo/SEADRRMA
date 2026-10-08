"""Set relay coil 0 over Modbus RTU and verify its electrical/mechanical state."""

from __future__ import annotations

import argparse
import time

import serial


def modbus_crc16(payload: bytes) -> int:
    crc = 0xFFFF
    for value in payload:
        crc ^= value
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc


def add_crc(payload: bytes) -> bytes:
    crc = modbus_crc16(payload)
    return payload + bytes((crc & 0xFF, (crc >> 8) & 0xFF))


def exchange(stream: serial.Serial, request: bytes, response_length: int) -> bytes:
    stream.reset_input_buffer()
    stream.write(request)
    stream.flush()
    response = stream.read(response_length)
    if len(response) != response_length:
        raise RuntimeError(
            f"Expected a {response_length}-byte response, received {len(response)} bytes"
        )
    if modbus_crc16(response[:-2]) != int.from_bytes(response[-2:], "little"):
        raise RuntimeError("Relay response CRC mismatch")
    return response


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "state",
        choices=("on", "off", "engage", "release"),
        help=(
            "Electrical coil state, or mechanical clutch state. On this rig, "
            "coil OFF engages the clutch and coil ON releases it."
        ),
    )
    parser.add_argument(
        "--port",
        default="/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0",
    )
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--slave", type=int, default=1)
    args = parser.parse_args()

    enabled = args.state in ("on", "release")
    expected_mechanical_state = "released" if enabled else "engaged"
    value = 0xFF00 if enabled else 0x0000
    write_request = add_crc(
        bytes(
            (
                args.slave,
                0x05,
                0x00,
                0x00,
                (value >> 8) & 0xFF,
                value & 0xFF,
            )
        )
    )
    read_request = add_crc(bytes((args.slave, 0x01, 0x00, 0x00, 0x00, 0x01)))

    with serial.Serial(
        port=args.port,
        baudrate=args.baudrate,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=0.5,
    ) as stream:
        write_response = exchange(stream, write_request, 8)
        if write_response != write_request:
            raise RuntimeError(
                "Relay write response does not echo the request: "
                f"{write_response.hex(' ')}"
            )
        time.sleep(0.1)
        read_response = exchange(stream, read_request, 6)

    if (
        read_response[0] != args.slave
        or read_response[1] != 0x01
        or read_response[2] != 0x01
    ):
        raise RuntimeError(f"Unexpected relay read response: {read_response.hex(' ')}")
    actual_enabled = bool(read_response[3] & 0x01)
    print(f"write_request={write_request.hex(' ')}")
    print(f"write_response={write_response.hex(' ')}")
    print(f"read_response={read_response.hex(' ')}")
    print(f"coil_0={'ON' if actual_enabled else 'OFF'}")
    print(
        "mechanical_clutch="
        f"{'RELEASED' if actual_enabled else 'ENGAGED'}"
    )
    if actual_enabled != enabled:
        raise RuntimeError(
            f"Relay verification failed: requested {args.state}, "
            f"read back {'on' if actual_enabled else 'off'}"
        )
    if ("released" if actual_enabled else "engaged") != expected_mechanical_state:
        raise RuntimeError("Internal clutch-state mapping error")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
