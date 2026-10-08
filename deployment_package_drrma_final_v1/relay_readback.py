"""Read relay coil 0 over Modbus RTU without changing its state."""

from __future__ import annotations

import argparse

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


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--port",
        default="/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0",
    )
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--slave", type=int, default=1)
    args = parser.parse_args()

    request = add_crc(bytes((args.slave, 0x01, 0x00, 0x00, 0x00, 0x01)))
    with serial.Serial(
        port=args.port,
        baudrate=args.baudrate,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=0.5,
    ) as stream:
        stream.reset_input_buffer()
        stream.write(request)
        stream.flush()
        response = stream.read(6)

    print(f"request={request.hex(' ')}")
    print(f"response={response.hex(' ')}")
    if len(response) != 6:
        raise RuntimeError(f"Expected a 6-byte response, received {len(response)} bytes")
    if modbus_crc16(response[:-2]) != int.from_bytes(response[-2:], "little"):
        raise RuntimeError("Relay response CRC mismatch")
    if response[0] != args.slave or response[1] != 0x01 or response[2] != 0x01:
        raise RuntimeError(f"Unexpected relay response: {response.hex(' ')}")

    print(f"coil_0={'ON' if response[3] & 0x01 else 'OFF'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
