#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path

if __package__ in {None, ""}:
    import sys

    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from communication.ecoder35_transport import ThirdEncoderRuntimeConfig, ThirdEncoderSensor
from controllers.spring_sensor import SpringSensorConfig, SpringSensorEstimator


def print_slave_inventory(sensor: ThirdEncoderSensor) -> None:
    print(f"EtherCAT network on {sensor.config.ifname}: expected_wkc={sensor.expected_wkc}", flush=True)
    for row in sensor.describe_slaves():
        print(f"  {row}", flush=True)


def print_encoder_sample(prefix: str, raw_counts: int, delta_theta_rad: float, spring_torque_nm: float) -> None:
    print(
        (
            f"{prefix} pos21={raw_counts} "
            f"delta_theta={delta_theta_rad:.6f}rad spring_torque={spring_torque_nm:.3f}Nm "
        ),
        flush=True,
    )


def warmup_sensor(sensor: ThirdEncoderSensor, warmup_cycles: int = 100, required_valid: int = 5) -> None:
    valid_count = 0
    for attempt in range(warmup_cycles):
        try:
            sample = sensor.sample_position_21bit(valid=attempt & 0xFF)
            print(
                f"[warmup] attempt={attempt} ok wkc={sample.wkc} pos21={sample.response.position_21bit} "
                f"crc_ok={int(sample.response.crc_ok)} enc_err={int(sample.response.encoder_error)} "
                f"comm_alarm={int(sample.response.communication_alarm)}",
                flush=True,
            )
            valid_count += 1
            if valid_count >= required_valid:
                return
        except Exception as exc:
            print(f"[warmup] attempt={attempt} failed: {exc}", flush=True)

    raise RuntimeError(
        f"Third encoder warm-up failed: only got {valid_count} valid frames in {warmup_cycles} attempts"
    )


def collect_zero_counts(sensor: ThirdEncoderSensor, sample_count: int) -> int:
    zero_samples: list[int] = []
    attempt_limit = max(sample_count * 20, 100)
    last_error: Exception | None = None
    for attempt in range(attempt_limit):
        if len(zero_samples) >= sample_count:
            break
        try:
            sample = sensor.sample_position_21bit(valid=attempt & 0xFF)
            raw_counts = sample.response.position_21bit
            print(
                f"[zero] attempt={attempt} ok wkc={sample.wkc} pos21={raw_counts} "
                f"crc_ok={int(sample.response.crc_ok)} enc_err={int(sample.response.encoder_error)} "
                f"comm_alarm={int(sample.response.communication_alarm)}",
                flush=True,
            )
        except Exception as exc:
            last_error = exc
            print(f"[zero] attempt={attempt} failed: {exc}", flush=True)
            continue
        zero_samples.append(raw_counts)
    if len(zero_samples) < sample_count:
        raise RuntimeError(
            f"Failed to collect enough valid zero samples. got={len(zero_samples)} need={sample_count} "
            f"last_error={last_error}"
        )
    return round(sum(zero_samples) / len(zero_samples))


def main() -> int:
    parser = argparse.ArgumentParser(description="SEA PD-DOB entry using eCoder35 spring estimation.")
    parser.add_argument("--ifname", default="eno1")
    parser.add_argument("--cycle-time", type=float, default=0.010)
    parser.add_argument("--calibration-samples", type=int, default=32)
    parser.add_argument("--cycles", type=int, default=200)
    parser.add_argument("--print-every", type=int, default=1)
    parser.add_argument("--counts-per-rev", type=int, default=1 << 19)
    parser.add_argument("--sign", type=int, default=1)
    parser.add_argument("--spring-stiffness", type=float, default=3800.0)
    args = parser.parse_args()

    sensor_config = ThirdEncoderRuntimeConfig(
        ifname=args.ifname,
        cycle_time_s=args.cycle_time,
    )
    estimator = SpringSensorEstimator(
        SpringSensorConfig(
            counts_per_rev=args.counts_per_rev,
            zero_counts=0,
            sign=args.sign,
            spring_stiffness_nm_per_rad=args.spring_stiffness,
        )
    )
    sensor = ThirdEncoderSensor(sensor_config)

    try:
        print("Stage 0: third encoder EtherCAT connect", flush=True)
        count = sensor.connect()
        print(f"Connected on {args.ifname}. expected_wkc={sensor.expected_wkc}", flush=True)
        print(f"Found {count} EtherCAT slaves on {args.ifname}.", flush=True)
        print_slave_inventory(sensor)
        print("Third encoder channel reached OP.", flush=True)

        print("Stage 1: third encoder RS485 warmup", flush=True)
        print("Warming up third encoder RS485 channel...", flush=True)
        warmup_sensor(sensor, warmup_cycles=100, required_valid=5)

        print("Stage 2: zero calibration", flush=True)
        zero_counts = collect_zero_counts(sensor, args.calibration_samples)
        estimator.tare_zero(zero_counts)
        print(
            f"Calibration done: zero_counts={zero_counts} "
            f"counts_per_rev={args.counts_per_rev} spring_stiffness={args.spring_stiffness}",
            flush=True,
        )

        print("Stage 3: online estimation", flush=True)
        for cycle_index in range(args.cycles):
            raw_counts = sensor.read_position_21bit(valid=cycle_index & 0xFF)
            estimate = estimator.estimate(raw_counts)
            if cycle_index % max(args.print_every, 1) == 0:
                print_encoder_sample(
                    prefix=f"cycle={cycle_index}",
                    raw_counts=raw_counts,
                    delta_theta_rad=estimate.delta_theta_rad,
                    spring_torque_nm=estimate.spring_torque_nm,
                )

        print("PD-DOB third encoder estimation done.", flush=True)
        return 0
    finally:
        sensor.close()


if __name__ == "__main__":
    raise SystemExit(main())
