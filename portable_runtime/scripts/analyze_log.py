from __future__ import annotations

import argparse
import csv
from pathlib import Path

import numpy as np


def column(rows: list[dict[str, str]], name: str) -> np.ndarray:
    return np.asarray([float(row[name]) for row in rows], dtype=np.float64)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("log", type=Path)
    args = parser.parse_args()
    with args.log.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise SystemExit("empty log")

    error = column(rows, "target_position_rad") - column(rows, "load_position_rad")
    compute_ms = column(rows, "compute_ms")
    period_ms = column(rows, "period_ms")
    torque = column(rows, "command_torque_nm")
    misses = column(rows, "deadline_missed")
    print(f"samples={len(rows)}")
    print(f"tracking_rmse_rad={np.sqrt(np.mean(error**2)):.6f}")
    print(f"tracking_max_abs_rad={np.max(np.abs(error)):.6f}")
    print(f"torque_rms_nm={np.sqrt(np.mean(torque**2)):.6f}")
    print(f"torque_max_abs_nm={np.max(np.abs(torque)):.6f}")
    print(f"compute_mean_ms={np.mean(compute_ms):.6f}")
    print(f"compute_p99_ms={np.percentile(compute_ms, 99):.6f}")
    print(f"compute_max_ms={np.max(compute_ms):.6f}")
    print(f"period_mean_ms={np.mean(period_ms):.6f}")
    print(f"period_p99_ms={np.percentile(period_ms, 99):.6f}")
    print(f"period_max_ms={np.max(period_ms):.6f}")
    print(f"deadline_miss_rate={np.mean(misses):.6f}")


if __name__ == "__main__":
    main()
