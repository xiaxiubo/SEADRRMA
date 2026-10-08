from __future__ import annotations

import csv
from pathlib import Path
from typing import Any

FIELDS = (
    "time_s",
    "compute_ms",
    "period_ms",
    "schedule_lateness_ms",
    "deadline_missed",
    "output_enabled",
    "target_position_rad",
    "target_velocity_rad_s",
    "motor_position_rad",
    "load_position_rad",
    "motor_velocity_rad_s",
    "load_velocity_rad_s",
    "spring_deflection_rad",
    "spring_velocity_rad_s",
    "normalized_action",
    "command_torque_nm",
    "inertia_hat",
    "inertia_raw",
    "inertia_short",
    "inertia_long",
    "jump_gate",
    "confidence",
)


class CsvLogger:
    def __init__(self, path: str | Path) -> None:
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.stream = self.path.open("w", newline="", encoding="utf-8")
        self.writer = csv.DictWriter(self.stream, fieldnames=FIELDS)
        self.writer.writeheader()

    def write(self, row: dict[str, Any]) -> None:
        self.writer.writerow(row)

    def close(self) -> None:
        self.stream.flush()
        self.stream.close()

    def __enter__(self) -> CsvLogger:
        return self

    def __exit__(self, *_: object) -> None:
        self.close()
