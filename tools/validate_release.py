#!/usr/bin/env python3
from __future__ import annotations

import ast
import hashlib
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
REQUIRED_FILES = (
    "communication/sea_motor_comm.py",
    "communication/ecoder35_transport.py",
    "control_flow/TRACE_hardware_final_v1_main.py",
    "control_flow/DRRMA_final_v1_main.py",
    "control_flow/SEA_friction_identification.py",
    "control_flow/SEA_spring_calibration_locked.py",
    "controllers/trace_onnx_controller.py",
    "deployment_package_trace_hardware_final_v1_20260726/calibration/hardware_inertia_calibrator_v1.json",
)

OPTIONAL_MODELS = (
    "deployment_package_drrma_final_v1/onnx/drrma_control_only.onnx",
    "deployment_package_trace_hardware_final_v1_20260726/onnx/trace_hardware_control.onnx",
)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def main() -> int:
    missing = [name for name in REQUIRED_FILES if not (ROOT / name).is_file()]
    if missing:
        raise SystemExit("Missing required files:\n" + "\n".join(missing))

    python_files = sorted(ROOT.glob("communication/*.py"))
    python_files += sorted(ROOT.glob("control_flow/*.py"))
    python_files += sorted(ROOT.glob("controllers/**/*.py"))
    python_files += sorted(ROOT.glob("scripts/*.py"))
    python_files += sorted(ROOT.glob("analysis/*.py"))
    for path in python_files:
        ast.parse(path.read_text(encoding="utf-8"), filename=str(path))

    calibration = json.loads(
        (ROOT / REQUIRED_FILES[-1]).read_text(encoding="utf-8")
    )
    if not isinstance(calibration, dict):
        raise SystemExit("Calibration JSON must contain an object")

    report = {
        "python_files_parsed": len(python_files),
        "required_files": len(REQUIRED_FILES),
        "optional_models": {
            name: {
                "available": True,
                "bytes": (ROOT / name).stat().st_size,
                "sha256": sha256(ROOT / name),
            }
            if (ROOT / name).is_file()
            else {"available": False}
            for name in OPTIONAL_MODELS
        },
        "hardware_io_exercised": False,
    }
    print(json.dumps(report, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
