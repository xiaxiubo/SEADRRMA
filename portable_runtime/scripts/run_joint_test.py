from __future__ import annotations

import argparse
from datetime import datetime
from pathlib import Path

from seadrrma_joint.backend import load_backend
from seadrrma_joint.config import load_profile
from seadrrma_joint.logger import CsvLogger
from seadrrma_joint.model import TraceOnnxController
from seadrrma_joint.runner import run_experiment


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    parser.add_argument("--model", required=True)
    parser.add_argument("--backend", default="mock")
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--threads", type=int, default=2)
    parser.add_argument("--log", default=None)
    parser.add_argument(
        "--enable-output",
        action="store_true",
        help="Permit nonzero torque commands. Omit for dry-run commissioning.",
    )
    args = parser.parse_args()

    profile = load_profile(args.config)
    backend = load_backend(args.backend, profile)
    controller = TraceOnnxController(args.model, profile.history_length, args.threads)
    log_path = args.log or (
        Path("logs") / f"joint_test_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    )
    print(f"backend={args.backend} output_enabled={args.enable_output} log={log_path}")
    with CsvLogger(log_path) as logger:
        run_experiment(
            backend,
            controller,
            profile,
            args.duration,
            logger,
            args.enable_output,
        )
    print("experiment complete; drive disabled")


if __name__ == "__main__":
    main()

