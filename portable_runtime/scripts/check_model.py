from __future__ import annotations

import argparse
import hashlib
from pathlib import Path

import numpy as np

from seadrrma_joint.model import TraceOnnxController


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", required=True, type=Path)
    parser.add_argument("--expected-sha256", default=None)
    parser.add_argument("--steps", type=int, default=100)
    args = parser.parse_args()

    actual_hash = sha256(args.model)
    print(f"sha256={actual_hash}")
    if args.expected_sha256 and actual_hash.lower() != args.expected_sha256.lower():
        raise SystemExit("model checksum mismatch")

    controller = TraceOnnxController(args.model)
    rng = np.random.default_rng(20261008)
    actions = []
    for index in range(args.steps):
        output = controller.step(
            rng.normal(0.0, 0.05, size=9).astype(np.float32),
            startup=index < 10,
        )
        actions.append(output.action)
    actions_array = np.asarray(actions)
    if not np.all(np.isfinite(actions_array)):
        raise SystemExit("model produced non-finite action")
    if np.max(np.abs(actions_array)) > 1.0001:
        raise SystemExit("model action exceeded normalized bounds")
    print(f"steps={args.steps} max_abs_action={np.max(np.abs(actions_array)):.6f}")


if __name__ == "__main__":
    main()

