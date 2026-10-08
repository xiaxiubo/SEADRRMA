"""Benchmark the DR-RMA final_v1 ONNX graph with one CPU thread."""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import numpy as np
import onnxruntime as ort


HISTORY_LENGTH = 100
OBS_DIM = 9
OUTPUT_NAMES = ["action", "j_hat", "j_short", "j_long", "jump_gate", "confidence"]


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--onnx-dir", default="onnx")
    parser.add_argument("--threads", type=int, default=1)
    parser.add_argument("--warmup", type=int, default=500)
    parser.add_argument("--steps", type=int, default=5000)
    parser.add_argument("--seed", type=int, default=20260715)
    parser.add_argument("--deadline-ms", type=float, default=5.0)
    parser.add_argument(
        "--outputs",
        choices=("action", "diagnostics"),
        default="diagnostics",
        help="Request only the control action or all scalar diagnostic outputs.",
    )
    parser.add_argument("--output", default="latency_results_cpu_1thread.json")
    args = parser.parse_args()

    onnx_dir = Path(args.onnx_dir).resolve()
    model_path = onnx_dir / "drrma_control_only.onnx"
    options = ort.SessionOptions()
    options.intra_op_num_threads = args.threads
    options.inter_op_num_threads = 1
    options.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
    options.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
    session = ort.InferenceSession(
        str(model_path),
        sess_options=options,
        providers=["CPUExecutionProvider"],
    )

    rng = np.random.default_rng(args.seed)
    observations = rng.normal(
        0.0,
        np.asarray([0.1, 0.5, 0.1, 0.5, 0.001, 0.2, 0.3, 0.7, 0.25]),
        size=(args.warmup + args.steps, OBS_DIM),
    ).astype(np.float32)
    observations[:, 8] = np.clip(observations[:, 8], -1.0, 1.0)
    history = np.zeros((1, HISTORY_LENGTH, OBS_DIM), dtype=np.float32)
    samples_ms: list[float] = []
    requested_outputs = ["action"] if args.outputs == "action" else OUTPUT_NAMES

    for index, observation in enumerate(observations):
        history[:, :-1, :] = history[:, 1:, :]
        history[:, -1, :] = observation
        base_obs = observation.reshape(1, OBS_DIM)
        start = time.perf_counter_ns()
        outputs = session.run(
            requested_outputs,
            {"history": history, "base_obs": base_obs},
        )
        end = time.perf_counter_ns()
        if not all(np.isfinite(output).all() for output in outputs):
            raise RuntimeError(f"Non-finite output at step {index}")
        if index >= args.warmup:
            samples_ms.append((end - start) / 1e6)

    values = np.asarray(samples_ms, dtype=np.float64)
    deadline_misses = values > args.deadline_ms
    summary = {
        "mode": f"onnxruntime_final_v1_{args.outputs}",
        "model": str(model_path),
        "onnxruntime_version": ort.__version__,
        "providers": session.get_providers(),
        "threads": args.threads,
        "requested_outputs": requested_outputs,
        "steps": args.steps,
        "warmup": args.warmup,
        "mean_ms": float(values.mean()),
        "median_ms": float(np.percentile(values, 50)),
        "p95_ms": float(np.percentile(values, 95)),
        "p99_ms": float(np.percentile(values, 99)),
        "max_ms": float(values.max()),
        "deadline_ms": args.deadline_ms,
        "deadline_miss_count": int(deadline_misses.sum()),
        "deadline_miss_rate": float(deadline_misses.mean()),
        "estimated_max_hz_from_mean": float(1000.0 / values.mean()),
    }
    output_path = onnx_dir / args.output
    output_path.write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(json.dumps(summary, indent=2))
    print(f"Saved: {output_path}")


if __name__ == "__main__":
    main()
