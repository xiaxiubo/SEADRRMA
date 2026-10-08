from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import numpy as np
import onnxruntime as ort


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--onnx-dir", default="onnx")
    parser.add_argument("--threads", type=int, default=4)
    parser.add_argument("--warmup", type=int, default=300)
    parser.add_argument("--steps", type=int, default=3000)
    parser.add_argument("--deadline-ms", type=float, default=5.0)
    parser.add_argument("--seed", type=int, default=20260726)
    args = parser.parse_args()

    model_path = Path(args.onnx_dir).resolve() / "trace_hardware_control.onnx"
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
    scales = np.asarray(
        [0.1, 0.5, 0.1, 0.5, 0.001, 0.2, 0.3, 0.7, 0.25],
        dtype=np.float32,
    )
    observations = rng.normal(
        0.0, scales, size=(args.warmup + args.steps, 9)
    ).astype(np.float32)
    observations[:, 8] = np.clip(observations[:, 8], -1.0, 1.0)
    history = np.zeros((1, 100, 9), dtype=np.float32)
    startup = np.zeros((1, 1), dtype=np.float32)
    samples = []
    for index, observation in enumerate(observations):
        history[:, :-1] = history[:, 1:]
        history[:, -1] = observation
        start = time.perf_counter_ns()
        outputs = session.run(
            None,
            {
                "history": history,
                "base_obs": observation.reshape(1, 9),
                "startup_override": startup,
            },
        )
        elapsed = (time.perf_counter_ns() - start) / 1e6
        if not all(np.isfinite(output).all() for output in outputs):
            raise RuntimeError(f"Non-finite output at step {index}")
        if index >= args.warmup:
            samples.append(elapsed)
    values = np.asarray(samples, dtype=np.float64)
    result = {
        "model": str(model_path),
        "threads": args.threads,
        "steps": args.steps,
        "mean_ms": float(values.mean()),
        "p95_ms": float(np.percentile(values, 95)),
        "p99_ms": float(np.percentile(values, 99)),
        "max_ms": float(values.max()),
        "deadline_ms": args.deadline_ms,
        "deadline_miss_count": int(np.sum(values > args.deadline_ms)),
        "deadline_miss_rate": float(np.mean(values > args.deadline_ms)),
        "onnxruntime_version": ort.__version__,
    }
    output = Path(args.onnx_dir) / f"latency_cpu_{args.threads}thread.json"
    output.write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps(result, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
