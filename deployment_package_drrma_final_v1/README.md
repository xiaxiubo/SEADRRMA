# DR-RMA final_v1 ONNX deployment package

This package contains the frozen `final_v1` controller exported as one ONNX graph for
the NVIDIA Orin CPU execution provider. Training-time dependencies such as PyTorch,
Stable-Baselines3, and VecNormalize are not required at runtime.

## Files

- `onnx/drrma_control_only.onnx`: complete Fast Attention v2 + Slow TCN + PPO graph
- `onnx/metadata.json`: model interface, provenance, and hashes
- `onnx/validation_results.json`: PyTorch/ONNX numerical comparison
- `onnx/latency_results_cpu_1thread.json`: latest local single-thread benchmark
- `benchmark_onnx.py`: hardware-independent ONNX Runtime benchmark
- `requirements.txt`: minimal Python dependencies

## Interface

Inputs use fixed batch size 1:

- `history`: `float32[1,100,9]`
- `base_obs`: `float32[1,9]`

The 9D order is:

```text
[pos_error_load, vel_error_load,
 pos_error_motor, vel_error_motor,
 spring_defl, spring_vel,
 target_pos, target_vel,
 prev_action]
```

Append the current `base_obs` to `history` before each inference. `prev_action` is the
previous normalized action and must be initialized to zero after controller reset.

Outputs are `float32[1,1]`:

- `action`: normalized PPO action in `[-1,1]`
- `j_hat`: bounded inertia estimate used by the PPO actor
- `j_short`: short-window inertia estimate
- `j_long`: long-window inertia estimate
- `jump_gate`: short/long fusion gate
- `confidence`: Fast Attention v2 diagnostic confidence

The graph does not convert `action` to drive torque units. Keep torque scaling and
safety saturation in the real-time controller. It also does not contain action EMA,
inertia rate limiting, or a startup inertia override.

## Runtime example

```python
import numpy as np
import onnxruntime as ort

options = ort.SessionOptions()
options.intra_op_num_threads = 1
options.inter_op_num_threads = 1
options.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
session = ort.InferenceSession(
    "onnx/drrma_control_only.onnx",
    sess_options=options,
    providers=["CPUExecutionProvider"],
)

history = np.zeros((1, 100, 9), dtype=np.float32)
base_obs = np.zeros((1, 9), dtype=np.float32)
history[:, :-1, :] = history[:, 1:, :]
history[:, -1, :] = base_obs

action, j_hat, j_short, j_long, jump_gate, confidence = session.run(
    None,
    {"history": history, "base_obs": base_obs},
)
```

Run a single-thread benchmark with:

```bash
python3 benchmark_onnx.py --onnx-dir onnx --threads 1 --outputs action --warmup 500 --steps 5000
```

## Verified Orin latency

The package was verified on the target aarch64 Orin with ONNX Runtime 1.23.2
`CPUExecutionProvider` on 2026-07-15. Each row used 300 warmup and 3000 measured
inferences:

| Outputs | Threads | Mean (ms) | P95 (ms) | P99 (ms) | Max (ms) | >5 ms |
|---|---:|---:|---:|---:|---:|---:|
| action | 1 | 4.718 | 4.849 | 5.045 | 6.727 | 48/3000 |
| action | 2 | 3.421 | 3.520 | 3.669 | 4.742 | 0/3000 |
| action | 4 | 2.581 | 2.683 | 2.793 | 4.318 | 0/3000 |
| all diagnostics | 4 | 2.526 | 2.625 | 2.720 | 4.561 | 0/3000 |

Use four intra-op threads for the first closed-loop integration. These values cover
ONNX inference only; EtherCAT communication and logging still require a complete
5 ms loop timing test.
