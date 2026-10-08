# TRACE Hardware ONNX Local Initial Test

Date: 2026-07-27

## Scope

This is a local CPU-only preliminary test of
`onnx/trace_hardware_control.onnx`. It verifies package integrity, the ONNX
interface, numerical behavior, startup handling, model-only latency, and the
local deployment wrapper. It does not verify EtherCAT communication, serial
clutch control, actuator safety, or complete 200 Hz closed-loop timing.

## Model Configuration

- Version: `trace_hardware_final_v1_20260726`
- ONNX opset: 18
- Control period: 5 ms
- Spring stiffness: 2400 Nm/rad
- Startup override: 0.6 s with `J_hat = 0.3 kg m^2`
- Inputs: `history [1,100,9]`, `base_obs [1,9]`,
  `startup_override [1,1]`
- Outputs: `action`, `j_hat`, `j_raw`, `j_short`, `j_long`, `jump_gate`,
  and `confidence`, each with shape `[1,1]`
- ONNX SHA-256:
  `58c453cb53702784874165b94cd934beabd835026ce35ccca94ad01c26449780`

The 12 files in the supplied package passed the SHA-256 manifest check. This
report and its JSON companion are post-package local test artifacts and are
not part of that original manifest.

## Results

### Interface and basic inference

- ONNX Runtime 1.28.0 loaded the model successfully on the CPU provider.
- Actual input names, output names, shapes, and float32 types match the
  metadata.
- All tested outputs were finite.
- Twenty repeated inferences with identical inputs were bitwise
  deterministic (`max_abs_diff = 0`).
- Invalid history shape, invalid observation shape, and float64 input were
  rejected.

With zero observations and `startup_override = 1`, the reported inertia was
`J_hat = 0.30000001 kg m^2`. With the override disabled, the same zero history
gave `J_hat = J_raw = 0.8478813 kg m^2`. The deployment loop must therefore
provide the startup input for the first 0.6 s; zero-filled history alone is
not a valid initialization substitute.

### Local latency

The model was benchmarked for 3000 timed calls after warm-up:

| CPU threads | Mean (ms) | P95 (ms) | P99 (ms) | Max (ms) | Calls over 5 ms |
|---:|---:|---:|---:|---:|---:|
| 1 | 1.2005 | 1.3905 | 1.5681 | 1.9020 | 0 / 3000 |
| 4 | 0.7678 | 0.9001 | 1.0385 | 1.6105 | 0 / 3000 |

The full local wrapper, including history update, observation assembly, and
ONNX inference with four threads, measured:

- Mean: 0.7948 ms
- P99: 1.1418 ms
- Maximum: 1.9426 ms
- Calls over 5 ms: 0 / 3000

These values establish local model feasibility only. They do not include
EtherCAT I/O, serial I/O, logging, scheduling jitter, or actuator command
execution on Orin.

### Stress behavior

Two thousand bounded but deliberately incoherent random observations produced
finite outputs. The action remained in `[-1,1]`, the gating outputs remained
bounded, and `J_hat` remained in the model's configured `[0.03,0.3]` interval.
Frequent action or inertia-bound saturation under such random inputs is an
out-of-distribution response, not evidence of trajectory-level accuracy.
Boundary occupancy should be monitored during the first hardware runs.

## Deployment Integration

The previous Orin entry script supplied only `history` and `base_obs`, so it
is incompatible with this three-input model. The local integration now:

- supplies `startup_override` during the first 0.6 s;
- records `j_raw` in addition to the existing estimator diagnostics;
- validates model names, shapes, and input types at startup;
- uses the new package as the default local model path; and
- uses `K_s = 2400 Nm/rad` directly, with no additional 0.628 scaling.

The reusable wrapper is
`controllers/controllers/trace_onnx_controller.py`. The Orin entry script has
been adapted locally but has not been uploaded or run on Orin in this test.

## Preliminary Decision

The ONNX file passes local integrity, interface, startup, determinism,
finite-output, and model-latency checks. It is suitable for the next staged
Orin software test. Hardware operation still requires an Orin-side offline
load test, an end-to-end timing test, a command-disabled data-acquisition run,
and then a guarded low-amplitude closed-loop test.
