# SEADRRMA Joint Test Toolkit

Portable runtime and test utilities for deploying the TRACE/SEADRRMA controller
on a single-degree-of-freedom series elastic actuator (SEA).

The repository is intentionally separated from the research training workspace.
It contains controller-side source code, configuration templates, safety checks,
log analysis, and a mock backend. Large checkpoints, raw experiment logs, vendor
SDKs, credentials, and machine-specific calibration files are not committed.

## Runtime contract

- Control period: 5 ms (200 Hz)
- History: 100 observations, newest sample last
- Observation order:

```text
[load_position_error, load_velocity_error,
 motor_position_error, motor_velocity_error,
 spring_deflection, spring_velocity,
 target_position, target_velocity,
 previous_normalized_action]
```

- ONNX inputs: `history`, `base_obs`, `startup_override`
- The ONNX action is normalized to `[-1, 1]`; torque scaling and all hardware
  safety limits remain outside the graph.

## Install

Python 3.10 or newer is recommended.

```bash
python -m venv .venv
# Windows: .venv\Scripts\activate
# Linux:   source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -e .
```

Copy the deployment model to `models/trace_hardware_control.onnx`, then compare
its SHA-256 digest with your release manifest. Model files are excluded from Git
by default.

## Safe smoke test

The default command uses the built-in mock backend and never enables physical
torque output:

```bash
python scripts/check_model.py --model models/trace_hardware_control.onnx
python scripts/run_joint_test.py --config config/joint_profile.example.json \
  --model models/trace_hardware_control.onnx --duration 10
python scripts/analyze_log.py logs/latest.csv
```

## Port to another controller

1. Implement the `JointBackend` protocol in `src/seadrrma_joint/backend.py`.
2. Expose a factory as `your_module:create_backend`.
3. Verify encoder signs, radians, velocity units, spring-deflection sign, torque
   conversion, watchdog behavior, and emergency stop with output disabled.
4. Run low-torque fixed-inertia tests before any inertia-switching experiment.
5. Enable output explicitly only after completing `docs/SAFETY.md`.

```bash
python scripts/run_joint_test.py \
  --backend your_module:create_backend \
  --config config/joint_profile.local.json \
  --model models/trace_hardware_control.onnx \
  --duration 10 --enable-output
```

`--enable-output` is deliberately required for physical torque commands. The
example profile uses conservative limits and must not be treated as calibrated
hardware settings.

## Repository layout

```text
config/                 Shareable configuration templates
docs/                   Porting and safety checklists
scripts/                Model check, experiment runner, and log analysis
src/seadrrma_joint/     Portable runtime package
tests/                  Hardware-free unit tests
```

See `docs/PORTING.md` for the backend contract and `docs/SAFETY.md` before
connecting a real actuator.
