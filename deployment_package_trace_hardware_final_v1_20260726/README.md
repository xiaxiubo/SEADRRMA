# TRACE hardware controller ONNX package

Version: `trace_hardware_final_v1_20260726`

This package contains the selected hardware-deployment TRACE controller as one
ONNX graph: Fast Attention v2, Slow TCN, frozen VecNormalize statistics, and the
deterministic PPO actor.

## Runtime dependencies

```bash
pip install -r requirements.txt
```

Only NumPy and ONNX Runtime are required. PyTorch and Stable-Baselines3 are not
required on the target controller.

## Model interface

Inputs, all `float32` with fixed batch size 1:

- `history`: `[1,100,9]`, newest observation at index 99
- `base_obs`: `[1,9]`, the same newest observation
- `startup_override`: `[1,1]`; use 1 during the first 0.6 s after
  reset and 0 afterwards

The 9D observation order is:

```text
[load_position_error, load_velocity_error,
 motor_position_error, motor_velocity_error,
 spring_deflection, spring_velocity,
 target_position, target_velocity,
 previous_normalized_action]
```

Outputs:

- `action`: normalized motor command in `[-1,1]`
- `j_hat`: inertia supplied to the PPO actor, including startup override
- `j_raw`: raw Fast estimate
- `j_short`, `j_long`, `jump_gate`, `confidence`: diagnostics

At reset, clear all 100 history rows and set previous action to zero. At every
5 ms control tick, append `base_obs`, run ONNX, then store the returned action as
the next `previous_normalized_action`.

The simulated actuator mapping is `torque_Nm = clip(action,-1,1) * 61`. Keep the
real drive torque/current conversion, saturation, watchdog, communication fault
handling, and emergency stop outside the ONNX graph.

## Calibrated SEA domain

- motor/harmonic Coulomb friction: 3.24 Nm
- load-side Coulomb friction: 0.34 Nm
- motor/harmonic viscous friction: 10.43 Nm s/rad
- load-side viscous friction: 0.76 Nm s/rad
- spring stiffness: 2400 Nm/rad

Friction values are not online Fast inputs. The Fast graph uses measured motion,
spring signals, previous action, and the known spring stiffness. The profile is
included to document the training domain.

## Minimal call

```python
import numpy as np
import onnxruntime as ort

session = ort.InferenceSession(
    "onnx/trace_hardware_control.onnx",
    providers=["CPUExecutionProvider"],
)
history = np.zeros((1, 100, 9), dtype=np.float32)
base_obs = np.zeros((1, 9), dtype=np.float32)
history[:, :-1] = history[:, 1:]
history[:, -1] = base_obs
outputs = session.run(
    None,
    {
        "history": history,
        "base_obs": base_obs,
        "startup_override": np.ones((1, 1), dtype=np.float32),
    },
)
action = outputs[0]
```

Run `python benchmark_onnx.py --onnx-dir onnx --threads 4` on the target computer
and verify the complete EtherCAT loop remains below the 5 ms deadline.
