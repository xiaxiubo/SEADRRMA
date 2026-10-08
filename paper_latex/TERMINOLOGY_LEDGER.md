# TRACE terminology ledger

Use these terms consistently in the manuscript, figures, captions, and deployment notes.

| Concept | Canonical form | Avoid |
|---|---|---|
| Proposed method | TRACE | DR-RMA when referring to the final method |
| Full name | Timescale-Resolved Adaptation via Cross-Attention and Temporal Encoding | Alternative expansions of TRACE |
| Explicit estimator | Fast Attention v2 Branch | fast network, attention fast, physics fast |
| Residual estimator | Slow TCN Branch | slow network, low-frequency branch |
| Reference baseline | RMA | treating RMA as a strict TRACE ablation |
| Load inertia | load inertia, $J_l$ | payload weight when inertia is meant |
| Inertia estimate | $\hat{J}_l$ | estimated mass |
| Controller output | motor torque, $\tau_m$ | force for this rotational SEA |
| Elastic signal | spring deflection, $\theta_s$ | spring position unless defined |
| Timing target | 200 Hz control rate; 5-ms period | claiming full-loop compliance from inference alone |

## Domain separation

- **Paper simulation profile:** $K_s=3800$ N m/rad, $F_{c,m}=6.0$ N m,
  $F_{c,l}=0.2$ N m, $B_m=8.0$ N m s/rad, and $B_l=0.01$ N m s/rad.
- **Hardware-oriented plant profile:** physical $K_s=2400$ N m/rad and the
  Stribeck ranges documented in `FRICTION_TRAINING_HANDOFF_20260728.md`.
- **Current hardware-v2 Fast feature scale:** 3800 N m/rad for checkpoint
  compatibility. This is an input calibration constant learned into the current
  weights, not a revised physical-stiffness claim. A clean 2400-scale retraining
  remains desirable before final hardware deployment.

## Evidence language

- Use **deterministic condition coverage** for the 108-case suite; do not call it
  statistical robustness without repeated seeds.
- Use **neural-inference latency** for the reported ONNX timings; do not call them
  end-to-end control-cycle latency.
- State that hardware experiments are in progress until raw closed-loop logs and
  repeated-trial statistics are available.
