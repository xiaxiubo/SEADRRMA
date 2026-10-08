# Remaining work before submission

This list contains only evidence or author information that cannot be completed
from the current repository without new measurements or decisions.

## Required evidence

- [ ] Run repeated physical H2 inertia-switching trials for TRACE, RMA, PPO,
  PD+DOB, and LQR under the same reference and clutch timing. Report trial count,
  mean, standard deviation or confidence interval, tracking RMSE, post-event
  RMSE, torque RMS/increment, saturation, and recovery time.
- [ ] Run H1, H3, and H4 hardware checks for TRACE, including raw encoder,
  current/torque, clutch-command, inertia-estimate, and loop-period logs.
- [ ] Measure the complete 200-Hz hardware cycle from sensor acquisition through
  EtherCAT output. Report mean, P95, P99, maximum, and deadline-miss share; the
  current paper reports neural inference only.
- [ ] Complete a strict component study using one controlled training/evaluation
  protocol: full TRACE, fixed nominal inertia, Fast-only, Slow-only, short-only,
  long-only, fixed fusion, and pre-DAgger.
- [ ] Retrain the Slow branch with either pure target MSE or a genuinely
  sequence-aware temporal regularizer, then compare it against the frozen legacy
  minibatch-consistency checkpoint under the same closed-loop protocol.
- [ ] Repeat the decisive simulation cases over multiple seeds and report
  uncertainty. The current 108-case suite is deterministic coverage with one seed
  per condition.

## Hardware-model closure

- [ ] Independently revalidate the motor/load dynamic-friction split using the
  final 2400 N m/rad stiffness.
- [ ] Retrain or fine-tune the hardware Fast branch with an explicit 2400 N m/rad
  feature scale. The current checkpoint was trained with a 3800 feature scale,
  although its simulated plant correctly used 2400.
- [ ] Test sensor noise, encoder quantization, 5/10/20-ms delay, clutch-timing
  variation, and unmodeled compliance before claiming broad sim-to-real
  robustness.
- [ ] Verify the exported hardware Student in closed loop on the physical joint,
  including startup/reset behavior and both clutch transitions.

## Author-supplied items

- [ ] Confirm author affiliations, corresponding author, postal address, and all
  email addresses.
- [ ] Supply funding, facility, and collaborator acknowledgments if applicable.
- [ ] Confirm the target IEEE journal and apply its current page-limit,
  supplementary-material, data-availability, and author-biography requirements.

## Completed in the 2026-08-02 audit

- [x] Preserved the high-friction paper simulation profile independently of the
  hardware Stribeck profile.
- [x] Corrected the reward equation, final Teacher refinement scale, Slow-loss
  coefficient, and inference-latency wording against executable evidence.
- [x] Replaced the inaccurate temporal-smoothing claim with the exact legacy
  shuffled-minibatch consistency objective and disabled that term by default for
  future training runs.
- [x] Removed inactive legacy simulation/discussion drafts and their stale TODOs.
- [x] Verified all labels, references, and bibliography keys used by the active
  manuscript.
- [x] Reproduced the five-controller paper benchmark after explicitly restoring
  the 3800 N m/rad model stiffness for PD+DOB and LQR.
- [x] Regenerated all five simulation figures and obtained zero pixel difference
  from the frozen manuscript PNGs.
- [x] Compiled and visually inspected the 12-page manuscript with no missing
  references, overfull boxes, overlaps, or clipped content.
- [x] Selected the hardware-v2 Student over 20 deterministic closed-loop cases,
  froze the causal Slow-latent EMA runtime, and documented the non-uniform
  inertia-estimation tradeoff.
- [x] Re-exported the selected hardware Student as a stateful ONNX graph with its
  exact runtime settings, verified PyTorch/ONNX parity, and recorded the graph
  checksum and workstation CPU latency separately from end-to-end timing.
