# Controller Migration Checklist

## 1. Platform

- Install Python 3.10 and `requirements.txt`.
- Confirm the target architecture has a working ONNX Runtime build.
- Confirm the real-time kernel, CPU affinity, memory locking, and scheduler
  permissions available on the target controller.
- Run `python3 tools/validate_release.py`.

## 2. EtherCAT

- Replace `eno1` with the target interface using `--ifname`.
- Run `scripts/test_scan.py` and compare slave identity and PDO size.
- Run `scripts/drive_feedback_probe.py` without enabling torque.
- Verify the runtime-detected expected WKC; do not hard-code the old value.
- Confirm target-torque units for object `0x6071` and the joint rated torque.

## 3. Sensors

- Verify motor and load encoder units, signs, offsets, and gear-side meaning.
- Verify the third encoder count width. The current joint uses 19 bits.
- Recalibrate the spring zero using
  `control_flow/SEA_spring_calibration_locked.py`.
- Replace `2400 Nm/rad` only after a new bidirectional load calibration.

## 4. Clutch

- Identify the relay through `/dev/serial/by-id/`.
- Use `control_flow/baseline_relay_control.py` for readback and manual switching.
- Verify which electrical state engages the clutch. The original bench used
  `OFF = engaged`, `ON = released`.
- Keep automatic relay control disabled during initial controller tests.

## 5. Controller integration

- Start with the ONNX benchmarks.
- Run DRRMA/TRACE in `--no-execute` mode and inspect generated observations.
- Check observation order and SI units against the deployment README.
- Check action-to-torque scaling and apply the hardware torque limit outside the
  model graph.
- Begin at a low torque limit and short duration with an emergency stop ready.

## 6. Acceptance evidence

For each controller and inertia state, retain:

- exact command and Git commit
- hardware/configuration description
- reference and measured position/velocity
- spring deflection and feedback torque
- commanded and measured motor torque
- clutch command/readback and inertia state
- WKC, cycle time, deadline misses, drive status, and stop reason
- raw CSV plus generated metrics/figures

