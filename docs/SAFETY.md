# Hardware Safety Checklist

Complete this checklist before every first run on a new controller or joint.

## Mechanical and electrical

- The joint is supported and cannot strike people or equipment.
- The external inertia and removable weights are mechanically secured.
- The emergency stop is reachable and verified independently of software.
- Drive overcurrent, overtemperature, and torque protections are enabled.
- Encoder wiring, signs, units, and zero positions are verified at zero torque.
- The clutch state is confirmed mechanically, not inferred only from relay state.

## Communication

- The intended NIC is passed through `--ifname`.
- Slave count, vendor/product identifiers, PDO sizes, and expected WKC match.
- EtherCAT OP and CiA 402 Operation Enabled are treated as separate states.
- Repeated WKC mismatch causes zero torque and shutdown.
- Loss or invalidity of the third encoder causes zero torque and shutdown.

## First powered motion

- Start with monitor-only or `--no-execute` mode.
- Use a low torque limit and a short duration.
- Use a small-amplitude, low-frequency trajectory.
- Keep relay control disabled until its electrical and mechanical meanings are
  checked on the new bench.
- Review the CSV log for position error, velocity, spring deflection, feedback
  torque, command torque, WKC, and missed deadlines before increasing limits.

## Stop conditions

Stop immediately on unexpected motion, encoder discontinuity, increasing spring
deflection, drive warning/fault, repeated deadline misses, WKC degradation, or a
command/feedback sign mismatch. Do not bypass a stop condition merely to finish
an experiment.

