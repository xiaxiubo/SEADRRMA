# Hardware safety checklist

This software is research code. It is not a certified safety controller. A
physical emergency stop, drive-level current/torque limits, and independent
mechanical travel protection are required.

## Before enabling torque

- [ ] The joint is mechanically restrained or operated in a guarded area.
- [ ] The physical emergency stop has been tested without this software.
- [ ] Drive over-current, over-speed, and over-temperature protection is active.
- [ ] Encoder zero, direction, units, and gear ratio are verified manually.
- [ ] Spring-deflection direction and torque sign are verified at low current.
- [ ] The configured torque limit is below the commissioning limit.
- [ ] The configured position, velocity, and spring limits are conservative.
- [ ] Backend communication loss disables the drive independently.
- [ ] Dry-run logs show finite observations and a stable 5-ms loop.
- [ ] The ONNX SHA-256 digest matches the intended release.

## Staged test sequence

1. Output disabled, stationary joint, inspect all channels.
2. Output disabled, move the joint manually through a small range.
3. Low fixed torque with the learned controller disconnected.
4. Learned controller at low torque and fixed inertia.
5. Small-amplitude, low-frequency trajectory.
6. Increase amplitude/frequency only after reviewing tracking, torque, spring
   deflection, loop timing, saturation, and watchdog events.
7. Test inertia switching last.

Any non-finite model output, limit violation, timing overrun, communication
fault, or operator abort must command zero torque and disable the drive.

