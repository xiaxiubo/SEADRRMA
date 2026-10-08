# Porting to another controller

The portable runtime does not depend on EtherCAT, CANopen, a drive vendor SDK,
or a particular real-time Linux distribution. Hardware access is isolated in a
small backend implementation.

## Required backend methods

Implement `JointBackend` from `seadrrma_joint.backend`:

```python
class MyBackend:
    def read(self) -> JointSample: ...
    def write_torque(self, torque_nm: float, enable: bool) -> None: ...
    def disable(self) -> None: ...
    def close(self) -> None: ...
```

Expose a factory:

```python
def create_backend(profile: JointProfile) -> MyBackend:
    return MyBackend(...)
```

Run it with `--backend my_backend:create_backend`.

## Unit and sign contract

- Positions: radians at the motor/output coordinates used during training.
- Velocities: radians per second.
- Torque: N m at the modelled motor-torque coordinate.
- Spring deflection: motor-side equivalent angle minus load angle, with the
  exact sign used by the trained observation pipeline.
- Timestamp: monotonic seconds, not wall-clock time.

Do not hide gear-ratio conversion inside several layers. Perform every encoder,
gear-ratio, current-to-torque, and sign conversion once in the backend, document
it, and log both raw and converted quantities during commissioning.

## Timing

The runner uses an absolute 5-ms schedule to avoid accumulating sleep drift.
This is convenient for integration but is not a hard real-time scheduler. On the
target controller, measure the complete cycle from sensor acquisition through
actuator output and report mean, P95, P99, maximum, and missed-deadline share.

## Startup and reset

At reset, the runtime clears the 100-sample history and previous action. It
asserts `startup_override` for the configured startup duration. Never reuse
history state across drive faults or emergency-stop events.
