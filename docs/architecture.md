# Architecture

## Layering

```
        user code / policy / script
                   │
        ┌──────────▼───────────┐
        │        SOArm         │  arm.py       device lifetime, torque, control loop
        └──────────┬───────────┘
     ┌─────────────┼──────────────┬─────────────────┐
     ▼             ▼              ▼                 ▼
 safety.py    trajectory.py  calibration.py   kinematics/
 (guards)     (interpolators) (deg ↔ ticks)   (optional FK/IK)
     └─────────────┴──────────────┴─────────────────┘
                   │  raw ticks, sync transactions
        ┌──────────▼───────────┐
        │         Bus          │  bus/base.py  Protocol — the only I/O boundary
        └──────────┬───────────┘
        ┌──────────┼───────────┐
        ▼          ▼           ▼
     scs.py   rustypot.py   fake.py
   (pyserial)  (optional)   (tests/dev)
```

`make_bus(port, ids, backend="scs" | "rustypot" | "auto")` picks one; `auto` prefers
`rustypot` when it is importable. `scs` is the default so behaviour does not depend on what
happens to be installed (2.8 ms per control step vs 1.4 ms).

**Rule:** nothing above `bus/` may import `serial` or `rustypot`; nothing below `bus/` may
know about degrees, calibration or limits. That boundary is what makes the whole control
path testable on a laptop. What belongs inside this boundary — and what is deliberately
left to the application layer — is decided in
[ADR 0001](adr/0001-scope-device-layer.md).

## Control step

```
read_once():   sync_read  Present_Position (+1 health register/tick)  1.0-2.2 ms
               → ArmState (immutable)
guards:        validate → soft limits → per-step clip → thermal/supply
write_goal():  sync_write Goal_Position (one broadcast frame)          ~0.26 ms
               → returns the goal actually sent
```

Timing is owned by `SOArm.run_control_loop`, never by the trajectory: absolute-deadline
scheduling on a monotonic clock, overruns counted and reported (not silently absorbed).

## Error model

| Situation | Exception | Arm state afterwards |
|---|---|---|
| port busy / cannot open | `BusConnectionError` | unchanged, no torque change |
| reply missing after retries | `CommunicationError` | goal held, watchdog counts up |
| watchdog trips | `WatchdogTimeout` | **torque off** |
| goal would be unsafe | `SafetyViolation` or clamped goal | last safe goal |
| bad configuration/calibration | `ConfigurationError` / `CalibrationError` | no I/O performed |

`stop()` is idempotent and safe from a signal handler and from `finally`; leaving torque
enabled after an exception is treated as a bug (see `SECURITY.md`).
