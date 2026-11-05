# pysoarm

A **lean, hardware-level Python API** for the [SO-100 / SO-101](https://github.com/TheRobotStudio/SO-ARM100)
robot arm: six Feetech **STS3215** serial-bus servos on a single 1 Mbaud TTL bus.

`pysoarm` talks to the bus directly and stops there — no datasets, no policies, no training
stack. The ergonomics follow `pylibfranka` / `pyAgxArm` (`connect()` → `read_once()` →
`write_goal()` / control loop) within what position-servo hardware can actually do.

!!! note "Version 0.1.3"
    Transport (`scs` and the optional `rustypot` backend), the motor-side calibration flow,
    safety, trajectories, Cartesian kinematics, `SOArm` (follower and read-only leader),
    diagnostics (`health()`, `doctor --stress`), the `ArmBridge`/ROS 2 adapter and the CLI are
    implemented. What is *not* here is by design: teleoperation, dataset recording and
    policies ([ADR 0001](adr/0001-scope-device-layer.md)). See `CHANGELOG.md` for the full
    list.

## Design targets

| Target | Value | How it is enforced |
|---|---|---|
| Control step (read 6 + write 6) | ≤ 4 ms (`scs`), ≤ 2 ms (`rustypot`) | hardware tests assert both |
| Position loop | 200 Hz default | 1.44 ms/step and 0 overruns with `rustypot` |
| Import side effects | none | `import pysoarm` only reads its version |
| Untested hardware paths | none | everything above the transport runs on `FakeBus` |
| Silent clipping | never | `write_goal()` returns the goal actually sent |

## Start here

1. [Install](guides/install.md)
2. [Quickstart](guides/quickstart.md)
3. [Hardware notes](hardware.md) — including the honest capability matrix
4. [API reference](reference/api.md)
