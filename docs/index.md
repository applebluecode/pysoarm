# pysoarm

A **lean, hardware-level Python API** for the [SO-100 / SO-101](https://github.com/TheRobotStudio/SO-ARM100)
robot arm: six Feetech **STS3215** serial-bus servos on a single 1 Mbaud TTL bus.

`pysoarm` talks to the bus directly and stops there — no datasets, no policies, no training
stack. The ergonomics follow `pylibfranka` / `pyAgxArm` (`connect()` → `read_once()` →
`write_goal()` / control loop) within what position-servo hardware can actually do.

!!! note "Pre-release (0.1.0-dev)"
    Transport, calibration, safety, trajectories, `SOArm` and the CLI are implemented. The
    faster `rustypot` backend, Cartesian kinematics and a ROS adapter remain on the roadmap
    (`CHANGELOG.md`).

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
