# Changelog

All notable changes to this project are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- **Bus layer** (`pysoarm/bus/`): a `Bus` Protocol as the single I/O boundary, the STS3215
  register table with sign-magnitude codecs and unit scaling, a `pyserial` SCS backend
  (`read`/`write`/`sync_read`/`sync_write`/`scan`, retries, checksum and status validation,
  frame resynchronisation), and an in-memory `FakeBus` with fault injection and a
  transaction log for tests.
- **Calibration** (`pysoarm/calibration.py`): LeRobot-compatible JSON in **and** out,
  validation against an arm model, `raw ↔ deg` conversion with the 4095 full-scale, travel
  clamping and mirrored (`drive_mode = 1`) joints.
- **Safety** (`pysoarm/safety.py`): validity, watchdog, over-temperature, undervoltage,
  soft-limit clamping and per-step clipping, expressed as pure functions returning a
  `SafetyReport` so clipping is observable.
- **Trajectories** (`pysoarm/trajectory.py`): minimum-jerk, linear ramp and
  `ResampledTrajectory` (replay of any recorded `(t, q)` table — a primitive, not a format).
- **`SOArm`** (`pysoarm/arm.py`): `connect` (configure → hold measured pose → enable torque),
  `read_once`, `write_goal`, `hold`, `move_to`, `run_control_loop`, idempotent `stop()`,
  context-manager support, health-register caching spread over ticks, and a communication
  watchdog that disables torque before raising.
- **CLI** (`pysoarm` console script): `scan`, `read`, `monitor`, `hold`, `doctor` with stable
  exit codes; `doctor` checks the tty, permissions, other processes holding the port and
  `ModemManager`.
- **Models** (`pysoarm/models/`): SO-100/SO-101 layout, motor ids, gripper protection caps,
  aliases for LeRobot's `so_follower`/`so_leader` names.
- **`rustypot` backend** (`pysoarm/bus/rustypot.py`): the same `Bus` protocol on top of the
  Rust driver LeRobot uses, selectable with `SOArm(backend="rustypot")`, `make_bus(...)` or
  `--backend` on every CLI sub-command (`auto` prefers it when importable). Measured on the
  reference arm: six-joint read 0.98 ms (vs 2.18 ms), control step 1.44 ms (vs 2.79 ms),
  200 Hz loop with **0 overruns** over 3 × 200 ticks (vs 3–6). Adapter tests run against a
  stub of the pyo3 module, plus opt-in hardware tests for correctness (both backends must
  report the same positions) and the latency budget. Retries are handled by the adapter with
  rustypot's own retry loop disabled, so both backends retry identically and an intermittent
  "Parsing error" from the Rust layer is retried instead of tripping the watchdog.
- **Tests**: 155 unit tests (protocol-level wire emulation, fake-bus control paths, safety
  tables, calibration round trips, CLI) plus 3 opt-in hardware tests, at ~90 % coverage.
- **Measured on the reference arm**: single register read 0.36 ms, six-joint `sync_read`
  2.18 ms, six-goal `sync_write` 0.26 ms, full control step 2.71 ms → 200 Hz loop with
  2–4 overruns per 200 ticks. Hardware tests assert the step stays under 4 ms.
- **Hardware finding**: the TTL bus is half-duplex, so pipelining READ requests corrupts the
  stream; `sync_read` issues sequential request/response pairs and the wire-level tests model
  the collision so a regression fails CI.

### Documentation

- Added [ADR 0001](docs/adr/0001-scope-device-layer.md): pysoarm is a device-layer SDK. It
  records the rule for what belongs in the package, lists teleoperation, dataset recording,
  policies and ROS drivers as explicit non-goals (with the reasoning and the alternatives
  considered), and documents replay as a *primitive* (`ResampledTrajectory`) rather than a
  dataset format. Mirrored in `README.md` under "Non-goals" and linked from the
  architecture page and the mkdocs navigation.

### Changed

- Development commands (`make lint|type|test|cov|docs|build`) now run through `uv run`;
  `make check` runs the full CI set locally and `make install-dev` does
  `uv sync --locked` + `pre-commit install`. A plain-pip path
  (`pip install --group dev -e .`) remains documented for contributors without uv.

### Planned

- Cartesian goals through an optional `kinematics` extra (placo + user-supplied URDF).
- ROS 2 adapter as a separate package, if there is demand.

[Unreleased]: https://github.com/USER/pysoarm/compare/main...HEAD
