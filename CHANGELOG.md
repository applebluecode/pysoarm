# Changelog

All notable changes to this project are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

_Nothing yet._

## [0.1.2] - 2026-11-05

Follow-up to the 0.1.1 hardware re-test (`PYSOARM_IMPROVEMENTS.md` §11, items L1–L9):
rate-independent speed limits, role-aware voltage defaults, gripper arrival semantics,
resilient connect, per-source loop errors and machine-readable diagnostics.

### Changed (behaviour defaults)

- **No software speed/step cap by default** (L1, LeRobot parity). `SafetyLimits()` now defaults
  to `max_speed_deg_s=None` and `max_step_deg=None`, so `write_goal()` sends the requested goal
  (soft-limit clamped) instead of clipping it to a per-tick step. This matches LeRobot's SO
  follower (`max_relative_target=None`) and removes the artificial speed ceiling during
  teleoperation. A rate-independent ceiling remains one keyword away:
  `SafetyLimits(max_speed_deg_s=720)` (14.4°/tick at 50 Hz, 3.6°/tick at 200 Hz) or
  `SafetyLimits(max_step_deg=5.0)` for a fixed cap; when both are set the stricter wins.
  `set_speed_scale()` only affects a configured cap. `connect()` logs the effective settings at
  INFO (including "no software speed/step cap").
- **Voltage threshold follows the role** (L3). `SOArm(role="leader")` with the default
  limits uses `SafetyLimits.for_role("leader")` (4.0 V) instead of the follower's 10.0 V, so
  a healthy 4.7–4.9 V leader is no longer flagged `voltage` by `health()` or `doctor`.
  Callers that constructed `SafetyLimits()` explicitly keep 10.0 V.

### Added

- **`doctor --json` and role-aware stress probe** (L2, L9). `_stress()` now reports
  `timeouts` (no reply) and `status_alarms` (a reply carrying an `ERROR` bit) as separate
  counters with an `alarm_counts` histogram and failed-read latency; `doctor --json` emits
  `port`, `role`, `min_voltage_v`, `backend`, `checks`, `warnings`, `timeouts`,
  `status_alarms`, `alarm_counts`, `latency` and `failed_latency`. `doctor --role leader`
  defaults `--min-voltage` to 4.0 V.
- **Blocking gripper moves** (L4). `gripper.move(percent, wait=True, duration=0.5, hz=200)`
  (also `open`/`close`) runs a trajectory until the jaws arrive; without `wait` the call
  keeps the historical one-step, non-blocking behaviour. `homing()` is documented as
  changing the closed end only.
- **`goal_speed` in the tuning surface** (L5). It is now part of `MOTOR_CONFIG_REGISTERS`,
  so `motor_config()` reads it and `set_motor_config(joint, goal_speed=...)` writes it.
  Default remains `0` (servo-unlimited), so the LeRobot-compatible configuration is unchanged.
- **`connect()` retries with a backoff** (L6). `connect(retries=2, backoff_s=0.25)` retries
  the whole connect sequence after a transient bus failure (closing/reopening the port and
  clearing per-attempt health state). The final error includes the attempt count, a hint to
  check supply/wiring/port ownership and the matching `pysoarm doctor` command.
- **Per-source loop errors** (L7). `LoopStats` gains `read_errors`, `callback_errors` and
  `write_errors` (summing to `errors`, included in `as_dict()`), so a failed leader read inside
  a teleoperation callback is distinguishable from a follower read failure. The docs now spell
  out the precedence: arm watchdog (3) before the loop cutoff (10).
- **`SafetyLimits.min_step_deg`** (L8). Commands below the configured deadband are not sent;
  the measured position is kept and the skip is reported in `SafetyReport.reasons`. Default
  `0.0` keeps the exact previous `write_goal` output.
- **`SafetyLimits.for_role(role, **overrides)`** for role-appropriate defaults.
- **Docs**: precision/deadband/drift tables (`docs/hardware.md`), speed and failure-precedence
  tables (`docs/reference/api.md`), role-aware supply voltages, and the 0.1.2 version stamp.
- **Tests**: 21 new cases for L1–L9 (default no-limit parity + opt-in speed preset, stress
  classification, leader voltage, gripper arrival, `goal_speed`, connect retry, error
  attribution, deadband skip and `doctor --json`), bringing the offline suite to 305.

## [0.1.1] - 2026-11-05

Reliability, calibration and diagnostics: the follow-up to the first hardware report.

### Added

- **Error classification** (`CommunicationError`): structured `motor_id`, `register`,
  `status`, `transient`, `attempts` and a decoded `flags` set (`voltage`, `sensor`,
  `overheat`, `overcurrent`, `angle`, `overload`). Status alarms are now retried by both
  backends (`RustypotBus(retry_status=False)` opts out), and every `Bus` call accepts a
  per-call `retries=` override.
- **Low-latency loop primitives**: `run_control_loop(read_state_every=..., health_period_s=...)`,
  per-tick latency (`mean/p95/max`), `deadline_misses`, `errors` and
  `consecutive_errors_max` on `LoopStats`, plus `LoopStats.as_dict()`.
- **Error policies** for the loop: `on_error="abort" | "hold" | "skip"` with
  `max_consecutive_errors`, so a lost frame no longer ends a session (P0-1/P1-2).
- **Public calibration flow** (`pysoarm.calibration`): `read_calibration`,
  `write_calibration`, `reset_calibration`, `set_half_turn_homings`, `record_ranges`,
  `calibrate`, with the Feetech `position - 2047` convention and `wrist_roll` full-turn
  handling. New CLI: `pysoarm calibrate`, `pysoarm calibration show|export`.
- **Safety surface**: `emergency_stop()` (keeps the connection), `enable/disable(joints)`,
  `is_enabled()`, `reset_faults()`, and the documented `stop`/`emergency_stop`/`hold`
  semantics.
- **Roles**: `SOArm(role="leader")` is a read-only arm: torque stays off, no goal is
  written, and only readability registers are configured.
- **Diagnostics**: `SOArm.health()` returns structured `MotorHealth` (voltage,
  temperature, current, latched/derived flags), `ArmState.health_age_s`, and
  `pysoarm doctor --stress SECONDS` reports per-motor timeouts, status alarms, latency and
  low-voltage/hot-motor warnings.
- **Tuning**: `SafetyLimits.max_speed_deg_s` + `limits.step_for(hz)` decouple the step
  guard from the loop rate; `set_speed_scale()`, `motor_config()` and
  `set_motor_config()` expose the tuning registers.
- **Gripper**: a thin `arm.gripper` facade (`open/close/move(percent)/hold/homing`); force
  remains servo-limited, not closed-loop.
- **Examples**: all six now runnable offline with `--fake`; CI smoke-tests them.
- **Structure**: internal refactor with no public API or behaviour change: the control loop
  moved to `pysoarm.loop`, health caching/tuning to `diagnostics.py`/`tuning.py`, calibration
  became a `model` + `flow` package, and the CLI split into `parser.py`, `factories.py` and
  `commands/`.
- **Tests**: 283 offline tests (was 207), covering status retry parity, calibration flow,
  roles, the safety surface, loop policies/latency, diagnostics, CLI commands, examples and
  the extracted modules.

## [0.1.0] - 2026-11-04

First public release: the device layer for one SO-100/SO-101 arm.

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
- **Tests**: 207 unit tests (protocol-level wire emulation, fake-bus control paths, safety
  tables, calibration round trips, CLI) plus 3 opt-in hardware tests, at ~90 % coverage.
- **Measured on the reference arm**: single register read 0.36 ms, six-joint `sync_read`
  2.18 ms, six-goal `sync_write` 0.26 ms, full control step 2.71 ms → 200 Hz loop with
  2–4 overruns per 200 ticks. Hardware tests assert the step stays under 4 ms.
- **Hardware finding**: the TTL bus is half-duplex, so pipelining READ requests corrupts the
  stream; `sync_read` issues sequential request/response pairs and the wire-level tests model
  the collision so a regression fails CI.

- **Cartesian control** (`pysoarm/kinematics/`): a dependency-free `Pose` (metres + unit
  quaternion, RPY helpers), the `Kinematics` protocol, and `PlacoKinematics` behind the
  optional `kinematics` extra. `SOArm.forward()` and `SOArm.move_to_pose()` solve IK **before**
  writing any goal, so an unreachable target raises `NotReachableError` with the residual and
  leaves the arm untouched; the resulting joint goal still passes every safety guard.
  Defaults are tuned from measurement: a placo task gain of `1e-3` converges in ~5 iterations
  on the reference two-link URDF, while `1.0` overshoots into a wrong branch. The placo import
  is lazy, so `import pysoarm` still needs neither numpy nor placo.
- **CLI**: `pysoarm pose` (FK, from `--q` or read-only from `--port`) and `pysoarm ik`
  (dry-run inverse kinematics), both with `--json`.
- **`ArmBridge`** (`pysoarm/bridge.py`): framework-agnostic snapshot (`JointStateFrame`, with
  radians for ROS), command path in degrees or radians, torque enable/disable, and a
  staleness rule — a command based on a state older than `max_command_age_s` is refused.
- **ROS 2 package** (`ros2/pysoarm_ros2`, separate `ament_python` package): publishes
  `joint_states` (radians), subscribes to `joint_commands`, exposes `enable`/`disable`
  services, and routes everything through `ArmBridge` so the guards cannot be bypassed. CI has
  no ROS installation: the node is syntax-checked only and its tests skip without `rclpy`, while
  all ROS-independent logic is covered by the main suite. The node itself is covered by
  `tests/test_ros2_node.py`, which loads it against stub `rclpy`/message modules and checks
  parameter declarations, endpoints, degree→radian conversion, command routing, stale-command
  rejection, the torque services and torque release on shutdown.

### Documentation

- Added [ADR 0001](docs/adr/0001-scope-device-layer.md): pysoarm is a device-layer SDK. It
  records the rule for what belongs in the package, lists teleoperation, dataset recording,
  policies and ROS drivers as explicit non-goals (with the reasoning and the alternatives
  considered), and documents replay as a *primitive* (`ResampledTrajectory`) rather than a
  dataset format. Mirrored in `README.md` under "Non-goals" and linked from the
  architecture page and the mkdocs navigation.

### Changed

- Project metadata now points at the public repository
  <https://github.com/applebluecode/pysoarm> (author `pysoarm contributors`, security reports via GitHub private
  advisories, ROS 2 maintainer `pysoarm contributors`).
- Development commands (`make lint|type|test|cov|docs|build`) now run through `uv run`;
  `make check` runs the full CI set locally and `make install-dev` does
  `uv sync --locked` + `pre-commit install`. A plain-pip path
  (`pip install --group dev -e .`) remains documented for contributors without uv.
- `uv.lock` is resolved against pypi.org (the index pinned in `pyproject.toml`), so
  `uv sync --locked` works on CI rather than only next to a local mirror config. `numpy`
  joined the `dev` group because the kinematics adapter imports it at module level while
  `placo` stays optional — its tests skip when the extra is absent.

[Unreleased]: https://github.com/applebluecode/pysoarm/compare/v0.1.1...HEAD
[0.1.1]: https://github.com/applebluecode/pysoarm/releases/tag/v0.1.1
[0.1.0]: https://pypi.org/project/pysoarm/0.1.0/
