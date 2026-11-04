# ADR 0001 — Scope: pysoarm is a device-layer SDK

- **Status:** Accepted
- **Date:** 2026-11-02
- **Deciders:** jw
- **Related:** `SECURITY.md` (repo root), [Architecture](../architecture.md), [Hardware notes](../hardware.md)

## Context

The SO-100/SO-101 stack already has an application layer: LeRobot provides teleoperation,
dataset recording (v3.0 format, video encoding, Hub upload), replay, policies and training.
What it does **not** provide is a small, dependency-light, typesafe driver for the bus that
a user can build on without importing torch.

Every new capability added to a library is a long-term maintenance contract. Recording in
particular implies: camera capture, video encoding (NVENC/VAAPI/SVT-AV1 fallbacks), a
parquet/JSONL dataset schema owned by another project, statistics computation, resumable
episodes and Hub upload. That contract is versioned by upstream, not by us.

We therefore need an explicit rule for what belongs in pysoarm, to keep scope creep and
dependency growth in check.

## Decision

**A feature belongs in pysoarm if, and only if, it is a physical fact of a single device:**
a register address, an encoder scale, a calibration constant, a limit, a timing constraint,
a status bit. Multi-device coordination, data formats and learning stacks stay outside.

| Function | Classification | Where it lives |
|---|---|---|
| Register access, sync read/write, torque enable | device fact | `pysoarm/bus/` |
| Calibration (homing offset, ranges, ticks ↔ degrees) | device fact — defines the unit | `pysoarm/calibration.py` |
| Limits, per-step clipping, thermal/supply guards, watchdog | device fact | `pysoarm/safety.py` |
| Joint-space trajectory interpolation | single-device timing | `pysoarm/trajectory.py` |
| State snapshots | device fact | `pysoarm/state.py` |
| FK/IK, Cartesian goals | model fact, needs a third-party solver | `pysoarm/kinematics/` (optional extra) |
| **Teleoperation** (leader → follower, input events) | two devices + an interaction policy | user code, or LeRobot |
| **Dataset recording** (cameras, video, parquet, Hub) | data-format contract + codecs | LeRobot |
| **Policies / training / inference** | learning stack | LeRobot |

Teleoperation is deliberately a ~10-line user program, because it is a policy:

```python
follower.write_goal(leader.read_once().q_deg)
```

Putting that in the SDK would bake a specific master/slave topology, joint order and gripper
convention into the library, and force a background-thread architecture on the synchronous
`read_once()` / `run_control_loop()` API.

Replay is supported as a *primitive*, not a format: `ResampledTrajectory` replays any
`(t, q)` table, and `examples/replay_trajectory.py` shows it. No dataset schema is assumed.

## Consequences

**Positive**

- Runtime dependencies stay at `pyserial` (+ optional `rustypot`); the wheel stays tiny and
  installs anywhere, including machines that cannot build or run torch.
- `import pysoarm` needs no camera, no codec, no network, no GPU.
- The whole control path is testable on a laptop with `FakeBus`; only the transport needs
  hardware.
- No version coupling to an external dataset schema.

**Negative / accepted costs**

- Users who want a one-command "record an episode" must use LeRobot (or ~50 lines of their
  own code). We accept the seam.
- Two processes must not hold the serial port at the same time: LeRobot recording and a
  pysoarm script cannot run concurrently on the same arm. Documented as a known trap.
- If we ever add recording, it starts as a separate package/adapter, not as a new module in
  the core import path.

**Explicit non-goals**

1. Teleoperation, master/slave modes, input devices.
2. Dataset recording, video encoding, dataset schema compatibility, Hub upload.
3. Policy inference, training, RL environments.
4. ROS/ROS2 nodes or drivers *inside this package*: the adapter lives in
   `ros2/pysoarm_ros2` (a separate ament package) and keeps all ROS-independent
   logic in `pysoarm.bridge`, which is covered by the main test suite.
5. Any claim of torque or impedance control on hardware that only has a position loop.

## Alternatives considered

| Option | Why not |
|---|---|
| Grow pysoarm into a LeRobot replacement | Duplicates an active upstream, inherits its schema churn, and multiplies our dependency and CI surface |
| Bundle a minimal recorder (npz/hdf5, no video) in core | Reasonable as a *separate* package (`pysoarm-collect`); in core it still adds filesystem/format policy to a driver |
| Add `pysoarm[teleop]` extra | Worth doing only when someone needs teleoperation *without* LeRobot; the seam is now documented, so it can be added without breaking the layer rule |

## References

- LeRobot dataset format v3.0 (upstream, evolves independently)
- `docs/architecture.md` — the layering rule this ADR formalises
- `SECURITY.md` (repo root) — treats "torque left enabled" and "watchdog not firing" as security issues
