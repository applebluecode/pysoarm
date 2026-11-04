# API reference

Complete reference for the public API of `pysoarm` **0.1.0**. Every symbol documented here is
importable from the top-level package:

```python
from pysoarm import SOArm, ArmState, Calibration, SafetyLimits, Pose
```

The reference sections are generated from the docstrings by
[mkdocstrings](https://mkdocstrings.github.io/), so this page and the code cannot drift apart.
Start with [Conventions](#conventions) and [Public API at a glance](#public-api-at-a-glance);
task-oriented examples live in the [Quickstart](../guides/quickstart.md).

---

## Conventions

The rules below hold across the whole API. Read them once — the rest of the reference assumes
them.

### Units

| Quantity | Unit | Where |
|---|---|---|
| Joint positions, goals, limits, step guard | **degrees** | `SOArm`, `ArmState.q_deg`, `Calibration`, `SafetyLimits` |
| Joint positions, ROS bridge input | radians | only `ArmBridge.command_from_radians` |
| Cartesian position | metres | `Pose.x/y/z` |
| Orientation | unit quaternion `(w, x, y, z)` | `Pose.qw/qx/qy/qz` |
| Euler angles, `Pose.angle_to` | degrees (radians for `angle_to`) | `Pose.from_xyz_rpy`, `Pose.to_xyz_rpy` |
| Time, timestamps, durations | seconds | `ArmState.t`, `duration=`, `hz=` in Hz |
| Temperature, voltage | °C, V | `ArmState.temperature_c`, `ArmState.voltage_v` |

Raw servo quantities are **not** SI and are never silently converted: `ArmState.speed_raw` and
`ArmState.current` are `present_speed` / `present_current` register counts. Use
`Register.to_unit()` when a physical unit matters.

### Joint order

Every joint tuple and every sequence argument uses the model's joint order, which is the same
for SO-100 and SO-101:

| Index | 0 | 1 | 2 | 3 | 4 | 5 |
|---|---|---|---|---|---|---|
| Joint | `shoulder_pan` | `shoulder_lift` | `elbow_flex` | `wrist_flex` | `wrist_roll` | `gripper` |
| Motor id | 1 | 2 | 3 | 4 | 5 | 6 |

Pass a different [`RobotModel`](#pysoarm.models.base.RobotModel) to change order, ids or count.
Sequences that are too short, too long, non-finite, or not in this order raise
`ConfigurationError` before anything is sent to the bus.

### Values are immutable

`ArmState`, `Pose`, `SafetyReport`, `JointStateFrame`, `LoopStats`, `Register`,
`JointCalibration` and `RobotModel` are frozen dataclasses (`JointStateFrame`, `LoopStats` and
`SafetyReport` are plain frozen containers): they are snapshots, never mutated in place. The
only stateful objects are `SOArm`, `ArmBridge`, the bus transports and `Calibration`.

### Nothing is clipped silently

`SOArm.write_goal()` and `ArmBridge.command()` return the goal that was **actually written**.
Guards clip or derate, and the difference is visible in `SafetyReport` and in the returned
tuple. `SOArm.move_to()` returns the state measured after the move.

### Lifetime and threading

`SOArm` owns its bus: construct it, `connect()` (or use `with`), use it, `stop()`.
`stop()` is idempotent and never raises, so it is safe in a `finally` block.

```python
from pysoarm import SOArm

with SOArm(port="/dev/so101_follower", calibration="follower.json") as arm:
    arm.hold()
# leaving the block disables torque and closes the port
```

Instances are **not** thread-safe: one owner thread per arm. The serial port is exclusive
(`BusConnectionError` if another process holds it). Callbacks passed to
`SOArm.run_control_loop()` run on the loop thread and must not block.

### Clock and injection seams

Construction takes `clock=` (defaults to `time.monotonic`), `sleep=` and — where a dependency
is optional — an explicit module seam (`placo_module`, `rustypot_module`, `serial_factory`).
Tests use these to run the real code paths with `FakeBus` and no hardware.

---

## Public API at a glance

| Symbol | Kind | Purpose |
|---|---|---|
| [`SOArm`](#pysoarm.arm.SOArm) | class | The arm: connect, read, goal, move, control loop |
| [`LoopStats`](#pysoarm.arm.LoopStats) | dataclass | Tick count, overruns and elapsed time of a control loop |
| [`ArmState`](#pysoarm.state.ArmState) | dataclass | One consistent snapshot (positions + health) |
| [`Calibration`](#pysoarm.calibration.Calibration) | class | Joint travel, homing; LeRobot JSON interop |
| [`JointCalibration`](#pysoarm.calibration.JointCalibration) | dataclass | Per-joint `motor_id`, travel, offset, direction |
| [`SafetyLimits`](#pysoarm.safety.SafetyLimits) | dataclass | Guard thresholds |
| [`SafetyReport`](#pysoarm.safety.SafetyReport) | dataclass | What the guards did to one goal |
| [`apply_guards`](#pysoarm.safety.apply_guards) | function | Run every guard on a goal without an arm |
| [`Trajectory`](#pysoarm.trajectory.Trajectory) | protocol | `sample(t) -> joint tuple` |
| [`MinimumJerk`](#pysoarm.trajectory.MinimumJerk) | class | Default motion profile of `move_to` |
| [`LinearRamp`](#pysoarm.trajectory.LinearRamp) | class | Constant-velocity profile |
| [`ResampledTrajectory`](#pysoarm.trajectory.ResampledTrajectory) | class | Piecewise-linear replay of a recorded table |
| [`ArmBridge`](#pysoarm.bridge.ArmBridge) | class | Framework-agnostic command/state bridge (ROS 2) |
| [`JointStateFrame`](#pysoarm.bridge.JointStateFrame) | dataclass | Bridge-level snapshot (names + SI units) |
| [`Bus`](#pysoarm.bus.base.Bus) | protocol | The only transport interface control code sees |
| [`ScsBus`](#pysoarm.bus.scs.ScsBus) | class | Reference SCS-protocol transport on `pyserial` |
| [`RustypotBus`](#pysoarm.bus.rustypot.RustypotBus) | class | Optional Rust transport (faster reads) |
| [`FakeBus`](#pysoarm.bus.fake.FakeBus) | class | Deterministic in-memory bus for tests |
| [`make_bus`](#pysoarm.bus.make_bus) | function | Build a transport for a port and backend |
| `Backend` | type alias | `"scs"`, `"rustypot"` or `"auto"` |
| `REGISTERS` / [`Register`](#pysoarm.bus.registers.Register) / [`get_register`](#pysoarm.bus.registers.get_register) | mapping/class/function | STS3215 memory table and codecs |
| [`RegisterKind`](#pysoarm.bus.registers.RegisterKind) | enum | `unsigned`, `sign_magnitude`, … |
| [`RobotModel`](#pysoarm.models.base.RobotModel) / `MODELS` / [`get_model`](#pysoarm.models.base.get_model) | dataclass/mapping/function | Joints, ids, gripper caps, baudrate |
| `SO100` / `SO101` | constants | Ready-made models |
| [`Pose`](#pysoarm.kinematics.base.Pose) | dataclass | End-effector pose (metres + quaternion) |
| [`Kinematics`](#pysoarm.kinematics.base.Kinematics) | protocol | `forward()` / `inverse()` |
| [`PlacoKinematics`](#pysoarm.kinematics.placo_solver.PlacoKinematics) | class | placo-backed FK/IK on your URDF |
| [`PysoarmError`](#pysoarm.exceptions.PysoarmError) | exception | Base of every error this package raises |
| `DEFAULT_BAUDRATE`, `DEFAULT_LOOP_HZ` | constants | `1_000_000`, `200.0` |

---

## Device layer: `pysoarm.arm`

### The lifecycle

```
construct ──► connect() ──► read_once()/write_goal()/move_to() ──► stop()
              │             └── run_control_loop() (owns the loop)
              └── open bus → configure motors (torque off) → hold measured pose → torque on
```

`connect()` is deliberately conservative: torque stays off while the motors are configured,
the measured pose is written back as the first goal, and only then is torque enabled — the arm
cannot jump on connect. The steps are idempotent where possible; calling `connect()` twice is
harmless, and `stop()` may be called at any time in any state.

### Typical calls

```python
from pysoarm import SOArm

arm = SOArm(port="/dev/so101_follower", calibration="follower.json")
arm.connect()

state = arm.read_once()          # q_deg, temperature_c, voltage_v, t, ...
arm.hold()                       # re-read and command the measured pose

goal = arm.write_goal([0, -30, 60, 0, 90, 40])   # returns what was written
arm.move_to([0, -20, 60, 0, 90, 40], duration=2.0, profile="minjerk")

def policy(state, elapsed_s):
    return state.q_deg           # None means "hold, send nothing"

stats = arm.run_control_loop(policy, hz=200, duration=5.0)
print(stats.ticks, stats.overruns)
arm.stop()
```

### Reading states

| `health=` | Registers refreshed | Transactions per tick |
|---|---|---|
| `False` | none (cache reused) | 1 sync read — the hot path |
| `True` | all health registers | 1 sync read + 1 read |
| `None` (default) | one health register per tick, round-robin, once the cache is older than `health_period_s` | 1 sync read (+1 periodically) |

That amortisation is what keeps a 200 Hz loop inside its 5 ms budget while temperature and
voltage stay fresh enough for the guards.

::: pysoarm.arm
    options:
      members_order: source
      filters: ["!^_"]

## State: `pysoarm.state`

::: pysoarm.state

## Calibration: `pysoarm.calibration`

Calibration maps raw ticks to degrees and defines each joint's usable travel. It comes from
one of three places:

1. a `Calibration` object you build,
2. a LeRobot JSON file (`.from_lerobot(path)` / `to_lerobot(path)` — pysoarm reads LeRobot's
   format directly, so a calibrated arm does not need calibrating again),
3. the motors themselves (`calibration=None` at construction reads the stored calibration).

`validate(model)` is called by `connect()`: missing joints, extra joints or duplicated motor
ids raise `CalibrationError` **before** any motor is commanded.

::: pysoarm.calibration

## Safety: `pysoarm.safety`

Every goal passes the same guard chain in `write_goal()` — there is no bypass flag:

| Order | Guard | Threshold | On trip |
|---|---|---|---|
| 1 | Finiteness | — | `SafetyViolation` |
| 2 | Watchdog | `max_consecutive_failures` | `WatchdogTimeout` (torque already off) |
| 3 | Temperature | `temp_max_c` | `SafetyViolation` |
| 4 | Voltage | `voltage_min_v` | `SafetyViolation` |
| 5 | Calibrated limits | `limit_margin_deg` | goal clamped inward, `clipped=True` |
| 6 | Step size | `max_step_deg` (scaled by the derate factor) | goal clipped to `state.q_deg ± max_step` |
| 7 | Derating | `temp_warn_c` | `SafetyLimits.derate()` returns a factor in `(0.25, 1.0]` from the hottest motor, which shrinks the allowed step |

`SafetyLimits()` defaults are the reference values used on the reference arm:

| Field | Default | Meaning |
|---|---|---|
| `max_step_deg` | `5.0` | Largest change of a joint between two goals |
| `limit_margin_deg` | `1.0` | Margin kept inside the calibrated travel |
| `temp_warn_c` | `55.0` | Above this, goals are derated |
| `temp_max_c` | `70.0` | At this, goals are refused |
| `voltage_min_v` | `10.0` | Below this, goals are refused |
| `max_consecutive_failures` | `3` | Read failures before the watchdog trips |

The contract on tripping is part of the API: `WatchdogTimeout` leaves torque **off**, and
`SafetyViolation`/`CalibrationError`/`NotReachableError` are raised **before** a goal is
written, so the arm keeps its last safe target. Temperature is judged on the hottest motor
(`ArmState.max_temperature_c`) and voltage on the lowest one (`ArmState.min_voltage_v`).

::: pysoarm.safety

## Trajectories: `pysoarm.trajectory`

A trajectory is anything with `sample(t) -> tuple[float, ...]`. `move_to(profile=...)` selects
one of the built-ins:

| `profile=` | Class | Shape |
|---|---|---|
| `"minjerk"` (default) | `MinimumJerk` | quintic `10s³ − 15s⁴ + 6s⁵`, zero velocity/acceleration at both ends |
| `"linear"` | `LinearRamp` | constant velocity, discontinuous acceleration |

`ResampledTrajectory` is the replay primitive: it takes any time-stamped joint table (`.npz`,
CSV, LeRobot parquet — your loader, your choice) and interpolates it. No dataset format is
assumed or produced.

::: pysoarm.trajectory

## Bridge: `pysoarm.bridge`

A thin, framework-agnostic wrapper for callers that use their own control loop (ROS 2 nodes,
servers): names + radians in, commands out, with a staleness check on the cached state.
`ArmBridge` never calls `connect()`/`stop()`; the owner controls the lifetime.

::: pysoarm.bridge

## Bus transports: `pysoarm.bus`

The rest of the package only ever sees [`Bus`](#pysoarm.bus.base.Bus), so transports are
swappable without touching control logic.

| `backend=` | Implementation | Notes |
|---|---|---|
| `"scs"` (default) | `ScsBus` | SCS protocol on `pyserial`, the only hard dependency; ~2.2 ms to read six joints |
| `"rustypot"` | `RustypotBus` | Optional Rust driver (the one LeRobot uses), ~1.0 ms; needs the `rustypot` extra |
| `"auto"` | — | `rustypot` when importable, `scs` otherwise |
| `"fake"` | `FakeBus` | Deterministic in-memory bus; injected directly, not via `make_bus` |

::: pysoarm.bus.base

::: pysoarm.bus.make_bus

### Register table

`REGISTERS` is the STS3215 memory table with the protocol details each access needs
(`kind` selects the codec, `scale`/`unit` the conversion). Addresses and sizes are from the
datasheet; `get_register(name)` resolves a name case-insensitively.

| Name | Addr | Size | Kind | Unit | Access |
|---|---|---|---|---|---|
| `model_number` | 3 | 2 | unsigned | — | ro |
| `id` | 5 | 1 | unsigned | — | rw |
| `baudrate` | 6 | 1 | unsigned | — | rw |
| `return_delay_time` | 7 | 1 | unsigned | — | rw |
| `min_position_limit` | 9 | 2 | unsigned | — | rw |
| `max_position_limit` | 11 | 2 | unsigned | — | rw |
| `max_torque_limit` | 16 | 2 | unsigned | — | rw |
| `phase` | 18 | 1 | unsigned | — | rw |
| `p_coefficient` | 21 | 1 | unsigned | — | rw |
| `d_coefficient` | 22 | 1 | unsigned | — | rw |
| `i_coefficient` | 23 | 1 | unsigned | — | rw |
| `protection_current` | 28 | 2 | unsigned | — | rw |
| `homing_offset` | 31 | 2 | sign_magnitude | — | rw |
| `operating_mode` | 33 | 1 | unsigned | — | rw |
| `overload_torque` | 36 | 1 | unsigned | — | rw |
| `torque_enable` | 40 | 1 | unsigned | — | rw |
| `acceleration` | 41 | 1 | unsigned | — | rw |
| `goal_position` | 42 | 2 | sign_magnitude | — | rw |
| `goal_time` | 44 | 2 | unsigned | — | rw |
| `goal_speed` | 46 | 2 | sign_magnitude | — | rw |
| `torque_limit` | 48 | 2 | unsigned | — | rw |
| `present_position` | 56 | 2 | sign_magnitude | — | ro |
| `present_speed` | 58 | 2 | sign_magnitude | — | ro |
| `present_load` | 60 | 2 | sign_magnitude | — | ro |
| `present_voltage` | 62 | 1 | unsigned | V | ro |
| `present_temperature` | 63 | 1 | unsigned | C | ro |
| `present_current` | 69 | 2 | unsigned | — | ro |
| `maximum_acceleration` | 85 | 1 | unsigned | — | rw |

::: pysoarm.bus.registers

::: pysoarm.bus.scs

::: pysoarm.bus.rustypot

::: pysoarm.bus.fake

## Models: `pysoarm.models`

`MODELS` maps the accepted `model=` strings (`so101`, `so101_follower`, `so101_leader`,
`so100`, `so100_follower`, `so100_leader`, `so_follower`, `so_leader` — case-insensitive) to a
`RobotModel`; `SO100` and `SO101` are the same values as ready-made constants.

::: pysoarm.models.base

::: pysoarm.models.so101

## Kinematics: `pysoarm.kinematics`

Kinematics is optional and out of the control loop: the position loop works in joint space,
and kinematics only converts to and from Cartesian goals. A SO-101 URDF is **not** shipped
(licensing and the accuracy of community meshes vary) — pass your own.

```python
from pysoarm import SOArm
from pysoarm.kinematics import PlacoKinematics

arm = SOArm(
    port="/dev/so101_follower",
    calibration="follower.json",
    kinematics=PlacoKinematics("so101.urdf", target_frame="gripper_frame_link"),
)
with arm:
    pose = arm.forward()                     # FK of the current joints
    arm.move_to_pose(pose, duration=2.0)     # IK, then a joint-space move
```

`import pysoarm` stays dependency-free: the placo adapter is imported lazily on first use, and
a missing dependency raises `KinematicsError` with the install command. `move_to_pose()` solves
IK **before** writing any goal, so an unreachable target raises `NotReachableError` and leaves
the arm where it was.

::: pysoarm.kinematics.base

::: pysoarm.kinematics.placo_solver

## Constants

| Constant | Value | Notes |
|---|---|---|
| `DEFAULT_BAUDRATE` | `1_000_000` | STS3215 factory default, same as LeRobot |
| `DEFAULT_LOOP_HZ` | `200.0` | Default control rate |
| `DEFAULT_TIMEOUT_S` | `0.05` | Per-transaction timeout (~0.3 ms measured) |
| `DEFAULT_RETRIES` | `2` | Extra attempts after the first |
| `BROADCAST_ID` | `0xFE` | Bus-wide id used by SYNC WRITE |
| `TICKS_PER_TURN` / `RESOLUTION_MAX` | `4096` / `4095` | Encoder resolution |
| `DEG_PER_TICK` | `≈0.0879` | Smallest representable joint step |
| `VOLTAGE_LSB_V` | `0.1` | `present_voltage` unit |
| `DEFAULT_HEALTH_PERIOD_S` | `0.5` | Health-refresh interval of `read_once()` |
| `DEFAULT_P_COEFFICIENT` / `DEFAULT_I_COEFFICIENT` / `DEFAULT_D_COEFFICIENT` | `16` / `0` / `32` | PID written at connect (LeRobot defaults) |

## Command line: `pysoarm.cli`

Installed as the `pysoarm` script:

```console
$ pysoarm --help
$ pysoarm scan  --port /dev/so101_follower
$ pysoarm read  --port /dev/so101_follower --calibration follower.json
$ pysoarm hold  --port /dev/so101_follower --calibration follower.json
$ pysoarm monitor --port /dev/so101_follower --hz 5
$ pysoarm pose  --urdf so101.urdf --joints 0 -30 60 0 90 40
$ pysoarm ik    --urdf so101.urdf --x 0.2 --y 0 --z 0.15     # dry run, no hardware
$ pysoarm doctor --port /dev/so101_follower
```

Exit codes: `0` success, `1` runtime error (bus, communication, safety), `2` usage error.

::: pysoarm.cli.main

## Exceptions: `pysoarm.exceptions`

Every error derives from `PysoarmError`, so one `except` can catch this package without
swallowing unrelated `OSError`/`ValueError`:

```
PysoarmError
├── ConfigurationError        bad arguments, unknown joint/register, bad calibration
├── BusConnectionError        port unavailable or held by another process
├── CommunicationError        no reply / corrupt packet; arm holds its last goal
│                             (.motor_id, .register, .raw)
├── CalibrationError          calibration missing, incomplete or mismatched
├── SafetyViolation           non-finite goal, over-temperature, undervoltage
│   └── WatchdogTimeout       too many consecutive read failures; torque is off
└── KinematicsError           kinematics unavailable or failed
    └── NotReachableError     IK did not converge; no goal was written
```

The recovery contract is part of the type: `WatchdogTimeout` means torque is **already off**,
and the errors raised during a move mean **no goal was written**, so a `try/except` around a
move leaves the arm in a defined state.

::: pysoarm.exceptions
