# API reference

Complete reference for the public API of `pysoarm` **0.1.3**. Every symbol documented here is
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
cannot jump on connect. A transient bus failure (no reply, checksum, alarm bit) is retried as
a whole sequence with a short backoff (`connect(retries=2, backoff_s=0.25)`), which covers the
serial-reopen race seen in containers; the final error names the attempt count and the
`pysoarm doctor` command. Non-transient errors (wrong model, missing calibration) are never
retried.

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

### Gripper, roles and tuning

`arm.gripper` is a percent-of-travel facade over the sixth joint (`0` = closed end of the
calibrated range, `100` = open end):

```python
arm.gripper.close(wait=True)     # timed trajectory (default duration 0.5 s)
arm.gripper.open(wait=True)
arm.gripper.move(40, wait=True)  # 40 % open
arm.gripper.close()              # one command; immediate with the default limits
arm.gripper.percent()            # current opening, 0..100
```

Without `wait=True` the target is written once. With the default limits there is no step cap,
so the servo heads straight there; if you configure `max_speed_deg_s`/`max_step_deg`, the goal
is clipped to one step and the jaws may need several calls to arrive. `wait=True` runs a timed
trajectory, which is the smooth option regardless of limits. `homing()` redefines the closed end at the current position; it
does **not** move the jaws.

`role="leader"` makes the whole object read-only (no goal writes, torque stays off). Tuning
registers are exposed per joint, including the servo's own velocity ceiling:

```python
arm.motor_config("gripper")                              # dict of registers
arm.set_motor_config("shoulder_pan", goal_speed=1000)    # 0 = servo-unlimited (default)
```

`goal_speed` (register 46) is the servo's internal limit and is independent of the software
`step_for(hz)` guard and of the acceleration ramp; leaving it at `0` keeps the
LeRobot-compatible configuration.

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

The motor-side flow is public in `pysoarm.calibration`:

| Function | Purpose |
|---|---|
| `read_calibration(bus, model)` | read limits + homing offsets from the servos |
| `write_calibration(bus, model, cal)` | persist a calibration to the servos |
| `reset_calibration(bus, model)` | clear offsets, widen travel to `0..4095` |
| `set_half_turn_homings(bus, model)` | Feetech `offset = position - 2047` |
| `record_ranges(bus, model, stop=...)` | poll travel while the operator moves the arm |
| `calibrate(bus, model, role=..., middle=True, wait_middle=...)` | the full flow above |

`calibrate()` runs torque off → reset → **move to mid-travel** → half-turn homing → record
travel → write. The middle stage is on by default to match LeRobot's SO calibration: `2048`
then means the mechanical middle, and `wait_middle` is the injection point for an interactive
console or a headless fixture (it must only block, not read stdin). `middle=False` reproduces
the 0.1.2 behaviour (homing at the current pose); `pysoarm calibrate --no-middle` exposes the
same switch. Do not mix the two conventions across arms that are later teleoperated together:
their `q_deg` zero would differ. See [Homing and calibration offsets](../hardware.md#homing-and-calibration-offsets)
for the register semantics.

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
| 6 | Step size (**opt-in**) | `step_for(hz) = min(max_step_deg, max_speed_deg_s / hz)` (scaled by the derate factor) | goal clipped to `state.q_deg ± step`; no clipping when both limits are `None` |
| 7 | Deadband | `min_step_deg` | command smaller than the deadband is not sent; measured position kept, reported in `reasons` |
| 8 | Derating | `temp_warn_c` | `SafetyLimits.derate()` returns a factor in `(0.25, 1.0]` from the hottest motor, which shrinks a configured step limit |

`SafetyLimits()` defaults put **no software cap on the per-tick step or joint speed**, matching
LeRobot's SO follower (`max_relative_target=None`). Each guard-passing goal is sent as
requested; the calibrated soft limits, thermal/voltage guards, watchdog and the servo itself
still apply:

| Field | Default | Meaning |
|---|---|---|
| `max_step_deg` | `None` | No fixed per-tick cap. Set e.g. `5.0` for a fixed absolute limit |
| `max_speed_deg_s` | `None` | No speed cap. Set `720.0` for the recommended teleoperation preset (14.4°/tick @ 50 Hz, 3.6° @ 200 Hz) |
| `min_step_deg` | `0.0` | Commands below this are not sent (servo deadband); `0` keeps the historical behaviour |
| `limit_margin_deg` | `1.0` | Margin kept inside the calibrated travel |
| `temp_warn_c` | `55.0` | Above this, goals are derated |
| `temp_max_c` | `70.0` | At this, goals are refused |
| `voltage_min_v` | `10.0` follower / `4.0` leader | Below this, goals are refused; `SOArm(limits=None)` picks it from `role` via `SafetyLimits.for_role()` |
| `max_consecutive_failures` | `3` | Read failures before the watchdog trips |

Choosing a software limit is opt-in:

```python
SafetyLimits()                       # no speed/step cap (LeRobot parity, default)
SafetyLimits(max_speed_deg_s=720)    # 720 deg/s at any loop rate
SafetyLimits(max_step_deg=5.0)       # fixed 5 deg/tick at any loop rate
SafetyLimits(max_step_deg=5.0, max_speed_deg_s=720)  # the stricter one wins
```

When both are set, `step_for(hz)` returns the stricter value. `SOArm.set_speed_scale(s)` scales
a configured limit; with the default unbounded limits there is nothing to scale, so configure
`max_speed_deg_s`/`max_step_deg` first if you want to use it.

The contract on tripping is part of the API: `WatchdogTimeout` leaves torque **off**, and
`SafetyViolation`/`CalibrationError`/`NotReachableError` are raised **before** a goal is
written, so the arm keeps its last safe target. Temperature is judged on the hottest motor
(`ArmState.max_temperature_c`) and voltage on the lowest one (`ArmState.min_voltage_v`).

### Failure thresholds and their precedence

A teleoperation loop has two independent cutoffs. The **arm watchdog** sees bus failures and
trips first; the **loop cutoff** is the second line of defence for errors the watchdog cannot
see (a leader read inside the callback, a user-code bug):

| Layer | Setting | Default | Effect when hit |
|---|---|---|---|
| Arm watchdog | `SafetyLimits.max_consecutive_failures` | `3` | `WatchdogTimeout`, torque disabled |
| Loop cutoff | `run_control_loop(max_consecutive_errors=...)` | `10` | re-raises the original error |
| Loop policy | `run_control_loop(on_error=...)` | `"abort"` | `"hold"`/`"skip"` count the tick and continue |

`LoopStats` attributes every handled failure to its source: `read_errors`,
`callback_errors` and `write_errors` (their sum is `errors`). In teleoperation a failed
*leader* read inside the callback is a `callback_error`, not a follower `read_error`.

### Precision and the smallest useful step

One encoder tick is `360 / 4095 = 0.0879°`. The servo's positioning deadband is larger than
that: on the reference arm, a stream of 5°/tick commands converged to a residual of
**0.22–0.84°**, and `max_step_deg=0.5` moved only ~0.18° in 0.3 s because the tiny steps fell
inside the deadband. Practical guidance:

* treat **≥ 1°** as the repeatability floor for goals and tolerance checks;
* do not try to close a sub-degree error with repeated `write_goal()` calls — it will not move;
* if you must suppress deadband-sized commands, set `SafetyLimits.min_step_deg` (default `0`,
  so behaviour is unchanged) and inspect `SafetyReport.reasons`;
* a leader arm is limp by design, so its pose **drifts** between sessions (gravity); the
  follower faithfully follows that drift. Re-zero or hold the leader when this matters.

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
$ pysoarm pose  --urdf so101.urdf --q "0,-30,60,0,90,40"
$ pysoarm ik    --urdf so101.urdf --xyz 0.2 0 0.15           # dry run, no hardware
$ pysoarm calibrate   --port /dev/ttyACM0 --role follower --id my_follower
$ pysoarm calibration show   --port /dev/ttyACM0
$ pysoarm calibration export --port /dev/ttyACM0 --output my_follower.json
$ pysoarm bench   --port /dev/ttyACM0 --seconds 5
$ pysoarm doctor  --port /dev/so101_follower
$ pysoarm doctor  --port /dev/ttyACM0 --stress 3             # electrical + status self-check
$ pysoarm doctor  --port /dev/ttyACM0 --role leader --json   # 5 V thresholds, machine readable
```

The JSON report contains `port`, `role`, `min_voltage_v`, `backend`, `checks`,
`warnings`, `timeouts`, `status_alarms`, `alarm_counts`, `latency` and `failed_latency`.
Timeouts (no reply) and status alarms (a reply with an `ERROR` bit) are counted separately;
`--min-voltage` defaults to 10.0 V for a follower and 4.0 V for a leader.

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
