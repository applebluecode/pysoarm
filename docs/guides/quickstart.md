# Quickstart

!!! note "Version 0.1.2"
    The snippets below describe the current API and are kept in sync with `arm.py`.

## Calibrate a fresh arm

```console
$ pysoarm calibrate --port /dev/ttyACM0 --role follower --id my_follower
$ pysoarm calibration show --port /dev/ttyACM0
$ pysoarm calibration export --port /dev/ttyACM0 --output my_follower.json
```

The flow disables torque, resets the motor-side calibration, sets half-turn homings
(`homing_offset = position - 2047`), records each joint's travel while you move it, then
writes the result back. `wrist_roll` is recorded over `0..4095` unless `--full-turn` is used.

## Leader (read-only) arm

```python
from pysoarm import SOArm

with SOArm("/dev/ttyACM1", role="leader", calibration="my_leader.json") as leader:
    state = leader.read_once()   # no goal is ever written, torque stays off
```

## Hold the current pose (safest first program)

```python
from pysoarm import SOArm

with SOArm(port="/dev/so101_follower", calibration="my_follower.json") as arm:
    state = arm.read_once()
    arm.write_goal(state.q_deg)   # goal == measured position: nothing moves
    input("holding — press Enter to release")
# exiting the block disables torque
```

## Read telemetry

```python
state = arm.read_once()
print(state.q_deg, state.temperature_c, state.voltage_v, state.t)
```

## Smooth move

```python
arm.move_to([0, -30, 60, 0, 90, 40], duration=2.0)   # degrees, model joint order
```

## Control loop

```python
def policy(state, t):
    return state.q_deg          # returns the goal for the next tick

arm.run_control_loop(policy, hz=200, duration=5.0)
```

Joint order (both SO-100 and SO-101): `shoulder_pan`, `shoulder_lift`, `elbow_flex`,
`wrist_flex`, `wrist_roll`, `gripper`.

## Reusing an existing LeRobot calibration

`pysoarm` reads LeRobot's JSON format directly (e.g.
`~/.cache/huggingface/lerobot/calibration/robots/so_follower/<id>.json`), so a calibrated
arm does not need to be calibrated again.
