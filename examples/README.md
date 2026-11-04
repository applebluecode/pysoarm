# Examples

Each file is a small, self-contained program. They are placeholders until the 0.1.0
implementation lands (see `CHANGELOG.md`); the docstring of each file states what it will
print/do so the intended usage is reviewable now.

| File | What it shows |
|---|---|
| `read_joints.py` | one snapshot: joint angles, temperatures, supply voltage |
| `hold_position.py` | enable torque and hold the current pose (the safest first move) |
| `move_to.py` | a smooth minimum-jerk move to a target pose |
| `control_loop.py` | 200 Hz loop, callback returns the next goal, overrun statistics |
| `replay_trajectory.py` | replay a recorded `(t, q)` table from a `.npz` file |
| `scan_bus.py` | find motor ids and model numbers on a port |

All examples take the port from `PYSOARM_HARDWARE_PORT` (default `/dev/ttyACM0`) and the
calibration file from `PYSOARM_CALIBRATION`.
