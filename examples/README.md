# Examples

Each file is a small, self-contained program. Every example accepts `--fake`, which swaps the
serial bus for `FakeBus`, so the whole set runs in CI and on a laptop without servos.

| File | What it shows |
|---|---|
| `read_joints.py` | one snapshot: joint angles, temperatures, supply voltage |
| `hold_position.py` | enable torque and hold the current pose (the safest first move) |
| `move_to.py` | a smooth minimum-jerk move to a target pose |
| `control_loop.py` | 200 Hz loop, callback returns the next goal, latency/overrun statistics |
| `replay_trajectory.py` | replay a recorded `(t, q)` table from a `.npz` file |
| `scan_bus.py` | find motor ids and model numbers on a port |

Hardware runs take the port from `PYSOARM_HARDWARE_PORT` (default `/dev/ttyACM0`) and the
calibration file from `PYSOARM_CALIBRATION`:

```bash
python examples/read_joints.py --fake
python examples/move_to.py --port /dev/ttyACM0 --calibration follower.json
```

`_common.py` is example-only glue (argument defaults plus fake-bus setup); it is not part of
the installed package.
