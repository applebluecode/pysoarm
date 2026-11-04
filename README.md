# pysoarm

[![CI](https://github.com/applebluecode/pysoarm/actions/workflows/ci.yml/badge.svg)](https://github.com/applebluecode/pysoarm/actions/workflows/ci.yml)
[![PyPI](https://img.shields.io/pypi/v/pysoarm.svg)](https://pypi.org/project/pysoarm/)
[![Python](https://img.shields.io/pypi/pyversions/pysoarm.svg)](https://pypi.org/project/pysoarm/)
[![License](https://img.shields.io/badge/license-Apache--2.0-blue.svg)](LICENSE)
[![Typed](https://img.shields.io/badge/typing-mypy%20strict-informational.svg)](https://mypy.readthedocs.io/)

A **lean, hardware-level Python API** for the [SO-100 / SO-101](https://github.com/TheRobotStudio/SO-ARM100)
robot arm: six Feetech **STS3215** serial-bus servos on a single 1 Mbaud TTL bus.

`pysoarm` talks to the bus directly and deliberately stops there: no datasets, no policies,
no training stack. It targets the ergonomics of `pylibfranka` / `pyAgxArm`
(`connect()` → `read_once()` → `write_goal()` / control loop) within what position-servo
hardware can actually do.

> **Status: 0.1.0-dev.** Bus transport (SCS over `pyserial`), calibration, safety guards,
> trajectories, the `SOArm` device object and the CLI are implemented and covered by tests,
> including opt-in hardware tests on a real SO-101.

## Why

* **1.4 ms** per control step with the optional `rustypot` backend (0.98 ms to read six
  joints, 0.26 ms to write six goals) → a 200 Hz loop with **zero overruns**; 2.8 ms and
  ~2 % overruns with the pure-Python default.
* **Single small dependency** (`pyserial`) by default; `pip install "pysoarm[rustypot]"`
  adds the Rust backend (what LeRobot uses) behind the same `Bus` protocol.
* **Explicit safety layer**: per-step clipping, soft limits, temperature/voltage
  guards, communication watchdog — the parts that usually get skipped in quick scripts.
* **Reuses your existing calibration** (LeRobot-compatible JSON) instead of forcing
  a re-calibration.

## What it is not

The STS3215 is a position servo: there is no torque sensor, no force/torque feedback
and no hard real-time bus. Impedance/torque control à la Franka is **not** possible;
see [`docs/hardware.md`](docs/hardware.md) for the honest capability matrix.

### Non-goals (deliberate, see [ADR 0001](docs/adr/0001-scope-device-layer.md))

pysoarm is a **device-layer** SDK. A feature belongs here only if it is a physical fact of a
single device (register, scale, calibration, limit, timing, status bit). Everything that
needs a second device, an external data format or a learning stack stays outside:

| Not in pysoarm | Use instead |
|---|---|
| Teleoperation / leader-follower / input devices | ~10 lines of your own code (`follower.write_goal(leader.read_once().q_deg)`), or `lerobot-teleoperate` |
| Dataset recording (cameras, video encoding, parquet, Hub upload) | LeRobot (`lerobot-record`) |
| Dataset-format replay | `ResampledTrajectory` + your loader (any `(t, q)` table) |
| Policies, training, RL environments | LeRobot / your framework |
| ROS / ROS2 driver | a separate adapter package (not planned here) |
| Torque / impedance control | not available on position-servo hardware |

The reason is maintenance, not modesty: recording would pull in `av`, `pyarrow`, `opencv`
and `huggingface_hub`, and couple us to a dataset schema owned by upstream. The seam is
documented so the decision can be revisited without breaking the layer rule.

## Install

```bash
pip install pysoarm                # reference backend (pyserial)
pip install "pysoarm[rustypot]"    # + fast Rust backend
pip install "pysoarm[dev]"         # lint, types, tests, build
```

From source, for development ([uv](https://docs.astral.sh/uv/) is the supported workflow):

```bash
git clone https://github.com/applebluecode/pysoarm && cd pysoarm
make install-dev        # = uv sync --locked && uv run pre-commit install
```

Without uv (pip 25.1+, PEP 735 groups):

```bash
python -m venv .venv && . .venv/bin/activate
pip install --group dev -e .
pre-commit install
```

## Quickstart

```python
from pysoarm import SOArm  # noqa: F401  (API lands in 0.1.0)

arm = SOArm(port="/dev/so101_follower", calibration="my_follower.json")
arm.connect()
try:
    state = arm.read_once()
    print(state.q_deg)
finally:
    arm.stop()
```

## Repository layout

```
src/pysoarm/          # the importable package (src layout)
  arm.py              # SOArm: high-level device object (pylibfranka-like surface)
  state.py            # immutable ArmState snapshots
  calibration.py      # LeRobot-compatible calibration JSON + degrees <-> raw ticks
  safety.py           # limits, guards, watchdog
  trajectory.py       # interpolators / ramps for move_to()
  bus/                # transport backends behind one Protocol (base, scs, rustypot, fake)
  models/             # per-arm descriptions (joint names, motor ids, directions)
  kinematics/         # optional FK/IK (placo) on top of a SO-101 URDF
  cli/                # `pysoarm` command line entry point
  devices/udev/       # udev rule for stable /dev/so101_* names
tests/                # unit tests (hardware tests are opt-in)
examples/             # runnable examples
docs/                 # mkdocs site + docs/adr/ (scope and other decisions)
ros2/                 # separate ament_python package: pysoarm_ros2 node
.github/workflows/    # CI (lint + types + tests + build) and PyPI release
```

## ROS 2

`ros2/pysoarm_ros2` is a thin rclpy node over `pysoarm.bridge.ArmBridge`: it publishes
`joint_states` (radians), accepts `joint_commands` (radians) and exposes `enable` / `disable`
services. It stays a separate package so the device layer keeps zero ROS dependencies.

```bash
colcon build --packages-select pysoarm_ros2
ros2 run pysoarm_ros2 arm_node --ros-args -p port:=/dev/so101_follower -p backend:=rustypot
```

See [docs/guides/ros2.md](docs/guides/ros2.md) for parameters and safety behaviour.

## Cartesian control

Kinematics is optional (`pip install "pysoarm[kinematics]"`, needs your own URDF):

```python
from pysoarm.kinematics import PlacoKinematics

arm = SOArm("/dev/so101_follower", kinematics=PlacoKinematics("so101.urdf"))
print(arm.forward())  # current end-effector pose
arm.move_to_pose(target_pose, duration=2.0)  # IK, then a joint-space move
```

Unreachable targets raise `NotReachableError` **before** anything is written.

## Development

```bash
make install-dev   # uv sync --locked + pre-commit hooks
make check         # lint + type + test (what CI runs)
make lint          # ruff check + ruff format --check
make type          # mypy --strict
make test          # pytest
make cov           # pytest + coverage
make build         # sdist + wheel, then twine check --strict
make docs          # serve the mkdocs site locally
```

Everything runs through `uv run`, so you never activate an environment manually.

### Dependency management

* **`pyproject.toml` is the only source of published metadata** — `pip install pysoarm`
  never looks at anything else. Dev-only tooling lives in PEP 735 *dependency groups*
  (`dev`, `docs`, `hardware`), which are not part of the distribution.
* **`uv.lock` makes the dev environment reproducible** and is committed. CI runs with
  `UV_FROZEN=1` and `uv sync --locked`, so a stale lock fails the build instead of silently
  resolving different versions.
* The index is pinned in `pyproject.toml` (`[[tool.uv.index]]` → `pypi.org`) so the lock
  names the same artefacts for everyone. To resolve through a mirror locally:

  ```bash
  UV_DEFAULT_INDEX=https://pypi.tuna.tsinghua.edu.cn/simple uv lock
  ```

  …but do **not** commit that lock: it would pin mirror URLs for downstream contributors.
* `make upgrade` (`uv lock --upgrade`) is the way to bump dependencies; Dependabot also
  opens `uv.lock` PRs.

Hardware tests are marked `hardware` and skipped unless `PYSOARM_HARDWARE_PORT` is set:

```bash
PYSOARM_HARDWARE_PORT=/dev/ttyACM0 pytest -m hardware
```

## Publishing

1. bump `version` in `pyproject.toml` and `CHANGELOG.md`;
2. `git tag v0.1.0 && git push --tags`;
3. the `release.yml` workflow builds, runs `twine check`, and publishes to PyPI via
   **trusted publishing** (OIDC, no API token stored).

## License

Apache-2.0 — see [`LICENSE`](LICENSE).
