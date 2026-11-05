"""Replay a recorded trajectory stored as ``npz(t=..., q_deg=...)``.

The table is validated (strictly increasing time, consistent width) and then resampled by
the control loop at ``--hz`` using :class:`~pysoarm.trajectory.ResampledTrajectory`.
Without ``--file`` and with ``--fake`` a small synthetic trajectory is used, so the example
runs offline:

    python examples/replay_trajectory.py --fake
    python examples/replay_trajectory.py --file demo.npz --port /dev/ttyACM0
"""

from __future__ import annotations

import argparse
import math

from _common import add_connection_args, open_arm
from pysoarm.trajectory import ResampledTrajectory


def synthetic() -> ResampledTrajectory:
    """A 1 s, six-joint, gentle sine wave (fake-mode demo)."""
    times = tuple(index / 100 for index in range(101))
    positions = tuple(tuple(10.0 * math.sin(2 * math.pi * t) for _ in range(6)) for t in times)
    return ResampledTrajectory(times=times, positions=positions)


def load(path: str) -> ResampledTrajectory:
    """Load ``npz(t, q_deg)`` without assuming a dataset format."""
    try:
        import numpy as np  # noqa: PLC0415 - optional dependency, only for --file
    except ImportError as exc:  # pragma: no cover - depends on the environment
        raise SystemExit("numpy is needed for --file: pip install numpy") from exc
    with np.load(path) as data:
        return ResampledTrajectory(
            times=tuple(float(value) for value in data["t"]),
            positions=tuple(tuple(float(value) for value in row) for row in data["q_deg"]),
        )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    add_connection_args(parser)
    parser.add_argument("--file", default=None, help="npz with arrays t and q_deg")
    parser.add_argument("--hz", type=float, default=200.0)
    args = parser.parse_args()
    if args.file is None and not args.fake:
        parser.error("pass --file, or --fake to replay the synthetic trajectory")
    trajectory = synthetic() if args.file is None else load(args.file)
    arm = open_arm(args)

    def replay(state: object, elapsed: float) -> tuple[float, ...]:
        del state
        return trajectory.sample(elapsed)

    try:
        stats = arm.run_control_loop(
            replay,
            hz=args.hz,
            duration=trajectory.duration,  # type: ignore[arg-type]
        )
        print(stats.as_dict())
    finally:
        arm.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
