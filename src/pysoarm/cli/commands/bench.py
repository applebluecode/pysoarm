"""The ``bench`` command: read/write latency and loop overruns."""

from __future__ import annotations

import argparse
import json
import time

from pysoarm.cli import factories
from pysoarm.cli.commands import print_error
from pysoarm.cli.constants import EXIT_OK, EXIT_USAGE
from pysoarm.cli.metrics import latency_summary
from pysoarm.models.base import get_model

__all__ = ["_cmd_bench"]


def _cmd_bench(args: argparse.Namespace) -> int:
    if args.seconds <= 0 or args.hz <= 0 or args.samples <= 0:
        print_error("--seconds, --hz and --samples must be positive")
        return EXIT_USAGE
    model = get_model(args.model)
    ids = model.motor_ids
    bus = factories.make_bus(args)
    reads: list[float] = []
    writes: list[float] = []
    with bus:
        positions = bus.sync_read(ids, "present_position")
        for _ in range(args.samples):
            started = time.perf_counter()
            positions = bus.sync_read(ids, "present_position")
            reads.append(time.perf_counter() - started)
            started = time.perf_counter()
            bus.sync_write("goal_position", {mid: positions[mid] for mid in ids})
            writes.append(time.perf_counter() - started)

        period = 1.0 / args.hz
        started = time.perf_counter()
        ticks = 0
        overruns = 0
        while True:
            elapsed = time.perf_counter() - started
            if elapsed >= args.seconds:
                break
            positions = bus.sync_read(ids, "present_position")
            bus.sync_write("goal_position", positions)
            deadline = started + (ticks + 1) * period
            slack = deadline - time.perf_counter()
            if slack > 0:
                time.sleep(slack)
            else:
                overruns += 1
            ticks += 1
        loop_elapsed = time.perf_counter() - started

    report = {
        "port": args.port,
        "backend": bus.backend,
        "read": latency_summary(reads),
        "write": latency_summary(writes),
        "loop": {
            "hz": args.hz,
            "ticks": ticks,
            "overruns": overruns,
            "elapsed_s": loop_elapsed,
            "mean_hz": ticks / loop_elapsed if loop_elapsed > 0 else 0.0,
        },
    }
    if args.json:
        print(json.dumps(report, indent=2))
    else:
        read, write, loop = report["read"], report["write"], report["loop"]
        print(f"backend {report['backend']}")
        print(
            f"read6   mean {read['mean_ms']:.2f} ms  p50 {read['p50_ms']:.2f}  "
            f"p95 {read['p95_ms']:.2f}  max {read['max_ms']:.2f}"
        )
        print(
            f"write6  mean {write['mean_ms']:.2f} ms  p50 {write['p50_ms']:.2f}  "
            f"p95 {write['p95_ms']:.2f}  max {write['max_ms']:.2f}"
        )
        print(
            f"loop    {loop['hz']:.0f} Hz target, {loop['mean_hz']:.1f} Hz actual, "
            f"{loop['ticks']} ticks, {loop['overruns']} overruns"
        )
    return EXIT_OK
