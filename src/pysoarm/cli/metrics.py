"""Latency summaries shared by ``bench`` and ``doctor --stress``."""

from __future__ import annotations

import math
import statistics

__all__ = ["latency_summary", "percentile"]


def percentile(values: list[float], fraction: float) -> float:
    """Nearest-rank percentile (``0.0`` for an empty list)."""
    if not values:
        return 0.0
    ordered = sorted(values)
    index = min(len(ordered) - 1, max(0, math.ceil(fraction * len(ordered)) - 1))
    return ordered[index]


def latency_summary(samples: list[float]) -> dict[str, float]:
    """Mean/p50/p95/max in milliseconds from second samples."""
    scale = 1e3
    return {
        "mean_ms": statistics.fmean(samples) * scale if samples else 0.0,
        "p50_ms": percentile(samples, 0.50) * scale,
        "p95_ms": percentile(samples, 0.95) * scale,
        "max_ms": max(samples, default=0.0) * scale,
    }
