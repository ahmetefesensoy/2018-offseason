"""Robust bounding-box depth sampling and pinhole projection."""

from __future__ import annotations

from dataclasses import dataclass
import math
import statistics


@dataclass(frozen=True)
class BoundingBox:
    min_x: int
    min_y: int
    max_x: int
    max_y: int


@dataclass(frozen=True)
class ProjectedPoint:
    x: float
    y: float
    z: float
    variance: float


def project_detection(
    box: BoundingBox,
    depth,
    *,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
    depth_scale: float,
    min_depth_m: float = 0.15,
    max_depth_m: float = 8.0,
) -> ProjectedPoint:
    if fx <= 0.0 or fy <= 0.0 or depth_scale <= 0.0:
        raise ValueError("camera intrinsics and depth scale must be positive")
    height = len(depth); width = len(depth[0]) if height else 0
    x0 = max(0, min(width, box.min_x)); x1 = max(x0, min(width, box.max_x))
    y0 = max(0, min(height, box.min_y)); y1 = max(y0, min(height, box.max_y))
    values = []
    for row in range(y0, y1):
        for column in range(x0, x1):
            value = float(depth[row][column]) * depth_scale
            if math.isfinite(value) and min_depth_m <= value <= max_depth_m:
                values.append(value)
    if not values:
        raise ValueError("detection has no valid depth samples")
    z = statistics.median(values)
    u = (box.min_x + box.max_x) / 2.0
    v = (box.min_y + box.max_y) / 2.0
    spread = statistics.median(abs(value - z) for value in values)
    variance = max(0.0025, (0.01 * z + spread) ** 2)
    return ProjectedPoint((u - cx) * z / fx, (v - cy) * z / fy, z, variance)
