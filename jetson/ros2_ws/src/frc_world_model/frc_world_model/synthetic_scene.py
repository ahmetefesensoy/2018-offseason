"""Deterministic field observations for a hardware-free RViz demonstration."""

import math

from .model import Observation


def sample_scene(elapsed_s: float, timestamp_us: int) -> tuple[Observation, ...]:
    triangle = 6.0 - abs((elapsed_s % 12.0) - 6.0)
    ally_x = 0.8 + 0.45 * triangle
    opponent_y = 2.4 + 1.2 * math.sin(2.0 * math.pi * elapsed_s / 8.0)
    return (
        _observation("cube-left", "power_cube", 1.8, 1.2, 0, 0.33, 0.96, 0.01, timestamp_us),
        _observation("cube-center", "power_cube", 3.0, 2.7, 0, 0.33, 0.94, 0.01, timestamp_us),
        _observation("cube-right", "power_cube", 4.7, 1.4, 0, 0.33, 0.92, 0.01, timestamp_us),
        _observation("ally-254", "robot", ally_x, 4.5, 1, 0.90, 0.98, 0.04, timestamp_us),
        _observation(
            "opponent-999",
            "robot",
            3.5,
            opponent_y,
            2,
            0.90,
            0.97,
            0.04,
            timestamp_us,
            yaw=math.pi / 2.0,
        ),
    )


def _observation(
    detection_id: str,
    class_name: str,
    x: float,
    y: float,
    affiliation: int,
    size: float,
    confidence: float,
    variance: float,
    timestamp_us: int,
    yaw: float = 0.0,
) -> Observation:
    return Observation(
        detection_id=detection_id,
        class_name=class_name,
        affiliation=affiliation,
        x=x,
        y=y,
        yaw=yaw,
        size_x=size,
        size_y=size,
        confidence=confidence,
        variance_x=variance,
        variance_y=variance,
        timestamp_us=timestamp_us,
    )
