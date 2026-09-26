"""ROS-independent planar motion used by the lightweight roboRIO simulator."""

from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class PoseState:
    x: float
    y: float
    yaw: float

    def integrate(
        self,
        vx_mps: float,
        vy_mps: float,
        omega_radps: float,
        dt_seconds: float,
    ) -> "PoseState":
        values = (vx_mps, vy_mps, omega_radps, dt_seconds)
        if not all(math.isfinite(value) for value in values) or dt_seconds < 0.0:
            raise ValueError("motion sample must be finite and dt must be non-negative")
        cosine = math.cos(self.yaw)
        sine = math.sin(self.yaw)
        world_vx = cosine * vx_mps - sine * vy_mps
        world_vy = sine * vx_mps + cosine * vy_mps
        yaw = _wrap_angle(self.yaw + omega_radps * dt_seconds)
        return PoseState(
            self.x + world_vx * dt_seconds,
            self.y + world_vy * dt_seconds,
            yaw,
        )


def _wrap_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi
