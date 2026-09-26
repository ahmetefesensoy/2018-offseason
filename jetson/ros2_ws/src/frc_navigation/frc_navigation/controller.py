"""Bounded holonomic path tracking in the robot coordinate frame."""

from __future__ import annotations

import math

from .model import PathPoint, Pose2, VelocityCommand


class HolonomicController:
    def __init__(
        self,
        max_speed_mps: float = 0.75,
        max_omega_radps: float = 1.5,
        position_gain: float = 1.5,
        heading_gain: float = 2.5,
    ) -> None:
        self._max_speed = max_speed_mps
        self._max_omega = max_omega_radps
        self._position_gain = position_gain
        self._heading_gain = heading_gain

    def command(
        self,
        pose: Pose2,
        path: tuple[PathPoint, ...],
        perception_fresh: bool,
    ) -> VelocityCommand:
        if not path:
            return VelocityCommand(False, 0.0, 0.0, 0.0, 0, False, 0.0)
        target = next(
            (point for point in path if math.hypot(point.x - pose.x, point.y - pose.y) > 0.20),
            path[-1],
        )
        dx = target.x - pose.x
        dy = target.y - pose.y
        field_vx = dx * self._position_gain
        field_vy = dy * self._position_gain
        limit = self._max_speed if perception_fresh else min(self._max_speed, 0.25)
        magnitude = math.hypot(field_vx, field_vy)
        if magnitude > limit and magnitude > 0.0:
            field_vx *= limit / magnitude
            field_vy *= limit / magnitude
        cosine = math.cos(pose.heading)
        sine = math.sin(pose.heading)
        robot_vx = cosine * field_vx + sine * field_vy
        robot_vy = -sine * field_vx + cosine * field_vy
        omega = _clamp(self._heading_gain * _wrap(target.heading - pose.heading), self._max_omega)
        return VelocityCommand(True, robot_vx, robot_vy, omega, 0, False, 0.0)


def _wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))

