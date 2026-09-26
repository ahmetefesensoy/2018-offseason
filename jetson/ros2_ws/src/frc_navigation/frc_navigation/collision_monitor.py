"""Planner-independent dynamic slow/stop safety envelope."""

from __future__ import annotations

from dataclasses import replace
import math

from .model import CircleObstacle, MonitorResult, Pose2, VelocityCommand


class CollisionMonitor:
    def __init__(
        self,
        stop_clearance_m: float = 0.20,
        slow_clearance_m: float = 0.80,
        horizon_s: float = 1.0,
        stale_speed_mps: float = 0.25,
    ) -> None:
        self._stop = stop_clearance_m
        self._slow = slow_clearance_m
        self._horizon = horizon_s
        self._stale_speed = stale_speed_mps

    def filter(
        self,
        command: VelocityCommand,
        pose: Pose2,
        obstacles: tuple[CircleObstacle, ...],
        robot_radius_m: float,
        perception_fresh: bool,
    ) -> MonitorResult:
        if not command.armed:
            return MonitorResult(command, math.inf, "DISARMED")
        cosine = math.cos(pose.heading)
        sine = math.sin(pose.heading)
        field_vx = cosine * command.vx_mps - sine * command.vy_mps
        field_vy = sine * command.vx_mps + cosine * command.vy_mps
        minimum = math.inf
        for obstacle in obstacles:
            for step in range(21):
                time_s = self._horizon * step / 20.0
                robot_x = pose.x + field_vx * time_s
                robot_y = pose.y + field_vy * time_s
                obstacle_x = obstacle.x + obstacle.vx_mps * time_s
                obstacle_y = obstacle.y + obstacle.vy_mps * time_s
                clearance = math.hypot(robot_x - obstacle_x, robot_y - obstacle_y) - robot_radius_m - obstacle.radius_m
                minimum = min(minimum, clearance)
        if minimum <= self._stop:
            stopped = VelocityCommand(False, 0.0, 0.0, 0.0, 0, False, 0.0)
            return MonitorResult(stopped, minimum, "DYNAMIC_COLLISION_STOP")
        if minimum < self._slow:
            scale = max(0.15, (minimum - self._stop) / (self._slow - self._stop))
            return MonitorResult(replace(
                command,
                vx_mps=command.vx_mps * scale,
                vy_mps=command.vy_mps * scale,
                omega_radps=command.omega_radps * scale,
            ), minimum, "DYNAMIC_COLLISION_SLOW")
        if not perception_fresh:
            speed = math.hypot(command.vx_mps, command.vy_mps)
            if speed > self._stale_speed:
                scale = self._stale_speed / speed
                return MonitorResult(replace(
                    command,
                    vx_mps=command.vx_mps * scale,
                    vy_mps=command.vy_mps * scale,
                ), minimum, "STALE_PERCEPTION_SLOW")
        return MonitorResult(command, minimum, "CLEAR")
