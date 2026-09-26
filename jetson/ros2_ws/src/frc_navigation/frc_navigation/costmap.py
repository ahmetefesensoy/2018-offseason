"""Time-dependent field, robot, and teammate-reservation costs."""

from __future__ import annotations

import math

from .model import NavigationWorld


class TemporalCostmap:
    def __init__(self, world: NavigationWorld) -> None:
        self.world = world

    def cost(self, x: float, y: float, time_us: int) -> float:
        radius = self.world.robot_radius_m
        if not (radius <= x <= self.world.field_length_m - radius):
            return math.inf
        if not (radius <= y <= self.world.field_width_m - radius):
            return math.inf

        for rectangle in self.world.static_rectangles:
            if (
                rectangle.min_x - radius <= x <= rectangle.max_x + radius
                and rectangle.min_y - radius <= y <= rectangle.max_y + radius
            ):
                return math.inf

        total = 0.0
        horizon_s = max(0.0, (time_us - self.world.now_us) / 1e6)
        for obstacle in self.world.obstacles:
            obstacle_x = obstacle.x + obstacle.vx_mps * horizon_s
            obstacle_y = obstacle.y + obstacle.vy_mps * horizon_s
            clearance = math.hypot(x - obstacle_x, y - obstacle_y) - radius - obstacle.radius_m
            if obstacle.hard and clearance <= 0.10:
                return math.inf
            if clearance < 1.25:
                total += obstacle.confidence * (1.25 - max(clearance, 0.0)) * 20.0

        for reservation in self.world.reservations:
            if reservation.start_us <= time_us <= reservation.end_us:
                clearance = math.hypot(x - reservation.x, y - reservation.y) - radius - reservation.radius_m
                if clearance < 0.75:
                    total += reservation.confidence * (0.75 - max(clearance, 0.0)) * 12.0
        return total

