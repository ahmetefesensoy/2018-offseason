"""Deterministic temporal A* for an omnidirectional swerve footprint."""

from __future__ import annotations

import heapq
import math

from .costmap import TemporalCostmap
from .model import NavigationGoal, PathPoint, Pose2


class TemporalAStarPlanner:
    _NEIGHBORS = (
        (-1, -1), (-1, 0), (-1, 1), (0, -1),
        (0, 1), (1, -1), (1, 0), (1, 1), (0, 0),
    )

    def __init__(
        self,
        resolution_m: float = 0.25,
        time_step_s: float = 0.2,
        nominal_speed_mps: float = 0.75,
        max_nodes: int = 25_000,
        max_horizon_s: float = 15.0,
    ) -> None:
        self._resolution = resolution_m
        self._time_step_us = round(time_step_s * 1e6)
        self._nominal_speed = nominal_speed_mps
        self._max_nodes = max_nodes
        self._max_steps = math.ceil(max_horizon_s / time_step_s)

    def plan(
        self,
        start: Pose2,
        goal: NavigationGoal,
        costmap: TemporalCostmap,
        start_us: int,
    ) -> tuple[PathPoint, ...]:
        start_key = (*self._grid(start.x, start.y), 0)
        goal_grid = self._grid(goal.pose.x, goal.pose.y)
        frontier = [(self._heuristic(start_key[:2], goal_grid), 0.0, start_key)]
        parents = {start_key: None}
        costs = {start_key: 0.0}
        visited = 0
        final_key = None

        while frontier and visited < self._max_nodes:
            _, current_cost, current = heapq.heappop(frontier)
            if current_cost != costs.get(current):
                continue
            visited += 1
            x, y = self._world(current[0], current[1])
            if math.hypot(x - goal.pose.x, y - goal.pose.y) <= max(goal.position_tolerance_m, self._resolution * 0.75):
                final_key = current
                break
            if current[2] >= self._max_steps:
                continue

            for dx, dy in self._NEIGHBORS:
                distance = self._resolution * math.hypot(dx, dy)
                step_count = 1 if distance == 0.0 else max(
                    1,
                    math.ceil((distance / self._nominal_speed * 1e6) / self._time_step_us),
                )
                neighbor = (current[0] + dx, current[1] + dy, current[2] + step_count)
                if neighbor[2] > self._max_steps:
                    continue
                nx, ny = self._world(neighbor[0], neighbor[1])
                arrival_us = start_us + neighbor[2] * self._time_step_us
                cell_cost = costmap.cost(nx, ny, arrival_us)
                if not math.isfinite(cell_cost):
                    continue
                move_cost = (distance if distance > 0.0 else 0.08) + cell_cost * 0.02
                tentative = current_cost + move_cost
                if tentative + 1e-12 >= costs.get(neighbor, math.inf):
                    continue
                costs[neighbor] = tentative
                parents[neighbor] = current
                priority = tentative + self._heuristic(neighbor[:2], goal_grid)
                heapq.heappush(frontier, (priority, tentative, neighbor))

        if final_key is None:
            return ()
        keys = []
        current = final_key
        while current is not None:
            keys.append(current)
            current = parents[current]
        keys.reverse()
        points = []
        for index, key in enumerate(keys):
            x, y = self._world(key[0], key[1])
            if index + 1 < len(keys):
                nx, ny = self._world(keys[index + 1][0], keys[index + 1][1])
                heading = math.atan2(ny - y, nx - x)
            else:
                heading = goal.pose.heading
            points.append(PathPoint(start_us + key[2] * self._time_step_us, x, y, heading))
        return tuple(points)

    def _grid(self, x: float, y: float) -> tuple[int, int]:
        return round(x / self._resolution), round(y / self._resolution)

    def _world(self, x: int, y: int) -> tuple[float, float]:
        return x * self._resolution, y * self._resolution

    def _heuristic(self, value: tuple[int, int], goal: tuple[int, int]) -> float:
        return self._resolution * math.hypot(value[0] - goal[0], value[1] - goal[1])

