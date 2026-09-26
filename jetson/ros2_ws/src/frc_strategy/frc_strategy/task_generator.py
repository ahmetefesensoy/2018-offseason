"""Generate only strategy tasks whose hard preconditions are currently valid."""

from __future__ import annotations

import math
from typing import Iterable

from .model import CandidateTask, DynamicRobot, FieldTarget, Pose2, StrategySnapshot


class TaskGenerator:
    def __init__(
        self,
        field_targets: Iterable[FieldTarget],
        min_object_confidence: float = 0.55,
        assumed_drive_speed_mps: float = 0.75,
    ) -> None:
        self._targets = {target.target_id: target for target in field_targets}
        self._min_object_confidence = min_object_confidence
        self._drive_speed_mps = assumed_drive_speed_mps

    def generate(self, snapshot: StrategySnapshot) -> tuple[CandidateTask, ...]:
        tasks: list[CandidateTask] = []
        if "drive" in snapshot.capabilities:
            cross_line = self._targets.get("auto_line")
            if snapshot.autonomous and cross_line is not None:
                tasks.append(self._from_field_target(cross_line, snapshot))

        if (
            not snapshot.has_cube
            and snapshot.perception_fresh
            and "intake" in snapshot.capabilities
        ):
            pickup = self._nearest_cube(snapshot)
            if pickup is not None:
                distance = _distance(snapshot.robot_pose, pickup.pose)
                duration = max(0.5, distance / self._drive_speed_mps) + 0.6
                tasks.append(CandidateTask(
                    task_id=f"pickup:{pickup.object_id}",
                    task_type="PICKUP",
                    target_id=pickup.object_id,
                    target_pose=pickup.pose,
                    estimated_duration_s=duration,
                    duration_std_s=0.2,
                    success_probability=min(0.97, 0.65 + 0.3 * pickup.confidence),
                    collision_probability=self._opponent_risk(snapshot, pickup.pose, duration),
                    reservation_cost=self._reservation_cost(snapshot, pickup.pose, duration),
                    required_capability="intake",
                    base_score=0.0,
                    future_value=18.0,
                    intake_action="INTAKE",
                ))

        if snapshot.has_cube:
            self._add_scoring_task(tasks, snapshot, "switch", snapshot.our_switch_side)
            self._add_scoring_task(tasks, snapshot, "scale", snapshot.scale_side)

        tasks.append(CandidateTask(
            task_id="wait:safe",
            task_type="WAIT",
            target_id="current_pose",
            target_pose=snapshot.robot_pose,
            estimated_duration_s=0.25,
            duration_std_s=0.0,
            success_probability=1.0,
            collision_probability=0.0,
            reservation_cost=0.0,
            required_capability="",
            base_score=0.0,
            intake_action="HOLD" if snapshot.has_cube else "STOP",
        ))
        return tuple(sorted(tasks, key=lambda task: task.task_id))

    def _nearest_cube(self, snapshot: StrategySnapshot):
        cubes = [
            item for item in snapshot.game_objects
            if item.class_name == "power_cube" and item.confidence >= self._min_object_confidence
        ]
        return min(
            cubes,
            key=lambda item: (_distance(snapshot.robot_pose, item.pose), item.object_id),
            default=None,
        )

    def _add_scoring_task(
        self,
        tasks: list[CandidateTask],
        snapshot: StrategySnapshot,
        capability: str,
        side: str,
    ) -> None:
        if capability not in snapshot.capabilities or side not in {"LEFT", "RIGHT"}:
            return
        target = self._targets.get(f"{capability}_{side.lower()}")
        if target is not None:
            tasks.append(self._from_field_target(target, snapshot))

    def _from_field_target(
        self,
        target: FieldTarget,
        snapshot: StrategySnapshot,
    ) -> CandidateTask:
        distance = _distance(snapshot.robot_pose, target.pose)
        drive_duration = distance / self._drive_speed_mps
        duration = max(target.duration_s, drive_duration)
        return CandidateTask(
            task_id=f"{target.task_type.lower()}:{target.target_id}",
            task_type=target.task_type,
            target_id=target.target_id,
            target_pose=target.pose,
            estimated_duration_s=duration,
            duration_std_s=0.25,
            success_probability=max(0.35, 0.94 - 0.025 * distance),
            collision_probability=self._opponent_risk(snapshot, target.pose, duration),
            reservation_cost=self._reservation_cost(snapshot, target.pose, duration),
            required_capability=target.required_capability,
            base_score=target.base_score,
            ownership_value=target.base_score * 0.25 if target.task_type.startswith("SCORE") else 0.0,
            future_value=5.0 if target.task_type.startswith("SCORE") else 0.0,
            intake_action=target.intake_action,
            elevator_target_m=target.elevator_target_m,
        )

    @staticmethod
    def _reservation_cost(
        snapshot: StrategySnapshot,
        target: Pose2,
        duration_s: float,
    ) -> float:
        arrival_us = snapshot.now_us + round(duration_s * 1_000_000)
        cost = 0.0
        for reservation in snapshot.reservations:
            if reservation.start_us <= arrival_us <= reservation.end_us:
                if _distance(reservation.pose, target) <= reservation.radius_m + 0.6:
                    cost += reservation.confidence
        return min(cost, 1.0)

    @staticmethod
    def _opponent_risk(
        snapshot: StrategySnapshot,
        target: Pose2,
        duration_s: float,
    ) -> float:
        risk = 0.0
        for robot in snapshot.robots:
            if robot.affiliation == "ALLY":
                continue
            horizon = min(duration_s, 2.0)
            predicted = Pose2(
                robot.pose.x + robot.vx_mps * horizon,
                robot.pose.y + robot.vy_mps * horizon,
                robot.pose.heading,
            )
            clearance = _distance_to_segment(predicted, snapshot.robot_pose, target) - robot.radius_m
            proximity = max(0.0, min(1.0, (1.25 - clearance) / 1.25))
            risk = max(risk, proximity * robot.confidence)
        return risk


def _distance(first: Pose2, second: Pose2) -> float:
    return math.hypot(first.x - second.x, first.y - second.y)


def _distance_to_segment(point: Pose2, start: Pose2, end: Pose2) -> float:
    dx = end.x - start.x
    dy = end.y - start.y
    length_squared = dx * dx + dy * dy
    if length_squared <= 1e-12:
        return _distance(point, start)
    ratio = ((point.x - start.x) * dx + (point.y - start.y) * dy) / length_squared
    ratio = max(0.0, min(1.0, ratio))
    closest = Pose2(start.x + ratio * dx, start.y + ratio * dy, 0.0)
    return _distance(point, closest)
