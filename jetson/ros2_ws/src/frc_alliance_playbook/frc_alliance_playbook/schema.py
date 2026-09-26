"""Strict canonical schema for pre-match alliance plans."""

from __future__ import annotations

from dataclasses import dataclass, replace
import hashlib
import json
import math
from typing import Any, Mapping


SCHEMA_VERSION = 1
FIELD_VERSION = "2018-power-up-v1"
FIELD_LENGTH_M = 16.46
FIELD_WIDTH_M = 8.23
VALID_ALLIANCES = frozenset(("blue", "red"))
VALID_TASKS = frozenset(
    (
        "PICKUP",
        "SCORE_SWITCH",
        "SCORE_SCALE",
        "DELIVER_VAULT",
        "CROSS_LINE",
        "PARK",
        "CLIMB",
        "WAIT",
    )
)


class PlanValidationError(ValueError):
    """A plan cannot be accepted as a complete canonical document."""


@dataclass(frozen=True)
class Pose2d:
    x: float
    y: float
    heading: float


@dataclass(frozen=True)
class TimedPose:
    time_us: int
    pose: Pose2d


@dataclass(frozen=True)
class PlanSegment:
    segment_id: str
    task: str
    target_id: str
    earliest_start_us: int
    latest_start_us: int
    expected_duration_us: int
    corridor_radius_m: float
    fallback_segment_id: str
    path: tuple[TimedPose, ...]


@dataclass(frozen=True)
class RobotPlan:
    team_number: int
    robot_label: str
    start_pose: Pose2d
    footprint_length_m: float
    footprint_width_m: float
    max_velocity_mps: float
    max_acceleration_mps2: float
    initial_confidence: float
    segments: tuple[PlanSegment, ...]


@dataclass(frozen=True)
class AlliancePlan:
    schema_version: int
    field_version: str
    alliance: str
    plan_id: str
    robots: tuple[RobotPlan, ...]
    content_sha256: str = ""


def parse_plan(data: Mapping[str, Any]) -> AlliancePlan:
    root = _mapping(
        data,
        "plan",
        required=("schema_version", "field_version", "alliance", "plan_id", "robots"),
        optional=("content_sha256",),
    )
    robots_value = root["robots"]
    if not isinstance(robots_value, list):
        raise PlanValidationError("plan.robots must be an array")
    plan = AlliancePlan(
        schema_version=_integer(root["schema_version"], "plan.schema_version"),
        field_version=_string(root["field_version"], "plan.field_version"),
        alliance=_string(root["alliance"], "plan.alliance"),
        plan_id=_string(root["plan_id"], "plan.plan_id"),
        robots=tuple(
            _parse_robot(value, f"plan.robots[{index}]")
            for index, value in enumerate(robots_value)
        ),
        content_sha256=_optional_string(root.get("content_sha256"), "plan.content_sha256"),
    )
    _validate_plan(plan)
    calculated_hash = content_sha256(plan)
    if plan.content_sha256 and plan.content_sha256 != calculated_hash:
        raise PlanValidationError("plan.content_sha256 does not match canonical content")
    return replace(plan, content_sha256=calculated_hash)


def canonical_payload(plan: AlliancePlan) -> dict[str, Any]:
    return {
        "schema_version": plan.schema_version,
        "field_version": plan.field_version,
        "alliance": plan.alliance,
        "plan_id": plan.plan_id,
        "robots": [_robot_payload(robot) for robot in plan.robots],
    }


def canonical_document(plan: AlliancePlan) -> dict[str, Any]:
    document = canonical_payload(plan)
    document["content_sha256"] = content_sha256(plan)
    return document


def canonical_json(plan: AlliancePlan) -> str:
    return json.dumps(
        canonical_document(plan),
        ensure_ascii=False,
        separators=(",", ":"),
        sort_keys=True,
    )


def content_sha256(plan: AlliancePlan) -> str:
    encoded = json.dumps(
        canonical_payload(plan),
        ensure_ascii=False,
        separators=(",", ":"),
        sort_keys=True,
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _parse_robot(value: Any, context: str) -> RobotPlan:
    data = _mapping(
        value,
        context,
        required=(
            "team_number",
            "robot_label",
            "start_pose",
            "footprint",
            "max_velocity_mps",
            "max_acceleration_mps2",
            "initial_confidence",
            "segments",
        ),
    )
    footprint = _mapping(
        data["footprint"],
        f"{context}.footprint",
        required=("length_m", "width_m"),
    )
    segments_value = data["segments"]
    if not isinstance(segments_value, list):
        raise PlanValidationError(f"{context}.segments must be an array")
    return RobotPlan(
        team_number=_integer(data["team_number"], f"{context}.team_number"),
        robot_label=_string(data["robot_label"], f"{context}.robot_label"),
        start_pose=_parse_pose(data["start_pose"], f"{context}.start_pose"),
        footprint_length_m=_number(footprint["length_m"], f"{context}.footprint.length_m"),
        footprint_width_m=_number(footprint["width_m"], f"{context}.footprint.width_m"),
        max_velocity_mps=_number(data["max_velocity_mps"], f"{context}.max_velocity_mps"),
        max_acceleration_mps2=_number(
            data["max_acceleration_mps2"],
            f"{context}.max_acceleration_mps2",
        ),
        initial_confidence=_number(data["initial_confidence"], f"{context}.initial_confidence"),
        segments=tuple(
            _parse_segment(segment, f"{context}.segments[{index}]")
            for index, segment in enumerate(segments_value)
        ),
    )


def _parse_segment(value: Any, context: str) -> PlanSegment:
    data = _mapping(
        value,
        context,
        required=(
            "segment_id",
            "task",
            "target_id",
            "earliest_start_us",
            "latest_start_us",
            "expected_duration_us",
            "corridor_radius_m",
            "fallback_segment_id",
            "path",
        ),
    )
    path_value = data["path"]
    if not isinstance(path_value, list):
        raise PlanValidationError(f"{context}.path must be an array")
    return PlanSegment(
        segment_id=_string(data["segment_id"], f"{context}.segment_id"),
        task=_string(data["task"], f"{context}.task"),
        target_id=_optional_string(data["target_id"], f"{context}.target_id"),
        earliest_start_us=_integer(data["earliest_start_us"], f"{context}.earliest_start_us"),
        latest_start_us=_integer(data["latest_start_us"], f"{context}.latest_start_us"),
        expected_duration_us=_integer(
            data["expected_duration_us"],
            f"{context}.expected_duration_us",
        ),
        corridor_radius_m=_number(data["corridor_radius_m"], f"{context}.corridor_radius_m"),
        fallback_segment_id=_optional_string(
            data["fallback_segment_id"],
            f"{context}.fallback_segment_id",
        ),
        path=tuple(
            _parse_timed_pose(point, f"{context}.path[{index}]")
            for index, point in enumerate(path_value)
        ),
    )


def _parse_timed_pose(value: Any, context: str) -> TimedPose:
    data = _mapping(
        value,
        context,
        required=("time_us", "x", "y", "heading"),
    )
    return TimedPose(
        time_us=_integer(data["time_us"], f"{context}.time_us"),
        pose=Pose2d(
            x=_number(data["x"], f"{context}.x"),
            y=_number(data["y"], f"{context}.y"),
            heading=_number(data["heading"], f"{context}.heading"),
        ),
    )


def _parse_pose(value: Any, context: str) -> Pose2d:
    data = _mapping(value, context, required=("x", "y", "heading"))
    return Pose2d(
        x=_number(data["x"], f"{context}.x"),
        y=_number(data["y"], f"{context}.y"),
        heading=_number(data["heading"], f"{context}.heading"),
    )


def _validate_plan(plan: AlliancePlan) -> None:
    if plan.schema_version != SCHEMA_VERSION:
        raise PlanValidationError(f"unsupported schema_version {plan.schema_version}")
    if plan.field_version != FIELD_VERSION:
        raise PlanValidationError(f"unsupported field_version {plan.field_version}")
    if plan.alliance not in VALID_ALLIANCES:
        raise PlanValidationError(f"unsupported alliance {plan.alliance}")
    if not plan.plan_id.strip():
        raise PlanValidationError("plan_id cannot be blank")
    if not 1 <= len(plan.robots) <= 2:
        raise PlanValidationError("an alliance playbook must contain one or two teammates")

    team_numbers: set[int] = set()
    robot_labels: set[str] = set()
    for robot in plan.robots:
        if robot.team_number in team_numbers:
            raise PlanValidationError(f"duplicate team_number {robot.team_number}")
        if robot.robot_label in robot_labels:
            raise PlanValidationError(f"duplicate robot_label {robot.robot_label}")
        team_numbers.add(robot.team_number)
        robot_labels.add(robot.robot_label)
        _validate_robot(robot)


def _validate_robot(robot: RobotPlan) -> None:
    if robot.team_number <= 0:
        raise PlanValidationError("team_number must be positive")
    if not robot.robot_label.strip():
        raise PlanValidationError("robot_label cannot be blank")
    _validate_field_pose(robot.start_pose, f"robot {robot.team_number} start_pose")
    if robot.footprint_length_m <= 0.0 or robot.footprint_width_m <= 0.0:
        raise PlanValidationError("robot footprint must be positive")
    if robot.max_velocity_mps <= 0.0 or robot.max_acceleration_mps2 <= 0.0:
        raise PlanValidationError("robot motion limits must be positive")
    if not 0.0 <= robot.initial_confidence <= 1.0:
        raise PlanValidationError("initial_confidence must be within [0, 1]")
    if not robot.segments:
        raise PlanValidationError("robot plan must contain at least one segment")

    segment_ids: set[str] = set()
    previous_end_us = -1
    for segment in robot.segments:
        if segment.segment_id in segment_ids:
            raise PlanValidationError(f"duplicate segment_id {segment.segment_id}")
        segment_ids.add(segment.segment_id)
        _validate_segment(segment, robot)
        if segment.path[0].time_us < previous_end_us:
            raise PlanValidationError("robot plan segments overlap in time")
        previous_end_us = segment.path[-1].time_us

    for segment in robot.segments:
        if segment.fallback_segment_id and segment.fallback_segment_id not in segment_ids:
            raise PlanValidationError(
                f"fallback segment {segment.fallback_segment_id} does not exist"
            )
    _validate_fallback_graph(robot.segments)


def _validate_segment(segment: PlanSegment, robot: RobotPlan) -> None:
    if not segment.segment_id.strip():
        raise PlanValidationError("segment_id cannot be blank")
    if segment.task not in VALID_TASKS:
        raise PlanValidationError(f"unsupported task {segment.task}")
    if segment.earliest_start_us < 0 or segment.latest_start_us < segment.earliest_start_us:
        raise PlanValidationError("segment start window is invalid")
    if segment.expected_duration_us <= 0:
        raise PlanValidationError("segment expected duration must be positive")
    if segment.corridor_radius_m < 0.0:
        raise PlanValidationError("segment corridor radius cannot be negative")
    if len(segment.path) < 2:
        raise PlanValidationError("segment path must contain at least two poses")
    if not segment.earliest_start_us <= segment.path[0].time_us <= segment.latest_start_us:
        raise PlanValidationError("segment path does not start inside its start window")
    if segment.path[-1].time_us - segment.path[0].time_us > segment.expected_duration_us:
        raise PlanValidationError("segment path exceeds expected duration")

    velocities: list[tuple[float, float, float]] = []
    previous = segment.path[0]
    _validate_field_pose(previous.pose, f"segment {segment.segment_id}")
    for point in segment.path[1:]:
        _validate_field_pose(point.pose, f"segment {segment.segment_id}")
        delta_us = point.time_us - previous.time_us
        if delta_us <= 0:
            raise PlanValidationError("path timestamps must be strictly increasing")
        delta_seconds = delta_us / 1e6
        vx = (point.pose.x - previous.pose.x) / delta_seconds
        vy = (point.pose.y - previous.pose.y) / delta_seconds
        speed = math.hypot(vx, vy)
        if speed > robot.max_velocity_mps + 1e-9:
            raise PlanValidationError("path exceeds robot velocity limit")
        velocities.append((vx, vy, delta_seconds))
        previous = point

    for prior, current in zip(velocities, velocities[1:]):
        sample_seconds = (prior[2] + current[2]) / 2.0
        acceleration = math.hypot(current[0] - prior[0], current[1] - prior[1]) / sample_seconds
        if acceleration > robot.max_acceleration_mps2 + 1e-9:
            raise PlanValidationError("path exceeds robot acceleration limit")


def _validate_fallback_graph(segments: tuple[PlanSegment, ...]) -> None:
    edges = {
        segment.segment_id: segment.fallback_segment_id
        for segment in segments
        if segment.fallback_segment_id
    }
    for start in edges:
        visited: set[str] = set()
        current = start
        while current in edges:
            if current in visited:
                raise PlanValidationError("fallback graph contains a cycle")
            visited.add(current)
            current = edges[current]


def _validate_field_pose(pose: Pose2d, context: str) -> None:
    if not 0.0 <= pose.x <= FIELD_LENGTH_M or not 0.0 <= pose.y <= FIELD_WIDTH_M:
        raise PlanValidationError(f"{context} is outside the 2018 field")


def _robot_payload(robot: RobotPlan) -> dict[str, Any]:
    return {
        "team_number": robot.team_number,
        "robot_label": robot.robot_label,
        "start_pose": _pose_payload(robot.start_pose),
        "footprint": {
            "length_m": robot.footprint_length_m,
            "width_m": robot.footprint_width_m,
        },
        "max_velocity_mps": robot.max_velocity_mps,
        "max_acceleration_mps2": robot.max_acceleration_mps2,
        "initial_confidence": robot.initial_confidence,
        "segments": [_segment_payload(segment) for segment in robot.segments],
    }


def _segment_payload(segment: PlanSegment) -> dict[str, Any]:
    return {
        "segment_id": segment.segment_id,
        "task": segment.task,
        "target_id": segment.target_id,
        "earliest_start_us": segment.earliest_start_us,
        "latest_start_us": segment.latest_start_us,
        "expected_duration_us": segment.expected_duration_us,
        "corridor_radius_m": segment.corridor_radius_m,
        "fallback_segment_id": segment.fallback_segment_id,
        "path": [
            {"time_us": point.time_us, **_pose_payload(point.pose)}
            for point in segment.path
        ],
    }


def _pose_payload(pose: Pose2d) -> dict[str, float]:
    return {"x": pose.x, "y": pose.y, "heading": pose.heading}


def _mapping(
    value: Any,
    context: str,
    required: tuple[str, ...],
    optional: tuple[str, ...] = (),
) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise PlanValidationError(f"{context} must be an object")
    missing = set(required) - set(value)
    unknown = set(value) - set(required) - set(optional)
    if missing:
        raise PlanValidationError(f"{context} is missing keys: {sorted(missing)}")
    if unknown:
        raise PlanValidationError(f"{context} has unknown keys: {sorted(unknown)}")
    return value


def _string(value: Any, context: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise PlanValidationError(f"{context} must be a non-empty string")
    return value


def _optional_string(value: Any, context: str) -> str:
    if value is None:
        return ""
    if not isinstance(value, str):
        raise PlanValidationError(f"{context} must be a string")
    return value


def _integer(value: Any, context: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise PlanValidationError(f"{context} must be an integer")
    return value


def _number(value: Any, context: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise PlanValidationError(f"{context} must be numeric")
    result = float(value)
    if not math.isfinite(result):
        raise PlanValidationError(f"{context} must be finite")
    return result
