"""Normalize external teammate strategy formats into the canonical schema."""

from __future__ import annotations

import csv
from dataclasses import dataclass
import io
import math
import re
from typing import Any, Callable, Mapping

from .schema import (
    AlliancePlan,
    PlanSegment,
    Pose2d,
    RobotPlan,
    TimedPose,
    canonical_payload,
    parse_plan,
)


class PlanImportError(ValueError):
    """An external plan cannot be normalized safely."""


@dataclass(frozen=True)
class ImportMetadata:
    plan_id: str
    alliance: str
    team_number: int
    robot_label: str
    footprint_length_m: float
    footprint_width_m: float
    initial_confidence: float
    corridor_radius_m: float
    default_task: str
    default_target_id: str
    max_velocity_mps: float
    max_acceleration_mps2: float


def import_csv(text: str, metadata: ImportMetadata) -> AlliancePlan:
    reader = csv.DictReader(io.StringIO(text))
    required = ("time", "x", "y", "heading", "vx", "vy", "omega", "event")
    if reader.fieldnames is None or tuple(reader.fieldnames) != required:
        raise PlanImportError(f"CSV columns must be exactly {required}")
    rows = list(reader)
    if len(rows) < 2:
        raise PlanImportError("CSV must contain at least two samples")

    samples: list[tuple[TimedPose, float, float, str]] = []
    for index, row in enumerate(rows):
        try:
            time_seconds = _finite_float(row["time"], f"row {index + 2} time")
            x = _finite_float(row["x"], f"row {index + 2} x")
            y = _finite_float(row["y"], f"row {index + 2} y")
            heading_degrees = _finite_float(row["heading"], f"row {index + 2} heading")
            vx = _finite_float(row["vx"], f"row {index + 2} vx")
            vy = _finite_float(row["vy"], f"row {index + 2} vy")
            _finite_float(row["omega"], f"row {index + 2} omega")
        except (TypeError, ValueError) as error:
            raise PlanImportError(str(error)) from error
        timestamp_us = round(time_seconds * 1e6)
        samples.append(
            (
                TimedPose(
                    timestamp_us,
                    Pose2d(x=x, y=y, heading=math.radians(heading_degrees)),
                ),
                vx,
                vy,
                (row["event"] or "").strip(),
            )
        )

    for previous, current in zip(samples, samples[1:]):
        delta_us = current[0].time_us - previous[0].time_us
        if delta_us <= 0:
            raise PlanImportError("CSV time must be strictly increasing")
        delta_seconds = delta_us / 1e6
        derived_vx = (current[0].pose.x - previous[0].pose.x) / delta_seconds
        derived_vy = (current[0].pose.y - previous[0].pose.y) / delta_seconds
        if math.hypot(derived_vx - current[1], derived_vy - current[2]) > 0.25:
            raise PlanImportError("CSV position and declared velocity disagree")

    event_indices = [index for index, sample in enumerate(samples) if sample[3]]
    if not event_indices or event_indices[0] != 0:
        event_indices.insert(0, 0)
    segments: list[PlanSegment] = []
    for event_number, start_index in enumerate(event_indices):
        end_index = (
            event_indices[event_number + 1]
            if event_number + 1 < len(event_indices)
            else len(samples) - 1
        )
        if end_index <= start_index:
            raise PlanImportError("each CSV event segment needs at least two samples")
        event = samples[start_index][3]
        task, target_id = _event_task(event, metadata)
        path = tuple(sample[0] for sample in samples[start_index : end_index + 1])
        segments.append(
            _segment(
                segment_id=f"{_slug(task)}-{event_number + 1}",
                task=task,
                target_id=target_id,
                corridor_radius_m=metadata.corridor_radius_m,
                path=path,
            )
        )
    return _make_plan(metadata, tuple(segments))


def import_pathplanner_path(
    data: Mapping[str, Any],
    metadata: ImportMetadata,
) -> AlliancePlan:
    segment = _pathplanner_segment(data, metadata, 0, "path-1")
    return _make_plan(metadata, (segment,))


def import_pathplanner_auto(
    data: Mapping[str, Any],
    path_loader: Callable[[str], Mapping[str, Any]],
    metadata: ImportMetadata,
) -> AlliancePlan:
    if not isinstance(data, Mapping) or data.get("version") != "2025.0":
        raise PlanImportError("only PathPlanner auto version 2025.0 is supported")
    root_command = data.get("command")
    segments: list[PlanSegment] = []
    offset_us = 0
    current_pose: Pose2d | None = None

    def consume(command: Any) -> None:
        nonlocal offset_us, current_pose
        if not isinstance(command, Mapping):
            raise PlanImportError("PathPlanner command must be an object")
        command_type = command.get("type")
        command_data = command.get("data")
        if not isinstance(command_data, Mapping):
            raise PlanImportError("PathPlanner command data must be an object")
        if command_type == "sequential":
            commands = command_data.get("commands")
            if not isinstance(commands, list):
                raise PlanImportError("sequential command requires a commands array")
            for child in commands:
                consume(child)
            return
        if command_type in ("parallel", "race", "deadline"):
            raise PlanImportError(f"unsupported concurrent drive structure: {command_type}")
        if command_type == "named":
            return
        if command_type == "wait":
            if current_pose is None:
                raise PlanImportError("wait cannot appear before the first path")
            wait_seconds = _finite_float(command_data.get("waitTime"), "waitTime")
            if wait_seconds <= 0.0:
                raise PlanImportError("waitTime must be positive")
            duration_us = round(wait_seconds * 1e6)
            path = (
                TimedPose(offset_us, current_pose),
                TimedPose(offset_us + duration_us, current_pose),
            )
            segments.append(
                _segment(
                    segment_id=f"wait-{len(segments) + 1}",
                    task="WAIT",
                    target_id="",
                    corridor_radius_m=metadata.corridor_radius_m,
                    path=path,
                )
            )
            offset_us += duration_us
            return
        if command_type == "path":
            path_name = command_data.get("pathName")
            if not isinstance(path_name, str) or not path_name:
                raise PlanImportError("path command requires pathName")
            try:
                path_data = path_loader(path_name)
            except Exception as error:
                raise PlanImportError(f"unable to load PathPlanner path '{path_name}'") from error
            segment = _pathplanner_segment(
                path_data,
                metadata,
                offset_us,
                f"{_slug(path_name)}-{len(segments) + 1}",
            )
            segments.append(segment)
            offset_us = segment.path[-1].time_us
            current_pose = segment.path[-1].pose
            return
        raise PlanImportError(f"unsupported PathPlanner command type: {command_type}")

    consume(root_command)
    if not segments:
        raise PlanImportError("PathPlanner auto contains no path segments")
    return _make_plan(metadata, tuple(segments))


def _pathplanner_segment(
    data: Mapping[str, Any],
    metadata: ImportMetadata,
    time_offset_us: int,
    segment_id: str,
) -> PlanSegment:
    if not isinstance(data, Mapping) or data.get("version") != "2025.0":
        raise PlanImportError("only PathPlanner path version 2025.0 is supported")
    waypoints = data.get("waypoints")
    constraints = data.get("globalConstraints")
    if not isinstance(waypoints, list) or len(waypoints) < 2:
        raise PlanImportError("PathPlanner path requires at least two waypoints")
    if not isinstance(constraints, Mapping):
        raise PlanImportError("PathPlanner path is missing globalConstraints")
    file_velocity = _finite_float(constraints.get("maxVelocity"), "maxVelocity")
    file_acceleration = _finite_float(constraints.get("maxAcceleration"), "maxAcceleration")
    max_velocity = min(file_velocity, metadata.max_velocity_mps)
    if max_velocity <= 0.0 or file_acceleration <= 0.0:
        raise PlanImportError("PathPlanner motion limits must be positive")

    points: list[tuple[float, float, float]] = []
    for span_index in range(len(waypoints) - 1):
        first = _waypoint(waypoints[span_index], f"waypoint {span_index}")
        second = _waypoint(waypoints[span_index + 1], f"waypoint {span_index + 1}")
        p0 = first[0]
        p1 = first[2] or p0
        p2 = second[1] or second[0]
        p3 = second[0]
        start_sample = 0 if span_index == 0 else 1
        for sample_index in range(start_sample, 21):
            t = sample_index / 20.0
            x, y = _cubic_point(p0, p1, p2, p3, t)
            dx, dy = _cubic_derivative(p0, p1, p2, p3, t)
            heading = math.atan2(dy, dx) if abs(dx) + abs(dy) > 1e-12 else 0.0
            points.append((x, y, heading))

    timed: list[TimedPose] = []
    current_time_us = time_offset_us
    for index, point in enumerate(points):
        if index:
            distance = math.hypot(point[0] - points[index - 1][0], point[1] - points[index - 1][1])
            if distance <= 1e-9:
                continue
            # Quantize toward a longer interval so microsecond rounding can
            # never make the normalized path exceed its velocity limit.
            current_time_us += max(1, math.ceil(distance / max_velocity * 1e6))
        timed.append(TimedPose(current_time_us, Pose2d(*point)))
    if len(timed) < 2:
        raise PlanImportError("PathPlanner path has no measurable length")
    return _segment(
        segment_id=segment_id,
        task=metadata.default_task,
        target_id=metadata.default_target_id,
        corridor_radius_m=metadata.corridor_radius_m,
        path=tuple(timed),
    )


def _make_plan(metadata: ImportMetadata, segments: tuple[PlanSegment, ...]) -> AlliancePlan:
    if not segments:
        raise PlanImportError("import produced no segments")
    robot = RobotPlan(
        team_number=metadata.team_number,
        robot_label=metadata.robot_label,
        start_pose=segments[0].path[0].pose,
        footprint_length_m=metadata.footprint_length_m,
        footprint_width_m=metadata.footprint_width_m,
        max_velocity_mps=metadata.max_velocity_mps,
        max_acceleration_mps2=metadata.max_acceleration_mps2,
        initial_confidence=metadata.initial_confidence,
        segments=segments,
    )
    raw_plan = AlliancePlan(
        schema_version=1,
        field_version="2018-power-up-v1",
        alliance=metadata.alliance,
        plan_id=metadata.plan_id,
        robots=(robot,),
    )
    try:
        return parse_plan(canonical_payload(raw_plan))
    except ValueError as error:
        raise PlanImportError(f"imported plan failed canonical validation: {error}") from error


def _segment(
    segment_id: str,
    task: str,
    target_id: str,
    corridor_radius_m: float,
    path: tuple[TimedPose, ...],
) -> PlanSegment:
    duration_us = path[-1].time_us - path[0].time_us
    return PlanSegment(
        segment_id=segment_id,
        task=task,
        target_id=target_id,
        earliest_start_us=path[0].time_us,
        latest_start_us=path[0].time_us,
        expected_duration_us=duration_us,
        corridor_radius_m=corridor_radius_m,
        fallback_segment_id="",
        path=path,
    )


def _waypoint(value: Any, context: str):
    if not isinstance(value, Mapping) or "anchor" not in value:
        raise PlanImportError(f"{context} is missing anchor")
    anchor = _xy(value["anchor"], f"{context}.anchor")
    previous = _xy(value["prevControl"], f"{context}.prevControl") if value.get("prevControl") is not None else None
    following = _xy(value["nextControl"], f"{context}.nextControl") if value.get("nextControl") is not None else None
    return anchor, previous, following


def _xy(value: Any, context: str) -> tuple[float, float]:
    if not isinstance(value, Mapping) or "x" not in value or "y" not in value:
        raise PlanImportError(f"{context} must contain x and y")
    return _finite_float(value["x"], f"{context}.x"), _finite_float(value["y"], f"{context}.y")


def _cubic_point(p0, p1, p2, p3, t: float) -> tuple[float, float]:
    inverse = 1.0 - t
    weights = (
        inverse**3,
        3.0 * inverse**2 * t,
        3.0 * inverse * t**2,
        t**3,
    )
    return (
        sum(weight * point[0] for weight, point in zip(weights, (p0, p1, p2, p3))),
        sum(weight * point[1] for weight, point in zip(weights, (p0, p1, p2, p3))),
    )


def _cubic_derivative(p0, p1, p2, p3, t: float) -> tuple[float, float]:
    inverse = 1.0 - t
    return (
        3.0 * inverse**2 * (p1[0] - p0[0])
        + 6.0 * inverse * t * (p2[0] - p1[0])
        + 3.0 * t**2 * (p3[0] - p2[0]),
        3.0 * inverse**2 * (p1[1] - p0[1])
        + 6.0 * inverse * t * (p2[1] - p1[1])
        + 3.0 * t**2 * (p3[1] - p2[1]),
    )


def _event_task(event: str, metadata: ImportMetadata) -> tuple[str, str]:
    if not event:
        return metadata.default_task, metadata.default_target_id
    task, separator, target = event.partition(":")
    return task.strip().upper(), target.strip() if separator else metadata.default_target_id


def _finite_float(value: Any, context: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as error:
        raise PlanImportError(f"{context} must be numeric") from error
    if not math.isfinite(result):
        raise PlanImportError(f"{context} must be finite")
    return result


def _slug(value: str) -> str:
    slug = re.sub(r"[^a-z0-9]+", "-", value.lower()).strip("-")
    return slug or "segment"
