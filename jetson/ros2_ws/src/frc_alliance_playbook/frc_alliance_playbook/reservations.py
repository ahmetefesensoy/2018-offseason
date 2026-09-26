"""Time-dependent teammate reservations and plan-observation confidence fusion."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Mapping, Sequence

from .schema import AlliancePlan, PlanSegment, Pose2d, RobotPlan, TimedPose


@dataclass(frozen=True)
class ReservationSample:
    time_us: int
    pose: Pose2d
    radius_m: float


@dataclass(frozen=True)
class ReservationTube:
    plan_id: str
    team_number: int
    robot_label: str
    confidence: float
    samples: tuple[ReservationSample, ...]


@dataclass(frozen=True)
class ReservationConflict:
    first_team_number: int
    second_team_number: int
    start_us: int
    end_us: int
    minimum_clearance_m: float


def build_reservations(
    plan: AlliancePlan,
    sample_period_us: int = 100_000,
) -> tuple[ReservationTube, ...]:
    """Sample each teammate route into deterministic circular occupancy tubes."""
    if isinstance(sample_period_us, bool) or sample_period_us <= 0:
        raise ValueError("sample_period_us must be positive")

    tubes: list[ReservationTube] = []
    for robot in sorted(plan.robots, key=lambda value: value.team_number):
        if not robot.segments:
            raise ValueError(f"robot {robot.team_number} has no segments")
        segments = tuple(
            sorted(robot.segments, key=lambda value: value.path[0].time_us)
        )
        start_us = segments[0].path[0].time_us
        end_us = segments[-1].path[-1].time_us
        sample_times = list(range(start_us, end_us + 1, sample_period_us))
        if not sample_times or sample_times[-1] != end_us:
            sample_times.append(end_us)
        samples = tuple(
            _sample_robot(robot, segments, time_us) for time_us in sample_times
        )
        tubes.append(
            ReservationTube(
                plan_id=plan.plan_id,
                team_number=robot.team_number,
                robot_label=robot.robot_label,
                confidence=robot.initial_confidence,
                samples=samples,
            )
        )
    return tuple(tubes)


def find_conflicts(
    tubes: Sequence[ReservationTube],
) -> tuple[ReservationConflict, ...]:
    """Return continuous time intervals where two linearly moving discs overlap."""
    ordered = sorted(tubes, key=lambda value: value.team_number)
    conflicts: list[ReservationConflict] = []
    for first_index, first in enumerate(ordered):
        if not first.samples:
            continue
        for second in ordered[first_index + 1 :]:
            if not second.samples:
                continue
            overlap_start = max(first.samples[0].time_us, second.samples[0].time_us)
            overlap_end = min(first.samples[-1].time_us, second.samples[-1].time_us)
            if overlap_start > overlap_end:
                continue
            times = {overlap_start, overlap_end}
            times.update(
                sample.time_us
                for sample in first.samples
                if overlap_start <= sample.time_us <= overlap_end
            )
            times.update(
                sample.time_us
                for sample in second.samples
                if overlap_start <= sample.time_us <= overlap_end
            )
            raw_intervals: list[tuple[float, float, float]] = []
            breakpoints = sorted(times)
            if len(breakpoints) == 1:
                first_sample = _sample_tube(first, breakpoints[0])
                second_sample = _sample_tube(second, breakpoints[0])
                clearance = _clearance(first_sample, second_sample)
                if clearance <= 0.0:
                    raw_intervals.append(
                        (float(breakpoints[0]), float(breakpoints[0]), clearance)
                    )
            for start_us, end_us in zip(breakpoints, breakpoints[1:]):
                first_start = _sample_tube(first, start_us)
                first_end = _sample_tube(first, end_us)
                second_start = _sample_tube(second, start_us)
                second_end = _sample_tube(second, end_us)
                for ratio_start, ratio_end, clearance in _linear_collision_intervals(
                    first_start,
                    first_end,
                    second_start,
                    second_end,
                ):
                    duration_us = end_us - start_us
                    raw_intervals.append(
                        (
                            start_us + duration_us * ratio_start,
                            start_us + duration_us * ratio_end,
                            clearance,
                        )
                    )

            for interval_start, interval_end, minimum_clearance in _merge_intervals(
                raw_intervals
            ):
                conflicts.append(
                    ReservationConflict(
                        first.team_number,
                        second.team_number,
                        math.floor(interval_start),
                        math.ceil(interval_end),
                        minimum_clearance,
                    )
                )
    return tuple(conflicts)


def sample_reservation(tube: ReservationTube, time_us: int) -> ReservationSample:
    """Sample a tube at match time, holding its first/last pose outside its span."""
    if not tube.samples:
        raise ValueError("reservation tube contains no samples")
    clamped_time_us = min(
        max(time_us, tube.samples[0].time_us),
        tube.samples[-1].time_us,
    )
    sample = _sample_tube(tube, clamped_time_us)
    if clamped_time_us == time_us:
        return sample
    return ReservationSample(time_us, sample.pose, sample.radius_m)


class ConfidenceTracker:
    """Monotonically reduce plan trust as observations deviate or disappear."""

    def __init__(
        self,
        initial_confidence: Mapping[int, float],
        *,
        corridor_tolerance_m: float,
        deviation_scale_m: float,
        missing_half_life_us: int,
    ) -> None:
        if corridor_tolerance_m < 0.0 or not math.isfinite(corridor_tolerance_m):
            raise ValueError("corridor_tolerance_m must be finite and non-negative")
        if deviation_scale_m <= 0.0 or not math.isfinite(deviation_scale_m):
            raise ValueError("deviation_scale_m must be finite and positive")
        if isinstance(missing_half_life_us, bool) or missing_half_life_us <= 0:
            raise ValueError("missing_half_life_us must be positive")
        self._initial: dict[int, float] = {}
        for team_number, confidence in initial_confidence.items():
            if not math.isfinite(confidence) or not 0.0 <= confidence <= 1.0:
                raise ValueError("initial confidence must be within [0, 1]")
            self._initial[team_number] = confidence
        self._current = dict(self._initial)
        self._last_update_us: dict[int, int | None] = {
            team_number: None for team_number in self._initial
        }
        self._corridor_tolerance_m = corridor_tolerance_m
        self._deviation_scale_m = deviation_scale_m
        self._missing_half_life_us = missing_half_life_us

    def confidence(self, team_number: int) -> float:
        self._check_team(team_number)
        return self._current[team_number]

    def update(
        self,
        team_number: int,
        planned_pose: Pose2d,
        observed_pose: Pose2d,
        observed_at_us: int,
    ) -> float:
        self._check_timestamp(team_number, observed_at_us)
        deviation = math.hypot(
            planned_pose.x - observed_pose.x,
            planned_pose.y - observed_pose.y,
        )
        if not math.isfinite(deviation):
            raise ValueError("pose deviation must be finite")
        excess = max(0.0, deviation - self._corridor_tolerance_m)
        candidate = self._initial[team_number] * math.exp(
            -excess / self._deviation_scale_m
        )
        self._current[team_number] = _clamp_confidence(
            min(self._current[team_number], candidate),
            self._initial[team_number],
        )
        self._last_update_us[team_number] = observed_at_us
        return self._current[team_number]

    def missing(self, team_number: int, now_us: int) -> float:
        self._check_timestamp(team_number, now_us)
        previous_us = self._last_update_us[team_number]
        if previous_us is not None:
            elapsed_us = now_us - previous_us
            decay = 0.5 ** (elapsed_us / self._missing_half_life_us)
            self._current[team_number] = _clamp_confidence(
                self._current[team_number] * decay,
                self._initial[team_number],
            )
        self._last_update_us[team_number] = now_us
        return self._current[team_number]

    def _check_team(self, team_number: int) -> None:
        if team_number not in self._initial:
            raise KeyError(f"unknown team number: {team_number}")

    def _check_timestamp(self, team_number: int, time_us: int) -> None:
        self._check_team(team_number)
        if isinstance(time_us, bool) or not isinstance(time_us, int) or time_us < 0:
            raise ValueError("observation time must be a non-negative integer")
        previous_us = self._last_update_us[team_number]
        if previous_us is not None and time_us < previous_us:
            raise ValueError("observation time cannot move backwards")


def _sample_robot(
    robot: RobotPlan,
    segments: tuple[PlanSegment, ...],
    time_us: int,
) -> ReservationSample:
    footprint_radius = math.hypot(
        robot.footprint_length_m,
        robot.footprint_width_m,
    ) / 2.0
    previous_segment: PlanSegment | None = None
    for segment in segments:
        segment_start = segment.path[0].time_us
        segment_end = segment.path[-1].time_us
        if time_us < segment_start:
            if previous_segment is None:
                selected = segment
                pose = segment.path[0].pose
            else:
                selected = previous_segment
                pose = previous_segment.path[-1].pose
            return ReservationSample(
                time_us,
                pose,
                footprint_radius + selected.corridor_radius_m,
            )
        if time_us <= segment_end:
            return ReservationSample(
                time_us,
                _interpolate_path(segment.path, time_us),
                footprint_radius + segment.corridor_radius_m,
            )
        previous_segment = segment
    selected = segments[-1]
    return ReservationSample(
        time_us,
        selected.path[-1].pose,
        footprint_radius + selected.corridor_radius_m,
    )


def _sample_tube(tube: ReservationTube, time_us: int) -> ReservationSample:
    if not tube.samples:
        raise ValueError("reservation tube contains no samples")
    if time_us < tube.samples[0].time_us or time_us > tube.samples[-1].time_us:
        raise ValueError("sample time lies outside reservation tube")
    for first, second in zip(tube.samples, tube.samples[1:]):
        if time_us == first.time_us:
            return first
        if time_us <= second.time_us:
            span_us = second.time_us - first.time_us
            ratio = (time_us - first.time_us) / span_us
            return ReservationSample(
                time_us,
                _interpolate_pose(first.pose, second.pose, ratio),
                first.radius_m + (second.radius_m - first.radius_m) * ratio,
            )
    return tube.samples[-1]


def _linear_collision_intervals(
    first_start: ReservationSample,
    first_end: ReservationSample,
    second_start: ReservationSample,
    second_end: ReservationSample,
) -> tuple[tuple[float, float, float], ...]:
    relative_x = first_start.pose.x - second_start.pose.x
    relative_y = first_start.pose.y - second_start.pose.y
    velocity_x = (
        first_end.pose.x
        - first_start.pose.x
        - second_end.pose.x
        + second_start.pose.x
    )
    velocity_y = (
        first_end.pose.y
        - first_start.pose.y
        - second_end.pose.y
        + second_start.pose.y
    )
    radius = first_start.radius_m + second_start.radius_m
    radius_delta = (
        first_end.radius_m
        + second_end.radius_m
        - first_start.radius_m
        - second_start.radius_m
    )
    quadratic = velocity_x**2 + velocity_y**2 - radius_delta**2
    linear = 2.0 * (
        relative_x * velocity_x
        + relative_y * velocity_y
        - radius * radius_delta
    )
    constant = relative_x**2 + relative_y**2 - radius**2
    roots = _quadratic_roots(quadratic, linear, constant)
    cuts = [0.0] + [root for root in roots if 0.0 < root < 1.0] + [1.0]
    cuts = sorted(set(cuts))
    intervals: list[tuple[float, float, float]] = []
    for start, end in zip(cuts, cuts[1:]):
        midpoint = (start + end) / 2.0
        if _polynomial(quadratic, linear, constant, midpoint) <= 1e-12:
            intervals.append(
                (
                    start,
                    end,
                    _minimum_linear_clearance(
                        relative_x,
                        relative_y,
                        velocity_x,
                        velocity_y,
                        radius,
                        radius_delta,
                        start,
                        end,
                    ),
                )
            )
    for root in roots:
        if 0.0 <= root <= 1.0 and abs(
            _polynomial(quadratic, linear, constant, root)
        ) <= 1e-10:
            intervals.append((root, root, 0.0))
    return tuple(_merge_intervals(intervals))


def _quadratic_roots(quadratic: float, linear: float, constant: float) -> tuple[float, ...]:
    if abs(quadratic) <= 1e-14:
        if abs(linear) <= 1e-14:
            return ()
        return (-constant / linear,)
    discriminant = linear**2 - 4.0 * quadratic * constant
    if discriminant < -1e-12:
        return ()
    square_root = math.sqrt(max(0.0, discriminant))
    return (
        (-linear - square_root) / (2.0 * quadratic),
        (-linear + square_root) / (2.0 * quadratic),
    )


def _polynomial(quadratic: float, linear: float, constant: float, value: float) -> float:
    return (quadratic * value + linear) * value + constant


def _minimum_linear_clearance(
    relative_x: float,
    relative_y: float,
    velocity_x: float,
    velocity_y: float,
    radius: float,
    radius_delta: float,
    start: float,
    end: float,
) -> float:
    def clearance(ratio: float) -> float:
        return math.hypot(
            relative_x + velocity_x * ratio,
            relative_y + velocity_y * ratio,
        ) - (radius + radius_delta * ratio)

    low = start
    high = end
    for _ in range(64):
        first_third = (2.0 * low + high) / 3.0
        second_third = (low + 2.0 * high) / 3.0
        if clearance(first_third) <= clearance(second_third):
            high = second_third
        else:
            low = first_third
    return min(clearance(start), clearance(end), clearance((low + high) / 2.0))


def _merge_intervals(
    intervals: Sequence[tuple[float, float, float]],
) -> tuple[tuple[float, float, float], ...]:
    merged: list[tuple[float, float, float]] = []
    for start, end, clearance in sorted(intervals):
        if not merged or start > merged[-1][1] + 1e-9:
            merged.append((start, end, clearance))
            continue
        previous_start, previous_end, previous_clearance = merged[-1]
        merged[-1] = (
            previous_start,
            max(previous_end, end),
            min(previous_clearance, clearance),
        )
    return tuple(merged)


def _clearance(first: ReservationSample, second: ReservationSample) -> float:
    return math.hypot(
        first.pose.x - second.pose.x,
        first.pose.y - second.pose.y,
    ) - (first.radius_m + second.radius_m)


def _interpolate_path(path: tuple[TimedPose, ...], time_us: int) -> Pose2d:
    for first, second in zip(path, path[1:]):
        if time_us == first.time_us:
            return first.pose
        if time_us <= second.time_us:
            ratio = (time_us - first.time_us) / (second.time_us - first.time_us)
            return _interpolate_pose(first.pose, second.pose, ratio)
    return path[-1].pose


def _interpolate_pose(first: Pose2d, second: Pose2d, ratio: float) -> Pose2d:
    heading_delta = (second.heading - first.heading + math.pi) % (2.0 * math.pi) - math.pi
    return Pose2d(
        first.x + (second.x - first.x) * ratio,
        first.y + (second.y - first.y) * ratio,
        first.heading + heading_delta * ratio,
    )


def _clamp_confidence(value: float, initial: float) -> float:
    return min(initial, max(0.0, value))
