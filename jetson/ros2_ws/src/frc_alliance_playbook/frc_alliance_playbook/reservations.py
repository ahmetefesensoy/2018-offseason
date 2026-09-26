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
    """Return sampled time intervals where two reservation discs overlap."""
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
            active_start: int | None = None
            active_end = 0
            minimum_clearance = math.inf
            for time_us in sorted(times):
                first_sample = _sample_tube(first, time_us)
                second_sample = _sample_tube(second, time_us)
                center_distance = math.hypot(
                    first_sample.pose.x - second_sample.pose.x,
                    first_sample.pose.y - second_sample.pose.y,
                )
                clearance = center_distance - (
                    first_sample.radius_m + second_sample.radius_m
                )
                if clearance <= 0.0:
                    if active_start is None:
                        active_start = time_us
                        minimum_clearance = clearance
                    else:
                        minimum_clearance = min(minimum_clearance, clearance)
                    active_end = time_us
                elif active_start is not None:
                    conflicts.append(
                        ReservationConflict(
                            first.team_number,
                            second.team_number,
                            active_start,
                            active_end,
                            minimum_clearance,
                        )
                    )
                    active_start = None
                    minimum_clearance = math.inf
            if active_start is not None:
                conflicts.append(
                    ReservationConflict(
                        first.team_number,
                        second.team_number,
                        active_start,
                        active_end,
                        minimum_clearance,
                    )
                )
    return tuple(conflicts)


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
