"""ROS-independent semantic object tracking primitives."""

from __future__ import annotations

from dataclasses import dataclass, replace
import math
from typing import Iterable


class ObservationError(ValueError):
    """A semantic observation batch is unsafe to apply."""


@dataclass(frozen=True)
class TrackerConfig:
    association_gate_m: float = 0.75
    position_gain: float = 0.65
    velocity_gain: float = 0.35
    track_timeout_us: int = 750_000
    max_observation_age_us: int = 250_000
    min_confidence: float = 0.20
    prediction_horizon_s: float = 2.0
    prediction_step_s: float = 0.1
    process_noise_variance: float = 0.16


@dataclass(frozen=True)
class Observation:
    detection_id: str
    class_name: str
    affiliation: int
    x: float
    y: float
    yaw: float
    size_x: float
    size_y: float
    confidence: float
    variance_x: float
    variance_y: float
    timestamp_us: int


@dataclass(frozen=True)
class Prediction:
    horizon_s: float
    x: float
    y: float
    yaw: float
    variance_x: float
    variance_y: float


@dataclass(frozen=True)
class Track:
    track_id: str
    class_name: str
    affiliation: int
    x: float
    y: float
    yaw: float
    vx: float
    vy: float
    size_x: float
    size_y: float
    confidence: float
    variance_x: float
    variance_y: float
    last_seen_us: int
    predictions: tuple[Prediction, ...] = ()


class MultiObjectTracker:
    def __init__(self, config: TrackerConfig) -> None:
        self._config = config
        self._tracks: dict[str, Track] = {}
        self._next_track_number = 1

    def update(
        self,
        observations: Iterable[Observation],
        now_us: int,
    ) -> tuple[Track, ...]:
        batch = tuple(observations)
        for item in batch:
            self._validate(item, now_us)

        self._expire_tracks(now_us)
        available_track_ids = set(self._tracks)
        for item in sorted(
            batch,
            key=lambda value: (
                value.class_name,
                value.detection_id,
                value.x,
                value.y,
            ),
        ):
            track_id = self._nearest_compatible_track(item, available_track_ids)
            if track_id is None:
                track_id = self._create_track(item)
            else:
                self._tracks[track_id] = self._correct_track(
                    self._tracks[track_id],
                    item,
                )
                available_track_ids.remove(track_id)
        return self.snapshot(now_us)

    def snapshot(self, now_us: int) -> tuple[Track, ...]:
        self._expire_tracks(now_us)
        return tuple(
            self._with_predictions(self._tracks[key]) for key in sorted(self._tracks)
        )

    def _create_track(self, item: Observation) -> str:
        track_id = f"trk-{self._next_track_number:06d}"
        self._next_track_number += 1
        self._tracks[track_id] = Track(
            track_id=track_id,
            class_name=item.class_name,
            affiliation=item.affiliation,
            x=item.x,
            y=item.y,
            yaw=item.yaw,
            vx=0.0,
            vy=0.0,
            size_x=item.size_x,
            size_y=item.size_y,
            confidence=item.confidence,
            variance_x=item.variance_x,
            variance_y=item.variance_y,
            last_seen_us=item.timestamp_us,
        )
        return track_id

    def _nearest_compatible_track(
        self,
        item: Observation,
        available_track_ids: set[str],
    ) -> str | None:
        candidates: list[tuple[float, str]] = []
        for track_id in available_track_ids:
            track = self._tracks[track_id]
            if not self._is_compatible(track, item):
                continue
            dt_seconds = max((item.timestamp_us - track.last_seen_us) / 1e6, 0.0)
            predicted_x = track.x + track.vx * dt_seconds
            predicted_y = track.y + track.vy * dt_seconds
            distance = math.hypot(item.x - predicted_x, item.y - predicted_y)
            if distance <= self._config.association_gate_m:
                candidates.append((distance, track_id))
        return min(candidates)[1] if candidates else None

    @staticmethod
    def _is_compatible(track: Track, item: Observation) -> bool:
        if track.class_name != item.class_name:
            return False
        if item.class_name != "robot":
            return True
        return (
            track.affiliation == item.affiliation
            or track.affiliation == 0
            or item.affiliation == 0
        )

    def _correct_track(self, track: Track, item: Observation) -> Track:
        dt_seconds = (item.timestamp_us - track.last_seen_us) / 1e6
        if dt_seconds <= 0.0:
            measured_vx = track.vx
            measured_vy = track.vy
            predicted_x = track.x
            predicted_y = track.y
        else:
            measured_vx = (item.x - track.x) / dt_seconds
            measured_vy = (item.y - track.y) / dt_seconds
            predicted_x = track.x + track.vx * dt_seconds
            predicted_y = track.y + track.vy * dt_seconds

        position_gain = self._config.position_gain
        velocity_gain = self._config.velocity_gain
        predicted_variance_x = (
            track.variance_x
            + self._config.process_noise_variance * dt_seconds * dt_seconds
        )
        predicted_variance_y = (
            track.variance_y
            + self._config.process_noise_variance * dt_seconds * dt_seconds
        )
        affiliation = (
            item.affiliation if track.affiliation == 0 else track.affiliation
        )
        return Track(
            track_id=track.track_id,
            class_name=track.class_name,
            affiliation=affiliation,
            x=predicted_x + position_gain * (item.x - predicted_x),
            y=predicted_y + position_gain * (item.y - predicted_y),
            yaw=item.yaw,
            vx=track.vx + velocity_gain * (measured_vx - track.vx),
            vy=track.vy + velocity_gain * (measured_vy - track.vy),
            size_x=track.size_x + position_gain * (item.size_x - track.size_x),
            size_y=track.size_y + position_gain * (item.size_y - track.size_y),
            confidence=track.confidence
            + position_gain * (item.confidence - track.confidence),
            variance_x=(1.0 - position_gain) * predicted_variance_x
            + position_gain * item.variance_x,
            variance_y=(1.0 - position_gain) * predicted_variance_y
            + position_gain * item.variance_y,
            last_seen_us=item.timestamp_us,
        )

    def _expire_tracks(self, now_us: int) -> None:
        expired = [
            track_id
            for track_id, track in self._tracks.items()
            if now_us - track.last_seen_us > self._config.track_timeout_us
        ]
        for track_id in expired:
            del self._tracks[track_id]

    def _with_predictions(self, track: Track) -> Track:
        if track.class_name != "robot":
            return track
        step_count = round(
            self._config.prediction_horizon_s / self._config.prediction_step_s
        )
        predictions = tuple(
            self._prediction_at(track, (index + 1) * self._config.prediction_step_s)
            for index in range(step_count)
        )
        return replace(track, predictions=predictions)

    def _prediction_at(self, track: Track, horizon_s: float) -> Prediction:
        covariance_growth = self._config.process_noise_variance * horizon_s**2
        return Prediction(
            horizon_s=horizon_s,
            x=track.x + track.vx * horizon_s,
            y=track.y + track.vy * horizon_s,
            yaw=track.yaw,
            variance_x=track.variance_x + covariance_growth,
            variance_y=track.variance_y + covariance_growth,
        )

    def _validate(self, item: Observation, now_us: int) -> None:
        numeric_values = (
            item.x,
            item.y,
            item.yaw,
            item.size_x,
            item.size_y,
            item.confidence,
            item.variance_x,
            item.variance_y,
        )
        if not item.detection_id or not item.class_name:
            raise ObservationError("detection and class identifiers are required")
        if item.affiliation not in (0, 1, 2):
            raise ObservationError("affiliation is outside the message contract")
        if not all(math.isfinite(value) for value in numeric_values):
            raise ObservationError("observation contains a non-finite value")
        if item.size_x <= 0.0 or item.size_y <= 0.0:
            raise ObservationError("observation footprint must be positive")
        if not self._config.min_confidence <= item.confidence <= 1.0:
            raise ObservationError("observation confidence is outside the accepted range")
        if item.variance_x < 0.0 or item.variance_y < 0.0:
            raise ObservationError("observation covariance cannot be negative")
        if item.timestamp_us <= 0 or item.timestamp_us > now_us:
            raise ObservationError("observation timestamp is invalid")
        if now_us - item.timestamp_us > self._config.max_observation_age_us:
            raise ObservationError("observation is stale")
