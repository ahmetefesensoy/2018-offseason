"""Atomic, versioned snapshots built from semantic observation batches."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable

from .model import MultiObjectTracker, Observation, ObservationError, Track


class WorldStoreError(ValueError):
    """An observation frame cannot be committed to the world model."""


@dataclass(frozen=True)
class WorldSnapshot:
    version: int
    source_timestamp_us: int
    perception_fresh: bool
    newest_source_age_us: int
    game_objects: tuple[Track, ...]
    robots: tuple[Track, ...]


class WorldStore:
    def __init__(
        self,
        tracker: MultiObjectTracker,
        source_timeout_us: int,
    ) -> None:
        if source_timeout_us <= 0:
            raise ValueError("source timeout must be positive")
        self._tracker = tracker
        self._source_timeout_us = source_timeout_us
        self._version = 0
        self._last_source_timestamp_us = 0

    def ingest(
        self,
        observations: Iterable[Observation],
        source_timestamp_us: int,
        received_timestamp_us: int,
    ) -> WorldSnapshot:
        batch = tuple(observations)
        self._validate_frame_timestamps(
            batch,
            source_timestamp_us,
            received_timestamp_us,
        )
        try:
            self._tracker.update(batch, received_timestamp_us)
        except ObservationError as error:
            raise WorldStoreError(str(error)) from error

        self._last_source_timestamp_us = source_timestamp_us
        self._version += 1
        return self.snapshot(received_timestamp_us)

    def snapshot(self, now_us: int) -> WorldSnapshot:
        tracks = self._tracker.snapshot(now_us)
        game_objects = tuple(track for track in tracks if track.class_name != "robot")
        robots = tuple(track for track in tracks if track.class_name == "robot")
        if self._last_source_timestamp_us == 0:
            source_age_us = -1
            perception_fresh = False
        else:
            source_age_us = now_us - self._last_source_timestamp_us
            perception_fresh = 0 <= source_age_us <= self._source_timeout_us
        return WorldSnapshot(
            version=self._version,
            source_timestamp_us=self._last_source_timestamp_us,
            perception_fresh=perception_fresh,
            newest_source_age_us=source_age_us,
            game_objects=game_objects,
            robots=robots,
        )

    def _validate_frame_timestamps(
        self,
        batch: tuple[Observation, ...],
        source_timestamp_us: int,
        received_timestamp_us: int,
    ) -> None:
        if source_timestamp_us <= 0 or received_timestamp_us <= 0:
            raise WorldStoreError("source and receive timestamps must be positive")
        if source_timestamp_us > received_timestamp_us:
            raise WorldStoreError("source timestamp is in the future")
        if received_timestamp_us - source_timestamp_us > self._source_timeout_us:
            raise WorldStoreError("observation frame arrived stale")
        if source_timestamp_us <= self._last_source_timestamp_us:
            raise WorldStoreError("source timestamp is not strictly increasing")
        if any(item.timestamp_us != source_timestamp_us for item in batch):
            raise WorldStoreError("observations do not share the frame timestamp")
