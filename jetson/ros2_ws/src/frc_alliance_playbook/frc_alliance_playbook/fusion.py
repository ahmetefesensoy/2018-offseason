"""ROS-independent match clock and stable teammate-to-plan association."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence

from .reservations import ReservationTube, sample_reservation
from .schema import Pose2d


AUTONOMOUS_MODES = frozenset(("AUTONOMOUS", "AUTONOMOUS_SIM"))


class MatchClock:
    """Derive plan time from explicit roboRIO autonomous mode transitions."""

    def __init__(self) -> None:
        self._epoch_us: int | None = None
        self._last_update_us: int | None = None

    @property
    def active(self) -> bool:
        return self._epoch_us is not None

    def update(self, mode: str, now_us: int) -> bool:
        _validate_time(now_us)
        rewound = self._last_update_us is not None and now_us < self._last_update_us
        if rewound:
            self._epoch_us = None
        autonomous = mode.strip().upper() in AUTONOMOUS_MODES
        started = autonomous and self._epoch_us is None
        if started:
            self._epoch_us = now_us
        elif not autonomous:
            self._epoch_us = None
        self._last_update_us = now_us
        return started

    def elapsed_us(self, now_us: int) -> int | None:
        _validate_time(now_us)
        if self._epoch_us is None or now_us < self._epoch_us:
            return None
        return now_us - self._epoch_us


@dataclass(frozen=True)
class ObservedAlly:
    track_id: str
    pose: Pose2d


class StableAllyMatcher:
    """Bind stable perception track IDs to teammate plans for one match."""

    def __init__(self, *, missing_binding_grace_us: int) -> None:
        if (
            isinstance(missing_binding_grace_us, bool)
            or not isinstance(missing_binding_grace_us, int)
            or missing_binding_grace_us < 0
        ):
            raise ValueError("missing_binding_grace_us must be a non-negative integer")
        self._grace_us = missing_binding_grace_us
        self._team_to_track: dict[int, str] = {}
        self._last_seen_us: dict[int, int] = {}
        self._last_now_us: int | None = None

    def reset(self) -> None:
        self._team_to_track.clear()
        self._last_seen_us.clear()
        self._last_now_us = None

    def assign(
        self,
        tubes: Sequence[ReservationTube],
        observations: Sequence[ObservedAlly],
        now_us: int,
    ) -> dict[int, ObservedAlly]:
        _validate_time(now_us)
        if self._last_now_us is not None and now_us < self._last_now_us:
            self.reset()
        self._last_now_us = now_us

        by_track: dict[str, ObservedAlly] = {}
        for observation in observations:
            if not observation.track_id:
                raise ValueError("ally observation track_id cannot be blank")
            if observation.track_id in by_track:
                raise ValueError(f"duplicate ally track_id: {observation.track_id}")
            if not all(
                math.isfinite(value)
                for value in (
                    observation.pose.x,
                    observation.pose.y,
                    observation.pose.heading,
                )
            ):
                raise ValueError("ally observation pose must be finite")
            by_track[observation.track_id] = observation

        assignments: dict[int, ObservedAlly] = {}
        for team_number, track_id in tuple(self._team_to_track.items()):
            observation = by_track.get(track_id)
            if observation is not None:
                assignments[team_number] = observation
                self._last_seen_us[team_number] = now_us
            elif now_us - self._last_seen_us[team_number] > self._grace_us:
                del self._team_to_track[team_number]
                del self._last_seen_us[team_number]

        bound_tracks = set(self._team_to_track.values())
        bound_teams = set(self._team_to_track)
        candidates: list[tuple[float, str, int, ObservedAlly]] = []
        for observation in observations:
            if observation.track_id in bound_tracks:
                continue
            for tube in tubes:
                if tube.team_number in bound_teams:
                    continue
                planned = sample_reservation(tube, now_us).pose
                distance = math.hypot(
                    observation.pose.x - planned.x,
                    observation.pose.y - planned.y,
                )
                candidates.append(
                    (distance, observation.track_id, tube.team_number, observation)
                )
        used_tracks = set(bound_tracks)
        used_teams = set(bound_teams)
        for _, track_id, team_number, observation in sorted(candidates):
            if track_id in used_tracks or team_number in used_teams:
                continue
            self._team_to_track[team_number] = track_id
            self._last_seen_us[team_number] = now_us
            assignments[team_number] = observation
            used_tracks.add(track_id)
            used_teams.add(team_number)
        return dict(sorted(assignments.items()))


def _validate_time(value: int) -> None:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ValueError("time must be a non-negative integer")
