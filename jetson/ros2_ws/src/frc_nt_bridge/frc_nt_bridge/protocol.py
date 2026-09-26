"""Pure, ROS-independent command framing for deterministic unit tests."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Optional, Tuple


class ProtocolError(ValueError):
    """A command cannot be represented safely on the roboRIO link."""


@dataclass(frozen=True)
class CommandRequest:
    armed: bool
    vx_mps: float
    vy_mps: float
    omega_radps: float
    valid_for_us: int


@dataclass(frozen=True)
class CommandFrame:
    session_id: str
    armed: bool
    sent_at_us: int
    valid_until_us: int
    vx_mps: float
    vy_mps: float
    omega_radps: float
    sequence: int

    def publish_operations(self) -> Tuple[Tuple[str, object], ...]:
        """Return the strict publish order; commit_sequence must remain last."""
        return (
            ("session_id", self.session_id),
            ("armed", self.armed),
            ("sent_at_us", self.sent_at_us),
            ("valid_until_us", self.valid_until_us),
            ("vx_mps", self.vx_mps),
            ("vy_mps", self.vy_mps),
            ("omega_radps", self.omega_radps),
            ("sequence", self.sequence),
            ("commit_sequence", self.sequence),
        )


class FrameBuilder:
    def __init__(
        self,
        max_translation_speed_mps: float,
        max_rotation_speed_radps: float,
        max_validity_us: int,
    ) -> None:
        self._max_translation_speed_mps = max_translation_speed_mps
        self._max_rotation_speed_radps = max_rotation_speed_radps
        self._max_validity_us = max_validity_us
        self._sequence = 0

    def synchronize_accepted_sequence(self, accepted_sequence: int) -> None:
        """Keep monotonicity when the Jetson process restarts mid-roboRIO boot."""
        self._sequence = max(self._sequence, accepted_sequence)

    def build(
        self,
        request: CommandRequest,
        session_id: str,
        now_server_us: int,
    ) -> CommandFrame:
        if not session_id:
            raise ProtocolError("roboRIO boot session is unavailable")
        if now_server_us <= 0:
            raise ProtocolError("invalid NT4 server timestamp")
        if request.valid_for_us <= 0 or request.valid_for_us > self._max_validity_us:
            raise ProtocolError("validity window is outside the safety limit")

        velocity = (request.vx_mps, request.vy_mps, request.omega_radps)
        if not all(math.isfinite(value) for value in velocity):
            raise ProtocolError("velocity contains a non-finite value")
        if math.hypot(request.vx_mps, request.vy_mps) > self._max_translation_speed_mps:
            raise ProtocolError("translation velocity exceeds the safety limit")
        if abs(request.omega_radps) > self._max_rotation_speed_radps:
            raise ProtocolError("rotation velocity exceeds the safety limit")

        self._sequence += 1
        if request.armed:
            vx_mps, vy_mps, omega_radps = velocity
        else:
            vx_mps, vy_mps, omega_radps = (0.0, 0.0, 0.0)

        return CommandFrame(
            session_id=session_id,
            armed=request.armed,
            sent_at_us=now_server_us,
            valid_until_us=now_server_us + request.valid_for_us,
            vx_mps=vx_mps,
            vy_mps=vy_mps,
            omega_radps=omega_radps,
            sequence=self._sequence,
        )


def server_time_us(local_monotonic_us: int, server_offset_us: Optional[int]) -> int:
    """Convert local monotonic time to the NT4 server/roboRIO FPGA time domain."""
    if server_offset_us is None:
        raise ProtocolError("NT4 time synchronization has not completed")
    return local_monotonic_us + server_offset_us
