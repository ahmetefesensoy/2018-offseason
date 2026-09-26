"""ROS-independent navigation value types."""

from dataclasses import dataclass
from typing import Tuple


@dataclass(frozen=True)
class Pose2:
    x: float
    y: float
    heading: float


@dataclass(frozen=True)
class Rectangle:
    min_x: float
    min_y: float
    max_x: float
    max_y: float


@dataclass(frozen=True)
class CircleObstacle:
    obstacle_id: str
    x: float
    y: float
    vx_mps: float
    vy_mps: float
    radius_m: float
    hard: bool
    confidence: float


@dataclass(frozen=True)
class ReservationCircle:
    start_us: int
    end_us: int
    x: float
    y: float
    radius_m: float
    confidence: float


@dataclass(frozen=True)
class NavigationWorld:
    now_us: int
    perception_fresh: bool
    field_length_m: float
    field_width_m: float
    robot_radius_m: float
    static_rectangles: Tuple[Rectangle, ...]
    obstacles: Tuple[CircleObstacle, ...]
    reservations: Tuple[ReservationCircle, ...]


@dataclass(frozen=True)
class NavigationGoal:
    task_id: str
    pose: Pose2
    position_tolerance_m: float
    heading_tolerance_rad: float
    valid_until_us: int
    intake_action: int
    mechanism_enabled: bool
    elevator_target_m: float


@dataclass(frozen=True)
class PathPoint:
    time_us: int
    x: float
    y: float
    heading: float


@dataclass(frozen=True)
class VelocityCommand:
    armed: bool
    vx_mps: float
    vy_mps: float
    omega_radps: float
    intake_action: int
    mechanism_enabled: bool
    elevator_target_m: float


@dataclass(frozen=True)
class MonitorResult:
    command: VelocityCommand
    min_clearance_m: float
    reason: str

