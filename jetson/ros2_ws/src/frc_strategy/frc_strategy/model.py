"""ROS-independent strategy models shared by generation, evaluation, and replay."""

from __future__ import annotations

from dataclasses import dataclass
from typing import FrozenSet, Tuple


@dataclass(frozen=True, order=True)
class Pose2:
    x: float
    y: float
    heading: float


@dataclass(frozen=True)
class WorldObject:
    object_id: str
    class_name: str
    pose: Pose2
    confidence: float


@dataclass(frozen=True)
class DynamicRobot:
    track_id: str
    affiliation: str
    pose: Pose2
    vx_mps: float
    vy_mps: float
    radius_m: float
    confidence: float


@dataclass(frozen=True)
class ReservationZone:
    robot_label: str
    start_us: int
    end_us: int
    pose: Pose2
    radius_m: float
    confidence: float


@dataclass(frozen=True)
class FieldTarget:
    target_id: str
    task_type: str
    pose: Pose2
    required_capability: str
    base_score: float
    duration_s: float
    elevator_target_m: float = 0.0
    intake_action: str = "HOLD"


@dataclass(frozen=True)
class StrategySnapshot:
    world_version: int
    now_us: int
    match_time_remaining_s: float
    autonomous: bool
    perception_fresh: bool
    robot_pose: Pose2
    has_cube: bool
    game_objects: Tuple[WorldObject, ...]
    robots: Tuple[DynamicRobot, ...]
    reservations: Tuple[ReservationZone, ...]
    capabilities: FrozenSet[str]
    our_switch_side: str
    scale_side: str
    uncertainty: float


@dataclass(frozen=True)
class CandidateTask:
    task_id: str
    task_type: str
    target_id: str
    target_pose: Pose2
    estimated_duration_s: float
    duration_std_s: float
    success_probability: float
    collision_probability: float
    reservation_cost: float
    required_capability: str
    base_score: float
    ownership_value: float = 0.0
    ranking_point_value: float = 0.0
    future_value: float = 0.0
    rule_penalty_probability: float = 0.0
    intake_action: str = "STOP"
    elevator_target_m: float = 0.0
    hard_valid: bool = True
    rejection_reason: str = ""


@dataclass(frozen=True)
class StrategyWeights:
    ranking_point: float = 1.0
    future: float = 0.6
    time: float = 0.8
    collision: float = 60.0
    failure: float = 12.0
    overlap: float = 25.0
    penalty: float = 30.0
    uncertainty: float = 8.0
    hysteresis: float = 2.0


@dataclass(frozen=True)
class UtilityBreakdown:
    task_id: str
    direct_score: float
    ownership_score: float
    ranking_point_value: float
    future_value: float
    time_penalty: float
    collision_penalty: float
    failure_penalty: float
    overlap_penalty: float
    rule_penalty: float
    uncertainty_penalty: float
    total: float


@dataclass(frozen=True)
class Decision:
    world_version: int
    sequence: int
    seed: int
    chosen: CandidateTask
    ranked: Tuple[Tuple[CandidateTask, UtilityBreakdown], ...]
    replan_reason: str
    counterfactual: str
    decision_hash: str

