"""ROS adapter for the deterministic strategy core."""

from __future__ import annotations

import math

import rclpy
from rclpy.node import Node

from frc_autonomy_msgs.msg import (
    AutonomyStatus,
    CandidateTask as CandidateTaskMessage,
    DecisionTrace,
    NavigationGoal,
    NavigationState,
    ReservationTubeArray,
    RobotTrack,
    UtilityBreakdown as UtilityBreakdownMessage,
    WorldState,
)

from .evaluator import StrategyEvaluator
from .executive import StrategyExecutive
from .model import (
    DynamicRobot,
    FieldTarget,
    Pose2,
    ReservationZone,
    StrategySnapshot,
    StrategyWeights,
    WorldObject,
)
from .task_generator import TaskGenerator


class StrategyNode(Node):
    def __init__(self) -> None:
        super().__init__("strategy_node")
        self._declare_parameters()
        self._generator = TaskGenerator(_field_targets(self.get_parameter("field_targets").value))
        weights = StrategyWeights(
            ranking_point=float(self.get_parameter("weight_ranking_point").value),
            future=float(self.get_parameter("weight_future").value),
            time=float(self.get_parameter("weight_time").value),
            collision=float(self.get_parameter("weight_collision").value),
            failure=float(self.get_parameter("weight_failure").value),
            overlap=float(self.get_parameter("weight_overlap").value),
            penalty=float(self.get_parameter("weight_penalty").value),
            uncertainty=float(self.get_parameter("weight_uncertainty").value),
            hysteresis=float(self.get_parameter("hysteresis").value),
        )
        self._executive = StrategyExecutive(
            StrategyEvaluator(weights, int(self.get_parameter("rollout_count").value)),
            hysteresis=weights.hysteresis,
        )
        self._world = None
        self._status = None
        self._reservations = None
        self._match_start_us = None
        self._has_cube = bool(self.get_parameter("initial_has_cube").value)
        self._completed: set[str] = set()
        self._last_goal_task = ""
        self._navigation_emergency = False
        self._trace_publisher = self.create_publisher(DecisionTrace, "/strategy/decision", 10)
        self._goal_publisher = self.create_publisher(NavigationGoal, "/strategy/navigation_goal", 10)
        self.create_subscription(WorldState, "/world/state", self._on_world, 10)
        self.create_subscription(AutonomyStatus, "/autonomy/status", self._on_status, 10)
        self.create_subscription(ReservationTubeArray, "/alliance/reservations", self._on_reservations, 10)
        self.create_subscription(NavigationState, "/navigation/state", self._on_navigation, 10)
        self.create_timer(1.0 / float(self.get_parameter("tick_rate_hz").value), self._tick)

    def _declare_parameters(self) -> None:
        self.declare_parameter("seed", 2018)
        self.declare_parameter("tick_rate_hz", 5.0)
        self.declare_parameter("initial_has_cube", False)
        self.declare_parameter("our_switch_side", "LEFT")
        self.declare_parameter("scale_side", "LEFT")
        self.declare_parameter("match_duration_s", 15.0)
        self.declare_parameter("rollout_count", 64)
        self.declare_parameter("hysteresis", 2.0)
        self.declare_parameter("field_targets", [])
        self.declare_parameter("capabilities", ["drive", "intake", "switch", "scale"])
        for name, default in (
            ("ranking_point", 1.0), ("future", 0.6), ("time", 0.8),
            ("collision", 60.0), ("failure", 12.0), ("overlap", 25.0),
            ("penalty", 30.0), ("uncertainty", 8.0),
        ):
            self.declare_parameter(f"weight_{name}", default)

    def _on_world(self, message) -> None:
        self._world = message

    def _on_reservations(self, message) -> None:
        self._reservations = message

    def _on_status(self, message) -> None:
        self._status = message
        if message.mode in {"AUTONOMOUS", "AUTONOMOUS_SIM"}:
            if self._match_start_us is None:
                self._match_start_us = self._now_us()
        else:
            self._match_start_us = None
            self._completed.clear()

    def _on_navigation(self, message) -> None:
        self._navigation_emergency = (
            message.state == NavigationState.STOPPED
            and message.obstacle_reason == "DYNAMIC_COLLISION_STOP"
        )
        if not message.goal_reached or not message.goal_task_id:
            return
        if message.goal_task_id == self._last_goal_task:
            self._completed.add(message.goal_task_id)
            if message.goal_task_id.startswith("pickup:"):
                self._has_cube = True
            elif message.goal_task_id.startswith("score_"):
                self._has_cube = False

    def _tick(self) -> None:
        if self._world is None or self._status is None or self._match_start_us is None:
            return
        now_us = self._now_us()
        snapshot = self._snapshot(now_us)
        candidates = tuple(
            task for task in self._generator.generate(snapshot)
            if task.task_id not in self._completed
        )
        decision = self._executive.decide(
            snapshot,
            candidates,
            int(self.get_parameter("seed").value),
            emergency=self._navigation_emergency,
        )
        if decision is None:
            return
        self._last_goal_task = decision.chosen.task_id
        stamp = self.get_clock().now().to_msg()
        self._trace_publisher.publish(_trace_message(decision, stamp))
        self._goal_publisher.publish(_goal_message(decision, stamp, now_us))

    def _snapshot(self, now_us: int) -> StrategySnapshot:
        pose = Pose2(self._status.pose.x, self._status.pose.y, self._status.pose.theta)
        elapsed_s = max(0.0, (now_us - self._match_start_us) / 1e6)
        remaining_s = max(0.0, float(self.get_parameter("match_duration_s").value) - elapsed_s)
        objects = tuple(
            WorldObject(item.object_id, item.class_name, _message_pose(item.pose), item.confidence)
            for item in self._world.game_objects
        )
        robots = tuple(
            DynamicRobot(
                item.track_id,
                "ALLY" if item.affiliation == RobotTrack.AFFILIATION_ALLY else "OPPONENT",
                _message_pose(item.pose),
                item.velocity.twist.linear.x,
                item.velocity.twist.linear.y,
                max(item.size.x, item.size.y) / 2.0,
                item.confidence,
            )
            for item in self._world.robots
        )
        reservations = _reservation_zones(self._reservations, self._match_start_us)
        return StrategySnapshot(
            world_version=int(self._world.version), now_us=now_us,
            match_time_remaining_s=remaining_s, autonomous=True,
            perception_fresh=bool(self._world.perception_fresh), robot_pose=pose,
            has_cube=self._has_cube, game_objects=objects, robots=robots,
            reservations=reservations,
            capabilities=frozenset(self.get_parameter("capabilities").value),
            our_switch_side=str(self.get_parameter("our_switch_side").value).upper(),
            scale_side=str(self.get_parameter("scale_side").value).upper(),
            uncertainty=min(1.0, max(0.0, self._world.newest_source_age_us / 500_000.0)),
        )

    def _now_us(self) -> int:
        return self.get_clock().now().nanoseconds // 1_000


def _field_targets(values) -> tuple[FieldTarget, ...]:
    targets = []
    for value in values:
        parts = [part.strip() for part in value.split(",")]
        if len(parts) != 10:
            raise ValueError(f"invalid field target: {value}")
        targets.append(FieldTarget(
            parts[0], parts[1], Pose2(float(parts[2]), float(parts[3]), float(parts[4])),
            parts[5], float(parts[6]), float(parts[7]), float(parts[8]), parts[9],
        ))
    return tuple(targets)


def _message_pose(value) -> Pose2:
    q = value.pose.orientation
    heading = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y*q.y + q.z*q.z))
    return Pose2(value.pose.position.x, value.pose.position.y, heading)


def _reservation_zones(message, match_start_us) -> tuple[ReservationZone, ...]:
    if message is None or match_start_us is None:
        return ()
    zones = []
    for tube in message.tubes:
        for sample in tube.samples:
            zones.append(ReservationZone(
                tube.robot_label,
                match_start_us + sample.time_us - 50_000,
                match_start_us + sample.time_us + 50_000,
                Pose2(sample.pose.x, sample.pose.y, sample.pose.theta),
                sample.radius_m,
                tube.confidence,
            ))
    return tuple(zones)


def _candidate_message(task) -> CandidateTaskMessage:
    message = CandidateTaskMessage()
    message.task_id = task.task_id; message.task_type = task.task_type; message.target_id = task.target_id
    message.target_pose.x = task.target_pose.x; message.target_pose.y = task.target_pose.y; message.target_pose.theta = task.target_pose.heading
    message.estimated_duration_us = round(task.estimated_duration_s * 1e6)
    message.success_probability = task.success_probability; message.collision_probability = task.collision_probability
    message.reservation_cost = task.reservation_cost; message.required_capability = task.required_capability
    message.hard_valid = task.hard_valid; message.rejection_reason = task.rejection_reason
    message.intake_action = task.intake_action; message.elevator_target_m = task.elevator_target_m
    return message


def _utility_message(value) -> UtilityBreakdownMessage:
    message = UtilityBreakdownMessage()
    for field in (
        "task_id", "direct_score", "ownership_score", "ranking_point_value", "future_value",
        "time_penalty", "collision_penalty", "failure_penalty", "overlap_penalty",
        "rule_penalty", "uncertainty_penalty", "total",
    ):
        setattr(message, field, getattr(value, field))
    return message


def _trace_message(decision, stamp) -> DecisionTrace:
    message = DecisionTrace(); message.header.stamp = stamp; message.header.frame_id = "map"
    message.world_version = decision.world_version; message.decision_sequence = decision.sequence
    message.seed = decision.seed; message.chosen_task_id = decision.chosen.task_id
    message.candidates = [_candidate_message(item[0]) for item in decision.ranked]
    message.utilities = [_utility_message(item[1]) for item in decision.ranked]
    message.replan_reason = decision.replan_reason; message.counterfactual = decision.counterfactual
    message.decision_hash = decision.decision_hash
    return message


def _goal_message(decision, stamp, now_us) -> NavigationGoal:
    task = decision.chosen; message = NavigationGoal(); message.header.stamp = stamp; message.header.frame_id = "map"
    message.world_version = decision.world_version; message.task_id = task.task_id
    message.target_pose.x = task.target_pose.x; message.target_pose.y = task.target_pose.y; message.target_pose.theta = task.target_pose.heading
    message.position_tolerance_m = 0.15; message.heading_tolerance_rad = 0.15; message.valid_until_us = now_us + 500_000
    message.intake_action = {"STOP": 0, "INTAKE": 1, "HOLD": 2, "EJECT": 3}.get(task.intake_action, 0)
    message.mechanism_enabled = task.task_type not in {"WAIT", "CROSS_LINE"}
    message.elevator_target_m = task.elevator_target_m
    return message


def main(args=None) -> None:
    rclpy.init(args=args); node = StrategyNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()
