"""ROS node owning dynamic motion commands and the final Jetson collision filter."""

from __future__ import annotations

from dataclasses import replace
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node

from frc_autonomy_msgs.msg import (
    AutonomyCommand,
    AutonomyStatus,
    NavigationGoal as NavigationGoalMessage,
    NavigationState as NavigationStateMessage,
    ReservationTubeArray,
    RobotTrack,
    TimedPose,
    WorldState,
)

from .collision_monitor import CollisionMonitor
from .controller import HolonomicController
from .costmap import TemporalCostmap
from .model import (
    CircleObstacle, NavigationGoal, NavigationWorld, Pose2, Rectangle,
    ReservationCircle, VelocityCommand,
)
from .planner import TemporalAStarPlanner


class NavigationNode(Node):
    def __init__(self) -> None:
        super().__init__("navigation_node")
        self._declare_parameters()
        self._planner = TemporalAStarPlanner(
            float(self.get_parameter("grid_resolution_m").value),
            float(self.get_parameter("time_step_s").value),
            float(self.get_parameter("max_speed_mps").value),
        )
        self._controller = HolonomicController(
            float(self.get_parameter("max_speed_mps").value),
            float(self.get_parameter("max_omega_radps").value),
        )
        self._monitor = CollisionMonitor(
            float(self.get_parameter("stop_clearance_m").value),
            float(self.get_parameter("slow_clearance_m").value),
        )
        self._status = None; self._world = None; self._goal = None; self._reservations = None
        self._world_received_us = 0; self._goal_received_us = 0; self._match_start_us = None
        self._path = (); self._plan_revision = 0; self._last_reason = "NO_INPUT"
        self._command_publisher = self.create_publisher(AutonomyCommand, "/autonomy/command", 10)
        self._path_publisher = self.create_publisher(Path, "/navigation/path", 10)
        self._state_publisher = self.create_publisher(NavigationStateMessage, "/navigation/state", 10)
        self.create_subscription(AutonomyStatus, "/autonomy/status", self._on_status, 10)
        self.create_subscription(WorldState, "/world/state", self._on_world, 10)
        self.create_subscription(NavigationGoalMessage, "/strategy/navigation_goal", self._on_goal, 10)
        self.create_subscription(ReservationTubeArray, "/alliance/reservations", self._on_reservations, 10)
        self.create_timer(1.0 / float(self.get_parameter("planner_rate_hz").value), self._plan)
        self.create_timer(1.0 / float(self.get_parameter("controller_rate_hz").value), self._control)

    def _declare_parameters(self) -> None:
        defaults = {
            "field_length_m": 16.46, "field_width_m": 8.23, "robot_radius_m": 0.55,
            "max_speed_mps": 0.75, "max_omega_radps": 1.5, "planner_rate_hz": 5.0,
            "controller_rate_hz": 30.0, "world_timeout_us": 300_000,
            "goal_timeout_us": 500_000, "grid_resolution_m": 0.25, "time_step_s": 0.20,
            "stop_clearance_m": 0.20, "slow_clearance_m": 0.80,
            "static_rectangles": [],
        }
        for name, value in defaults.items(): self.declare_parameter(name, value)

    def _on_status(self, message) -> None:
        self._status = message
        if message.mode in {"AUTONOMOUS", "AUTONOMOUS_SIM"}:
            if self._match_start_us is None: self._match_start_us = self._now_us()
        else: self._match_start_us = None

    def _on_world(self, message) -> None:
        self._world = message; self._world_received_us = self._now_us()

    def _on_goal(self, message) -> None:
        self._goal = NavigationGoal(
            message.task_id, Pose2(message.target_pose.x, message.target_pose.y, message.target_pose.theta),
            message.position_tolerance_m, message.heading_tolerance_rad, message.valid_until_us,
            int(message.intake_action), bool(message.mechanism_enabled), message.elevator_target_m,
        )
        self._goal_received_us = self._now_us()

    def _on_reservations(self, message) -> None:
        self._reservations = message

    def _plan(self) -> None:
        now_us = self._now_us()
        if not self._inputs_valid(now_us):
            self._path = (); return
        world = self._navigation_world(now_us)
        pose = self._robot_pose()
        self._path = self._planner.plan(pose, self._goal, TemporalCostmap(world), now_us)
        self._plan_revision += 1
        self._last_reason = "PATH_READY" if self._path else "NO_SAFE_PATH"
        self._path_publisher.publish(_path_message(self._path, self.get_clock().now().to_msg()))

    def _control(self) -> None:
        now_us = self._now_us()
        if not self._inputs_valid(now_us) or not self._path:
            self._publish_command(VelocityCommand(False, 0.0, 0.0, 0.0, 0, False, 0.0))
            self._publish_state(now_us, False, math.inf, "INPUT_STALE" if self._inputs_valid(now_us) is False else self._last_reason)
            return
        pose = self._robot_pose()
        reached = _goal_reached(pose, self._goal)
        if reached:
            command = VelocityCommand(
                True, 0.0, 0.0, 0.0, self._goal.intake_action,
                self._goal.mechanism_enabled, self._goal.elevator_target_m,
            )
            self._publish_command(command); self._publish_state(now_us, True, math.inf, "GOAL_REACHED")
            return
        base = self._controller.command(pose, self._path, bool(self._world.perception_fresh))
        base = replace(base, intake_action=self._goal.intake_action,
                       mechanism_enabled=self._goal.mechanism_enabled,
                       elevator_target_m=self._goal.elevator_target_m)
        world = self._navigation_world(now_us)
        monitored = self._monitor.filter(
            base, pose, world.obstacles, world.robot_radius_m, world.perception_fresh,
        )
        self._last_reason = monitored.reason
        self._publish_command(monitored.command)
        self._publish_state(now_us, False, monitored.min_clearance_m, monitored.reason)

    def _inputs_valid(self, now_us: int) -> bool:
        return bool(
            self._status is not None and self._world is not None and self._goal is not None
            and self._status.mode in {"AUTONOMOUS", "AUTONOMOUS_SIM"}
            and self._status.gyro_healthy and self._status.drivetrain_healthy
            and now_us - self._world_received_us <= int(self.get_parameter("world_timeout_us").value)
            and now_us - self._goal_received_us <= int(self.get_parameter("goal_timeout_us").value)
            and now_us <= self._goal.valid_until_us
        )

    def _robot_pose(self) -> Pose2:
        return Pose2(self._status.pose.x, self._status.pose.y, self._status.pose.theta)

    def _navigation_world(self, now_us: int) -> NavigationWorld:
        obstacles = tuple(
            CircleObstacle(
                robot.track_id, robot.pose.pose.position.x, robot.pose.pose.position.y,
                robot.velocity.twist.linear.x, robot.velocity.twist.linear.y,
                max(0.35, max(robot.size.x, robot.size.y) / 2.0), True, robot.confidence,
            ) for robot in self._world.robots
        )
        return NavigationWorld(
            now_us, bool(self._world.perception_fresh),
            float(self.get_parameter("field_length_m").value),
            float(self.get_parameter("field_width_m").value),
            float(self.get_parameter("robot_radius_m").value),
            _rectangles(self.get_parameter("static_rectangles").value), obstacles,
            _reservations(self._reservations, self._match_start_us),
        )

    def _publish_command(self, value: VelocityCommand) -> None:
        message = AutonomyCommand(); message.header.stamp = self.get_clock().now().to_msg()
        message.armed = value.armed; message.vx_mps = value.vx_mps; message.vy_mps = value.vy_mps
        message.omega_radps = value.omega_radps; message.valid_for_us = 100_000
        message.intake_action = value.intake_action; message.mechanism_enabled = value.mechanism_enabled
        message.elevator_target_m = value.elevator_target_m; self._command_publisher.publish(message)

    def _publish_state(self, now_us: int, reached: bool, clearance: float, reason: str) -> None:
        message = NavigationStateMessage(); message.header.stamp = self.get_clock().now().to_msg(); message.header.frame_id = "map"
        message.world_version = int(self._world.version) if self._world else 0; message.plan_revision = self._plan_revision
        message.goal_task_id = self._goal.task_id if self._goal else ""; message.goal_reached = reached
        message.state = NavigationStateMessage.GOAL_REACHED if reached else (
            NavigationStateMessage.STOPPED if "STOP" in reason or "STALE" in reason else NavigationStateMessage.TRACKING
        )
        message.min_clearance_m = clearance; message.speed_limit_mps = float(self.get_parameter("max_speed_mps").value)
        message.obstacle_reason = reason
        message.planned_path = [_timed_pose(point) for point in self._path]
        self._state_publisher.publish(message)

    def _now_us(self) -> int: return self.get_clock().now().nanoseconds // 1_000


def _rectangles(values) -> tuple[Rectangle, ...]:
    result = []
    for value in values:
        parts = [float(part) for part in value.split(",")]
        if len(parts) != 4: raise ValueError(f"invalid rectangle: {value}")
        result.append(Rectangle(*parts))
    return tuple(result)


def _reservations(message, match_start_us) -> tuple[ReservationCircle, ...]:
    if message is None or match_start_us is None: return ()
    return tuple(
        ReservationCircle(match_start_us + sample.time_us - 50_000, match_start_us + sample.time_us + 50_000,
                          sample.pose.x, sample.pose.y, sample.radius_m, tube.confidence)
        for tube in message.tubes for sample in tube.samples
    )


def _goal_reached(pose: Pose2, goal: NavigationGoal) -> bool:
    position = math.hypot(pose.x - goal.pose.x, pose.y - goal.pose.y)
    heading = abs(math.atan2(math.sin(pose.heading - goal.pose.heading), math.cos(pose.heading - goal.pose.heading)))
    return position <= goal.position_tolerance_m and heading <= goal.heading_tolerance_rad


def _timed_pose(value) -> TimedPose:
    message = TimedPose(); message.time_us = value.time_us
    message.pose.x = value.x; message.pose.y = value.y; message.pose.theta = value.heading; return message


def _path_message(points, stamp) -> Path:
    message = Path(); message.header.stamp = stamp; message.header.frame_id = "map"
    for point in points:
        pose = PoseStamped(); pose.header = message.header; pose.pose.position.x = point.x; pose.pose.position.y = point.y
        pose.pose.orientation.z = math.sin(point.heading / 2.0); pose.pose.orientation.w = math.cos(point.heading / 2.0)
        message.poses.append(pose)
    return message


def main(args=None) -> None:
    rclpy.init(args=args); node = NavigationNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()
