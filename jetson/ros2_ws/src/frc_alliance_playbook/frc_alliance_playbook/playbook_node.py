"""ROS publisher for validated teammate plans and live reservation confidence."""

from __future__ import annotations

from dataclasses import replace
import json
import math
from pathlib import Path

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from frc_autonomy_msgs.msg import (
    AlliancePlan as AlliancePlanMessage,
    PlanSegment as PlanSegmentMessage,
    ReservationTubeArray as ReservationTubeArrayMessage,
    RobotPlan as RobotPlanMessage,
    RobotTrack,
    TimedPose as TimedPoseMessage,
    WorldState,
)

from .reservations import (
    ConfidenceTracker,
    ReservationTube,
    build_reservations,
    find_conflicts,
    sample_reservation,
)
from .schema import AlliancePlan, Pose2d, content_sha256, parse_plan


TASK_TYPES = {
    "PICKUP": PlanSegmentMessage.TASK_PICKUP,
    "SCORE_SWITCH": PlanSegmentMessage.TASK_SCORE_SWITCH,
    "SCORE_SCALE": PlanSegmentMessage.TASK_SCORE_SCALE,
    "DELIVER_VAULT": PlanSegmentMessage.TASK_DELIVER_VAULT,
    "CROSS_LINE": PlanSegmentMessage.TASK_CROSS_LINE,
    "PARK": PlanSegmentMessage.TASK_PARK,
    "CLIMB": PlanSegmentMessage.TASK_CLIMB,
    "WAIT": PlanSegmentMessage.TASK_WAIT,
}


class AlliancePlaybookNode(Node):
    def __init__(self) -> None:
        super().__init__("alliance_playbook")
        self._declare_parameters()
        self._plan = self._load_plan()
        self._base_tubes = build_reservations(
            self._plan,
            int(self._parameter("sample_period_us")),
        )
        self._tracker = ConfidenceTracker(
            {
                robot.team_number: robot.initial_confidence
                for robot in self._plan.robots
            },
            corridor_tolerance_m=float(self._parameter("corridor_tolerance_m")),
            deviation_scale_m=float(self._parameter("deviation_scale_m")),
            missing_half_life_us=int(self._parameter("missing_half_life_us")),
        )
        self._match_epoch_us: int | None = None
        self._revision = 0
        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._plan_publisher = self.create_publisher(
            AlliancePlanMessage,
            "/alliance/plan",
            latched_qos,
        )
        self._reservation_publisher = self.create_publisher(
            ReservationTubeArrayMessage,
            "/alliance/reservations",
            latched_qos,
        )
        self._diagnostics_publisher = self.create_publisher(
            DiagnosticArray,
            "/diagnostics",
            10,
        )
        self.create_subscription(
            WorldState,
            "/world/state",
            self._world_callback,
            10,
        )
        publish_rate_hz = float(self._parameter("publish_rate_hz"))
        if not math.isfinite(publish_rate_hz) or publish_rate_hz <= 0.0:
            raise ValueError("publish_rate_hz must be finite and positive")
        self.create_timer(1.0 / publish_rate_hz, self._publish)
        self.get_logger().info(
            f"Loaded alliance plan '{self._plan.plan_id}' "
            f"sha256={self._plan.content_sha256}"
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("plan_path", "")
        self.declare_parameter("expected_sha256", "")
        self.declare_parameter("sample_period_us", 100_000)
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("corridor_tolerance_m", 0.20)
        self.declare_parameter("deviation_scale_m", 0.75)
        self.declare_parameter("missing_half_life_us", 1_000_000)

    def _parameter(self, name: str):
        return self.get_parameter(name).value

    def _load_plan(self) -> AlliancePlan:
        plan_path = str(self._parameter("plan_path")).strip()
        if not plan_path:
            raise ValueError("plan_path is required")
        with Path(plan_path).open("r", encoding="utf-8-sig") as source:
            plan = parse_plan(json.load(source))
        expected = str(self._parameter("expected_sha256")).strip().lower()
        if expected and expected != content_sha256(plan):
            raise ValueError("expected_sha256 does not match the validated plan")
        return plan

    def _world_callback(self, world: WorldState) -> None:
        now_us = self.get_clock().now().nanoseconds // 1_000
        if self._match_epoch_us is None:
            if not world.perception_fresh:
                return
            self._match_epoch_us = now_us
        match_time_us = max(0, now_us - self._match_epoch_us)
        assignments = self._assign_allies(world, match_time_us)
        for tube in self._base_tubes:
            track = assignments.get(tube.team_number)
            planned = sample_reservation(tube, match_time_us).pose
            if track is None or not world.perception_fresh:
                self._tracker.missing(tube.team_number, match_time_us)
                continue
            observed = _track_pose(track)
            self._tracker.update(
                tube.team_number,
                planned,
                observed,
                match_time_us,
            )
        self._revision += 1

    def _assign_allies(self, world: WorldState, match_time_us: int):
        allies = [
            track
            for track in world.robots
            if int(track.affiliation) == RobotTrack.AFFILIATION_ALLY
        ]
        candidates = []
        for track in allies:
            observed = _track_pose(track)
            for tube in self._base_tubes:
                planned = sample_reservation(tube, match_time_us).pose
                distance = math.hypot(
                    observed.x - planned.x,
                    observed.y - planned.y,
                )
                candidates.append((distance, track.track_id, tube.team_number, track))
        assignments = {}
        used_tracks: set[str] = set()
        for _, track_id, team_number, track in sorted(candidates):
            if track_id in used_tracks or team_number in assignments:
                continue
            assignments[team_number] = track
            used_tracks.add(track_id)
        return assignments

    def _current_tubes(self) -> tuple[ReservationTube, ...]:
        return tuple(
            replace(tube, confidence=self._tracker.confidence(tube.team_number))
            for tube in self._base_tubes
        )

    def _publish(self) -> None:
        stamp = self.get_clock().now().to_msg()
        tubes = self._current_tubes()
        self._plan_publisher.publish(_plan_message(self._plan, stamp))
        self._reservation_publisher.publish(
            _reservation_message(
                tubes,
                self._revision,
                self._plan.content_sha256,
                stamp,
            )
        )
        self._diagnostics_publisher.publish(
            _diagnostics_message(self._plan, tubes, stamp)
        )


def _plan_message(plan: AlliancePlan, stamp) -> AlliancePlanMessage:
    message = AlliancePlanMessage()
    message.header.frame_id = "map"
    message.header.stamp = stamp
    message.schema_version = plan.schema_version
    message.field_version = plan.field_version
    message.alliance = plan.alliance
    message.plan_id = plan.plan_id
    message.content_sha256 = plan.content_sha256
    message.robots = [_robot_message(robot) for robot in plan.robots]
    return message


def _robot_message(robot) -> RobotPlanMessage:
    message = RobotPlanMessage()
    message.team_number = robot.team_number
    message.robot_label = robot.robot_label
    _set_pose2d(message.start_pose, robot.start_pose)
    message.footprint.x = robot.footprint_length_m
    message.footprint.y = robot.footprint_width_m
    message.max_velocity_mps = robot.max_velocity_mps
    message.max_acceleration_mps2 = robot.max_acceleration_mps2
    message.initial_confidence = robot.initial_confidence
    message.segments = []
    for segment in robot.segments:
        segment_message = PlanSegmentMessage()
        segment_message.segment_id = segment.segment_id
        segment_message.task_type = TASK_TYPES[segment.task]
        segment_message.target_id = segment.target_id
        segment_message.earliest_start_us = segment.earliest_start_us
        segment_message.latest_start_us = segment.latest_start_us
        segment_message.expected_duration_us = segment.expected_duration_us
        segment_message.corridor_radius_m = segment.corridor_radius_m
        segment_message.fallback_segment_id = segment.fallback_segment_id
        for point in segment.path:
            point_message = TimedPoseMessage()
            point_message.time_us = point.time_us
            _set_pose2d(point_message.pose, point.pose)
            segment_message.path.append(point_message)
        message.segments.append(segment_message)
    return message


def _reservation_message(
    tubes: tuple[ReservationTube, ...],
    revision: int,
    plan_hash: str,
    stamp,
) -> ReservationTubeArrayMessage:
    from frc_autonomy_msgs.msg import ReservationSample, ReservationTube as TubeMessage

    message = ReservationTubeArrayMessage()
    message.header.frame_id = "map"
    message.header.stamp = stamp
    message.revision = revision
    message.content_sha256 = plan_hash
    for tube in tubes:
        tube_message = TubeMessage()
        tube_message.plan_id = tube.plan_id
        tube_message.team_number = tube.team_number
        tube_message.robot_label = tube.robot_label
        tube_message.confidence = tube.confidence
        for sample in tube.samples:
            sample_message = ReservationSample()
            sample_message.time_us = sample.time_us
            _set_pose2d(sample_message.pose, sample.pose)
            sample_message.radius_m = sample.radius_m
            tube_message.samples.append(sample_message)
        message.tubes.append(tube_message)
    return message


def _diagnostics_message(plan: AlliancePlan, tubes, stamp) -> DiagnosticArray:
    conflicts = find_conflicts(tubes)
    array = DiagnosticArray()
    array.header.stamp = stamp
    status = DiagnosticStatus()
    status.name = "alliance_playbook"
    status.hardware_id = "jetson"
    status.level = DiagnosticStatus.WARN if conflicts else DiagnosticStatus.OK
    status.message = (
        f"{len(conflicts)} reservation conflict(s)"
        if conflicts
        else "validated plan active"
    )
    status.values = [
        KeyValue(key="plan_id", value=plan.plan_id),
        KeyValue(key="content_sha256", value=plan.content_sha256),
        KeyValue(key="robots", value=str(len(tubes))),
        KeyValue(key="conflicts", value=str(len(conflicts))),
    ] + [
        KeyValue(
            key=f"team_{tube.team_number}_confidence",
            value=f"{tube.confidence:.6f}",
        )
        for tube in tubes
    ]
    array.status = [status]
    return array


def _set_pose2d(message, pose: Pose2d) -> None:
    message.x = pose.x
    message.y = pose.y
    message.theta = pose.heading


def _track_pose(track) -> Pose2d:
    orientation = track.pose.pose.orientation
    heading = math.atan2(
        2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
        1.0 - 2.0 * (orientation.y**2 + orientation.z**2),
    )
    return Pose2d(
        float(track.pose.pose.position.x),
        float(track.pose.pose.position.y),
        heading,
    )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AlliancePlaybookNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
