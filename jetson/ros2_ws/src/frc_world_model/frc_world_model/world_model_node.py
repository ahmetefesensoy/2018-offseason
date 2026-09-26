"""ROS adapter for the deterministic perception world store."""

from __future__ import annotations

import math
import time

import rclpy
from rclpy.node import Node

from frc_autonomy_msgs.msg import (
    GameObject,
    PredictedPose,
    RobotTrack,
    SemanticObservationArray,
    WorldState,
)

from .model import MultiObjectTracker, Observation, Track, TrackerConfig
from .world_store import WorldSnapshot, WorldStore, WorldStoreError


class WorldModelNode(Node):
    def __init__(self) -> None:
        super().__init__("world_model")
        self._declare_parameters()
        tracker = MultiObjectTracker(
            TrackerConfig(
                association_gate_m=float(self._parameter("association_gate_m")),
                position_gain=float(self._parameter("position_gain")),
                velocity_gain=float(self._parameter("velocity_gain")),
                track_timeout_us=int(self._parameter("track_timeout_us")),
                max_observation_age_us=int(
                    self._parameter("max_observation_age_us")
                ),
                min_confidence=float(self._parameter("min_confidence")),
                prediction_horizon_s=float(
                    self._parameter("prediction_horizon_s")
                ),
                prediction_step_s=float(self._parameter("prediction_step_s")),
                process_noise_variance=float(
                    self._parameter("process_noise_variance")
                ),
            )
        )
        self._store = WorldStore(
            tracker,
            source_timeout_us=int(self._parameter("source_timeout_us")),
        )
        self._last_warning_monotonic_us = 0
        self._publisher = self.create_publisher(WorldState, "/world/state", 10)
        self.create_subscription(
            SemanticObservationArray,
            "/perception/observations",
            self._observation_callback,
            10,
        )
        publish_rate_hz = float(self._parameter("publish_rate_hz"))
        self.create_timer(1.0 / publish_rate_hz, self._publish_snapshot)

    def _declare_parameters(self) -> None:
        self.declare_parameter("association_gate_m", 0.75)
        self.declare_parameter("position_gain", 0.65)
        self.declare_parameter("velocity_gain", 0.35)
        self.declare_parameter("track_timeout_us", 750_000)
        self.declare_parameter("source_timeout_us", 250_000)
        self.declare_parameter("max_observation_age_us", 250_000)
        self.declare_parameter("min_confidence", 0.20)
        self.declare_parameter("prediction_horizon_s", 2.0)
        self.declare_parameter("prediction_step_s", 0.1)
        self.declare_parameter("process_noise_variance", 0.16)
        self.declare_parameter("publish_rate_hz", 10.0)

    def _parameter(self, name: str):
        return self.get_parameter(name).value

    def _observation_callback(self, message: SemanticObservationArray) -> None:
        if message.header.frame_id != "map":
            self._warn_throttled(
                f"Rejected perception frame '{message.header.frame_id}'; expected 'map'"
            )
            return
        source_timestamp_us = _stamp_to_us(message.header.stamp)
        received_timestamp_us = self._now_us()
        observations = tuple(
            _to_observation(item, source_timestamp_us)
            for item in message.observations
        )
        try:
            self._store.ingest(
                observations,
                source_timestamp_us,
                received_timestamp_us,
            )
        except WorldStoreError as error:
            self._warn_throttled(f"Rejected perception batch: {error}")

    def _publish_snapshot(self) -> None:
        snapshot = self._store.snapshot(self._now_us())
        self._publisher.publish(_to_world_message(snapshot))

    def _now_us(self) -> int:
        return self.get_clock().now().nanoseconds // 1_000

    def _warn_throttled(self, message: str) -> None:
        now_us = time.monotonic_ns() // 1_000
        if now_us - self._last_warning_monotonic_us >= 1_000_000:
            self.get_logger().warning(message)
            self._last_warning_monotonic_us = now_us


def _stamp_to_us(stamp) -> int:
    return int(stamp.sec) * 1_000_000 + int(stamp.nanosec) // 1_000


def _to_observation(message, timestamp_us: int) -> Observation:
    orientation = message.pose.pose.orientation
    yaw = math.atan2(
        2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
        1.0 - 2.0 * (orientation.y**2 + orientation.z**2),
    )
    return Observation(
        detection_id=message.detection_id,
        class_name=message.class_name,
        affiliation=int(message.affiliation),
        x=float(message.pose.pose.position.x),
        y=float(message.pose.pose.position.y),
        yaw=yaw,
        size_x=float(message.size.x),
        size_y=float(message.size.y),
        confidence=float(message.confidence),
        variance_x=float(message.pose.covariance[0]),
        variance_y=float(message.pose.covariance[7]),
        timestamp_us=timestamp_us,
    )


def _to_world_message(snapshot: WorldSnapshot) -> WorldState:
    message = WorldState()
    message.header.frame_id = "map"
    _set_stamp(message.header.stamp, snapshot.source_timestamp_us)
    message.version = snapshot.version
    message.perception_fresh = snapshot.perception_fresh
    message.newest_source_age_us = snapshot.newest_source_age_us
    message.game_objects = [_to_game_object(track) for track in snapshot.game_objects]
    message.robots = [_to_robot_track(track) for track in snapshot.robots]
    return message


def _to_game_object(track: Track) -> GameObject:
    message = GameObject()
    message.object_id = track.track_id
    message.class_name = track.class_name
    _populate_pose(message.pose, track)
    _populate_twist(message.velocity, track)
    message.size.x = track.size_x
    message.size.y = track.size_y
    message.size.z = min(track.size_x, track.size_y)
    message.confidence = track.confidence
    message.last_seen_us = track.last_seen_us
    return message


def _to_robot_track(track: Track) -> RobotTrack:
    message = RobotTrack()
    message.track_id = track.track_id
    message.affiliation = track.affiliation
    _populate_pose(message.pose, track)
    _populate_twist(message.velocity, track)
    message.size.x = track.size_x
    message.size.y = track.size_y
    message.size.z = 0.25
    message.predictions = [_to_prediction(value) for value in track.predictions]
    message.confidence = track.confidence
    message.last_seen_us = track.last_seen_us
    return message


def _to_prediction(prediction) -> PredictedPose:
    message = PredictedPose()
    total_nanoseconds = round(prediction.horizon_s * 1e9)
    message.horizon.sec = total_nanoseconds // 1_000_000_000
    message.horizon.nanosec = total_nanoseconds % 1_000_000_000
    _populate_pose_values(
        message.pose,
        prediction.x,
        prediction.y,
        prediction.yaw,
        prediction.variance_x,
        prediction.variance_y,
    )
    return message


def _populate_pose(message, track: Track) -> None:
    _populate_pose_values(
        message,
        track.x,
        track.y,
        track.yaw,
        track.variance_x,
        track.variance_y,
    )


def _populate_pose_values(
    message,
    x: float,
    y: float,
    yaw: float,
    variance_x: float,
    variance_y: float,
) -> None:
    message.pose.position.x = x
    message.pose.position.y = y
    message.pose.orientation.z = math.sin(yaw / 2.0)
    message.pose.orientation.w = math.cos(yaw / 2.0)
    message.covariance[0] = variance_x
    message.covariance[7] = variance_y
    message.covariance[35] = 0.05


def _populate_twist(message, track: Track) -> None:
    message.twist.linear.x = track.vx
    message.twist.linear.y = track.vy
    message.covariance[0] = 0.25
    message.covariance[7] = 0.25


def _set_stamp(stamp, timestamp_us: int) -> None:
    if timestamp_us <= 0:
        return
    stamp.sec = timestamp_us // 1_000_000
    stamp.nanosec = (timestamp_us % 1_000_000) * 1_000


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WorldModelNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
