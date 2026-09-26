"""Publish a deterministic perception-only scene for RViz and rosbag demos."""

import math

import rclpy
from rclpy.node import Node

from frc_autonomy_msgs.msg import SemanticObservation, SemanticObservationArray

from .synthetic_scene import sample_scene


class SyntheticPerceptionNode(Node):
    def __init__(self) -> None:
        super().__init__("synthetic_perception")
        self.declare_parameter("publish_rate_hz", 20.0)
        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        if publish_rate_hz <= 0.0:
            raise ValueError("publish_rate_hz must be positive")
        self._publisher = self.create_publisher(
            SemanticObservationArray,
            "/perception/observations",
            10,
        )
        self._start_nanoseconds = self.get_clock().now().nanoseconds
        self.create_timer(1.0 / publish_rate_hz, self._tick)

    def _tick(self) -> None:
        now = self.get_clock().now()
        elapsed_s = max(
            (now.nanoseconds - self._start_nanoseconds) / 1e9,
            0.0,
        )
        timestamp_us = now.nanoseconds // 1_000
        message = SemanticObservationArray()
        message.header.stamp = now.to_msg()
        message.header.frame_id = "map"
        message.observations = [
            _to_message(item) for item in sample_scene(elapsed_s, timestamp_us)
        ]
        self._publisher.publish(message)


def _to_message(item) -> SemanticObservation:
    message = SemanticObservation()
    message.detection_id = item.detection_id
    message.class_name = item.class_name
    message.affiliation = item.affiliation
    message.pose.pose.position.x = item.x
    message.pose.pose.position.y = item.y
    message.pose.pose.orientation.z = math.sin(item.yaw / 2.0)
    message.pose.pose.orientation.w = math.cos(item.yaw / 2.0)
    message.pose.covariance[0] = item.variance_x
    message.pose.covariance[7] = item.variance_y
    message.pose.covariance[35] = 0.05
    message.size.x = item.size_x
    message.size.y = item.size_y
    message.size.z = 0.25 if item.class_name == "robot" else item.size_x
    message.confidence = item.confidence
    return message


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SyntheticPerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
