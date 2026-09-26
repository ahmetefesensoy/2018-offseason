"""Explicitly armed, low-speed figure-eight source for simulation demos only."""

import math

import rclpy
from rclpy.node import Node

from frc_autonomy_msgs.msg import AutonomyCommand


class FakeAutonomyNode(Node):
    def __init__(self) -> None:
        super().__init__("fake_autonomy")
        self.declare_parameter("armed", False)
        self.declare_parameter("forward_speed_mps", 0.18)
        self.declare_parameter("lateral_speed_mps", 0.08)
        self.declare_parameter("rotation_speed_radps", 0.25)
        self._publisher = self.create_publisher(AutonomyCommand, "/autonomy/command", 10)
        self._started_ns = self.get_clock().now().nanoseconds
        self.create_timer(0.02, self._tick)

    def _tick(self) -> None:
        elapsed = (self.get_clock().now().nanoseconds - self._started_ns) / 1e9
        armed = bool(self.get_parameter("armed").value)
        message = AutonomyCommand()
        message.header.stamp = self.get_clock().now().to_msg()
        message.armed = armed
        message.valid_for_us = 100_000
        if armed:
            message.vx_mps = float(self.get_parameter("forward_speed_mps").value)
            message.vy_mps = float(self.get_parameter("lateral_speed_mps").value) * math.sin(elapsed)
            message.omega_radps = float(self.get_parameter("rotation_speed_radps").value) * math.sin(elapsed * 0.5)
        self._publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FakeAutonomyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
