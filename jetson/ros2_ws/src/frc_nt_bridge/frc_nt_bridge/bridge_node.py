"""ROS 2 node translating bounded autonomy commands and roboRIO status over NT4."""

from __future__ import annotations

import math
import time

import ntcore
import rclpy
from rclpy.node import Node

from frc_autonomy_msgs.msg import AutonomyCommand, AutonomyStatus

from .protocol import CommandRequest, FrameBuilder, ProtocolError, server_time_us


NT_ROOT = "/frc/autonomy/v1"


class FrcNtBridge(Node):
    def __init__(self) -> None:
        super().__init__("frc_nt_bridge")
        self.declare_parameter("team_number", 0)
        self.declare_parameter("server_address", "127.0.0.1")
        self.declare_parameter("command_timeout_us", 100_000)
        self.declare_parameter("default_validity_us", 100_000)
        self.declare_parameter("max_translation_speed_mps", 0.75)
        self.declare_parameter("max_rotation_speed_radps", 1.5)
        self.declare_parameter("max_validity_us", 150_000)

        self._command_timeout_us = int(self.get_parameter("command_timeout_us").value)
        self._default_validity_us = int(self.get_parameter("default_validity_us").value)
        self._builder = FrameBuilder(
            float(self.get_parameter("max_translation_speed_mps").value),
            float(self.get_parameter("max_rotation_speed_radps").value),
            int(self.get_parameter("max_validity_us").value),
        )
        self._latest_request = CommandRequest(False, 0.0, 0.0, 0.0, self._default_validity_us)
        self._last_ros_command_monotonic_us = 0
        self._last_sync_warning_monotonic_us = 0

        self._nt = ntcore.NetworkTableInstance.create()
        self._nt.startClient4("jetson-frc-autonomy")
        team_number = int(self.get_parameter("team_number").value)
        if team_number > 0:
            self._nt.setServerTeam(team_number)
        else:
            self._nt.setServer(str(self.get_parameter("server_address").value))

        self._command_publishers = {
            "session_id": self._nt.getStringTopic(self._path("command/session_id")).publish(),
            "armed": self._nt.getBooleanTopic(self._path("command/armed")).publish(),
            "sent_at_us": self._nt.getIntegerTopic(self._path("command/sent_at_us")).publish(),
            "valid_until_us": self._nt.getIntegerTopic(self._path("command/valid_until_us")).publish(),
            "vx_mps": self._nt.getDoubleTopic(self._path("command/vx_mps")).publish(),
            "vy_mps": self._nt.getDoubleTopic(self._path("command/vy_mps")).publish(),
            "omega_radps": self._nt.getDoubleTopic(self._path("command/omega_radps")).publish(),
            "sequence": self._nt.getIntegerTopic(self._path("command/sequence")).publish(),
            "commit_sequence": self._nt.getIntegerTopic(self._path("command/commit_sequence")).publish(),
        }
        self._status_session = self._nt.getStringTopic(self._path("status/session_id")).subscribe("")
        self._status_mode = self._nt.getStringTopic(self._path("status/mode")).subscribe("UNKNOWN")
        self._status_reject = self._nt.getStringTopic(self._path("status/reject_reason")).subscribe("NO_LINK")
        self._status_accepted = self._nt.getIntegerTopic(self._path("status/accepted_sequence")).subscribe(-1)
        self._status_robot_time = self._nt.getIntegerTopic(self._path("status/roborio_time_us")).subscribe(-1)
        self._status_command_age = self._nt.getIntegerTopic(self._path("status/command_age_us")).subscribe(-1)
        self._status_active = self._nt.getBooleanTopic(self._path("status/command_active")).subscribe(False)
        self._status_gyro = self._nt.getBooleanTopic(self._path("status/gyro_healthy")).subscribe(False)
        self._status_drivetrain = self._nt.getBooleanTopic(self._path("status/drivetrain_healthy")).subscribe(False)
        self._status_pose = self._nt.getDoubleArrayTopic(self._path("status/pose")).subscribe([])
        self._status_measured = self._nt.getDoubleArrayTopic(self._path("status/measured_chassis")).subscribe([])

        self._status_publisher = self.create_publisher(AutonomyStatus, "/autonomy/status", 10)
        self.create_subscription(AutonomyCommand, "/autonomy/command", self._command_callback, 10)
        self.create_timer(0.02, self._tick)

    @staticmethod
    def _path(suffix: str) -> str:
        return f"{NT_ROOT}/{suffix}"

    def _command_callback(self, message: AutonomyCommand) -> None:
        validity = int(message.valid_for_us) or self._default_validity_us
        self._latest_request = CommandRequest(
            bool(message.armed),
            float(message.vx_mps),
            float(message.vy_mps),
            float(message.omega_radps),
            validity,
        )
        self._last_ros_command_monotonic_us = time.monotonic_ns() // 1_000

    def _tick(self) -> None:
        local_now_us = time.monotonic_ns() // 1_000
        self._publish_status()

        session_id = self._status_session.get()
        if not session_id:
            return
        try:
            now_server_us = server_time_us(local_now_us, self._nt.getServerTimeOffset())
        except ProtocolError as error:
            if local_now_us - self._last_sync_warning_monotonic_us > 1_000_000:
                self.get_logger().warning(str(error))
                self._last_sync_warning_monotonic_us = local_now_us
            return

        self._builder.synchronize_accepted_sequence(self._status_accepted.get())
        request = self._latest_request
        if (
            self._last_ros_command_monotonic_us == 0
            or local_now_us - self._last_ros_command_monotonic_us > self._command_timeout_us
        ):
            request = CommandRequest(False, 0.0, 0.0, 0.0, self._default_validity_us)

        try:
            frame = self._builder.build(request, session_id, now_server_us)
        except ProtocolError as error:
            self.get_logger().error(f"Rejected ROS autonomy command: {error}")
            frame = self._builder.build(
                CommandRequest(False, 0.0, 0.0, 0.0, self._default_validity_us),
                session_id,
                now_server_us,
            )

        for field_name, value in frame.publish_operations():
            self._command_publishers[field_name].set(value)
        self._nt.flush()

    def _publish_status(self) -> None:
        message = AutonomyStatus()
        message.header.stamp = self.get_clock().now().to_msg()
        message.session_id = self._status_session.get()
        message.mode = self._status_mode.get()
        message.reject_reason = self._status_reject.get()
        message.accepted_sequence = self._status_accepted.get()
        message.roborio_time_us = self._status_robot_time.get()
        message.command_age_us = self._status_command_age.get()
        message.command_active = self._status_active.get()
        message.gyro_healthy = self._status_gyro.get()
        message.drivetrain_healthy = self._status_drivetrain.get()

        pose = self._status_pose.get()
        if len(pose) == 3 and all(math.isfinite(value) for value in pose):
            message.pose.x, message.pose.y, message.pose.theta = pose
        measured = self._status_measured.get()
        if len(measured) == 3 and all(math.isfinite(value) for value in measured):
            message.measured_chassis.linear.x = measured[0]
            message.measured_chassis.linear.y = measured[1]
            message.measured_chassis.angular.z = measured[2]
        self._status_publisher.publish(message)

    def destroy_node(self) -> bool:
        self._latest_request = CommandRequest(False, 0.0, 0.0, 0.0, self._default_validity_us)
        self._last_ros_command_monotonic_us = 0
        try:
            self._tick()
        finally:
            self._nt.stopClient()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FrcNtBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
