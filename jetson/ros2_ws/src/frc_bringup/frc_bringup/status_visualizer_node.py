"""Expose roboRIO status as TF, odometry, path, and readable RViz markers."""

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

from frc_autonomy_msgs.msg import AutonomyStatus, DecisionTrace, NavigationState


class StatusVisualizerNode(Node):
    def __init__(self) -> None:
        super().__init__("autonomy_status_visualizer")
        self._tf = TransformBroadcaster(self)
        self._odometry = self.create_publisher(Odometry, "/autonomy/odom", 10)
        self._path_publisher = self.create_publisher(Path, "/autonomy/executed_path", 10)
        self._markers = self.create_publisher(MarkerArray, "/autonomy/decision_markers", 10)
        self._path = Path()
        self._decision = None
        self._navigation = None
        self.create_subscription(AutonomyStatus, "/autonomy/status", self._status_callback, 10)
        self.create_subscription(DecisionTrace, "/strategy/decision", self._decision_callback, 10)
        self.create_subscription(NavigationState, "/navigation/state", self._navigation_callback, 10)

    def _decision_callback(self, message: DecisionTrace) -> None:
        self._decision = message

    def _navigation_callback(self, message: NavigationState) -> None:
        self._navigation = message

    def _status_callback(self, status: AutonomyStatus) -> None:
        stamp = self.get_clock().now().to_msg()
        quaternion_z = math.sin(status.pose.theta / 2.0)
        quaternion_w = math.cos(status.pose.theta / 2.0)

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = "map"
        transform.child_frame_id = "base_footprint"
        transform.transform.translation.x = status.pose.x
        transform.transform.translation.y = status.pose.y
        transform.transform.rotation.z = quaternion_z
        transform.transform.rotation.w = quaternion_w
        self._tf.sendTransform(transform)

        odometry = Odometry()
        odometry.header.stamp = stamp
        odometry.header.frame_id = "map"
        odometry.child_frame_id = "base_footprint"
        odometry.pose.pose.position.x = status.pose.x
        odometry.pose.pose.position.y = status.pose.y
        odometry.pose.pose.orientation.z = quaternion_z
        odometry.pose.pose.orientation.w = quaternion_w
        odometry.twist.twist = status.measured_chassis
        self._odometry.publish(odometry)

        self._path.header = odometry.header
        pose_stamped = odometry_to_pose_stamped(odometry)
        self._path.poses.append(pose_stamped)
        if len(self._path.poses) > 2_000:
            del self._path.poses[: len(self._path.poses) - 2_000]
        self._path_publisher.publish(self._path)
        self._publish_marker(status, stamp)

    def _publish_marker(self, status: AutonomyStatus, stamp) -> None:
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = "base_link"
        marker.ns = "autonomy_status"
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.z = 1.0
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.16
        marker.color.a = 1.0
        marker.color.g = 1.0 if status.command_active else 0.15
        marker.color.r = 0.15 if status.command_active else 1.0
        marker.text = (
            f"{status.mode} | seq={status.accepted_sequence} | "
            f"active={status.command_active} | {status.reject_reason}"
        )
        markers = [marker]
        if self._decision is not None:
            decision = Marker()
            decision.header.stamp = stamp; decision.header.frame_id = "base_link"
            decision.ns = "strategy_decision"; decision.id = 1
            decision.type = Marker.TEXT_VIEW_FACING; decision.action = Marker.ADD
            decision.pose.position.z = 1.25; decision.pose.orientation.w = 1.0
            decision.scale.z = 0.13; decision.color.a = 1.0; decision.color.b = 1.0; decision.color.g = 0.8
            navigation_reason = self._navigation.obstacle_reason if self._navigation else "NO_NAV"
            decision.text = (
                f"TASK: {self._decision.chosen_task_id} | {self._decision.replan_reason}\n"
                f"{self._decision.counterfactual}\nNAV: {navigation_reason}"
            )
            markers.append(decision)
        self._markers.publish(MarkerArray(markers=markers))


def odometry_to_pose_stamped(odometry):
    from geometry_msgs.msg import PoseStamped

    pose = PoseStamped()
    pose.header = odometry.header
    pose.pose = odometry.pose.pose
    return pose


def main(args=None) -> None:
    rclpy.init(args=args)
    node = StatusVisualizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
