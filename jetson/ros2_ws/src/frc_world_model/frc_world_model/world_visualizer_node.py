"""Render world snapshots as compact, explainable RViz markers."""

import math
import zlib

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray

from frc_autonomy_msgs.msg import RobotTrack, WorldState


class WorldVisualizerNode(Node):
    def __init__(self) -> None:
        super().__init__("world_visualizer")
        self._publisher = self.create_publisher(MarkerArray, "/world/markers", 10)
        self.create_subscription(WorldState, "/world/state", self._world_callback, 10)

    def _world_callback(self, world: WorldState) -> None:
        stamp = self.get_clock().now().to_msg()
        delete_all = Marker()
        delete_all.header.frame_id = "map"
        delete_all.header.stamp = stamp
        delete_all.action = Marker.DELETEALL
        markers = [delete_all]
        for game_object in world.game_objects:
            markers.extend(_game_object_markers(game_object, world.perception_fresh, stamp))
        for robot in world.robots:
            markers.extend(_robot_markers(robot, world.perception_fresh, stamp))
        self._publisher.publish(MarkerArray(markers=markers))


def _game_object_markers(game_object, fresh: bool, stamp) -> list[Marker]:
    body = _marker("game_objects", game_object.object_id, Marker.CUBE, stamp)
    body.pose = game_object.pose.pose
    body.scale.x = game_object.size.x
    body.scale.y = game_object.size.y
    body.scale.z = max(game_object.size.z, 0.08)
    _set_color(body, 1.0, 0.62, 0.05, 0.9, fresh)

    label = _marker("game_object_labels", game_object.object_id, Marker.TEXT_VIEW_FACING, stamp)
    label.pose.position.x = game_object.pose.pose.position.x
    label.pose.position.y = game_object.pose.pose.position.y
    label.pose.position.z = body.scale.z + 0.22
    label.pose.orientation.w = 1.0
    label.scale.z = 0.13
    _set_color(label, 1.0, 0.82, 0.25, 1.0, fresh)
    label.text = f"{game_object.class_name} {game_object.confidence:.0%}"
    return [body, label]


def _robot_markers(robot: RobotTrack, fresh: bool, stamp) -> list[Marker]:
    red, green, blue = _affiliation_color(robot.affiliation)
    body = _marker("robot_tracks", robot.track_id, Marker.CUBE, stamp)
    body.pose = robot.pose.pose
    body.scale.x = robot.size.x
    body.scale.y = robot.size.y
    body.scale.z = max(robot.size.z, 0.20)
    _set_color(body, red, green, blue, 0.72, fresh)

    covariance = _marker("covariance", robot.track_id, Marker.CYLINDER, stamp)
    covariance.pose.position.x = robot.pose.pose.position.x
    covariance.pose.position.y = robot.pose.pose.position.y
    covariance.pose.position.z = 0.02
    covariance.pose.orientation.w = 1.0
    covariance.scale.x = max(4.0 * math.sqrt(max(robot.pose.covariance[0], 0.0)), 0.05)
    covariance.scale.y = max(4.0 * math.sqrt(max(robot.pose.covariance[7], 0.0)), 0.05)
    covariance.scale.z = 0.025
    _set_color(covariance, red, green, blue, 0.20, fresh)

    velocity = _marker("velocities", robot.track_id, Marker.ARROW, stamp)
    velocity.points = [
        Point(
            x=robot.pose.pose.position.x,
            y=robot.pose.pose.position.y,
            z=0.38,
        ),
        Point(
            x=robot.pose.pose.position.x + robot.velocity.twist.linear.x,
            y=robot.pose.pose.position.y + robot.velocity.twist.linear.y,
            z=0.38,
        ),
    ]
    velocity.scale.x = 0.045
    velocity.scale.y = 0.09
    velocity.scale.z = 0.12
    _set_color(velocity, red, green, blue, 0.95, fresh)

    prediction = _marker("predictions", robot.track_id, Marker.LINE_STRIP, stamp)
    prediction.points = [
        Point(x=robot.pose.pose.position.x, y=robot.pose.pose.position.y, z=0.30)
    ] + [
        Point(x=value.pose.pose.position.x, y=value.pose.pose.position.y, z=0.30)
        for value in robot.predictions
    ]
    prediction.scale.x = 0.055
    _set_color(prediction, red, green, blue, 0.80, fresh)

    label = _marker("robot_labels", robot.track_id, Marker.TEXT_VIEW_FACING, stamp)
    label.pose.position.x = robot.pose.pose.position.x
    label.pose.position.y = robot.pose.pose.position.y
    label.pose.position.z = 0.78
    label.pose.orientation.w = 1.0
    label.scale.z = 0.16
    _set_color(label, red, green, blue, 1.0, fresh)
    label.text = f"{_affiliation_name(robot.affiliation)} {robot.track_id} {robot.confidence:.0%}"
    return [covariance, body, velocity, prediction, label]


def _marker(namespace: str, key: str, marker_type: int, stamp) -> Marker:
    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = stamp
    marker.ns = namespace
    marker.id = zlib.crc32(f"{namespace}:{key}".encode("utf-8")) & 0x7FFFFFFF
    marker.type = marker_type
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    return marker


def _set_color(
    marker: Marker,
    red: float,
    green: float,
    blue: float,
    alpha: float,
    fresh: bool,
) -> None:
    if fresh:
        marker.color.r = red
        marker.color.g = green
        marker.color.b = blue
    else:
        marker.color.r = 0.45
        marker.color.g = 0.45
        marker.color.b = 0.45
    marker.color.a = alpha


def _affiliation_color(affiliation: int) -> tuple[float, float, float]:
    if affiliation == RobotTrack.AFFILIATION_ALLY:
        return (0.08, 0.30, 1.0)
    if affiliation == RobotTrack.AFFILIATION_OPPONENT:
        return (1.0, 0.10, 0.08)
    return (1.0, 0.82, 0.08)


def _affiliation_name(affiliation: int) -> str:
    if affiliation == RobotTrack.AFFILIATION_ALLY:
        return "ALLY"
    if affiliation == RobotTrack.AFFILIATION_OPPONENT:
        return "OPPONENT"
    return "UNKNOWN"


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WorldVisualizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
