"""RViz markers for alliance paths, reservation corridors, and deviations."""

from __future__ import annotations

import math
import zlib

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray

from frc_autonomy_msgs.msg import ReservationTubeArray, RobotTrack, WorldState

from .reservations import (
    ReservationSample,
    ReservationTube,
    find_conflicts,
    sample_reservation,
)
from .schema import Pose2d


class AllianceVisualizerNode(Node):
    def __init__(self) -> None:
        super().__init__("alliance_visualizer")
        self._publisher = self.create_publisher(
            MarkerArray,
            "/alliance/markers",
            10,
        )
        self._tubes: tuple[ReservationTube, ...] = ()
        self._world: WorldState | None = None
        self.create_subscription(
            ReservationTubeArray,
            "/alliance/reservations",
            self._reservation_callback,
            10,
        )
        self.create_subscription(
            WorldState,
            "/world/state",
            self._world_callback,
            10,
        )

    def _reservation_callback(self, message: ReservationTubeArray) -> None:
        self._tubes = tuple(
            converted
            for converted in (_to_tube(tube) for tube in message.tubes)
            if converted.samples
        )
        self._publish()

    def _world_callback(self, message: WorldState) -> None:
        self._world = message
        if self._tubes:
            self._publish()

    def _publish(self) -> None:
        stamp = self.get_clock().now().to_msg()
        delete_all = Marker()
        delete_all.header.frame_id = "map"
        delete_all.header.stamp = stamp
        delete_all.action = Marker.DELETEALL
        markers = [delete_all]
        for index, tube in enumerate(self._tubes):
            markers.extend(_tube_markers(tube, index, stamp))
        markers.extend(_conflict_markers(self._tubes, stamp))
        if self._world is not None:
            markers.extend(_deviation_markers(self._tubes, self._world, stamp))
        self._publisher.publish(MarkerArray(markers=markers))


def _tube_markers(tube: ReservationTube, index: int, stamp) -> list[Marker]:
    red, green, blue = _team_color(index)
    points = [Point(x=value.pose.x, y=value.pose.y, z=0.08) for value in tube.samples]

    corridor = _marker("reservation_corridors", str(tube.team_number), Marker.LINE_STRIP, stamp)
    corridor.points = points
    corridor.scale.x = 2.0 * max(value.radius_m for value in tube.samples)
    _color(corridor, red, green, blue, 0.16 + 0.24 * tube.confidence)

    path = _marker("alliance_paths", str(tube.team_number), Marker.LINE_STRIP, stamp)
    path.points = [Point(x=value.pose.x, y=value.pose.y, z=0.12) for value in tube.samples]
    path.scale.x = 0.055
    _color(path, red, green, blue, 0.35 + 0.65 * tube.confidence)

    label = _marker("alliance_labels", str(tube.team_number), Marker.TEXT_VIEW_FACING, stamp)
    label.pose.position.x = tube.samples[0].pose.x
    label.pose.position.y = tube.samples[0].pose.y
    label.pose.position.z = 0.62
    label.scale.z = 0.17
    label.text = f"{tube.robot_label} #{tube.team_number} plan {tube.confidence:.0%}"
    _color(label, red, green, blue, 1.0)
    return [corridor, path, label]


def _conflict_markers(tubes: tuple[ReservationTube, ...], stamp) -> list[Marker]:
    by_team = {tube.team_number: tube for tube in tubes}
    markers = []
    for index, conflict in enumerate(find_conflicts(tubes)):
        time_us = (conflict.start_us + conflict.end_us) // 2
        first = sample_reservation(by_team[conflict.first_team_number], time_us)
        second = sample_reservation(by_team[conflict.second_team_number], time_us)
        x = (first.pose.x + second.pose.x) / 2.0
        y = (first.pose.y + second.pose.y) / 2.0

        body = _marker("reservation_conflicts", str(index), Marker.CYLINDER, stamp)
        body.pose.position.x = x
        body.pose.position.y = y
        body.pose.position.z = 0.035
        diameter = max(first.radius_m + second.radius_m, 0.1)
        body.scale.x = diameter
        body.scale.y = diameter
        body.scale.z = 0.07
        _color(body, 1.0, 0.04, 0.02, 0.55)

        label = _marker("conflict_labels", str(index), Marker.TEXT_VIEW_FACING, stamp)
        label.pose.position.x = x
        label.pose.position.y = y
        label.pose.position.z = 0.48
        label.scale.z = 0.15
        label.text = (
            f"CONFLICT #{conflict.first_team_number}/#{conflict.second_team_number} "
            f"{conflict.start_us / 1e6:.1f}-{conflict.end_us / 1e6:.1f}s"
        )
        _color(label, 1.0, 0.15, 0.08, 1.0)
        markers.extend((body, label))
    return markers


def _deviation_markers(
    tubes: tuple[ReservationTube, ...],
    world: WorldState,
    stamp,
) -> list[Marker]:
    allies = [
        robot
        for robot in world.robots
        if int(robot.affiliation) == RobotTrack.AFFILIATION_ALLY
    ]
    candidates = []
    for robot in allies:
        observed_x = float(robot.pose.pose.position.x)
        observed_y = float(robot.pose.pose.position.y)
        for tube in tubes:
            nearest = min(
                tube.samples,
                key=lambda value: math.hypot(
                    observed_x - value.pose.x,
                    observed_y - value.pose.y,
                ),
            )
            distance = math.hypot(observed_x - nearest.pose.x, observed_y - nearest.pose.y)
            candidates.append((distance, robot.track_id, tube.team_number, robot, nearest))

    markers = []
    used_tracks: set[str] = set()
    used_teams: set[int] = set()
    for distance, track_id, team_number, robot, nearest in sorted(candidates):
        if track_id in used_tracks or team_number in used_teams:
            continue
        used_tracks.add(track_id)
        used_teams.add(team_number)
        marker = _marker("plan_deviations", track_id, Marker.LINE_LIST, stamp)
        marker.points = [
            Point(
                x=float(robot.pose.pose.position.x),
                y=float(robot.pose.pose.position.y),
                z=0.32,
            ),
            Point(x=nearest.pose.x, y=nearest.pose.y, z=0.32),
        ]
        marker.scale.x = 0.035
        if distance <= nearest.radius_m:
            _color(marker, 0.12, 1.0, 0.25, 0.75)
        else:
            _color(marker, 1.0, 0.08, 0.04, 0.95)
        markers.append(marker)
    return markers


def _to_tube(message) -> ReservationTube:
    return ReservationTube(
        plan_id=message.plan_id,
        team_number=int(message.team_number),
        robot_label=message.robot_label,
        confidence=float(message.confidence),
        samples=tuple(
            ReservationSample(
                time_us=int(sample.time_us),
                pose=Pose2d(
                    float(sample.pose.x),
                    float(sample.pose.y),
                    float(sample.pose.theta),
                ),
                radius_m=float(sample.radius_m),
            )
            for sample in message.samples
        ),
    )


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


def _color(marker: Marker, red: float, green: float, blue: float, alpha: float) -> None:
    marker.color.r = red
    marker.color.g = green
    marker.color.b = blue
    marker.color.a = alpha


def _team_color(index: int) -> tuple[float, float, float]:
    colors = ((0.10, 0.52, 1.0), (0.60, 0.25, 1.0))
    return colors[index % len(colors)]


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AllianceVisualizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
