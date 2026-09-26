import math
import unittest

from frc_navigation.collision_monitor import CollisionMonitor
from frc_navigation.controller import HolonomicController
from frc_navigation.costmap import TemporalCostmap
from frc_navigation.model import (
    CircleObstacle,
    NavigationGoal,
    NavigationWorld,
    PathPoint,
    Pose2,
    VelocityCommand,
)
from frc_navigation.planner import TemporalAStarPlanner


def world(obstacles=(), fresh=True):
    return NavigationWorld(
        now_us=1_000_000,
        perception_fresh=fresh,
        field_length_m=8.0,
        field_width_m=5.0,
        robot_radius_m=0.45,
        static_rectangles=(),
        obstacles=tuple(obstacles),
        reservations=(),
    )


class NavigationCoreTest(unittest.TestCase):
    def test_planner_detours_around_blocking_robot_deterministically(self):
        obstacle = CircleObstacle("opponent", 3.0, 2.5, 0.0, 0.0, 0.55, True, 1.0)
        costmap = TemporalCostmap(world((obstacle,)))
        planner = TemporalAStarPlanner(resolution_m=0.25, time_step_s=0.2)
        start = Pose2(1.0, 2.5, 0.0)
        goal = NavigationGoal("score", Pose2(5.0, 2.5, 0.0), 0.15, 0.15, 5_000_000, 0, False, 0.0)

        first = planner.plan(start, goal, costmap, 1_000_000)
        second = planner.plan(start, goal, costmap, 1_000_000)

        self.assertEqual(first, second)
        self.assertGreater(len(first), 2)
        self.assertTrue(any(abs(point.y - 2.5) >= 0.5 for point in first))
        self.assertTrue(all(math.isfinite(point.x + point.y) for point in first))

    def test_controller_outputs_bounded_robot_relative_velocity(self):
        controller = HolonomicController(max_speed_mps=0.75, max_omega_radps=1.5)
        path = (
            PathPoint(1_000_000, 1.0, 1.0, 0.0),
            PathPoint(1_200_000, 2.0, 1.0, 0.0),
        )

        command = controller.command(Pose2(1.0, 1.0, math.pi / 2.0), path, True)

        self.assertAlmostEqual(command.vx_mps, 0.0, places=6)
        self.assertAlmostEqual(command.vy_mps, -0.75, places=6)
        self.assertLessEqual(abs(command.omega_radps), 1.5)

    def test_collision_monitor_stops_for_crossing_opponent(self):
        monitor = CollisionMonitor(stop_clearance_m=0.20, slow_clearance_m=0.80, horizon_s=1.0)
        obstacle = CircleObstacle("crossing", 1.0, -0.6, 0.0, 1.2, 0.45, True, 1.0)
        command = VelocityCommand(True, 0.75, 0.0, 0.0, 0, False, 0.0)

        result = monitor.filter(command, Pose2(0.0, 0.0, 0.0), (obstacle,), 0.45, True)

        self.assertFalse(result.command.armed)
        self.assertEqual((result.command.vx_mps, result.command.vy_mps), (0.0, 0.0))
        self.assertEqual(result.reason, "DYNAMIC_COLLISION_STOP")

    def test_stale_perception_never_increases_speed(self):
        monitor = CollisionMonitor(stop_clearance_m=0.20, slow_clearance_m=0.80, horizon_s=1.0)
        command = VelocityCommand(True, 0.75, 0.0, 0.0, 0, False, 0.0)

        result = monitor.filter(command, Pose2(0.0, 0.0, 0.0), (), 0.45, False)

        self.assertTrue(result.command.armed)
        self.assertAlmostEqual(math.hypot(result.command.vx_mps, result.command.vy_mps), 0.25)
        self.assertEqual(result.reason, "STALE_PERCEPTION_SLOW")


if __name__ == "__main__":
    unittest.main()
