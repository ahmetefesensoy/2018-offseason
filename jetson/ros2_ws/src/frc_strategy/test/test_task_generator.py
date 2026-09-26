import unittest

from frc_strategy.model import (
    FieldTarget,
    Pose2,
    ReservationZone,
    StrategySnapshot,
    WorldObject,
)
from frc_strategy.task_generator import TaskGenerator


TARGETS = (
    FieldTarget("auto_line", "CROSS_LINE", Pose2(3.2, 2.0, 0.0), "drive", 5.0, 2.0),
    FieldTarget("switch_left", "SCORE_SWITCH", Pose2(4.2, 2.1, 0.0), "switch", 25.0, 2.5, 0.55, "EJECT"),
    FieldTarget("switch_right", "SCORE_SWITCH", Pose2(4.2, 6.1, 0.0), "switch", 25.0, 2.5, 0.55, "EJECT"),
    FieldTarget("scale_left", "SCORE_SCALE", Pose2(7.0, 2.0, 0.0), "scale", 35.0, 3.5, 1.55, "EJECT"),
)


def snapshot(**overrides):
    values = dict(
        world_version=7,
        now_us=1_000_000,
        match_time_remaining_s=12.0,
        autonomous=True,
        perception_fresh=True,
        robot_pose=Pose2(1.0, 2.0, 0.0),
        has_cube=False,
        game_objects=(),
        robots=(),
        reservations=(),
        capabilities=frozenset({"drive", "intake", "switch"}),
        our_switch_side="LEFT",
        scale_side="LEFT",
        uncertainty=0.1,
    )
    values.update(overrides)
    return StrategySnapshot(**values)


class TaskGeneratorTest(unittest.TestCase):
    def setUp(self):
        self.generator = TaskGenerator(TARGETS)

    def test_stale_perception_suppresses_pickup_but_keeps_safe_fallbacks(self):
        state = snapshot(
            perception_fresh=False,
            game_objects=(WorldObject("cube-1", "power_cube", Pose2(2.0, 2.0, 0.0), 0.99),),
        )

        tasks = self.generator.generate(state)

        self.assertEqual([task.task_type for task in tasks], ["CROSS_LINE", "WAIT"])

    def test_fresh_cube_generates_nearest_pickup_with_intake_intent(self):
        state = snapshot(game_objects=(
            WorldObject("far", "power_cube", Pose2(5.0, 5.0, 0.0), 0.99),
            WorldObject("near", "power_cube", Pose2(2.0, 2.0, 0.0), 0.90),
        ))

        tasks = self.generator.generate(state)
        pickup = next(task for task in tasks if task.task_type == "PICKUP")

        self.assertEqual(pickup.target_id, "near")
        self.assertEqual(pickup.intake_action, "INTAKE")
        self.assertEqual(pickup.target_pose, Pose2(2.0, 2.0, 0.0))

    def test_cube_held_generates_owned_switch_and_reservation_cost(self):
        reservation = ReservationZone(
            "ally-left", 1_000_000, 6_000_000, Pose2(4.2, 2.1, 0.0), 0.8, 0.75,
        )
        state = snapshot(has_cube=True, reservations=(reservation,))

        tasks = self.generator.generate(state)
        score = next(task for task in tasks if task.task_type == "SCORE_SWITCH")

        self.assertEqual(score.target_id, "switch_left")
        self.assertAlmostEqual(score.reservation_cost, 0.75)
        self.assertEqual(score.intake_action, "EJECT")
        self.assertAlmostEqual(score.elevator_target_m, 0.55)

    def test_unsupported_scale_capability_never_generates_scale_task(self):
        tasks = self.generator.generate(snapshot(has_cube=True))

        self.assertNotIn("SCORE_SCALE", {task.task_type for task in tasks})


if __name__ == "__main__":
    unittest.main()
