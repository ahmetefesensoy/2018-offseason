from dataclasses import replace
import unittest

from frc_strategy.evaluator import StrategyEvaluator
from frc_strategy.executive import StrategyExecutive
from frc_strategy.model import CandidateTask, Pose2, StrategyWeights
from test_evaluator import candidate
from test_task_generator import snapshot


def safe_wait():
    return CandidateTask(
        "wait:safe", "WAIT", "current_pose", Pose2(1.0, 2.0, 0.0),
        0.25, 0.0, 1.0, 0.0, 0.0, "", 0.0,
    )


class StrategyExecutiveTest(unittest.TestCase):
    def setUp(self):
        evaluator = StrategyEvaluator(StrategyWeights(hysteresis=2.0), rollout_count=64)
        self.executive = StrategyExecutive(evaluator, min_tick_interval_us=200_000)

    def test_rechecks_at_five_hz_and_uses_hysteresis(self):
        current = candidate("score:current", base_score=20.0, collision_probability=0.0, reservation_cost=0.0)
        challenger = candidate("score:challenger", base_score=21.0, collision_probability=0.0, reservation_cost=0.0)
        state = snapshot(now_us=1_000_000)

        first = self.executive.decide(state, (current, safe_wait()), seed=44)
        too_soon = self.executive.decide(replace(state, now_us=1_100_000), (challenger, current), seed=44)
        reconsidered = self.executive.decide(replace(state, now_us=1_250_000), (challenger, current), seed=44)

        self.assertEqual(first.chosen.task_id, "score:current")
        self.assertIsNone(too_soon)
        self.assertEqual(reconsidered.chosen.task_id, "score:current")
        self.assertEqual(reconsidered.replan_reason, "HYSTERESIS_HOLD")

    def test_emergency_bypasses_tick_rate_and_selects_safe_wait(self):
        dangerous = candidate("score:danger", collision_probability=0.9, reservation_cost=0.0)
        state = snapshot(now_us=2_000_000)
        self.executive.decide(state, (dangerous, safe_wait()), seed=8)

        decision = self.executive.decide(
            replace(state, now_us=2_010_000),
            (dangerous, safe_wait()),
            seed=8,
            emergency=True,
        )

        self.assertEqual(decision.chosen.task_type, "WAIT")
        self.assertEqual(decision.replan_reason, "EMERGENCY_PREEMPT")

    def test_equal_inputs_produce_equal_decision_hashes(self):
        state = snapshot(now_us=3_000_000)
        tasks = (candidate("score:b"), candidate("score:a"), safe_wait())
        other = StrategyExecutive(
            StrategyEvaluator(StrategyWeights(hysteresis=2.0), rollout_count=64),
            min_tick_interval_us=200_000,
        )

        first = self.executive.decide(state, tasks, seed=101)
        second = other.decide(state, tasks, seed=101)

        self.assertEqual(first.chosen.task_id, "score:a")
        self.assertEqual(first.decision_hash, second.decision_hash)


if __name__ == "__main__":
    unittest.main()
