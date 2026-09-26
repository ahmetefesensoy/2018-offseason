from dataclasses import replace
import unittest

from frc_strategy.evaluator import StrategyEvaluator
from frc_strategy.model import CandidateTask, Pose2, StrategyWeights
from test_task_generator import snapshot


def candidate(task_id="score:a", **overrides):
    values = dict(
        task_id=task_id,
        task_type="SCORE_SWITCH",
        target_id=task_id,
        target_pose=Pose2(4.0, 2.0, 0.0),
        estimated_duration_s=2.0,
        duration_std_s=0.0,
        success_probability=1.0,
        collision_probability=0.1,
        reservation_cost=0.2,
        required_capability="switch",
        base_score=25.0,
        ownership_value=5.0,
        future_value=4.0,
        rule_penalty_probability=0.05,
        intake_action="EJECT",
        elevator_target_m=0.55,
    )
    values.update(overrides)
    return CandidateTask(**values)


class StrategyEvaluatorTest(unittest.TestCase):
    def test_reports_literal_explainable_utility_components(self):
        evaluator = StrategyEvaluator(StrategyWeights(), rollout_count=64)

        result = evaluator.evaluate(candidate(), snapshot(), seed=123)

        self.assertAlmostEqual(result.direct_score, 25.0)
        self.assertAlmostEqual(result.ownership_score, 5.0)
        self.assertAlmostEqual(result.future_value, 2.4)
        self.assertAlmostEqual(result.time_penalty, 1.6)
        self.assertAlmostEqual(result.collision_penalty, 6.0)
        self.assertAlmostEqual(result.overlap_penalty, 5.0)
        self.assertAlmostEqual(result.rule_penalty, 1.5)
        self.assertAlmostEqual(result.uncertainty_penalty, 0.8)
        self.assertAlmostEqual(result.total, 17.5)

    def test_probabilistic_rollouts_are_deterministic_for_same_seed(self):
        evaluator = StrategyEvaluator(StrategyWeights(), rollout_count=64)
        uncertain = candidate(
            success_probability=0.63,
            duration_std_s=0.45,
            collision_probability=0.22,
        )

        first = evaluator.evaluate(uncertain, snapshot(), seed=9981)
        second = evaluator.evaluate(uncertain, snapshot(), seed=9981)

        self.assertEqual(first, second)

    def test_ranking_prefers_unreserved_target_then_stable_task_id(self):
        evaluator = StrategyEvaluator(StrategyWeights(), rollout_count=64)
        reserved = candidate("score:reserved", reservation_cost=0.8)
        free_b = candidate("score:free-b", reservation_cost=0.0)
        free_a = candidate("score:free-a", reservation_cost=0.0)

        ranked = evaluator.rank((reserved, free_b, free_a), snapshot(), seed=5)

        self.assertEqual([item[0].task_id for item in ranked], [
            "score:free-a", "score:free-b", "score:reserved",
        ])

    def test_invalid_candidate_is_never_ranked(self):
        evaluator = StrategyEvaluator(StrategyWeights(), rollout_count=64)
        invalid = replace(candidate(), hard_valid=False, rejection_reason="capability")

        self.assertEqual(evaluator.rank((invalid,), snapshot(), seed=1), ())


if __name__ == "__main__":
    unittest.main()
