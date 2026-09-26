"""Deterministic Monte Carlo utility evaluation with explainable components."""

from __future__ import annotations

import hashlib
import random

from .model import CandidateTask, StrategySnapshot, StrategyWeights, UtilityBreakdown


class StrategyEvaluator:
    def __init__(self, weights: StrategyWeights, rollout_count: int = 64) -> None:
        if rollout_count <= 0:
            raise ValueError("rollout_count must be positive")
        self._weights = weights
        self._rollout_count = rollout_count

    def evaluate(
        self,
        candidate: CandidateTask,
        snapshot: StrategySnapshot,
        seed: int,
    ) -> UtilityBreakdown:
        rng = random.Random(_candidate_seed(seed, candidate.task_id))
        successes = 0
        duration_total = 0.0
        for _ in range(self._rollout_count):
            successes += rng.random() < candidate.success_probability
            duration_total += max(
                0.05,
                rng.gauss(candidate.estimated_duration_s, candidate.duration_std_s),
            )

        success_rate = successes / self._rollout_count
        mean_duration = duration_total / self._rollout_count
        direct_score = candidate.base_score * success_rate
        ownership_score = candidate.ownership_value * success_rate
        ranking_point_value = (
            self._weights.ranking_point * candidate.ranking_point_value * success_rate
        )
        future_value = self._weights.future * candidate.future_value * success_rate
        time_penalty = self._weights.time * mean_duration
        collision_penalty = self._weights.collision * candidate.collision_probability
        failure_penalty = self._weights.failure * (1.0 - success_rate)
        overlap_penalty = self._weights.overlap * candidate.reservation_cost
        rule_penalty = self._weights.penalty * candidate.rule_penalty_probability
        uncertainty_penalty = self._weights.uncertainty * snapshot.uncertainty
        total = (
            direct_score
            + ownership_score
            + ranking_point_value
            + future_value
            - time_penalty
            - collision_penalty
            - failure_penalty
            - overlap_penalty
            - rule_penalty
            - uncertainty_penalty
        )
        return UtilityBreakdown(
            task_id=candidate.task_id,
            direct_score=direct_score,
            ownership_score=ownership_score,
            ranking_point_value=ranking_point_value,
            future_value=future_value,
            time_penalty=time_penalty,
            collision_penalty=collision_penalty,
            failure_penalty=failure_penalty,
            overlap_penalty=overlap_penalty,
            rule_penalty=rule_penalty,
            uncertainty_penalty=uncertainty_penalty,
            total=total,
        )

    def rank(
        self,
        candidates: tuple[CandidateTask, ...],
        snapshot: StrategySnapshot,
        seed: int,
    ) -> tuple[tuple[CandidateTask, UtilityBreakdown], ...]:
        evaluated = [
            (candidate, self.evaluate(candidate, snapshot, seed))
            for candidate in candidates
            if candidate.hard_valid
        ]
        return tuple(sorted(evaluated, key=lambda item: (
            -item[1].total,
            item[0].collision_probability,
            item[0].estimated_duration_s,
            item[0].task_id,
        )))


def _candidate_seed(seed: int, task_id: str) -> int:
    payload = f"{seed}:{task_id}".encode("utf-8")
    return int.from_bytes(hashlib.sha256(payload).digest()[:8], "big", signed=False)
