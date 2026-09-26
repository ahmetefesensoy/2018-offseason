"""Rate-limited strategy executive with hysteresis and emergency preemption."""

from __future__ import annotations

import hashlib
import json

from .evaluator import StrategyEvaluator
from .model import CandidateTask, Decision, StrategySnapshot


class StrategyExecutive:
    def __init__(
        self,
        evaluator: StrategyEvaluator,
        min_tick_interval_us: int = 200_000,
        hysteresis: float = 2.0,
        emergency_collision_probability: float = 0.55,
    ) -> None:
        self._evaluator = evaluator
        self._min_tick_interval_us = min_tick_interval_us
        self._hysteresis = hysteresis
        self._emergency_collision_probability = emergency_collision_probability
        self._last_tick_us: int | None = None
        self._current_task_id: str | None = None
        self._sequence = 0

    def decide(
        self,
        snapshot: StrategySnapshot,
        candidates: tuple[CandidateTask, ...],
        seed: int,
        emergency: bool = False,
    ) -> Decision | None:
        if (
            not emergency
            and self._last_tick_us is not None
            and snapshot.now_us - self._last_tick_us < self._min_tick_interval_us
        ):
            return None
        self._last_tick_us = snapshot.now_us

        eligible = candidates
        if emergency:
            eligible = tuple(
                task for task in candidates
                if task.collision_probability < self._emergency_collision_probability
            )
        ranked = self._evaluator.rank(eligible, snapshot, seed)
        if not ranked:
            safe = _safe_wait(snapshot)
            ranked = self._evaluator.rank((safe,), snapshot, seed)

        proposed, proposed_utility = ranked[0]
        chosen = proposed
        reason = "EMERGENCY_PREEMPT" if emergency else "INITIAL"

        if not emergency and self._current_task_id is not None:
            current_entry = next(
                (entry for entry in ranked if entry[0].task_id == self._current_task_id),
                None,
            )
            if current_entry is not None:
                improvement = proposed_utility.total - current_entry[1].total
                if proposed.task_id != self._current_task_id and improvement <= self._hysteresis:
                    chosen = current_entry[0]
                    reason = "HYSTERESIS_HOLD"
                elif proposed.task_id == self._current_task_id:
                    reason = "WORLD_UPDATE"
                else:
                    reason = "UTILITY_SWITCH"
            else:
                reason = "TASK_INVALIDATED"

        self._current_task_id = chosen.task_id
        self._sequence += 1
        counterfactual = _counterfactual(chosen, ranked)
        decision_hash = _decision_hash(snapshot, seed, chosen, ranked, reason)
        return Decision(
            world_version=snapshot.world_version,
            sequence=self._sequence,
            seed=seed,
            chosen=chosen,
            ranked=ranked,
            replan_reason=reason,
            counterfactual=counterfactual,
            decision_hash=decision_hash,
        )


def _safe_wait(snapshot: StrategySnapshot) -> CandidateTask:
    return CandidateTask(
        task_id="wait:safe",
        task_type="WAIT",
        target_id="current_pose",
        target_pose=snapshot.robot_pose,
        estimated_duration_s=0.25,
        duration_std_s=0.0,
        success_probability=1.0,
        collision_probability=0.0,
        reservation_cost=0.0,
        required_capability="",
        base_score=0.0,
        intake_action="HOLD" if snapshot.has_cube else "STOP",
    )


def _counterfactual(
    chosen: CandidateTask,
    ranked: tuple,
) -> str:
    chosen_utility = next(item[1] for item in ranked if item[0].task_id == chosen.task_id)
    alternatives = [item for item in ranked if item[0].task_id != chosen.task_id]
    if not alternatives:
        return f"{chosen.task_id} tek geçerli görevdi."
    alternative, utility = alternatives[0]
    delta = chosen_utility.total - utility.total
    return (
        f"{chosen.task_id}, {alternative.task_id} görevine göre "
        f"{delta:.2f} fayda farkıyla seçildi."
    )


def _decision_hash(
    snapshot: StrategySnapshot,
    seed: int,
    chosen: CandidateTask,
    ranked: tuple,
    reason: str,
) -> str:
    payload = {
        "world_version": snapshot.world_version,
        "seed": seed,
        "chosen": chosen.task_id,
        "reason": reason,
        "ranked": [
            {
                "task_id": task.task_id,
                "total": round(utility.total, 12),
                "collision": round(task.collision_probability, 12),
            }
            for task, utility in ranked
        ],
    }
    canonical = json.dumps(payload, sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(canonical.encode("utf-8")).hexdigest()
