"""Hardware-free deterministic proof of the strategy/navigation safety loop."""

from __future__ import annotations

import hashlib
import json

from frc_navigation.collision_monitor import CollisionMonitor
from frc_navigation.controller import HolonomicController
from frc_navigation.costmap import TemporalCostmap
from frc_navigation.model import (
    CircleObstacle, NavigationGoal, NavigationWorld, Pose2 as NavPose, VelocityCommand,
)
from frc_navigation.planner import TemporalAStarPlanner
from frc_strategy.evaluator import StrategyEvaluator
from frc_strategy.executive import StrategyExecutive
from frc_strategy.model import FieldTarget, Pose2, StrategySnapshot, StrategyWeights, WorldObject
from frc_strategy.task_generator import TaskGenerator


def run_demo() -> dict:
    snapshot = StrategySnapshot(
        world_version=1, now_us=1_000_000, match_time_remaining_s=15.0,
        autonomous=True, perception_fresh=True, robot_pose=Pose2(1.0, 2.0, 0.0),
        has_cube=False,
        game_objects=(WorldObject("cube-demo", "power_cube", Pose2(4.0, 2.0, 0.0), 0.98),),
        robots=(), reservations=(), capabilities=frozenset({"drive", "intake", "switch"}),
        our_switch_side="LEFT", scale_side="LEFT", uncertainty=0.05,
    )
    targets = (FieldTarget("auto_line", "CROSS_LINE", Pose2(3.2, 3.0, 0.0), "drive", 5.0, 2.0),)
    tasks = TaskGenerator(targets).generate(snapshot)
    executive = StrategyExecutive(StrategyEvaluator(StrategyWeights(), 64))
    decision = executive.decide(snapshot, tasks, seed=2018)

    blocking = CircleObstacle("late-opponent", 2.5, 2.0, 0.0, 0.0, 0.55, True, 1.0)
    world = NavigationWorld(
        1_000_000, True, 8.0, 5.0, 0.45, (), (blocking,), (),
    )
    goal = NavigationGoal(
        decision.chosen.task_id,
        NavPose(decision.chosen.target_pose.x, decision.chosen.target_pose.y, decision.chosen.target_pose.heading),
        0.15, 0.15, 8_000_000, 1, True, 0.0,
    )
    planner = TemporalAStarPlanner(0.25, 0.2)
    path = planner.plan(NavPose(1.0, 2.0, 0.0), goal, TemporalCostmap(world), 1_000_000)
    if not path:
        raise RuntimeError("offline demo could not find a safe path")
    command = HolonomicController().command(NavPose(1.0, 2.0, 0.0), path, True)
    crossing = CircleObstacle("crossing", 1.0, 1.4, 0.0, 1.2, 0.45, True, 1.0)
    stopped = CollisionMonitor().filter(
        VelocityCommand(True, command.vx_mps, command.vy_mps, command.omega_radps, 1, True, 0.0),
        NavPose(1.0, 2.0, 0.0), (crossing,), 0.45, True,
    )
    path_payload = [(point.time_us, point.x, point.y, point.heading) for point in path]
    return {
        "chosen_task": decision.chosen.task_id,
        "decision_hash": decision.decision_hash,
        "path_hash": hashlib.sha256(json.dumps(path_payload, separators=(",", ":")).encode()).hexdigest(),
        "path_points": len(path),
        "detour": any(abs(point.y - 2.0) >= 0.5 for point in path),
        "first_command_mps": round((command.vx_mps**2 + command.vy_mps**2) ** 0.5, 3),
        "emergency_stop": not stopped.command.armed,
        "stop_reason": stopped.reason,
    }


def main() -> None:
    first = run_demo(); second = run_demo()
    first["deterministic_replay"] = (
        first["decision_hash"] == second["decision_hash"]
        and first["path_hash"] == second["path_hash"]
    )
    if not first["detour"] or not first["emergency_stop"] or not first["deterministic_replay"]:
        raise SystemExit("offline autonomy safety demonstration failed")
    print(json.dumps(first, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
