# End-to-End Dynamic Autonomy Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Complete a deterministic, explainable FRC 2018 autonomous loop that selects scoring tasks, plans around moving robots and alliance reservations, commands the swerve and mechanisms through the existing fail-closed NT4 link, and can be demonstrated with synthetic perception and RViz.

**Architecture:** Pure Python cores implement strategy, navigation, perception projection, and scenario replay so every safety decision can be unit tested without ROS. Thin ROS 2 nodes translate the existing `WorldState`, reservation, and roboRIO status topics into those cores, then publish a single safe `AutonomyCommand`; the roboRIO remains the final motor authority. A local model adapter validates immutable metadata before loading an ONNX/TensorRT detector, while the existing synthetic perception path remains the deterministic CI/demo source.

**Tech Stack:** Java 17, WPILib 2025, NT4, Python 3.12-compatible core modules, ROS 2 `rclpy`, standard ROS messages, YAML, RViz2, ONNX Runtime/TensorRT adapter boundary.

**Spec:** `docs/superpowers/specs/2026-09-26-ros2-strategic-autonomy-design.md`

## Global Constraints

- Jetson never accesses the motor CAN bus; all motion and mechanism commands cross the existing NT4 snapshot protocol.
- RoboRIO rejects stale, uncommitted, non-finite, out-of-limit, disabled, wrong-mode, or unhealthy commands and stops within the existing watchdog deadline.
- Strategy output is deterministic for the same world version, seed, match state, and configuration.
- The navigator is the only ROS node allowed to publish `/autonomy/command` in the dynamic profile.
- Dynamic obstacles always increase cost or cause slow/stop; degraded perception never permits a higher speed.
- The production path requires validated detector metadata; tests and the offline demo use synthetic observations and do not require a model binary.
- Unsupported robot capabilities never produce candidate tasks.

---

### Task 1: Strategy and navigation message contracts

**Files:**
- Create: `jetson/ros2_ws/src/frc_autonomy_msgs/msg/CandidateTask.msg`
- Create: `jetson/ros2_ws/src/frc_autonomy_msgs/msg/UtilityBreakdown.msg`
- Create: `jetson/ros2_ws/src/frc_autonomy_msgs/msg/DecisionTrace.msg`
- Create: `jetson/ros2_ws/src/frc_autonomy_msgs/msg/NavigationGoal.msg`
- Create: `jetson/ros2_ws/src/frc_autonomy_msgs/msg/NavigationState.msg`
- Modify: `jetson/ros2_ws/src/frc_autonomy_msgs/msg/AutonomyCommand.msg`
- Modify: `jetson/ros2_ws/src/frc_autonomy_msgs/CMakeLists.txt`
- Test: static message registration check in the final verification command

**Interfaces:**
- Consumes: existing `WorldState`, `ReservationTubeArray`, and `AutonomyStatus` messages.
- Produces: versioned task candidates, explainable decision traces, navigation goals/states, and bounded intake/elevator command fields.

- [ ] **Step 1: Add all message files to a failing registration check**

  Run a Python check that expects all five files and their `msg/<name>` entries in `CMakeLists.txt`; confirm it fails because the files are absent.

- [ ] **Step 2: Define exact wire fields**

  `CandidateTask` carries task/target IDs, target pose, duration, success probability, required capability, and hard-valid flag. `UtilityBreakdown` carries every signed utility component and total. `DecisionTrace` carries world version, seed, chosen task, ordered candidates/utilities, replan reason, and counterfactual. `NavigationGoal` carries task, pose, tolerances, and world version. `NavigationState` carries mode, path, clearance, speed limit, obstacle reason, and goal status.

- [ ] **Step 3: Extend `AutonomyCommand` safely**

  Add `intake_action` with `STOP/INTAKE/HOLD/EJECT`, `elevator_target_m`, and `mechanism_enabled`; disarmed frames must serialize STOP, ground height, and disabled.

- [ ] **Step 4: Run registration/XML validation and commit**

  Expected: all message files registered exactly once and package XML parses.

### Task 2: Deterministic strategic decision engine

**Files:**
- Create: `jetson/ros2_ws/src/frc_strategy/frc_strategy/model.py`
- Create: `jetson/ros2_ws/src/frc_strategy/frc_strategy/task_generator.py`
- Create: `jetson/ros2_ws/src/frc_strategy/frc_strategy/evaluator.py`
- Create: `jetson/ros2_ws/src/frc_strategy/frc_strategy/executive.py`
- Create: `jetson/ros2_ws/src/frc_strategy/frc_strategy/strategy_node.py`
- Create: `jetson/ros2_ws/src/frc_strategy/config/power_up.yaml`
- Create: `jetson/ros2_ws/src/frc_strategy/config/robot_capabilities.yaml`
- Create: `jetson/ros2_ws/src/frc_strategy/test/test_task_generator.py`
- Create: `jetson/ros2_ws/src/frc_strategy/test/test_evaluator.py`
- Create: `jetson/ros2_ws/src/frc_strategy/test/test_executive.py`
- Create: package metadata files under `jetson/ros2_ws/src/frc_strategy/`

**Interfaces:**
- Consumes: immutable `StrategySnapshot`, field targets, robot capabilities, reservations, and deterministic seed.
- Produces: `Decision` with chosen task, ranked utilities, counterfactual text, replan reason, and mechanism intent.

- [ ] **Step 1: Write failing task-generation tests**

  Cover fresh-cube pickup, cube-held switch/scale scoring, cross-line fallback, stale-perception suppression, capability suppression, and teammate-reserved target suppression.

- [ ] **Step 2: Implement immutable strategy models and task generation**

  Generate only tasks whose hard preconditions pass; sort by stable task ID before evaluation.

- [ ] **Step 3: Write failing evaluator tests**

  Assert direct score, time/risk/reservation penalties, deterministic 64-rollout results, stable tie-break, and a higher-utility unreserved switch target.

- [ ] **Step 4: Implement deterministic rolling-horizon evaluation**

  Use `random.Random(derived_seed)` per candidate, sample duration and success, compute every `UtilityBreakdown` term, and order by total descending, collision risk ascending, duration ascending, task ID ascending.

- [ ] **Step 5: Write failing executive tests**

  Prove 250 ms reevaluation, hysteresis against task thrashing, immediate emergency preemption, and safe WAIT when no candidate is valid.

- [ ] **Step 6: Implement executive and thin ROS node**

  The node publishes candidates, trace, and one `NavigationGoal`; it never publishes `/autonomy/command`.

- [ ] **Step 7: Run package tests and commit**

  Expected: all strategy tests pass twice with identical decision hashes.

### Task 3: Risk-aware dynamic navigation and collision monitor

**Files:**
- Create: `jetson/ros2_ws/src/frc_navigation/frc_navigation/model.py`
- Create: `jetson/ros2_ws/src/frc_navigation/frc_navigation/costmap.py`
- Create: `jetson/ros2_ws/src/frc_navigation/frc_navigation/planner.py`
- Create: `jetson/ros2_ws/src/frc_navigation/frc_navigation/controller.py`
- Create: `jetson/ros2_ws/src/frc_navigation/frc_navigation/collision_monitor.py`
- Create: `jetson/ros2_ws/src/frc_navigation/frc_navigation/navigation_node.py`
- Create: `jetson/ros2_ws/src/frc_navigation/config/navigation.yaml`
- Create: `jetson/ros2_ws/src/frc_navigation/test/test_costmap.py`
- Create: `jetson/ros2_ws/src/frc_navigation/test/test_planner.py`
- Create: `jetson/ros2_ws/src/frc_navigation/test/test_controller.py`
- Create: `jetson/ros2_ws/src/frc_navigation/test/test_collision_monitor.py`
- Create: package metadata files under `jetson/ros2_ws/src/frc_navigation/`

**Interfaces:**
- Consumes: robot pose, `NavigationGoal`, `WorldState` robot predictions, alliance reservations, and perception freshness.
- Produces: a collision-checked timed path, robot-relative velocity, mechanism intent, and the sole `/autonomy/command` stream.

- [ ] **Step 1: Write failing costmap and planner tests**

  Cover field bounds, static switch/scale geometry, moving-opponent prediction, soft alliance reservations, blocked direct path detour, deterministic path, and unreachable-goal failure.

- [ ] **Step 2: Implement temporal cost queries and deterministic A***

  Search an 8-connected 0.25 m lattice with 100 ms time buckets, hard collision rejection, soft reservation/risk costs, admissible Euclidean heuristic, and stable neighbor ordering.

- [ ] **Step 3: Write failing controller and collision-monitor tests**

  Assert holonomic tracking, heading control, speed degradation on stale perception, slow polygon behavior, emergency stop distance, opponent crossing interception, and disarmed zero output.

- [ ] **Step 4: Implement pure pursuit/heading control and independent monitor**

  Clamp translation to 0.75 m/s and rotation to 1.5 rad/s, transform field velocity into robot coordinates, and override the command with slow/stop based on time-to-collision and footprint clearance.

- [ ] **Step 5: Implement the navigation ROS node**

  Replan at 5 Hz, control at 30 Hz, fail closed on stale world/status/goal, publish `nav_msgs/Path`, `NavigationState`, and bounded `AutonomyCommand` with a 100 ms validity window.

- [ ] **Step 6: Run package tests and commit**

  Expected: deterministic paths and zero non-finite commands across the scenario matrix.

### Task 4: Mechanism command bridge and roboRIO execution

**Files:**
- Modify: `jetson/ros2_ws/src/frc_nt_bridge/frc_nt_bridge/protocol.py`
- Modify: `jetson/ros2_ws/src/frc_nt_bridge/frc_nt_bridge/bridge_node.py`
- Modify: `jetson/ros2_ws/src/frc_bringup/frc_bringup/fake_roborio_node.py`
- Modify: `src/main/java/frc/robot/autonomy/AutonomyCommandFrame.java`
- Modify: `src/main/java/frc/robot/autonomy/AutonomyLinkIONetworkTables.java`
- Modify: `src/main/java/frc/robot/autonomy/AutonomySafetyGate.java`
- Create: `src/main/java/frc/robot/autonomy/AutonomyMechanismCommand.java`
- Modify: `src/main/java/frc/robot/RobotContainer.java`
- Test: `jetson/ros2_ws/src/frc_nt_bridge/test/test_protocol.py`
- Test: `src/test/java/frc/robot/autonomy/AutonomyMechanismCommandTest.java`
- Test: existing autonomy safety and NT tests

**Interfaces:**
- Consumes: the navigation node's bounded mechanism intent.
- Produces: atomically committed NT fields and a roboRIO command that owns intake/elevator only while autonomous is scheduled.

- [ ] **Step 1: Write failing Python protocol tests**

  Assert commit-last ordering, invalid action rejection, non-finite/out-of-range elevator rejection, and disarmed safe mechanism fields.

- [ ] **Step 2: Extend Python framing and fake roboRIO**

  Publish mechanism fields before sequence/commit and reject the whole snapshot when any field is unsafe.

- [ ] **Step 3: Write failing Java NT/safety/executor tests**

  Assert atomic read, enum/range rejection, intake action dispatch, elevator clamping, and immediate stop on command end.

- [ ] **Step 4: Implement roboRIO mechanism execution**

  Run drive and mechanism commands in parallel from the same accepted frame; never let a stale frame keep intake motors active.

- [ ] **Step 5: Run Java and bridge tests and commit**

  Expected: all existing watchdog and drivetrain tests remain green.

### Task 5: Jetson detector adapter and metadata gate

**Files:**
- Create: `jetson/ros2_ws/src/frc_perception/frc_perception/model_contract.py`
- Create: `jetson/ros2_ws/src/frc_perception/frc_perception/projection.py`
- Create: `jetson/ros2_ws/src/frc_perception/frc_perception/detector_node.py`
- Create: `jetson/ros2_ws/src/frc_perception/config/detector.yaml`
- Create: `jetson/ros2_ws/src/frc_perception/config/model.example.json`
- Create: `jetson/ros2_ws/src/frc_perception/test/test_model_contract.py`
- Create: `jetson/ros2_ws/src/frc_perception/test/test_projection.py`
- Create: package metadata files under `jetson/ros2_ws/src/frc_perception/`

**Interfaces:**
- Consumes: synchronized color/depth image and `CameraInfo`, plus a validated local model artifact.
- Produces: `SemanticObservationArray` in `camera_optical_frame`; the world model remains tracking authority.

- [ ] **Step 1: Write failing metadata tests**

  Reject wrong schema, class order, artifact size/hash, unsupported backend, and non-finite thresholds; accept a matching temporary artifact.

- [ ] **Step 2: Implement immutable metadata validation**

  Require classes `power_cube`, `robot`, `switch_plate`, `scale_plate`, and `vault_opening`; verify SHA-256 before runtime creation.

- [ ] **Step 3: Write failing depth-projection tests**

  Cover median depth, invalid depth rejection, pinhole projection, covariance growth, and deterministic observation ordering.

- [ ] **Step 4: Implement projection and lazy backend loading**

  Keep TensorRT/ONNX imports inside runtime construction so core tests run without GPU libraries; refuse node activation when metadata or artifact validation fails.

- [ ] **Step 5: Run perception tests and commit**

  Expected: model-independent core tests pass; missing production engine fails closed with a clear diagnostic.

### Task 6: Integrated launch, RViz, scenarios, and replay

**Files:**
- Modify: `jetson/ros2_ws/src/frc_bringup/launch/foundation.launch.py`
- Modify: `jetson/ros2_ws/src/frc_robot_description/rviz/autonomy.rviz`
- Modify: `jetson/compose.yaml`
- Create: `jetson/ros2_ws/src/frc_bringup/frc_bringup/scenario_runner.py`
- Create: `jetson/ros2_ws/src/frc_bringup/test/test_end_to_end_scenario.py`
- Create: `scenarios/2018/open_field_cube.yaml`
- Create: `scenarios/2018/opponent_crossing.yaml`
- Create: `scenarios/2018/teammate_delay.yaml`
- Create: `docs/dynamic-autonomy.md`

**Interfaces:**
- Consumes: every package from Tasks 1-5.
- Produces: one launch command for synthetic end-to-end autonomy, stable replay hashes, RViz overlays, and operator instructions.

- [ ] **Step 1: Write a failing end-to-end scenario test**

  Feed deterministic robot/cube/opponent frames, require a scoring decision, a collision-free detour, bounded command output, emergency stop on interception, and an identical second-run decision/path hash.

- [ ] **Step 2: Implement the pure scenario runner**

  Execute strategy at 5 Hz and navigation at 30 Hz over a fixed clock; save trace hashes and safety events.

- [ ] **Step 3: Wire launch and RViz**

  Add `use_dynamic_autonomy`, `use_detector`, and scenario arguments. Disable `fake_autonomy` whenever dynamic autonomy is enabled. Display selected path, decision text markers, tracked robots, reservations, and navigation state.

- [ ] **Step 4: Document deploy and calibration workflow**

  Include model artifact placement, SHA generation, colcon build, synthetic launch, real NT4 launch, arm/disarm behavior, logging, expected topics, and field-test gates.

- [ ] **Step 5: Run the full verification matrix and commit**

  Run all Java tests/build, all Python package tests, Node Studio tests, Python compile, XML/YAML/JSON parsing, message registration, Docker Compose parsing, command-authority scan, deterministic scenario replay, `git diff --check`, and clean-status check.

