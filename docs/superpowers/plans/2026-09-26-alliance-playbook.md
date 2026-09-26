# Alliance Playbook Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Import, validate, hash, visualize, and continuously confidence-adjust pre-match teammate routes so strategy and navigation can treat them as deterministic time-dependent reservations.

**Architecture:** A ROS-independent canonical schema is the sole authority for plan validation and hashing. CSV and PathPlanner importers normalize external data into that schema; a reservation engine samples timed paths into circular occupancy tubes and decays plan confidence against observed teammate tracks. Thin ROS nodes publish the canonical plan, reservations, diagnostics, and RViz markers, while a dependency-free local web studio edits and exports the same JSON format.

**Tech Stack:** Python 3, ROS 2 Lyrical, `rclpy`, custom ROS messages, HTML5 Canvas, vanilla JavaScript, `unittest`

**Spec:** `docs/superpowers/specs/2026-09-26-ros2-strategic-autonomy-design.md` sections 3/Faz 3, 7/Takım planı, 8, 9/Takım niyet füzyonu, 13, 14, 15, and 16

## Global Constraints

- Canonical schema version is `1`; field version is exactly `2018-power-up-v1`.
- Canonical field coordinates use the blue-alliance origin and the 2018 field bounds `16.46 m × 8.23 m`.
- Imported plans never command the robot and never publish `/autonomy/command`.
- Unknown schema/field versions, non-finite values, out-of-bounds points, non-monotonic time, speed/acceleration violations, overlaps, missing fallback targets, and fallback cycles reject the whole plan.
- Plan SHA-256 is computed over UTF-8 canonical JSON with sorted keys and compact separators, excluding the `content_sha256` field itself.
- Current PathPlanner `2025.0` `.path` and `.auto` structures are accepted through an explicit importer; unsupported versions fail with a precise diagnostic.
- Reservation confidence can only stay equal or decrease when observations disagree or disappear; it never increases above the plan's initial confidence without matching observations.
- This phase consumes `/world/state` but does not alter the perception tracker.

---

### Task 1: Alliance Plan and Reservation ROS Interfaces

**Files:**
- Create: `jetson/ros2_ws/src/frc_autonomy_msgs/msg/TimedPose.msg`
- Create: `jetson/ros2_ws/src/frc_autonomy_msgs/msg/PlanSegment.msg`
- Create: `jetson/ros2_ws/src/frc_autonomy_msgs/msg/RobotPlan.msg`
- Create: `jetson/ros2_ws/src/frc_autonomy_msgs/msg/AlliancePlan.msg`
- Create: `jetson/ros2_ws/src/frc_autonomy_msgs/msg/ReservationSample.msg`
- Create: `jetson/ros2_ws/src/frc_autonomy_msgs/msg/ReservationTube.msg`
- Create: `jetson/ros2_ws/src/frc_autonomy_msgs/msg/ReservationTubeArray.msg`
- Modify: `jetson/ros2_ws/src/frc_autonomy_msgs/CMakeLists.txt`

**Interfaces:**
- Produces: `/alliance/plan` (`AlliancePlan`) and `/alliance/reservations` (`ReservationTubeArray`).

- [ ] **Step 1: Define timed path and segment messages**

`TimedPose` contains `int64 time_us` and `geometry_msgs/Pose2D pose`. `PlanSegment` contains identifiers, task constants (`PICKUP` through `WAIT`), earliest/latest/duration microseconds, corridor radius, fallback ID, and a `TimedPose[]`.

- [ ] **Step 2: Define robot, alliance, and reservation messages**

`RobotPlan` contains team number, label, start pose, footprint, speed/acceleration limits, initial confidence, and segments. `AlliancePlan` contains header, schema/field/alliance/plan/hash metadata and robots. Reservation messages contain sampled match times, poses, radii, team identity, and current confidence.

- [ ] **Step 3: Register all message files and statically validate the manifest**

- [ ] **Step 4: Commit**

```bash
git add jetson/ros2_ws/src/frc_autonomy_msgs
git commit -m "feat: define alliance playbook messages"
```

### Task 2: Canonical Schema, Validation, and Hashing

**Files:**
- Create: `jetson/ros2_ws/src/frc_alliance_playbook/frc_alliance_playbook/schema.py`
- Create: `jetson/ros2_ws/src/frc_alliance_playbook/test/test_schema.py`

**Interfaces:**
- Produces: immutable `TimedPose`, `PlanSegment`, `RobotPlan`, and `AlliancePlan`; `parse_plan(dict) -> AlliancePlan`; `canonical_payload(plan) -> dict`; `content_sha256(plan) -> str`.

- [ ] **Step 1: Write failing round-trip and hash tests**

```python
def test_canonical_round_trip_and_hash_are_order_independent(self):
    first = parse_plan(valid_plan_dict())
    reordered = parse_plan(json.loads(json.dumps(valid_plan_dict(), sort_keys=True)))
    self.assertEqual(first, reordered)
    self.assertEqual(content_sha256(first), content_sha256(reordered))
    self.assertEqual(64, len(content_sha256(first)))
```

- [ ] **Step 2: Verify RED, then implement immutable types, strict key/type parsing, canonical serialization, and SHA-256**

- [ ] **Step 3: Write failing table-driven validation tests**

Mutations cover unknown schema/field/alliance/task, duplicate robot or segment IDs, invalid footprints/limits/confidence, field overflow, non-increasing path time, speed/acceleration violation, segment overlap, missing fallback, and a two-node fallback cycle. Each mutation must raise `PlanValidationError` without returning a partial plan.

- [ ] **Step 4: Verify RED, implement validation, run GREEN, and commit**

```bash
git add jetson/ros2_ws/src/frc_alliance_playbook/frc_alliance_playbook/schema.py jetson/ros2_ws/src/frc_alliance_playbook/test/test_schema.py
git commit -m "feat: validate canonical alliance plans"
```

### Task 3: CSV and PathPlanner Importers

**Files:**
- Create: `jetson/ros2_ws/src/frc_alliance_playbook/frc_alliance_playbook/importers.py`
- Create: `jetson/ros2_ws/src/frc_alliance_playbook/test/test_importers.py`
- Create: `jetson/ros2_ws/src/frc_alliance_playbook/test/fixtures/Pickup.path`
- Create: `jetson/ros2_ws/src/frc_alliance_playbook/test/fixtures/Example.auto`
- Create: `jetson/ros2_ws/src/frc_alliance_playbook/test/fixtures/Score.path`

**Interfaces:**
- Produces: `import_csv(text, metadata)`, `import_pathplanner_path(data, metadata)`, and `import_pathplanner_auto(data, path_loader, metadata)` returning validated `AlliancePlan`.

- [ ] **Step 1: Write failing CSV tests**

Use literal `time,x,y,heading,vx,vy,omega,event` fixtures. Verify seconds-to-microseconds conversion, degrees-to-radians heading conversion, task event mapping, monotonic rejection, and declared speed agreement.

- [ ] **Step 2: Verify RED, implement CSV normalization, and run GREEN**

- [ ] **Step 3: Write failing PathPlanner `.path` tests**

Use the official `2025.0` anchor/prevControl/nextControl/globalConstraints layout. Verify cubic Bézier sampling includes both endpoints, timing respects maximum velocity, and unsupported versions or missing anchors fail.

- [ ] **Step 4: Verify RED, implement deterministic 20-samples-per-span Bézier normalization, and run GREEN**

- [ ] **Step 5: Write failing `.auto` tests**

Use the official `command: {type: sequential, data: {commands: [...]}}` structure with `pathName` references and waits. Verify referenced paths are concatenated in order, waits shift later timestamps, missing paths fail, and unsupported parallel/race/deadline drive structures fail closed.

- [ ] **Step 6: Implement auto command-tree import, run all importer tests, and commit**

```bash
git add jetson/ros2_ws/src/frc_alliance_playbook
git commit -m "feat: import teammate strategy files"
```

### Task 4: Reservation Tubes and Plan-Observation Fusion

**Files:**
- Create: `jetson/ros2_ws/src/frc_alliance_playbook/frc_alliance_playbook/reservations.py`
- Create: `jetson/ros2_ws/src/frc_alliance_playbook/test/test_reservations.py`

**Interfaces:**
- Produces: `build_reservations(plan, sample_period_us=100_000) -> tuple[ReservationTube, ...]`; `ConfidenceTracker.update(team_number, planned_pose, observed_pose, observed_at_us) -> float`; `ConfidenceTracker.missing(team_number, now_us) -> float`.

- [ ] **Step 1: Write failing sampling and collision tests**

Verify linear interpolation at 100 ms, footprint-plus-corridor radius, stable ordering, temporal overlap detection, and no conflict when spatial overlap occurs at different times.

- [ ] **Step 2: Verify RED, implement reservation sampling/conflict intervals, and run GREEN**

- [ ] **Step 3: Write failing confidence-fusion tests**

Verify on-corridor observations preserve but never increase confidence, 1 m deviation produces exponential decay, repeated missing intervals increase uncertainty, confidence clamps to `[0, initial_confidence]`, and equal inputs are deterministic.

- [ ] **Step 4: Implement confidence fusion, run GREEN, and commit**

```bash
git add jetson/ros2_ws/src/frc_alliance_playbook/frc_alliance_playbook/reservations.py jetson/ros2_ws/src/frc_alliance_playbook/test/test_reservations.py
git commit -m "feat: model teammate route reservations"
```

### Task 5: Package, CLI, ROS Publisher, and RViz Visualization

**Files:**
- Create: `jetson/ros2_ws/src/frc_alliance_playbook/package.xml`
- Create: `jetson/ros2_ws/src/frc_alliance_playbook/setup.py`
- Create: `jetson/ros2_ws/src/frc_alliance_playbook/setup.cfg`
- Create: `jetson/ros2_ws/src/frc_alliance_playbook/resource/frc_alliance_playbook`
- Create: `jetson/ros2_ws/src/frc_alliance_playbook/frc_alliance_playbook/cli.py`
- Create: `jetson/ros2_ws/src/frc_alliance_playbook/frc_alliance_playbook/playbook_node.py`
- Create: `jetson/ros2_ws/src/frc_alliance_playbook/frc_alliance_playbook/visualizer_node.py`
- Create: `jetson/ros2_ws/src/frc_alliance_playbook/config/playbook.yaml`
- Create: `config/alliance/example-plan.json`
- Modify: `jetson/ros2_ws/src/frc_bringup/launch/foundation.launch.py`
- Modify: `jetson/ros2_ws/src/frc_bringup/package.xml`
- Modify: `jetson/ros2_ws/src/frc_bringup/config/record_topics.txt`
- Modify: `jetson/ros2_ws/src/frc_robot_description/rviz/autonomy.rviz`

**Interfaces:**
- CLI: `playbook validate`, `playbook import-csv`, `playbook import-path`, `playbook import-auto`.
- ROS: publishes transient-local `/alliance/plan`, `/alliance/reservations`, and `/alliance/markers`; consumes `/world/state` for teammate confidence.

- [ ] **Step 1: Add package metadata, config, CLI commands, and example plan**

- [ ] **Step 2: Add the ROS publisher and confidence-fusion adapter**

Load and verify `plan_path` plus optional expected hash before publishing. Match ally tracks to the nearest planned teammate pose, update reservation confidence, and publish diagnostics without creating commands.

- [ ] **Step 3: Add RViz path, corridor, conflict, label, and deviation markers**

- [ ] **Step 4: Integrate launch, recording, and RViz configuration; statically validate and commit**

```bash
git add jetson/ros2_ws/src/frc_alliance_playbook jetson/ros2_ws/src/frc_bringup jetson/ros2_ws/src/frc_robot_description config/alliance
git commit -m "feat: publish alliance playbook reservations"
```

### Task 6: Dependency-Free Strategy Studio

**Files:**
- Create: `strategy_studio/index.html`
- Create: `strategy_studio/styles.css`
- Create: `strategy_studio/app.js`
- Create: `strategy_studio/README.md`

**Interfaces:**
- Imports canonical JSON and CSV in the browser.
- Exports schema-versioned canonical JSON, SHA-256 text, and a PNG preview.

- [ ] **Step 1: Build the 16.46 × 8.23 m Canvas field editor**

Support robot selection, click-to-add waypoints, drag-to-edit, segment task/time/corridor fields, delete/undo, blue/red canonical transform preview, and overlapping-corridor warnings.

- [ ] **Step 2: Add local file import and deterministic export**

Use browser File APIs only; do not upload plans. Canonicalize keys before Web Crypto SHA-256, download JSON, and export the canvas as PNG.

- [ ] **Step 3: Add keyboard/accessibility behavior and user-visible validation diagnostics**

- [ ] **Step 4: Run syntax/static checks and commit**

```bash
git add strategy_studio
git commit -m "feat: add local alliance strategy studio"
```

### Task 7: Documentation and Full Verification

**Files:**
- Create: `docs/alliance-playbook.md`
- Modify: `docs/autonomy-foundation.md`

- [ ] **Step 1: Document supported formats, canonical coordinates, validation failures, hashing, CLI, Studio, ROS, RViz, and rosbag flow**

- [ ] **Step 2: Run Java, all Python, XML/Xacro/YAML/message, JavaScript syntax, Compose, and diff checks**

- [ ] **Step 3: Confirm no playbook or Studio source references `/autonomy/command` and no live inter-robot networking exists**

- [ ] **Step 4: Self-review against the Phase 3 requirements and commit**

```bash
git add docs
git commit -m "docs: add alliance strategy workflow"
```
