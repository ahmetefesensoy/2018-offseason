# Perception World Model Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a deterministic, camera-independent ROS 2 perception and world-model slice that tracks simulated or adapter-provided 3D semantic observations and renders predicted dynamic occupancy in RViz without commanding the robot.

**Architecture:** Detector/VSLAM adapters publish one atomic `SemanticObservationArray` in the canonical `map` frame. A ROS-independent tracker performs deterministic gated nearest-neighbor association, constant-velocity filtering, covariance growth, expiry, and two-second prediction; a snapshot store partitions game objects and robots and increments one version per accepted observation batch. Thin ROS nodes translate messages, publish `WorldState`, provide a deterministic synthetic source, and render tracks/predictions as RViz markers.

**Tech Stack:** ROS 2 Lyrical, Python 3, `rclpy`, custom ROS interfaces, `geometry_msgs`, `visualization_msgs`, `unittest`, RViz 2

**Spec:** `docs/superpowers/specs/2026-09-26-ros2-strategic-autonomy-design.md` sections 6, 7, 9, 13, 14, 15, and 16

## Global Constraints

- Jetson never accesses motor CAN and this phase never publishes `/autonomy/command`.
- All observations consumed by the tracker are expressed in canonical `map` coordinates and use monotonic microsecond source timestamps.
- Invalid, non-finite, out-of-order, low-confidence, and over-age observations are rejected without mutating the current snapshot.
- Robot prediction horizon is exactly 2.0 seconds at 0.1-second steps.
- The same ordered observation batches and configuration produce byte-for-byte stable track identifiers and snapshot ordering.
- The package remains usable without Isaac ROS, a camera, CUDA, or a model engine; those components are upstream adapters.
- World-state publication continues with `perception_fresh=false` after source timeout so downstream strategy can fail closed.

---

### Task 1: Versioned Perception and World-State Messages

**Files:**
- Create: `jetson/ros2_ws/src/frc_autonomy_msgs/msg/SemanticObservation.msg`
- Create: `jetson/ros2_ws/src/frc_autonomy_msgs/msg/SemanticObservationArray.msg`
- Create: `jetson/ros2_ws/src/frc_autonomy_msgs/msg/PredictedPose.msg`
- Create: `jetson/ros2_ws/src/frc_autonomy_msgs/msg/GameObject.msg`
- Create: `jetson/ros2_ws/src/frc_autonomy_msgs/msg/RobotTrack.msg`
- Create: `jetson/ros2_ws/src/frc_autonomy_msgs/msg/WorldState.msg`
- Modify: `jetson/ros2_ws/src/frc_autonomy_msgs/CMakeLists.txt`
- Modify: `jetson/ros2_ws/src/frc_autonomy_msgs/package.xml`

**Interfaces:**
- Consumes: `std_msgs/Header`, `builtin_interfaces/Duration`, and geometry message types.
- Produces: `/perception/observations` payloads and `/world/state` snapshots used by every later task.

- [ ] **Step 1: Define the atomic observation contract**

```text
# SemanticObservation.msg
string detection_id
string class_name
uint8 AFFILIATION_UNKNOWN=0
uint8 AFFILIATION_ALLY=1
uint8 AFFILIATION_OPPONENT=2
uint8 affiliation
geometry_msgs/PoseWithCovariance pose
geometry_msgs/Vector3 size
float64 confidence

# SemanticObservationArray.msg
std_msgs/Header header
SemanticObservation[] observations
```

- [ ] **Step 2: Define tracked-object and prediction contracts**

```text
# PredictedPose.msg
builtin_interfaces/Duration horizon
geometry_msgs/PoseWithCovariance pose

# GameObject.msg
string object_id
string class_name
geometry_msgs/PoseWithCovariance pose
geometry_msgs/TwistWithCovariance velocity
geometry_msgs/Vector3 size
float64 confidence
int64 last_seen_us

# RobotTrack.msg
string track_id
uint8 AFFILIATION_UNKNOWN=0
uint8 AFFILIATION_ALLY=1
uint8 AFFILIATION_OPPONENT=2
uint8 affiliation
geometry_msgs/PoseWithCovariance pose
geometry_msgs/TwistWithCovariance velocity
geometry_msgs/Vector3 size
PredictedPose[] predictions
float64 confidence
int64 last_seen_us
```

- [ ] **Step 3: Define the atomic world snapshot**

```text
# WorldState.msg
std_msgs/Header header
uint64 version
bool perception_fresh
int64 newest_source_age_us
GameObject[] game_objects
RobotTrack[] robots
```

- [ ] **Step 4: Register all interfaces and dependencies**

Add all six files to `rosidl_generate_interfaces`, add `builtin_interfaces` to the CMake and package dependencies, then parse every XML/message file in static verification.

- [ ] **Step 5: Commit**

```bash
git add jetson/ros2_ws/src/frc_autonomy_msgs
git commit -m "feat: define perception world model messages"
```

### Task 2: Deterministic Multi-Object Tracker

**Files:**
- Create: `jetson/ros2_ws/src/frc_world_model/frc_world_model/model.py`
- Create: `jetson/ros2_ws/src/frc_world_model/test/test_tracker.py`

**Interfaces:**
- Consumes: `Observation(detection_id, class_name, affiliation, x, y, yaw, size_x, size_y, confidence, variance_x, variance_y, timestamp_us)`.
- Produces: `MultiObjectTracker.update(observations, now_us) -> tuple[Track, ...]` and `MultiObjectTracker.snapshot(now_us) -> tuple[Track, ...]`.

- [ ] **Step 1: Write failing validation and deterministic-creation tests**

```python
class MultiObjectTrackerTest(unittest.TestCase):
    def test_rejects_invalid_batch_without_mutating_tracks(self):
        tracker = MultiObjectTracker(TrackerConfig())
        tracker.update((observation("cube-a", "power_cube", 1.0, 2.0, 1_000_000),), 1_000_000)
        before = tracker.snapshot(1_000_000)
        with self.assertRaises(ObservationError):
            tracker.update((observation("bad", "robot", math.nan, 0.0, 1_020_000),), 1_020_000)
        self.assertEqual(before, tracker.snapshot(1_000_000))

    def test_creation_order_and_ids_do_not_depend_on_input_order(self):
        left = observation("z", "robot", 2.0, 0.0, 1_000_000)
        right = observation("a", "robot", 4.0, 0.0, 1_000_000)
        tracker = MultiObjectTracker(TrackerConfig())
        tracks = tracker.update((left, right), 1_000_000)
        self.assertEqual(("trk-000001", "trk-000002"), tuple(track.track_id for track in tracks))
        self.assertEqual((4.0, 2.0), tuple(track.x for track in tracks))
```

- [ ] **Step 2: Run the tracker tests and verify RED**

Run: `$env:PYTHONPATH=(Resolve-Path 'jetson/ros2_ws/src/frc_world_model').Path; python -m unittest jetson/ros2_ws/src/frc_world_model/test/test_tracker.py -v`

Expected: import failure because `frc_world_model.model` does not exist.

- [ ] **Step 3: Implement immutable types, validation, and deterministic creation**

Use frozen dataclasses for `Observation`, `TrackerConfig`, `Prediction`, and `Track`. Validate the complete batch before mutation, sort new observations by `(class_name, detection_id, x, y)`, and allocate zero-padded process-local IDs.

- [ ] **Step 4: Run the tests and verify GREEN**

Run the command from Step 2. Expected: both tests pass.

- [ ] **Step 5: Write failing association, velocity, expiry, and prediction tests**

```python
def test_associates_nearest_compatible_track_and_estimates_velocity(self):
    tracker = MultiObjectTracker(TrackerConfig(position_gain=1.0, velocity_gain=1.0))
    first = tracker.update((observation("r1", "robot", 1.0, 1.0, 1_000_000),), 1_000_000)[0]
    second = tracker.update((observation("r2", "robot", 1.2, 1.0, 1_200_000),), 1_200_000)[0]
    self.assertEqual(first.track_id, second.track_id)
    self.assertAlmostEqual(1.0, second.vx)
    self.assertAlmostEqual(0.0, second.vy)

def test_expires_stale_tracks(self):
    tracker = MultiObjectTracker(TrackerConfig(track_timeout_us=500_000))
    tracker.update((observation("cube", "power_cube", 0.0, 0.0, 1_000_000),), 1_000_000)
    self.assertEqual((), tracker.snapshot(1_500_001))

def test_robot_prediction_has_twenty_steps_and_growing_covariance(self):
    tracker = MultiObjectTracker(TrackerConfig(position_gain=1.0, velocity_gain=1.0))
    tracker.update((observation("r", "robot", 0.0, 0.0, 1_000_000),), 1_000_000)
    track = tracker.update((observation("r", "robot", 0.1, 0.0, 1_100_000),), 1_100_000)[0]
    self.assertEqual(20, len(track.predictions))
    self.assertAlmostEqual(2.1, track.predictions[-1].x)
    self.assertGreater(track.predictions[-1].variance_x, track.variance_x)
```

- [ ] **Step 6: Run the new tests and verify RED**

Expected failures: a second track is created, velocity remains zero, expiry does not occur, and predictions are empty.

- [ ] **Step 7: Implement gated association and constant-velocity prediction**

Associate only equal semantic classes; for robots also require compatible affiliation unless either side is unknown. Use Euclidean distance with a configurable gate, deterministic `(distance, track_id)` tie-breaking, alpha-beta state updates, and covariance growth `variance + process_noise * horizon^2` for horizons 0.1 through 2.0 seconds.

- [ ] **Step 8: Run the complete tracker tests and commit**

```bash
git add jetson/ros2_ws/src/frc_world_model/frc_world_model/model.py jetson/ros2_ws/src/frc_world_model/test/test_tracker.py
git commit -m "feat: add deterministic dynamic object tracker"
```

### Task 3: Atomic World Snapshot Store

**Files:**
- Create: `jetson/ros2_ws/src/frc_world_model/frc_world_model/world_store.py`
- Create: `jetson/ros2_ws/src/frc_world_model/test/test_world_store.py`

**Interfaces:**
- Consumes: validated batches through `WorldStore.ingest(observations, source_timestamp_us, received_timestamp_us)`.
- Produces: immutable `WorldSnapshot(version, source_timestamp_us, perception_fresh, newest_source_age_us, game_objects, robots)` through `WorldStore.snapshot(now_us)`.

- [ ] **Step 1: Write failing atomicity, partition, version, and freshness tests**

```python
def test_ingest_commits_one_partitioned_version(self):
    store = WorldStore(MultiObjectTracker(TrackerConfig()), source_timeout_us=250_000)
    snapshot = store.ingest(
        (observation("c", "power_cube", 1.0, 0.0, 1_000_000),
         observation("r", "robot", 2.0, 0.0, 1_000_000)),
        source_timestamp_us=1_000_000,
        received_timestamp_us=1_020_000,
    )
    self.assertEqual(1, snapshot.version)
    self.assertEqual(1, len(snapshot.game_objects))
    self.assertEqual(1, len(snapshot.robots))

def test_stale_source_keeps_tracks_but_marks_snapshot_unfresh(self):
    store = WorldStore(MultiObjectTracker(TrackerConfig(track_timeout_us=1_000_000)), 250_000)
    store.ingest((observation("r", "robot", 0.0, 0.0, 1_000_000),), 1_000_000, 1_010_000)
    snapshot = store.snapshot(1_300_001)
    self.assertFalse(snapshot.perception_fresh)
    self.assertEqual(300_001, snapshot.newest_source_age_us)
    self.assertEqual(1, len(snapshot.robots))
```

- [ ] **Step 2: Run the store tests and verify RED**

Expected: import failure because `frc_world_model.world_store` does not exist.

- [ ] **Step 3: Implement the store and snapshot ordering**

Reject source timestamps newer than receive time, older than the prior committed batch, or already over the source timeout. Increment `version` once only after the tracker accepts the entire batch. Sort `game_objects` and `robots` by stable ID.

- [ ] **Step 4: Run all pure world-model tests and commit**

```bash
git add jetson/ros2_ws/src/frc_world_model
git commit -m "feat: add atomic perception world snapshots"
```

### Task 4: ROS Package, Nodes, Synthetic Source, and RViz Markers

**Files:**
- Create: `jetson/ros2_ws/src/frc_world_model/package.xml`
- Create: `jetson/ros2_ws/src/frc_world_model/setup.py`
- Create: `jetson/ros2_ws/src/frc_world_model/setup.cfg`
- Create: `jetson/ros2_ws/src/frc_world_model/resource/frc_world_model`
- Create: `jetson/ros2_ws/src/frc_world_model/frc_world_model/__init__.py`
- Create: `jetson/ros2_ws/src/frc_world_model/frc_world_model/world_model_node.py`
- Create: `jetson/ros2_ws/src/frc_world_model/frc_world_model/synthetic_perception_node.py`
- Create: `jetson/ros2_ws/src/frc_world_model/frc_world_model/world_visualizer_node.py`
- Create: `jetson/ros2_ws/src/frc_world_model/config/world_model.yaml`
- Modify: `jetson/ros2_ws/src/frc_bringup/launch/foundation.launch.py`
- Modify: `jetson/ros2_ws/src/frc_bringup/package.xml`
- Modify: `jetson/ros2_ws/src/frc_robot_description/rviz/autonomy.rviz`
- Modify: `jetson/ros2_ws/src/frc_bringup/config/record_topics.txt`

**Interfaces:**
- Consumes: `/perception/observations` (`SemanticObservationArray`).
- Produces: `/world/state` (`WorldState`) and `/world/markers` (`MarkerArray`).

- [ ] **Step 1: Add package metadata and entry points**

Expose executables `world_model`, `synthetic_perception`, and `world_visualizer`; install `config/world_model.yaml`; declare runtime dependencies on `frc_autonomy_msgs`, `geometry_msgs`, `rclpy`, `std_msgs`, and `visualization_msgs`.

- [ ] **Step 2: Implement the thin world-model ROS adapter**

Convert each observation timestamp from `header.stamp`, reject any non-`map` frame, feed a complete tuple to `WorldStore.ingest`, publish at 10 Hz even when stale, and log invalid batches with throttling. Conversion functions remain pure and do not contain tracking logic.

- [ ] **Step 3: Add a deterministic synthetic perception source**

Publish three fixed cubes, one ally on a delayed straight path, and one opponent crossing the robot corridor. Use node time, deterministic analytic positions, fixed covariance, and `armed`-independent operation; the source publishes observations only and cannot publish velocity commands.

- [ ] **Step 4: Render observable tracker behavior**

Publish cube boxes, robot footprint cubes, velocity arrows, text labels, covariance cylinders, and prediction tubes. Use namespaces `game_objects`, `robot_tracks`, `velocities`, `covariance`, and `predictions`; color ally blue, opponent red, unknown yellow, and stale snapshots gray.

- [ ] **Step 5: Integrate launch, RViz, and recording**

Add launch argument `use_synthetic_perception:=false`, always start `world_model` and `world_visualizer`, conditionally start the synthetic source, add `/world/markers` to RViz, and record `/perception/observations`, `/world/state`, and `/world/markers`.

- [ ] **Step 6: Run static ROS package validation and commit**

```bash
python -m compileall -q jetson/ros2_ws/src/frc_world_model
docker compose -f jetson/compose.yaml config --quiet
git add jetson/ros2_ws/src/frc_world_model jetson/ros2_ws/src/frc_bringup jetson/ros2_ws/src/frc_robot_description
git commit -m "feat: visualize tracked perception world state"
```

### Task 5: Documentation and Full Verification

**Files:**
- Create: `docs/perception-world-model.md`
- Modify: `docs/autonomy-foundation.md`
- Modify: `jetson/Dockerfile`

**Interfaces:**
- Consumes: the completed Phase 2 interfaces and launch arguments.
- Produces: reproducible simulation instructions and explicit hardware adapter requirements.

- [ ] **Step 1: Document the hardware-independent demo**

Document `use_synthetic_perception:=true`, expected RViz objects, topic inspection commands, freshness failure demonstration, frame requirements, and the fact that this phase cannot command the drivetrain.

- [ ] **Step 2: Document real detector and localization adapter contracts**

Require camera calibration, measured `base_link -> camera_link`, output transformed into `map`, confidence/covariance population, 30 Hz target, model metadata/hash verification, and lifecycle refusal on mismatch. Give the official Isaac ROS YOLOv8 ONNX-to-TensorRT and nvblox depth/pose integration points without bundling an unvalidated model.

- [ ] **Step 3: Run all verification commands**

```powershell
$env:JAVA_HOME='C:\Program Files\Eclipse Adoptium\jdk-17.0.20.101-hotspot'
.\gradlew.bat clean test build --console=plain
$env:PYTHONPATH=(Resolve-Path 'jetson/ros2_ws/src/frc_nt_bridge').Path
python -m unittest discover -s jetson/ros2_ws/src/frc_nt_bridge/test -p 'test_*.py' -v
$env:PYTHONPATH=(Resolve-Path 'jetson/ros2_ws/src/frc_bringup').Path
python -m unittest discover -s jetson/ros2_ws/src/frc_bringup/test -p 'test_*.py' -v
$env:PYTHONPATH=(Resolve-Path 'jetson/ros2_ws/src/frc_world_model').Path
python -m unittest discover -s jetson/ros2_ws/src/frc_world_model/test -p 'test_*.py' -v
docker compose -f jetson/compose.yaml config --quiet
git diff --check
```

- [ ] **Step 4: Review against Phase 2 scope**

Confirm that no node publishes `/autonomy/command`, invalid batches do not increment version, stale perception is explicit, track ordering is deterministic, robot prediction is 20 steps, recording contains raw observations plus world snapshots, and camera/model-specific code is isolated upstream.

- [ ] **Step 5: Commit documentation**

```bash
git add docs jetson/Dockerfile
git commit -m "docs: add perception world model demo guide"
```
