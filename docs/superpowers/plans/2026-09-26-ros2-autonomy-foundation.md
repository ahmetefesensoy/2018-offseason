# ROS 2 Autonomy Foundation Implementation Plan

> **For Codex:** REQUIRED SUB-SKILL: Use executing-plans to implement this plan task-by-task. Apply test-driven-development for every behavior change and verification-before-completion before reporting success.

**Goal:** Build the first deployable slice of the approved ROS 2 autonomy architecture: a fail-closed Jetson-to-roboRIO velocity bridge, observable ROS 2 interfaces, and an RViz-ready robot model and simulation harness.

**Architecture:** The Jetson is an advisory planner and the roboRIO remains the only motor authority. ROS 2 publishes a complete command frame to NT4 primitive topics and publishes `commit_sequence` last; the roboRIO snapshots the frame only when that sequence changes, then independently validates mode, boot session, monotonic sequence, timestamps, finite/range-bounded velocity, gyro health, and watchdog age before commanding robot-relative swerve motion. Status flows back from the roboRIO to ROS 2 for RViz, rosbag, and operator diagnostics.

**Tech Stack:** Java 17, WPILib/NT4 2025.1.1, JUnit 5, ROS 2 Lyrical `rclpy`, RobotPy `ntcore`, URDF/Xacro, RViz2, `robot_state_publisher`, Python `unittest`.

**Scope boundary:** This plan implements Phase 1 only. It does not implement camera perception, opponent tracking, Nav2 planning, scoring strategy, or mechanism autonomy; those build on this safety foundation in later phase plans.

---

## NT4 protocol contract

All topics live below `/frc/autonomy/v1`.

Jetson to roboRIO command topics:

- `command/session_id` (`string`): current roboRIO boot-session identifier echoed from status.
- `command/armed` (`boolean`): explicit operator/launch arm state.
- `command/sent_at_us` (`int`): NT4 server-time timestamp in microseconds.
- `command/valid_until_us` (`int`): absolute NT4 server-time expiry in microseconds.
- `command/vx_mps`, `vy_mps`, `omega_radps` (`double`): robot-relative velocity request.
- `command/sequence` (`int`): sequence copied into the frame.
- `command/commit_sequence` (`int`): published last and equal to `sequence`; it is the snapshot barrier.

roboRIO to Jetson status topics:

- `status/session_id`, `mode`, `reject_reason` (`string`).
- `status/accepted_sequence`, `roborio_time_us`, `command_age_us` (`int`).
- `status/command_active`, `gyro_healthy` (`boolean`).
- `status/pose` and `status/measured_chassis` (`double[]`).

Frame limits for this first real-robot slice are intentionally conservative: 0.75 m/s translation, 1.5 rad/s rotation, 100 ms maximum age, 150 ms maximum validity horizon, and a 120 ms watchdog. These remain below the swerve subsystem's bring-up maxima.

### Task 1: Pure roboRIO protocol and safety gate

**Files:**

- Create: `src/main/java/frc/robot/AutonomyConstants.java`
- Create: `src/main/java/frc/robot/autonomy/AutonomyCommandFrame.java`
- Create: `src/main/java/frc/robot/autonomy/AutonomySafetyContext.java`
- Create: `src/main/java/frc/robot/autonomy/AutonomyRejectReason.java`
- Create: `src/main/java/frc/robot/autonomy/AutonomyDecision.java`
- Create: `src/main/java/frc/robot/autonomy/AutonomySafetyGate.java`
- Test: `src/test/java/frc/robot/autonomy/AutonomySafetyGateTest.java`

**Steps:**

1. Write failing tests for a valid command and every fail-closed condition: disabled/wrong mode, disarmed, boot-session mismatch, stale or future timestamp, expired/excessive validity horizon, non-increasing sequence, non-finite/out-of-range speed, unhealthy gyro, and watchdog expiration.
2. Run only `AutonomySafetyGateTest` and confirm behavioral failures, not setup failures.
3. Implement immutable command/context/decision records and the stateful safety gate with explicit reject reasons.
4. Re-run the focused test and commit when green.

### Task 2: NT4 adapter with atomic snapshot barrier

**Files:**

- Create: `src/main/java/frc/robot/autonomy/AutonomyLinkIO.java`
- Create: `src/main/java/frc/robot/autonomy/AutonomyStatus.java`
- Create: `src/main/java/frc/robot/autonomy/AutonomyLinkIONetworkTables.java`
- Test: `src/test/java/frc/robot/autonomy/AutonomyLinkIONetworkTablesTest.java`

**Steps:**

1. Write a failing local-NT-instance test proving incomplete/mismatched frames are ignored and a fully committed frame is returned once.
2. Write a failing status-publication test covering session, accepted sequence, rejection, pose, and chassis speed.
3. Implement type-specific NT4 subscribers/publishers with retained handles, 20 ms periodic options, duplicate sequence support, and `commit_sequence` read as the final snapshot barrier.
4. Re-run the focused NT test and commit when green.

### Task 3: Watchdog-controlled swerve command

**Files:**

- Create: `src/main/java/frc/robot/autonomy/AutonomyController.java`
- Create: `src/main/java/frc/robot/autonomy/AutonomyDriveCommand.java`
- Modify: `src/main/java/frc/robot/subsystems/SwerveDriveSubsystem.java`
- Test: `src/test/java/frc/robot/autonomy/AutonomyControllerTest.java`
- Test: `src/test/java/frc/robot/autonomy/AutonomyDriveCommandTest.java`

**Steps:**

1. Write failing tests for accepted robot-relative drive, rejection/stop, no-new-frame watchdog stop within 120 ms, end/cancel stop, and status feedback.
2. Add measured chassis speed and drivetrain health accessors to the swerve subsystem.
3. Implement a clock-injected controller that polls the bridge, applies the safety gate, owns the boot-session UUID, and publishes status every loop.
4. Implement the command that requires the drivetrain and never bypasses the controller decision.
5. Re-run focused tests and commit when green.

### Task 4: Robot lifecycle integration

**Files:**

- Modify: `src/main/java/frc/robot/RobotContainer.java`
- Modify: `src/main/java/frc/robot/Robot.java`
- Test: `src/test/java/frc/robot/RobotContainerAutonomyTest.java`

**Steps:**

1. Write a failing test that the container exposes an autonomous command requiring the drivetrain.
2. Construct the real NT4 adapter/controller in the production container while keeping an injectable constructor for tests.
3. Schedule the returned command in `autonomousInit`; cancel it and force a stop in `teleopInit`, `disabledInit`, and `testInit`.
4. Run all Java tests and commit when green.

### Task 5: ROS 2 messages and Jetson bridge

**Files:**

- Create: `jetson/ros2_ws/src/frc_autonomy_msgs/CMakeLists.txt`
- Create: `jetson/ros2_ws/src/frc_autonomy_msgs/package.xml`
- Create: `jetson/ros2_ws/src/frc_autonomy_msgs/msg/AutonomyCommand.msg`
- Create: `jetson/ros2_ws/src/frc_autonomy_msgs/msg/AutonomyStatus.msg`
- Create: `jetson/ros2_ws/src/frc_nt_bridge/package.xml`
- Create: `jetson/ros2_ws/src/frc_nt_bridge/setup.py`
- Create: `jetson/ros2_ws/src/frc_nt_bridge/setup.cfg`
- Create: `jetson/ros2_ws/src/frc_nt_bridge/resource/frc_nt_bridge`
- Create: `jetson/ros2_ws/src/frc_nt_bridge/frc_nt_bridge/__init__.py`
- Create: `jetson/ros2_ws/src/frc_nt_bridge/frc_nt_bridge/protocol.py`
- Create: `jetson/ros2_ws/src/frc_nt_bridge/frc_nt_bridge/bridge_node.py`
- Create: `jetson/ros2_ws/src/frc_nt_bridge/test/test_protocol.py`

**Steps:**

1. Write failing Python unit tests for sequence monotonicity, server-time stamping, validity limits, disarm zeroing, and publish-order generation.
2. Implement a ROS-independent protocol core, then make the `rclpy` node a thin adapter around it.
3. Make the node connect as NT4 client, echo the roboRIO boot session, publish command fields then commit last, translate status to ROS, and disarm on shutdown.
4. Run Python tests and validate ROS package XML/setup metadata.

### Task 6: Robot model, RViz, bringup, rosbag, and fake command source

**Files:**

- Create: `jetson/ros2_ws/src/frc_robot_description/CMakeLists.txt`
- Create: `jetson/ros2_ws/src/frc_robot_description/package.xml`
- Create: `jetson/ros2_ws/src/frc_robot_description/urdf/frc_2018_robot.urdf.xacro`
- Create: `jetson/ros2_ws/src/frc_robot_description/rviz/autonomy.rviz`
- Create: `jetson/ros2_ws/src/frc_bringup/package.xml`
- Create: `jetson/ros2_ws/src/frc_bringup/setup.py`
- Create: `jetson/ros2_ws/src/frc_bringup/setup.cfg`
- Create: `jetson/ros2_ws/src/frc_bringup/resource/frc_bringup`
- Create: `jetson/ros2_ws/src/frc_bringup/frc_bringup/__init__.py`
- Create: `jetson/ros2_ws/src/frc_bringup/frc_bringup/fake_autonomy_node.py`
- Create: `jetson/ros2_ws/src/frc_bringup/launch/foundation.launch.py`
- Create: `jetson/ros2_ws/src/frc_bringup/config/autonomy.yaml`
- Create: `jetson/ros2_ws/src/frc_bringup/config/record_topics.txt`
- Create: `jetson/ros2_ws/src/frc_bringup/rviz/autonomy.rviz`
- Create: `jetson/scripts/record_autonomy.sh`

**Steps:**

1. Build a parameterized, mesh-free Xacro model from the measured/commissioned square chassis footprint; make unverified geometry explicit as parameters rather than fabricated CAD detail.
2. Configure RViz for robot model, TF, odometry/path, command velocity, point cloud/obstacle topics, and diagnostic markers so later phases appear without redesigning the layout.
3. Add a bounded fake-autonomy node that requires an explicit arm parameter and produces a slow figure-eight command for simulation only.
4. Add one launch file for state publisher, bridge, fake source (optional), RViz (optional), and rosbag topic list.
5. Validate Python syntax, XML, YAML, and Xacro structure available on this workstation.

### Task 7: Operations guide and full verification

**Files:**

- Create: `docs/autonomy-foundation.md`
- Modify: `README.md` if present; otherwise keep the guide standalone.

**Steps:**

1. Document network addresses, launch/build commands, arming flow, expected status topics, RViz demo flow, rosbag capture, hardware-enable checklist, and fail-safe drills.
2. Clearly label geometry/CAN/team-number calibration items and real-hardware prerequisites.
3. Run the complete Gradle test/build suite and Python unit tests from clean processes.
4. Inspect `git diff --check`, worktree status, generated JAR, and test counts.
5. Self-review safety invariants against the master design, fix any gaps, then commit the complete Phase 1 slice.

## Primary references

- WPILib NetworkTables publish/subscribe: https://docs.wpilib.org/en/stable/docs/software/networktables/publish-and-subscribe.html
- WPILib NetworkTables overview and timestamps: https://docs.wpilib.org/en/latest/docs/software/networktables/networktables-intro.html
- WPILib client/server networking: https://docs.wpilib.org/en/stable/docs/software/networktables/networktables-networking.html
- WPILib multiple instances for tests: https://docs.wpilib.org/en/stable/docs/software/networktables/multiple-instances.html
- ROS 2 package and launch conventions: https://docs.ros.org/en/lyrical/
- NVIDIA Isaac ROS common/Jetson prerequisites: https://nvidia-isaac-ros.github.io/v/release-5.0/repositories_and_packages/isaac_ros_common/index.html
