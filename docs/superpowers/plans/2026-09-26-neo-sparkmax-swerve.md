# NEO/SparkMax Swerve Drive Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the temporary tank drive with a safe, testable four-module swerve drive using NEO/SparkMax motors, REV Through Bore absolute encoders, and a NavX gyro, without adding autonomous behavior.

**Architecture:** Keep hardware access behind narrow module and gyro interfaces so kinematics, control, and failure behavior can be tested on a desktop. A `SwerveDriveSubsystem` owns four modules in the fixed FL/FR/BL/BR order, converts chassis speeds with WPILib, updates a pose estimator from wheel positions plus NavX heading, and publishes a `Field2d` for diagnostics. `RobotContainer` supplies a shaped joystick command as the default drive command; autonomous remains empty.

**Tech Stack:** Java 17, WPILib/GradleRIO 2025.1.1, REVLib 2025.0.3, Studica NavX 2025.0.0, JUnit 5, WPILib HAL simulation.

**Spec:** `docs/superpowers/specs/2026-09-26-swerve-drive-design.md`

## Global Constraints

- The drivetrain is four NEO drive motors plus four NEO steering motors, all controlled by SparkMax controllers.
- Each steering SparkMax reads a REV Through Bore Encoder in absolute-duty-cycle mode through a REV Absolute Encoder Adapter.
- NavX is connected over the roboRIO MXP SPI interface and is the only heading source in this phase.
- CAN IDs are FL 10/11, FR 12/13, BL 14/15, BR 16/17, where each pair is drive/steer.
- Module ordering is always front-left, front-right, back-left, back-right in constructors, arrays, kinematics, telemetry, and tests.
- Coordinate convention follows WPILib: +x forward, +y left, positive rotation counter-clockwise, meters and radians internally.
- No Jetson, ROS, camera, object detection, PathPlanner, trajectory following, or autonomous routine is added in this plan.
- The initial hardware profile uses a 0.1016 m wheel, 6.75:1 drive reduction, a 0.60 m wheelbase and 0.60 m track width. All four absolute offsets start at 0 rad and the code exposes raw angles for physical commissioning before full-speed operation.
- Bring-up mode caps translation at 1.0 m/s and rotation at 2.0 rad/s until measured dimensions, drive reduction, inversion flags, and four absolute offsets are copied into `SwerveConstants`.
- Every production behavior is introduced only after its focused test has failed for the expected missing-behavior reason.
- Gradle is run with `JAVA_HOME=C:\Program Files\Eclipse Adoptium\jdk-17.0.20.101-hotspot` because the system-default Java is version 8.

---

### Task 1: Dependency and Hardware Configuration Contract

**Files:**
- Create: `vendordeps/Studica.json`
- Create: `src/main/java/frc/robot/SwerveConstants.java`
- Test: `src/test/java/frc/robot/SwerveConstantsTest.java`

**Interfaces:**
- Consumes: WPILib `Translation2d`, Java records.
- Produces: `SwerveConstants.ModuleConfig`, `MODULE_CONFIGS`, module translations, encoder conversion factors, current limits, bring-up speed limits, and controller gains.

- [ ] **Step 1: Install the official Studica 2025 vendordep**

Copy the WPILib-maintained Studica 2025 vendordep from
`https://raw.githubusercontent.com/wpilibsuite/vendor-json-repo/main/2025/Studica-2025.0.0.json`
to `vendordeps/Studica.json`. Confirm its `frcYear` is `2025`, its Java
dependency is `com.studica.frc:Studica-java:2025.0.0`, and the project contains
no legacy `StudicaLib` vendordep. The vendor's former `/2025/json/` URL returns
404, so it must not be used.

- [ ] **Step 2: Write failing conversion and CAN uniqueness tests**

Create `SwerveConstantsTest` with assertions equivalent to:

```java
@Test
void driveEncoderRotationsConvertToWheelMeters() {
    assertEquals(
        Math.PI * 0.1016 / 6.75,
        SwerveConstants.DRIVE_POSITION_FACTOR_METERS,
        1e-12);
}

@Test
void driveEncoderRpmConvertsToMetersPerSecond() {
    assertEquals(
        SwerveConstants.DRIVE_POSITION_FACTOR_METERS / 60.0,
        SwerveConstants.DRIVE_VELOCITY_FACTOR_MPS,
        1e-12);
}

@Test
void moduleCanIdsAreUniqueAndDoNotCollideWithMechanisms() {
    Set<Integer> ids = new HashSet<>();
    for (var module : SwerveConstants.MODULE_CONFIGS) {
        assertTrue(ids.add(module.driveCanId()));
        assertTrue(ids.add(module.turnCanId()));
    }
    assertTrue(Collections.disjoint(ids, Set.of(1, 2, 3, 9)));
}
```

- [ ] **Step 3: Run the focused test and verify RED**

Run:

```powershell
$env:JAVA_HOME='C:\Program Files\Eclipse Adoptium\jdk-17.0.20.101-hotspot'
.\gradlew.bat test --tests frc.robot.SwerveConstantsTest --console=plain
```

Expected: compilation fails because `SwerveConstants` does not exist.

- [ ] **Step 4: Implement the configuration contract**

Create a non-instantiable `SwerveConstants` containing:

```java
public record ModuleConfig(
    String name,
    int driveCanId,
    int turnCanId,
    boolean driveInverted,
    boolean turnInverted,
    double absoluteOffsetRadians) {}

public static final double WHEEL_DIAMETER_METERS = 0.1016;
public static final double DRIVE_REDUCTION = 6.75;
public static final double WHEEL_BASE_METERS = 0.60;
public static final double TRACK_WIDTH_METERS = 0.60;
public static final double DRIVE_POSITION_FACTOR_METERS =
    Math.PI * WHEEL_DIAMETER_METERS / DRIVE_REDUCTION;
public static final double DRIVE_VELOCITY_FACTOR_MPS =
    DRIVE_POSITION_FACTOR_METERS / 60.0;
public static final double TURN_POSITION_FACTOR_RADIANS = 2.0 * Math.PI;
public static final double TURN_VELOCITY_FACTOR_RAD_PER_SEC = 2.0 * Math.PI / 60.0;
public static final double MAX_SPEED_MPS = 1.0;
public static final double MAX_ANGULAR_SPEED_RAD_PER_SEC = 2.0;
```

Define FL/FR/BL/BR configs with CAN IDs 10/11 through 16/17. Set all four
absolute offsets to `0.0`, all four `driveInverted` flags to `false`, and all
four `turnInverted` flags to `false`; the commissioning procedure changes only
the measured values. Define four `Translation2d` locations using half wheelbase
and half track width. Use drive `kP=0.5`, `kI=0`, `kD=0`, `kS=0.20 V`,
`kV=2.4 V/(m/s)`, `kA=0`; turn `kP=4.0`, `kI=0`, `kD=0`, maximum steering
velocity `8 rad/s`, maximum steering acceleration `20 rad/s²`; 50 A drive and
30 A turn current limits; 12 V voltage compensation; and a 0.02 s loop period.

- [ ] **Step 5: Run the focused test and verify GREEN**

Run the Task 1 focused command and confirm all `SwerveConstantsTest` cases pass.

- [ ] **Step 6: Commit Task 1**

```powershell
git add vendordeps/Studica.json src/main/java/frc/robot/SwerveConstants.java src/test/java/frc/robot/SwerveConstantsTest.java
git commit -m "build: add swerve hardware configuration"
```

### Task 2: Testable Swerve Module Control

**Files:**
- Create: `src/main/java/frc/robot/subsystems/swerve/SwerveModuleIO.java`
- Create: `src/main/java/frc/robot/subsystems/swerve/SwerveModuleIOSparkMax.java`
- Create: `src/main/java/frc/robot/subsystems/swerve/SwerveModule.java`
- Test: `src/test/java/frc/robot/subsystems/swerve/SwerveModuleTest.java`

**Interfaces:**
- Consumes: `SwerveConstants.ModuleConfig`, `SwerveModuleState`, `SwerveModulePosition`, `Rotation2d`.
- Produces: `SwerveModule.updateInputs()`, `setDesiredState(SwerveModuleState)`, `getState()`, `getPosition()`, `getRawAbsoluteAngle()`, `stop()`, and `close()`.

- [ ] **Step 1: Write the failing module behavior tests**

Use a small in-test fake implementing this desired interface:

```java
interface SwerveModuleIO extends AutoCloseable {
    record Inputs(double drivePositionMeters,
                  double driveVelocityMetersPerSecond,
                  Rotation2d turnAngle,
                  double rawAbsolutePositionRotations) {}
    Inputs readInputs();
    void setDriveVoltage(double volts);
    void setTurnVoltage(double volts);
    void stop();
    @Override default void close() {}
}
```

Test real `SwerveModule` behavior:

```java
@Test
void optimizationReversesWheelInsteadOfTurningMoreThanNinetyDegrees() {
    io.inputs = new Inputs(0.0, 0.0, Rotation2d.kZero, 0.0);
    module.updateInputs();
    SwerveModuleState optimized = module.calculateOptimizedState(
        new SwerveModuleState(1.0, Rotation2d.fromDegrees(170.0)));
    assertEquals(-1.0, optimized.speedMetersPerSecond, 1e-9);
    assertEquals(-10.0, optimized.angle.getDegrees(), 1e-9);
}

@Test
void cosineCompensationSuppressesDriveWhileWheelIsSideways() {
    io.inputs = new Inputs(0.0, 0.0, Rotation2d.kZero, 0.0);
    module.updateInputs();
    SwerveModuleState state = module.calculateOptimizedState(
        new SwerveModuleState(1.0, Rotation2d.fromDegrees(90.0)));
    assertEquals(0.0, state.speedMetersPerSecond, 1e-9);
}

@Test
void stopCommandsBothOutputsToZero() {
    module.stop();
    assertEquals(0.0, io.driveVolts, 1e-9);
    assertEquals(0.0, io.turnVolts, 1e-9);
}
```

Also test that `getPosition()` uses measured drive distance and measured absolute angle, and that both voltage outputs are clamped to ±12 V.

- [ ] **Step 2: Run the focused module test and verify RED**

Run `.\gradlew.bat test --tests frc.robot.subsystems.swerve.SwerveModuleTest --console=plain` with the Task 1 Java 17 environment. Expected: compilation fails because module classes do not exist.

- [ ] **Step 3: Implement `SwerveModuleIO` and the pure controller**

Implement the exact interface shown in Step 1. `SwerveModule` caches one input snapshot per `updateInputs()`, uses `SwerveModuleState.optimize`, multiplies optimized speed by the cosine of steering error, uses a continuous-input `ProfiledPIDController` over `[-π, π]` for steering, and uses `PIDController + SimpleMotorFeedforward` for drive. Clamp each output with `MathUtil.clamp(volts, -12.0, 12.0)`. If requested speed is below 1% of `MAX_SPEED_MPS`, command zero drive voltage and retain the last steering target to prevent module jitter.

- [ ] **Step 4: Implement SparkMax hardware IO**

`SwerveModuleIOSparkMax` owns two brushless `SparkMax` objects, the drive motor relative encoder, and the steering motor `SparkAbsoluteEncoder`. Apply declarative `SparkMaxConfig` settings:

- brake idle mode on both motors;
- configured inversion flags and smart current limits;
- 12 V voltage compensation;
- drive relative-encoder factors in meters and meters/second;
- absolute-encoder factors in radians and radians/second;
- absolute offset subtraction followed by `MathUtil.angleModulus` in `readInputs()`;
- persistent safe-parameter configuration;
- `setVoltage` for outputs and `close()` for both controllers.

The raw absolute rotations value must remain un-offset and be published separately for commissioning.

- [ ] **Step 5: Run the module tests and verify GREEN**

Run the Task 2 focused test, then `.\gradlew.bat test --console=plain`. Confirm module tests and all pre-existing mechanism tests pass.

- [ ] **Step 6: Commit Task 2**

```powershell
git add src/main/java/frc/robot/subsystems/swerve src/test/java/frc/robot/subsystems/swerve
git commit -m "feat: add SparkMax swerve module control"
```

### Task 3: NavX, Four-Module Kinematics, and Pose Estimation

**Files:**
- Create: `src/main/java/frc/robot/subsystems/swerve/GyroIO.java`
- Create: `src/main/java/frc/robot/subsystems/swerve/GyroIONavX.java`
- Create: `src/main/java/frc/robot/subsystems/SwerveDriveSubsystem.java`
- Test: `src/test/java/frc/robot/subsystems/SwerveDriveSubsystemTest.java`

**Interfaces:**
- Consumes: four `SwerveModule` instances in FL/FR/BL/BR order, a `GyroIO`, `ChassisSpeeds`, `Pose2d`.
- Produces: `drive(double,double,double,boolean)`, `setModuleStates(SwerveModuleState[])`, `getPose()`, `resetPose(Pose2d)`, `zeroHeading()`, `getHeading()`, `stop()`, `close()`, and `createReal()`.

- [ ] **Step 1: Write failing drivetrain tests with fake gyro/module IO**

Define the gyro seam as:

```java
interface GyroIO extends AutoCloseable {
    Rotation2d getHeading();
    boolean isConnected();
    boolean isCalibrating();
    void zeroYaw();
    @Override default void close() {}
}
```

Test these behaviors:

```java
@Test
void forwardRobotRelativeCommandPointsAllModulesForward() {
    drive.drive(1.0, 0.0, 0.0, false);
    for (SwerveModuleState state : drive.getDesiredStatesForTest()) {
        assertEquals(1.0, state.speedMetersPerSecond, 1e-9);
        assertEquals(0.0, state.angle.getRadians(), 1e-9);
    }
}

@Test
void fieldRelativeCommandUsesGyroHeading() {
    gyro.heading = Rotation2d.fromDegrees(90.0);
    drive.drive(1.0, 0.0, 0.0, true);
    for (SwerveModuleState state : drive.getDesiredStatesForTest()) {
        assertEquals(-90.0, state.angle.getDegrees(), 1e-6);
    }
}

@Test
void disconnectedGyroFallsBackToRobotRelativeDrive() {
    gyro.connected = false;
    gyro.heading = Rotation2d.fromDegrees(90.0);
    drive.drive(1.0, 0.0, 0.0, true);
    for (SwerveModuleState state : drive.getDesiredStatesForTest()) {
        assertEquals(0.0, state.angle.getDegrees(), 1e-6);
    }
}
```

Also test speed desaturation, module-array length rejection, pose reset, gyro zero delegation, and `stop()` propagation.

- [ ] **Step 2: Run the focused drivetrain test and verify RED**

Run `.\gradlew.bat test --tests frc.robot.subsystems.SwerveDriveSubsystemTest --console=plain` with Java 17. Expected: compilation fails because `GyroIO` and `SwerveDriveSubsystem` do not exist.

- [ ] **Step 3: Implement NavX adapter**

`GyroIONavX` constructs `com.studica.frc.AHRS` with
`new AHRS(AHRS.NavXComType.kMXP_SPI, 100)`. The integer overload is the Studica
2025 API for a custom 100 Hz refresh rate. Convert NavX clockwise-positive yaw
into the WPILib counter-clockwise convention with
`Rotation2d.fromDegrees(-ahrs.getYaw())`. Delegate `isConnected()`,
`isCalibrating()`, `zeroYaw()`, and `close()` directly to the `AHRS` instance.

- [ ] **Step 4: Implement drivetrain control and pose estimation**

Create `SwerveDriveKinematics` from the four constant module translations. In `drive`, select robot-relative `ChassisSpeeds` directly or use `ChassisSpeeds.fromFieldRelativeSpeeds` only when the gyro is connected and not calibrating. Discretize at 20 ms, convert to module states, desaturate at `MAX_SPEED_MPS`, and dispatch in FL/FR/BL/BR order.

Own a `SwerveDrivePoseEstimator`; in `periodic()` update all module input snapshots first, update pose estimation with heading plus module positions, and publish:

- `Field2d` robot pose;
- gyro connected/calibrating/heading;
- measured and desired state per module;
- raw absolute encoder rotations per module.

`resetPose` must call `poseEstimator.resetPosition(currentHeading, currentPositions, pose)`. `createReal()` builds all four SparkMax module IOs from `MODULE_CONFIGS` and a `GyroIONavX`.

- [ ] **Step 5: Run focused and full tests and verify GREEN**

Run the Task 3 focused test, then `.\gradlew.bat test --console=plain` and confirm zero failures.

- [ ] **Step 6: Commit Task 3**

```powershell
git add vendordeps/Studica.json src/main/java/frc/robot/subsystems src/test/java/frc/robot/subsystems
git commit -m "feat: add NavX swerve drivetrain and odometry"
```

### Task 4: Driver Controls, Safety Defaults, and Robot Integration

**Files:**
- Create: `src/main/java/frc/robot/DriveInput.java`
- Modify: `src/main/java/frc/robot/RobotContainer.java`
- Modify: `src/main/java/frc/robot/Robot.java`
- Modify: `src/test/java/frc/robot/RobotContainerTest.java`
- Create: `src/test/java/frc/robot/DriveInputTest.java`

**Interfaces:**
- Consumes: joystick axes 1/0/4 and `SwerveDriveSubsystem`.
- Produces: shaped normalized translation/rotation, default field-relative drive command, heading-zero binding, subsystem access for later autonomous composition.

- [ ] **Step 1: Write failing joystick shaping tests**

Create `DriveInputTest` for a pure API:

```java
@Test
void deadbandRejectsStickNoise() {
    assertEquals(0.0, DriveInput.shapeAxis(0.04, 0.08), 1e-9);
}

@Test
void shapedAxisPreservesSignAndSquaresMagnitude() {
    assertEquals(-0.25, DriveInput.squareWithSign(-0.5), 1e-9);
    assertEquals(0.25, DriveInput.squareWithSign(0.5), 1e-9);
}

@Test
void translationVectorIsLimitedToUnitCircle() {
    Translation2d value = DriveInput.shapeTranslation(1.0, 1.0, 0.08);
    assertEquals(1.0, value.getNorm(), 1e-9);
}
```

Update `RobotContainerTest` to verify the container exposes a non-null drivetrain and constructs under HAL simulation.

- [ ] **Step 2: Run focused input/container tests and verify RED**

Run:

```powershell
.\gradlew.bat test --tests frc.robot.DriveInputTest --tests frc.robot.RobotContainerTest --console=plain
```

Expected: compilation fails because `DriveInput` and the drivetrain accessor do not exist.

- [ ] **Step 3: Implement pure drive-input shaping**

`DriveInput` applies a radial translation deadband, normalizes vectors larger than one, squares magnitude while preserving direction, and squares rotation with sign after deadband. The returned values remain normalized; only `RobotContainer` scales them to physical limits.

- [ ] **Step 4: Replace tank hardware with the swerve subsystem**

Remove the four drivetrain `SparkMax` fields and `driveWithJoystick()` from `RobotContainer`. Construct `SwerveDriveSubsystem.createReal()`. Set a default `RunCommand` that maps joystick axis 1 to forward, axis 0 to left/right, and axis 4 to rotation, uses `DriveInput`, scales by bring-up limits, and calls `drive(..., true)`. Bind button 4 to a run-once heading reset. Keep all intake, elevator, and climb bindings unchanged. Expose `getDrivetrain()` for future command composition.

Remove manual joystick driving from `Robot.teleopPeriodic()`; the command scheduler now runs the default drive command. Leave `autonomousInit()` without a drive command so this phase cannot accidentally execute autonomous movement.

- [ ] **Step 5: Run focused and full tests and verify GREEN**

Run the focused Task 4 tests and the complete test suite. Confirm existing mechanism button behavior is unchanged.

- [ ] **Step 6: Commit Task 4**

```powershell
git add src/main/java/frc/robot src/test/java/frc/robot
git commit -m "feat: integrate field-relative swerve teleop"
```

### Task 5: Commissioning Documentation and Final Verification

**Files:**
- Create: `docs/swerve-commissioning.md`
- Modify: `.gitignore` only if Gradle or simulation creates new local-only outputs not already ignored.

**Interfaces:**
- Consumes: dashboard telemetry implemented in Task 3.
- Produces: a deterministic wiring, offset calibration, wheel-direction, NavX, and low-speed validation procedure.

- [ ] **Step 1: Write the commissioning guide**

Document:

1. SparkMax firmware must match REVLib 2025 and each Through Bore Encoder must use REV-11-3326 on the steering SparkMax data port.
2. Verify CAN IDs 10–17 one controller at a time before installing belts/chains.
3. Lift the robot, point every wheel physically forward, record each raw absolute rotation from SmartDashboard, convert with `offsetRadians = rawRotations * 2π`, and enter the four values in `SwerveConstants`.
4. Rotate each module by hand and verify WPILib-positive angle direction; change only the matching `turnInverted` flag if direction is wrong.
5. Command +x at 0.2 m/s and verify all wheels pull toward the robot front; change only the matching `driveInverted` flag if one wheel is reversed.
6. Validate wheel diameter and drive reduction by commanding/measuring a straight 3 m traversal; update constants from measured error before raising speed limits.
7. Wait for NavX calibration with the robot motionless, zero heading, rotate the chassis counter-clockwise, and confirm dashboard heading increases.
8. Test robot-relative mode before field-relative mode, on blocks before floor, with an enabled operator beside the emergency stop.
9. Keep `MAX_SPEED_MPS=1.0` and `MAX_ANGULAR_SPEED_RAD_PER_SEC=2.0` until all checks pass.

- [ ] **Step 2: Run fresh full verification**

With Java 17 active, run:

```powershell
.\gradlew.bat clean test build --console=plain
```

Expected: exit code 0, all JUnit tests pass, and the deployable robot jar is produced.

- [ ] **Step 3: Inspect scope and repository state**

Run `git diff --check`, `git status --short`, and inspect `git diff --stat`. Confirm no Jetson, ROS, vision, trajectory, PathPlanner, or autonomous implementation was introduced.

- [ ] **Step 4: Commit documentation**

```powershell
git add docs/swerve-commissioning.md
git commit -m "docs: add swerve commissioning procedure"
```
