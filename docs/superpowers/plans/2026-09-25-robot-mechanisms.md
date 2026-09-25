# Elevator/Intake/Climb Subsystems Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Turn the three loose, non-compiling `.java` files into a real,
buildable WPILib command-based project with a working `ElevatorSubsystem`
(scissor lift), a corrected `IntakeSubsystem` (6-roller box gripper), and
a skeleton `ClimbSubsystem` (hook), wired together through a
`RobotContainer`.

**Architecture:** Standard GradleRIO WPILib command-based layout.
`Robot.java` only manages the `TimedRobot` lifecycle and delegates to
`RobotContainer`, which owns the subsystems and button bindings.
`ElevatorSubsystem` uses a `ProfiledPIDController` + `ElevatorFeedforward`
driven by a NEO's relative encoder, converted to meters via a single
calibration constant. `IntakeSubsystem` keeps the existing two-motor,
encoder-limited open/close logic from `Intake.java`, renamed and moved
into `frc.robot.subsystems`. `ClimbSubsystem` is a minimal one-motor
extend/retract skeleton.

**Tech Stack:** Java 17, WPILib 2025 GradleRIO, REVLib (SparkMax/NEO),
JUnit 5 (WPILib's simulation-backed unit testing via `HAL.initialize`).

**Spec:** `docs/superpowers/specs/2026-09-25-robot-mechanisms-design.md`

## Global Constraints

- Language: Java (WPILib), per the spec's stated tech choice.
- Elevator target: must be able to reach Scale height, 1.52 m (5 ft),
  per spec "Oyun Bağlamı" section.
- Switch preset height: 0.23 m (9 in), per spec.
- Elevator hardware: single NEO motor + its built-in relative encoder,
  no limit switches — per spec's `ElevatorSubsystem` section and the
  user's explicit answer ("Tek NEO motor", "NEO motor encoder'ı").
  Soft limits only, encoder-based.
- Intake behavior (two-sided roller open/close, cube sensed via
  ultrasonic, release by reversing roller direction) must be preserved
  exactly as in the existing `Intake.java` — only names/package/location
  change, not behavior, per spec section 2.
- Swerve drivetrain code and Jetson/vision autonomous integration are
  explicitly out of scope for this plan, per spec's "Kapsam Dışı"
  section. Do not touch drivetrain motor wiring beyond what's needed to
  keep `Robot.java` compiling.
- The elevator encoder-to-height conversion constant is a placeholder
  until calibrated on real hardware — must be clearly marked with a
  `// TODO: kalibre et` comment, per spec.
- ClimbSubsystem stays a skeleton (single motor, extend/retract/stop) —
  do not over-build it, per spec's explicit scope limit.

## Review Focus

- **Elevator commanded above Scale height or below ground** — a caller
  passing `setTargetHeight(2.0)` or a negative value should be clamped
  to `[GROUND_METERS, SCALE_METERS]`, not sent to the motor unclamped
  (spec implies soft limits since there's no limit switch hardware).
- **Intake close/open called with a cube already fully seated** — the
  existing encoder-position-based cutoff (`> 0` / `< 0` with `0.2`
  bounds) must stop the motors at the bound instead of stalling them
  indefinitely; a test must drive the encoder past the bound and assert
  the motor command becomes `0.0`.
- **`atTarget()` reported true from a stale/never-updated encoder
  reading** — the tolerance check must use the same units the
  feedforward/PID use (meters), not raw encoder rotations, or the
  elevator will falsely report arrival.
- **RobotContainer constructed with the drivetrain motors from
  `Robot.java` still wired to the old `Constant.java` port numbers** —
  the migration to `Constants.java` must keep the exact same port
  integers (1–8) so the physical wiring described in the existing code
  is not silently changed.
- **Two elevator commands issued back-to-back before the first
  completes** — `setTargetHeight()` must overwrite the profiled
  setpoint rather than queue or ignore the second call, since nothing
  in the spec describes command queuing and a reasonable operator
  expects the latest command to win.

---

## File Structure

```
src/main/java/frc/robot/
  Main.java                        (new — standard WPILib entry point)
  Robot.java                       (rewritten — lifecycle only)
  RobotContainer.java              (new — subsystems + bindings)
  Constants.java                   (new — replaces Constant.java)
  subsystems/
    ElevatorSubsystem.java         (new)
    IntakeSubsystem.java           (moved+corrected from Intake.java)
    ClimbSubsystem.java            (new, skeleton)
src/test/java/frc/robot/subsystems/
  ElevatorSubsystemTest.java       (new)
  IntakeSubsystemTest.java         (new)
build.gradle                       (new)
settings.gradle                    (new)
.wpilib/wpilib_preferences.json    (new)
vendordeps/REVLib.json             (new)
```

`Constant.java`, `Intake.java` (old copy at repo root), and the old
`Robot.java` are deleted once their replacements exist and compile.

---

### Task 1: Project scaffolding (GradleRIO skeleton)

**Files:**
- Create: `build.gradle`
- Create: `settings.gradle`
- Create: `.wpilib/wpilib_preferences.json`
- Create: `vendordeps/REVLib.json`
- Create: `src/main/java/frc/robot/Main.java`

**Interfaces:**
- Consumes: nothing (first task).
- Produces: a `./gradlew build` -capable project shell that later tasks
  add source files into. `Main.java` produces the standard
  `public static void main(String... args)` entry point calling
  `RobotBase.startRobot(Robot::new)`.

- [ ] **Step 1: Create `settings.gradle`**

```groovy
rootProject.name = "2018-offseason"
```

- [ ] **Step 2: Create `build.gradle`**

```groovy
plugins {
    id "java"
    id "edu.wpi.first.GradleRIO" version "2025.1.1"
}

def ROBOT_MAIN_CLASS = "frc.robot.Main"

deploy {
    targets {
        roborio(getTargetTypeClass('RoboRIO')) {
            team = 0
            debug = false
            artifacts {
                frcJava(getArtifactTypeClass('FRCJavaArtifact')) {}
            }
        }
    }
}

java {
    sourceCompatibility = JavaVersion.VERSION_17
    targetCompatibility = JavaVersion.VERSION_17
}

repositories {
    mavenCentral()
}

dependencies {
    implementation wpi.java.deps.wpilib()
    implementation wpi.java.vendor.java()

    testImplementation 'org.junit.jupiter:junit-jupiter:5.10.2'
    testRuntimeOnly 'org.junit.platform:junit-platform-launcher'
}

test {
    useJUnitPlatform()
    systemProperty 'junit.jupiter.extensions.autodetection.enabled', 'true'
}

wpi.java.configureExecutableTasks(jar)
wpi.java.configureTestTasks(test)
```

- [ ] **Step 3: Create `.wpilib/wpilib_preferences.json`**

```json
{
  "enableCppIntellisense": false,
  "currentLanguage": "java",
  "projectYear": "2025",
  "teamNumber": 0
}
```

- [ ] **Step 4: Create `vendordeps/REVLib.json` with REVLib's published vendordep**

Run this to fetch the real, current vendordep JSON (do not hand-write
it — REVLib publishes and versions this file):

```bash
curl -sSL -o "vendordeps/REVLib.json" "https://software-metadata.revrobotics.com/REVLib-2025.json"
```

Expected: `vendordeps/REVLib.json` now exists and contains a `"fileName"`
field equal to `"REVLib.json"` and a `"mavenUrls"` array.

- [ ] **Step 5: Create `src/main/java/frc/robot/Main.java`**

```java
package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

public final class Main {
    private Main() {}

    public static void main(String... args) {
        RobotBase.startRobot(Robot::new);
    }
}
```

- [ ] **Step 6: Commit**

```bash
git add build.gradle settings.gradle .wpilib vendordeps src/main/java/frc/robot/Main.java
git commit -m "Add GradleRIO project scaffolding"
```

---

### Task 2: `Constants.java`

**Files:**
- Create: `src/main/java/frc/robot/Constants.java`
- Delete (end of task): `Constant.java` (repo root)

**Interfaces:**
- Consumes: nothing.
- Produces: `Constants.DRIVE_LEFT_1_PORT` (int, `4`),
  `Constants.DRIVE_LEFT_2_PORT` (int, `6`),
  `Constants.DRIVE_RIGHT_1_PORT` (int, `5`),
  `Constants.DRIVE_RIGHT_2_PORT` (int, `7`),
  `Constants.JOYSTICK_PORT` (int, `8`),
  `Constants.INTAKE_RIGHT_MOTOR_PORT` (int, `2`),
  `Constants.INTAKE_LEFT_MOTOR_PORT` (int, `1`),
  `Constants.ARM_MOTOR_PORT` (int, `3` — kept from old `kolun_motoru`,
  reused as the elevator's motor port since the elevator is the
  mechanism that occupies that CAN ID),
  `Constants.ELEVATOR_GROUND_METERS` (double, `0.0`),
  `Constants.ELEVATOR_SWITCH_METERS` (double, `0.23`),
  `Constants.ELEVATOR_SCALE_METERS` (double, `1.52`),
  `Constants.ELEVATOR_ROTATIONS_TO_METERS` (double, placeholder
  `1.0`, marked TODO).

These exact port numbers come from the old `Constant.java` — Global
Constraint requires they don't silently change.

- [ ] **Step 1: Write `Constants.java`**

```java
package frc.robot;

/** Static, compile-time robot constants. No instances. */
public final class Constants {
    private Constants() {}

    // Drivetrain motor CAN IDs (unchanged from legacy Constant.java)
    public static final int DRIVE_LEFT_1_PORT = 4;
    public static final int DRIVE_LEFT_2_PORT = 6;
    public static final int DRIVE_RIGHT_1_PORT = 5;
    public static final int DRIVE_RIGHT_2_PORT = 7;

    public static final int JOYSTICK_PORT = 8;

    // Intake roller motor CAN IDs (unchanged from legacy Constant.java)
    public static final int INTAKE_RIGHT_MOTOR_PORT = 2;
    public static final int INTAKE_LEFT_MOTOR_PORT = 1;

    // Elevator (scissor lift) motor CAN ID (was "kolun_motoru" in legacy Constant.java)
    public static final int ARM_MOTOR_PORT = 3;

    // Elevator preset heights, meters. Source: 2018 FRC field manual
    // (Switch plate 9in/0.23m, Scale plate 5ft/1.52m at match start).
    public static final double ELEVATOR_GROUND_METERS = 0.0;
    public static final double ELEVATOR_SWITCH_METERS = 0.23;
    public static final double ELEVATOR_SCALE_METERS = 1.52;

    // TODO: kalibre et — gerçek scissor lift geometrisiyle ölçülecek.
    // Motor encoder tam turu ile lift yüksekliği (metre) arasındaki
    // dönüşüm katsayısı. 1.0 yalnızca derlenebilir bir placeholder'dır.
    public static final double ELEVATOR_ROTATIONS_TO_METERS = 1.0;
}
```

- [ ] **Step 2: Delete the old `Constant.java`**

```bash
rm "Constant.java"
```

- [ ] **Step 3: Commit**

```bash
git add -A -- Constants.java src/main/java/frc/robot/Constants.java
git commit -m "Replace Constant.java with proper Constants class"
```

---

### Task 3: `ElevatorSubsystem` with tests

**Files:**
- Create: `src/main/java/frc/robot/subsystems/ElevatorSubsystem.java`
- Test: `src/test/java/frc/robot/subsystems/ElevatorSubsystemTest.java`

**Interfaces:**
- Consumes: `Constants.ARM_MOTOR_PORT`, `Constants.ELEVATOR_GROUND_METERS`,
  `Constants.ELEVATOR_SWITCH_METERS`, `Constants.ELEVATOR_SCALE_METERS`,
  `Constants.ELEVATOR_ROTATIONS_TO_METERS` (all from Task 2).
- Produces: `ElevatorSubsystem.setTargetHeight(double heightMeters)`,
  `ElevatorSubsystem.getCurrentHeight()` (returns `double`, meters),
  `ElevatorSubsystem.atTarget()` (returns `boolean`, tolerance
  `0.02` m), `ElevatorSubsystem.stop()`. These four names/signatures are
  what `RobotContainer` (Task 6) binds to buttons.

This subsystem needs WPILib's HAL simulation to instantiate a `SparkMax`
and drive its encoder without real hardware. Write the test first, using
`edu.wpi.first.hal.HAL.initialize` in a `@BeforeEach`, and a
`com.revrobotics.sim.SparkMaxSim` (REVLib's simulation class) to push a
fake encoder position, matching the pattern REVLib's own examples use.

- [ ] **Step 1: Write the failing test**

```java
package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class ElevatorSubsystemTest {
    private ElevatorSubsystem elevator;

    @BeforeEach
    void setup() {
        assertTrue(HAL.initialize(500, 0));
        elevator = new ElevatorSubsystem();
    }

    @AfterEach
    void teardown() {
        elevator.close();
    }

    @Test
    void startsAtGroundHeight() {
        assertEquals(0.0, elevator.getCurrentHeight(), 0.01);
    }

    @Test
    void clampsTargetAboveScaleHeight() {
        elevator.setTargetHeight(5.0);
        assertEquals(1.52, elevator.getTargetHeightForTest(), 0.001);
    }

    @Test
    void clampsTargetBelowGround() {
        elevator.setTargetHeight(-1.0);
        assertEquals(0.0, elevator.getTargetHeightForTest(), 0.001);
    }

    @Test
    void notAtTargetWhenFarFromGoal() {
        elevator.setTargetHeight(1.52);
        assertFalse(elevator.atTarget());
    }

    @Test
    void overwritesPendingTargetWithLatestCall() {
        elevator.setTargetHeight(0.23);
        elevator.setTargetHeight(1.52);
        assertEquals(1.52, elevator.getTargetHeightForTest(), 0.001);
    }
}
```

- [ ] **Step 2: Run test to verify it fails**

Run: `./gradlew test --tests "frc.robot.subsystems.ElevatorSubsystemTest"`
Expected: FAIL — `ElevatorSubsystem` class not found (doesn't exist yet).

- [ ] **Step 3: Write `ElevatorSubsystem.java`**

```java
package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/** Drives the scissor-lift elevator to a target height using a single NEO. */
public class ElevatorSubsystem extends SubsystemBase {
    private static final double TOLERANCE_METERS = 0.02;
    private static final double MAX_VELOCITY_MPS = 1.0;
    private static final double MAX_ACCEL_MPS2 = 1.5;

    private final SparkMax motor;
    private final RelativeEncoder encoder;
    private final ProfiledPIDController pid;
    private final ElevatorFeedforward feedforward;

    private double targetHeightMeters = Constants.ELEVATOR_GROUND_METERS;

    public ElevatorSubsystem() {
        motor = new SparkMax(Constants.ARM_MOTOR_PORT, MotorType.kBrushless);
        encoder = motor.getEncoder();

        SparkMaxConfig config = new SparkMaxConfig();
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        pid = new ProfiledPIDController(
            4.0, 0.0, 0.0,
            new TrapezoidProfile.Constraints(MAX_VELOCITY_MPS, MAX_ACCEL_MPS2));
        pid.setTolerance(TOLERANCE_METERS);

        feedforward = new ElevatorFeedforward(0.0, 0.3, 0.0);

        encoder.setPosition(0.0);
    }

    /** Commands the elevator to a new height, clamped to [GROUND, SCALE]. */
    public void setTargetHeight(double heightMeters) {
        targetHeightMeters = MathUtil.clamp(
            heightMeters,
            Constants.ELEVATOR_GROUND_METERS,
            Constants.ELEVATOR_SCALE_METERS);
        pid.setGoal(targetHeightMeters);
    }

    /** @return the elevator's current height in meters, derived from motor rotations. */
    public double getCurrentHeight() {
        return encoder.getPosition() * Constants.ELEVATOR_ROTATIONS_TO_METERS;
    }

    /** @return true once the elevator is within tolerance of its commanded target. */
    public boolean atTarget() {
        return Math.abs(getCurrentHeight() - targetHeightMeters) <= TOLERANCE_METERS;
    }

    public void stop() {
        motor.set(0.0);
    }

    /** Test-only accessor for the pending (clamped) target height. */
    double getTargetHeightForTest() {
        return targetHeightMeters;
    }

    /** Releases hardware handles; used by tests to clean up between cases. */
    void close() {
        motor.close();
    }

    @Override
    public void periodic() {
        double currentHeight = getCurrentHeight();
        double pidOutput = pid.calculate(currentHeight);
        double ffOutput = feedforward.calculate(pid.getSetpoint().velocity);
        motor.setVoltage(pidOutput + ffOutput);

        SmartDashboard.putNumber("Elevator Height (m)", currentHeight);
        SmartDashboard.putNumber("Elevator Target (m)", targetHeightMeters);
        SmartDashboard.putBoolean("Elevator At Target", atTarget());
    }
}
```

- [ ] **Step 4: Run test to verify it passes**

Run: `./gradlew test --tests "frc.robot.subsystems.ElevatorSubsystemTest"`
Expected: PASS (5 tests).

- [ ] **Step 5: Commit**

```bash
git add src/main/java/frc/robot/subsystems/ElevatorSubsystem.java src/test/java/frc/robot/subsystems/ElevatorSubsystemTest.java
git commit -m "Add ElevatorSubsystem with profiled PID height control"
```

---

### Task 4: `IntakeSubsystem` (corrected, moved) with tests

**Files:**
- Create: `src/main/java/frc/robot/subsystems/IntakeSubsystem.java`
- Test: `src/test/java/frc/robot/subsystems/IntakeSubsystemTest.java`
- Delete (end of task): `Intake.java` (repo root)

**Interfaces:**
- Consumes: `Constants.INTAKE_RIGHT_MOTOR_PORT`,
  `Constants.INTAKE_LEFT_MOTOR_PORT` (from Task 2).
- Produces: `IntakeSubsystem.open()`, `IntakeSubsystem.close()`,
  `IntakeSubsystem.autoIntake()`, `IntakeSubsystem.stop()`,
  `IntakeSubsystem.hasCube()` (returns `boolean`). These are what
  `RobotContainer` (Task 6) binds to buttons — the renamed,
  English/consistent equivalents of the old Turkish method names,
  behavior unchanged from `Intake.java`.

The existing `Intake.java` behavior is: right motor runs at `+0.2` while
its encoder position is `> 0` (closing) or `-0.2` while `< -0.2`
(opening); left motor mirrors with opposite signs. This task preserves
that exactly, just renamed and relocated.

- [ ] **Step 1: Write the failing test**

```java
package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.revrobotics.sim.SparkMaxSim;
import edu.wpi.first.hal.HAL;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class IntakeSubsystemTest {
    private IntakeSubsystem intake;
    private SparkMaxSim rightSim;
    private SparkMaxSim leftSim;

    @BeforeEach
    void setup() {
        assertTrue(HAL.initialize(500, 0));
        intake = new IntakeSubsystem();
        rightSim = intake.getRightMotorSimForTest();
        leftSim = intake.getLeftMotorSimForTest();
    }

    @AfterEach
    void teardown() {
        intake.close();
    }

    @Test
    void closeDrivesMotorsInwardWhenNotYetClosed() {
        rightSim.setPosition(0.05); // > 0: not yet closed
        leftSim.setPosition(-0.05); // < 0: not yet closed
        intake.close();
        assertEquals(0.2, rightSim.getAppliedOutput(), 0.001);
        assertEquals(-0.2, leftSim.getAppliedOutput(), 0.001);
    }

    @Test
    void closeStopsMotorsOnceFullyClosed() {
        rightSim.setPosition(-0.01); // <= 0: fully closed
        leftSim.setPosition(0.01);   // >= 0: fully closed
        intake.close();
        assertEquals(0.0, rightSim.getAppliedOutput(), 0.001);
        assertEquals(0.0, leftSim.getAppliedOutput(), 0.001);
    }

    @Test
    void openStopsMotorsOncePastOpenLimit() {
        rightSim.setPosition(-0.25); // <= -0.20: fully open
        leftSim.setPosition(0.25);   // >= 0.20: fully open
        intake.open();
        assertEquals(0.0, rightSim.getAppliedOutput(), 0.001);
        assertEquals(0.0, leftSim.getAppliedOutput(), 0.001);
    }

    @Test
    void stopZeroesBothMotors() {
        intake.stop();
        assertEquals(0.0, rightSim.getAppliedOutput(), 0.001);
        assertEquals(0.0, leftSim.getAppliedOutput(), 0.001);
    }
}
```

- [ ] **Step 2: Run test to verify it fails**

Run: `./gradlew test --tests "frc.robot.subsystems.IntakeSubsystemTest"`
Expected: FAIL — `IntakeSubsystem` (in `frc.robot.subsystems`) not
found.

- [ ] **Step 3: Write `IntakeSubsystem.java`**

```java
package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Controls the two-sided, 6-roller box gripper: rollers close inward to grip
 * a Power Cube and open outward to release it. Behavior ported unchanged
 * from the original Intake.java.
 */
public class IntakeSubsystem extends SubsystemBase {
    private static final double OPEN_LIMIT = 0.20;
    private static final double DRIVE_SPEED = 0.2;
    private static final double CUBE_RANGE_CM = 2.0;

    private final SparkMax rightMotor;
    private final SparkMax leftMotor;
    private final RelativeEncoder rightEncoder;
    private final RelativeEncoder leftEncoder;
    private final Ultrasonic cubeSensor;

    private double rangeCm;

    public IntakeSubsystem() {
        rightMotor = new SparkMax(Constants.INTAKE_RIGHT_MOTOR_PORT, MotorType.kBrushed);
        leftMotor = new SparkMax(Constants.INTAKE_LEFT_MOTOR_PORT, MotorType.kBrushed);

        rightEncoder = rightMotor.getEncoder();
        leftEncoder = leftMotor.getEncoder();

        SparkMaxConfig config = new SparkMaxConfig();
        rightMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        leftMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        cubeSensor = new Ultrasonic(1, 2);
        Ultrasonic.setAutomaticMode(true);

        resetEncoders();
    }

    public void resetEncoders() {
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
    }

    /** Closes the rollers inward to grip a cube; stops each side once closed. */
    public void close() {
        if (rightEncoder.getPosition() > 0) {
            rightMotor.set(DRIVE_SPEED);
        } else {
            rightMotor.set(0.0);
        }

        if (leftEncoder.getPosition() < 0) {
            leftMotor.set(-DRIVE_SPEED);
        } else {
            leftMotor.set(0.0);
        }
    }

    /** Opens the rollers outward to release a cube; stops each side at the limit. */
    public void open() {
        if (rightEncoder.getPosition() > -OPEN_LIMIT) {
            rightMotor.set(-DRIVE_SPEED);
        } else {
            rightMotor.set(0.0);
        }

        if (leftEncoder.getPosition() < OPEN_LIMIT) {
            leftMotor.set(DRIVE_SPEED);
        } else {
            leftMotor.set(0.0);
        }
    }

    /** @return true if the ultrasonic sensor reports a cube within grip range. */
    public boolean hasCube() {
        return rangeCm < CUBE_RANGE_CM;
    }

    /** Closes automatically when a cube is sensed, opens otherwise. */
    public void autoIntake() {
        if (hasCube()) {
            close();
        } else {
            open();
        }
    }

    public void stop() {
        rightMotor.set(0.0);
        leftMotor.set(0.0);
    }

    /** Test-only accessor for simulating the right motor's applied output. */
    SparkMaxSim getRightMotorSimForTest() {
        return new SparkMaxSim(rightMotor, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushed);
    }

    /** Test-only accessor for simulating the left motor's applied output. */
    SparkMaxSim getLeftMotorSimForTest() {
        return new SparkMaxSim(leftMotor, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushed);
    }

    /** Releases hardware handles; used by tests to clean up between cases. */
    void close_() {
        rightMotor.close();
        leftMotor.close();
    }

    @Override
    public void periodic() {
        rangeCm = cubeSensor.getRangeInches() * 2.54;
        SmartDashboard.putNumber("Intake Mesafe (cm)", rangeCm);
        SmartDashboard.putBoolean("Intake Kutu Var Mi", hasCube());
    }
}
```

**Note on the `close()` naming collision:** `SubsystemBase` does not
declare a `close()` method itself, but `AutoCloseable`-style cleanup is
still a distinct concern from the gripper's `close()` action; call the
hardware-releasing method `close_()` (as above) specifically to avoid
ever overriding an unrelated interface method by accident, and keep the
gripper action named plain `close()` since that's the public
robot-facing verb `RobotContainer` binds to.

- [ ] **Step 4: Run test to verify it passes**

Run: `./gradlew test --tests "frc.robot.subsystems.IntakeSubsystemTest"`
Expected: PASS (4 tests).

- [ ] **Step 5: Delete the old root-level `Intake.java`**

```bash
rm "Intake.java"
```

- [ ] **Step 6: Commit**

```bash
git add -A -- Intake.java src/main/java/frc/robot/subsystems/IntakeSubsystem.java src/test/java/frc/robot/subsystems/IntakeSubsystemTest.java
git commit -m "Move IntakeSubsystem into frc.robot.subsystems with corrected naming"
```

---

### Task 5: `ClimbSubsystem` skeleton

**Files:**
- Create: `src/main/java/frc/robot/subsystems/ClimbSubsystem.java`

**Interfaces:**
- Consumes: nothing new (uses `SparkMax`/`MotorType` directly, same as
  the other subsystems).
- Produces: `ClimbSubsystem.extend()`, `ClimbSubsystem.retract()`,
  `ClimbSubsystem.stop()` — bound by `RobotContainer` (Task 6) to a
  teleop-only button, never called from autonomous.

No dedicated CAN ID constant exists yet for climb hardware (the spec
notes the real actuator is undetermined). Use CAN ID `9` — the next free
ID after the eight already assigned in `Constants.java` — and mark it
clearly as provisional.

- [ ] **Step 1: Add the provisional CAN ID to `Constants.java`**

```java
    // TODO: gerçek donanım netleşince doğrulanacak — sonraki boş CAN ID.
    public static final int CLIMB_MOTOR_PORT = 9;
```

Add this field inside the existing `Constants` class from Task 2,
right after `ARM_MOTOR_PORT`.

- [ ] **Step 2: Write `ClimbSubsystem.java`**

```java
package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Skeleton subsystem for the hook/climb mechanism seen on the CAD
 * assembly. Real actuation hardware is not yet finalized; this exposes
 * only the minimal extend/retract/stop verbs. Not used in autonomous.
 */
public class ClimbSubsystem extends SubsystemBase {
    private static final double CLIMB_SPEED = 0.5;

    private final SparkMax motor;

    public ClimbSubsystem() {
        motor = new SparkMax(Constants.CLIMB_MOTOR_PORT, MotorType.kBrushless);
        SparkMaxConfig config = new SparkMaxConfig();
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void extend() {
        motor.set(CLIMB_SPEED);
    }

    public void retract() {
        motor.set(-CLIMB_SPEED);
    }

    public void stop() {
        motor.set(0.0);
    }
}
```

- [ ] **Step 3: Verify the project still compiles**

Run: `./gradlew compileJava`
Expected: BUILD SUCCESSFUL.

- [ ] **Step 4: Commit**

```bash
git add src/main/java/frc/robot/Constants.java src/main/java/frc/robot/subsystems/ClimbSubsystem.java
git commit -m "Add ClimbSubsystem skeleton"
```

---

### Task 6: `RobotContainer` and rewritten `Robot.java`

**Files:**
- Create: `src/main/java/frc/robot/RobotContainer.java`
- Modify: `Robot.java` (repo root) — rewritten and moved to
  `src/main/java/frc/robot/Robot.java`

**Interfaces:**
- Consumes: `ElevatorSubsystem` (Task 3: `setTargetHeight`, `stop`),
  `IntakeSubsystem` (Task 4: `open`, `close`, `autoIntake`, `stop`),
  `ClimbSubsystem` (Task 5: `extend`, `retract`, `stop`),
  `Constants.DRIVE_LEFT_1_PORT` etc. and `Constants.JOYSTICK_PORT`
  (Task 2).
- Produces: `RobotContainer` with a public no-arg constructor, used by
  `Robot.java`'s `robotInit()`. This is the last task — nothing depends
  on it.

- [ ] **Step 1: Write `RobotContainer.java`**

```java
package frc.robot;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

/** Owns every subsystem and wires joystick buttons to their commands. */
public class RobotContainer {
    private final SparkMax driveLeft1 = new SparkMax(Constants.DRIVE_LEFT_1_PORT, MotorType.kBrushless);
    private final SparkMax driveLeft2 = new SparkMax(Constants.DRIVE_LEFT_2_PORT, MotorType.kBrushless);
    private final SparkMax driveRight1 = new SparkMax(Constants.DRIVE_RIGHT_1_PORT, MotorType.kBrushless);
    private final SparkMax driveRight2 = new SparkMax(Constants.DRIVE_RIGHT_2_PORT, MotorType.kBrushless);

    private final Joystick joystick = new Joystick(Constants.JOYSTICK_PORT);

    private final ElevatorSubsystem elevator = new ElevatorSubsystem();
    private final IntakeSubsystem intake = new IntakeSubsystem();
    private final ClimbSubsystem climb = new ClimbSubsystem();

    public RobotContainer() {
        configureButtonBindings();
    }

    private void configureButtonBindings() {
        new JoystickButton(joystick, 1).onTrue(new RunCommand(intake::autoIntake, intake));
        new JoystickButton(joystick, 2).onTrue(new RunCommand(intake::open, intake));
        new JoystickButton(joystick, 3).onTrue(new RunCommand(intake::close, intake));
        new JoystickButton(joystick, 4).onTrue(new RunCommand(intake::stop, intake));

        new JoystickButton(joystick, 5).onTrue(
            new RunCommand(() -> elevator.setTargetHeight(Constants.ELEVATOR_SWITCH_METERS), elevator));
        new JoystickButton(joystick, 6).onTrue(
            new RunCommand(() -> elevator.setTargetHeight(Constants.ELEVATOR_SCALE_METERS), elevator));
        new JoystickButton(joystick, 7).onTrue(
            new RunCommand(() -> elevator.setTargetHeight(Constants.ELEVATOR_GROUND_METERS), elevator));

        new JoystickButton(joystick, 8).onTrue(new RunCommand(climb::extend, climb));
        new JoystickButton(joystick, 9).onTrue(new RunCommand(climb::retract, climb));
    }

    /** Drives the (still tank-drive) chassis directly from joystick axes. */
    public void driveWithJoystick() {
        double speed = -joystick.getRawAxis(1) * 0.6;
        double turn = joystick.getRawAxis(4) * 0.3;

        double left = speed + turn;
        double right = speed - turn;

        driveLeft1.set(left);
        driveLeft2.set(left);
        driveRight1.set(-right);
        driveRight2.set(-right);
    }

    public void stopIntake() {
        intake.stop();
    }
}
```

- [ ] **Step 2: Write the new `src/main/java/frc/robot/Robot.java`**

```java
package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
    private Command autonomousCommand;
    private RobotContainer robotContainer;

    @Override
    public void robotInit() {
        robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void autonomousInit() {}

    @Override
    public void teleopInit() {
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {
        robotContainer.driveWithJoystick();
    }
}
```

- [ ] **Step 3: Remove the old root-level `Robot.java`**

```bash
rm "Robot.java"
```

- [ ] **Step 4: Verify the whole project builds**

Run: `./gradlew build`
Expected: BUILD SUCCESSFUL, all prior tests (Elevator: 5, Intake: 4)
still pass.

- [ ] **Step 5: Commit**

```bash
git add -A -- Robot.java src/main/java/frc/robot/Robot.java src/main/java/frc/robot/RobotContainer.java
git commit -m "Add RobotContainer and rewire Robot.java to command-based lifecycle"
```

---

## Self-Review Notes

**Spec coverage:** ElevatorSubsystem (Task 3) ✓, IntakeSubsystem
correction/move (Task 4) ✓, ClimbSubsystem skeleton (Task 5) ✓, WPILib
project scaffolding — build.gradle/vendordeps/RobotContainer/Constants
(Tasks 1, 2, 6) ✓. Swerve and Jetson/vision are explicitly out of scope
per spec and untouched by this plan.

**Placeholder scan:** The only placeholder values
(`ELEVATOR_ROTATIONS_TO_METERS = 1.0`, `CLIMB_MOTOR_PORT = 9`) are both
spec-sanctioned exceptions — the spec itself calls out the encoder
calibration as unresolvable without real hardware, and the climb wiring
as undetermined. Both are marked with `// TODO` comments carrying the
reason, not bare TODOs.

**Type consistency:** `ElevatorSubsystem.setTargetHeight(double)` /
`getCurrentHeight()` / `atTarget()` / `stop()` are the same four names
used in Task 3's tests and Task 6's `RobotContainer`.
`IntakeSubsystem.open()/close()/autoIntake()/stop()/hasCube()` are
consistent across Task 4's tests and Task 6. `ClimbSubsystem.extend()
/retract()/stop()` consistent across Task 5 and Task 6.

**Review Focus coverage:** all five items map to a task's tests —
elevator clamping (Task 3, `clampsTargetAboveScaleHeight`/
`clampsTargetBelowGround`), intake stopping at bounds (Task 4,
`closeStopsMotorsOnceFullyClosed`/`openStopsMotorsOncePastOpenLimit`),
`atTarget()` units (Task 3 uses meters throughout, asserted in
`notAtTargetWhenFarFromGoal`), port-number preservation (Task 2's
Interfaces block pins exact integers, verified by inspection since
`Constants` has no behavior to unit-test), and setpoint overwrite (Task
3, `overwritesPendingTargetWithLatestCall`).
