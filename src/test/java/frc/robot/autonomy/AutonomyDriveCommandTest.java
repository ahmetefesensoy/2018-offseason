package frc.robot.autonomy;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.AutonomyConstants;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.swerve.GyroIO;
import frc.robot.subsystems.swerve.SwerveModule;
import frc.robot.subsystems.swerve.SwerveModuleIO;
import java.util.Optional;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class AutonomyDriveCommandTest {
    private FakeSwerveModuleIO[] moduleIOs;
    private SwerveDriveSubsystem drivetrain;

    @BeforeEach
    void setup() {
        moduleIOs = new FakeSwerveModuleIO[] {
            new FakeSwerveModuleIO(), new FakeSwerveModuleIO(),
            new FakeSwerveModuleIO(), new FakeSwerveModuleIO()
        };
        drivetrain = new SwerveDriveSubsystem(
            new FakeGyroIO(),
            new SwerveModule(moduleIOs[0]),
            new SwerveModule(moduleIOs[1]),
            new SwerveModule(moduleIOs[2]),
            new SwerveModule(moduleIOs[3]));
    }

    @Test
    void acceptedCommandDrivesRobotRelativeWithAccelerationLimit() {
        StubController controller = new StubController(Optional.of(frame(0.25, 0.0, 0.0)));
        AutonomyDriveCommand command = new AutonomyDriveCommand(drivetrain, controller);

        command.execute();

        assertTrue(command.getRequirements().contains(drivetrain));
        for (SwerveModuleState state : drivetrain.getDesiredModuleStates()) {
            assertEquals(
                AutonomyConstants.MAX_TRANSLATION_ACCEL_MPS2
                    * AutonomyConstants.CONTROL_PERIOD_SECONDS,
                state.speedMetersPerSecond,
                1e-9);
            assertEquals(0.0, state.angle.getRadians(), 1e-9);
        }
    }

    @Test
    void missingCommandAndCommandEndStopEveryModule() {
        StubController controller = new StubController(Optional.empty());
        AutonomyDriveCommand command = new AutonomyDriveCommand(drivetrain, controller);

        command.execute();
        assertAllStopped();

        for (FakeSwerveModuleIO io : moduleIOs) {
            io.stopCalled = false;
        }
        command.end(true);
        assertAllStopped();
        assertTrue(controller.cancelCalled);
    }

    private void assertAllStopped() {
        for (FakeSwerveModuleIO io : moduleIOs) {
            assertTrue(io.stopCalled);
        }
    }

    private static AutonomyCommandFrame frame(double vx, double vy, double omega) {
        return new AutonomyCommandFrame("session", true, 1, 2, vx, vy, omega, 1, 1);
    }

    private static final class StubController implements AutonomyCommandSource {
        private final Optional<AutonomyCommandFrame> command;
        boolean cancelCalled;

        StubController(Optional<AutonomyCommandFrame> command) {
            this.command = command;
        }

        @Override
        public Optional<AutonomyCommandFrame> currentCommand() {
            return command;
        }

        @Override
        public void cancel() {
            cancelCalled = true;
        }
    }

    private static final class FakeGyroIO implements GyroIO {
        @Override
        public Rotation2d getHeading() {
            return Rotation2d.kZero;
        }

        @Override
        public boolean isConnected() {
            return true;
        }

        @Override
        public boolean isCalibrating() {
            return false;
        }

        @Override
        public void zeroYaw() {}
    }

    private static final class FakeSwerveModuleIO implements SwerveModuleIO {
        boolean stopCalled;

        @Override
        public Inputs readInputs() {
            return new Inputs(0.0, 0.0, Rotation2d.kZero, 0.0);
        }

        @Override
        public void setDriveVoltage(double volts) {}

        @Override
        public void setTurnVoltage(double volts) {}

        @Override
        public void stop() {
            stopCalled = true;
        }
    }
}
