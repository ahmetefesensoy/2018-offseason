package frc.robot;

import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.autonomy.AutonomyCommandFrame;
import frc.robot.autonomy.AutonomyCommandSource;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.swerve.GyroIO;
import frc.robot.subsystems.swerve.SwerveModule;
import frc.robot.subsystems.swerve.SwerveModuleIO;
import java.util.Optional;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class RobotContainerAutonomyTest {
    private FakeModuleIO[] moduleIOs;
    private SwerveDriveSubsystem drivetrain;

    @BeforeEach
    void setup() {
        assertTrue(HAL.initialize(500, 0));
        moduleIOs = new FakeModuleIO[] {
            new FakeModuleIO(), new FakeModuleIO(), new FakeModuleIO(), new FakeModuleIO()
        };
        drivetrain = new SwerveDriveSubsystem(
            new FakeGyro(),
            new SwerveModule(moduleIOs[0]),
            new SwerveModule(moduleIOs[1]),
            new SwerveModule(moduleIOs[2]),
            new SwerveModule(moduleIOs[3]));
    }

    @Test
    void exposesOwnedAutonomousCommandAndProvidesImmediateStop() {
        EmptyCommandSource source = new EmptyCommandSource();
        RobotContainer container = new RobotContainer(drivetrain, source);

        var command = container.getAutonomousCommand();

        assertNotNull(command);
        assertTrue(command.getRequirements().contains(drivetrain));

        container.stopAutonomy();

        assertTrue(source.cancelled);
        for (FakeModuleIO io : moduleIOs) {
            assertTrue(io.stopped);
        }
    }

    private static final class EmptyCommandSource implements AutonomyCommandSource {
        boolean cancelled;

        @Override
        public Optional<AutonomyCommandFrame> currentCommand() {
            return Optional.empty();
        }

        @Override
        public void cancel() {
            cancelled = true;
        }
    }

    private static final class FakeGyro implements GyroIO {
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

    private static final class FakeModuleIO implements SwerveModuleIO {
        boolean stopped;

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
            stopped = true;
        }
    }
}
