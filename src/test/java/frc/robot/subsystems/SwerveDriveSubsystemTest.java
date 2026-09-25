package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.SwerveConstants;
import frc.robot.subsystems.swerve.GyroIO;
import frc.robot.subsystems.swerve.SwerveModule;
import frc.robot.subsystems.swerve.SwerveModuleIO;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class SwerveDriveSubsystemTest {
    private FakeGyroIO gyro;
    private FakeSwerveModuleIO[] moduleIOs;
    private SwerveDriveSubsystem drive;

    @BeforeEach
    void setup() {
        gyro = new FakeGyroIO();
        moduleIOs = new FakeSwerveModuleIO[] {
            new FakeSwerveModuleIO(),
            new FakeSwerveModuleIO(),
            new FakeSwerveModuleIO(),
            new FakeSwerveModuleIO()
        };
        drive = new SwerveDriveSubsystem(
            gyro,
            new SwerveModule(moduleIOs[0]),
            new SwerveModule(moduleIOs[1]),
            new SwerveModule(moduleIOs[2]),
            new SwerveModule(moduleIOs[3]));
    }

    @Test
    void forwardRobotRelativeCommandPointsAllModulesForward() {
        drive.drive(1.0, 0.0, 0.0, false);

        for (SwerveModuleState state : drive.getDesiredModuleStates()) {
            assertEquals(1.0, state.speedMetersPerSecond, 1e-9);
            assertEquals(0.0, state.angle.getRadians(), 1e-9);
        }
    }

    @Test
    void fieldRelativeCommandUsesGyroHeading() {
        gyro.heading = Rotation2d.fromDegrees(90.0);

        drive.drive(1.0, 0.0, 0.0, true);

        for (SwerveModuleState state : drive.getDesiredModuleStates()) {
            assertEquals(-90.0, state.angle.getDegrees(), 1e-6);
        }
    }

    @Test
    void disconnectedGyroFallsBackToRobotRelativeDrive() {
        gyro.connected = false;
        gyro.heading = Rotation2d.fromDegrees(90.0);

        drive.drive(1.0, 0.0, 0.0, true);

        for (SwerveModuleState state : drive.getDesiredModuleStates()) {
            assertEquals(0.0, state.angle.getDegrees(), 1e-6);
        }
    }

    @Test
    void calibratingGyroFallsBackToRobotRelativeDrive() {
        gyro.calibrating = true;
        gyro.heading = Rotation2d.fromDegrees(90.0);

        drive.drive(1.0, 0.0, 0.0, true);

        for (SwerveModuleState state : drive.getDesiredModuleStates()) {
            assertEquals(0.0, state.angle.getDegrees(), 1e-6);
        }
    }

    @Test
    void wheelSpeedsAreDesaturatedToBringupLimit() {
        drive.drive(5.0, 0.0, 0.0, false);

        for (SwerveModuleState state : drive.getDesiredModuleStates()) {
            assertTrue(Math.abs(state.speedMetersPerSecond) <= SwerveConstants.MAX_SPEED_MPS);
            assertEquals(1.0, state.speedMetersPerSecond, 1e-9);
        }
    }

    @Test
    void moduleStateArrayMustUseExactlyFourEntries() {
        assertThrows(
            IllegalArgumentException.class,
            () -> drive.setModuleStates(new SwerveModuleState[3]));
    }

    @Test
    void resetPoseUsesCurrentHeadingAndWheelPositions() {
        Pose2d requestedPose = new Pose2d(2.0, 3.0, Rotation2d.fromDegrees(30.0));

        drive.resetPose(requestedPose);

        assertEquals(requestedPose.getX(), drive.getPose().getX(), 1e-9);
        assertEquals(requestedPose.getY(), drive.getPose().getY(), 1e-9);
        assertEquals(
            requestedPose.getRotation().getRadians(),
            drive.getPose().getRotation().getRadians(),
            1e-9);
    }

    @Test
    void periodicUpdatesPoseFromMeasuredWheelDistance() {
        for (FakeSwerveModuleIO io : moduleIOs) {
            io.inputs = new SwerveModuleIO.Inputs(1.0, 0.0, Rotation2d.kZero, 0.0);
        }

        drive.periodic();

        assertEquals(1.0, drive.getPose().getX(), 1e-6);
        assertEquals(0.0, drive.getPose().getY(), 1e-6);
    }

    @Test
    void zeroHeadingDelegatesToGyro() {
        drive.zeroHeading();

        assertTrue(gyro.zeroCalled);
    }

    @Test
    void stopPropagatesToEveryModule() {
        drive.stop();

        for (FakeSwerveModuleIO io : moduleIOs) {
            assertTrue(io.stopCalled);
        }
    }

    @Test
    void nonFiniteChassisCommandStopsEveryModule() {
        drive.drive(Double.NaN, 0.0, 0.0, false);

        for (FakeSwerveModuleIO io : moduleIOs) {
            assertTrue(io.stopCalled);
        }
    }

    private static final class FakeGyroIO implements GyroIO {
        Rotation2d heading = Rotation2d.kZero;
        boolean connected = true;
        boolean calibrating;
        boolean zeroCalled;

        @Override
        public Rotation2d getHeading() {
            return heading;
        }

        @Override
        public boolean isConnected() {
            return connected;
        }

        @Override
        public boolean isCalibrating() {
            return calibrating;
        }

        @Override
        public void zeroYaw() {
            zeroCalled = true;
            heading = Rotation2d.kZero;
        }
    }

    private static final class FakeSwerveModuleIO implements SwerveModuleIO {
        Inputs inputs = new Inputs(0.0, 0.0, Rotation2d.kZero, 0.0);
        boolean stopCalled;

        @Override
        public Inputs readInputs() {
            return inputs;
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
