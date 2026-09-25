package frc.robot.subsystems.swerve;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class SwerveModuleTest {
    private FakeSwerveModuleIO io;
    private SwerveModule module;

    @BeforeEach
    void setup() {
        io = new FakeSwerveModuleIO();
        module = new SwerveModule(io);
    }

    @Test
    void optimizationReversesWheelInsteadOfTurningMoreThanNinetyDegrees() {
        io.inputs = inputs(0.0, 0.0, 0.0, 0.0);
        module.updateInputs();

        SwerveModuleState optimized = module.optimizeState(
            new SwerveModuleState(1.0, Rotation2d.fromDegrees(170.0)));

        assertEquals(-1.0, optimized.speedMetersPerSecond, 1e-9);
        assertEquals(-10.0, optimized.angle.getDegrees(), 1e-9);
    }

    @Test
    void cosineCompensationSuppressesDriveWhileWheelIsSideways() {
        io.inputs = inputs(0.0, 0.0, 0.0, 0.0);
        module.updateInputs();

        SwerveModuleState optimized = module.applyCosineCompensation(
            new SwerveModuleState(1.0, Rotation2d.fromDegrees(90.0)));

        assertEquals(0.0, optimized.speedMetersPerSecond, 1e-9);
    }

    @Test
    void measuredPositionUsesDriveDistanceAndAbsoluteAngle() {
        io.inputs = inputs(2.75, 0.4, 37.0, 0.42);
        module.updateInputs();

        assertEquals(2.75, module.getPosition().distanceMeters, 1e-9);
        assertEquals(37.0, module.getPosition().angle.getDegrees(), 1e-9);
        assertEquals(0.42, module.getRawAbsolutePositionRotations(), 1e-9);
    }

    @Test
    void driveVoltageIsClampedToNominalBatteryVoltage() {
        io.inputs = inputs(0.0, -100.0, 0.0, 0.0);
        module.updateInputs();

        module.setDesiredState(new SwerveModuleState(100.0, Rotation2d.kZero));

        assertEquals(12.0, io.driveVolts, 1e-9);
        assertTrue(Math.abs(io.turnVolts) <= 12.0);
    }

    @Test
    void nearZeroSpeedRetainsMeasuredAngleAndStopsDrive() {
        io.inputs = inputs(0.0, 0.0, 25.0, 0.0);
        module.updateInputs();

        module.setDesiredState(
            new SwerveModuleState(0.005, Rotation2d.fromDegrees(-120.0)));

        assertEquals(0.0, io.driveVolts, 1e-9);
        assertEquals(25.0, module.getLastDesiredState().angle.getDegrees(), 1e-9);
    }

    @Test
    void stopCommandsBothOutputsToZero() {
        io.driveVolts = 6.0;
        io.turnVolts = -4.0;

        module.stop();

        assertTrue(io.stopCalled);
        assertEquals(0.0, io.driveVolts, 1e-9);
        assertEquals(0.0, io.turnVolts, 1e-9);
    }

    private static SwerveModuleIO.Inputs inputs(
            double drivePositionMeters,
            double driveVelocityMetersPerSecond,
            double turnAngleDegrees,
            double rawAbsolutePositionRotations) {
        return new SwerveModuleIO.Inputs(
            drivePositionMeters,
            driveVelocityMetersPerSecond,
            Rotation2d.fromDegrees(turnAngleDegrees),
            rawAbsolutePositionRotations);
    }

    private static final class FakeSwerveModuleIO implements SwerveModuleIO {
        Inputs inputs = new Inputs(0.0, 0.0, Rotation2d.kZero, 0.0);
        double driveVolts;
        double turnVolts;
        boolean stopCalled;

        @Override
        public Inputs readInputs() {
            return inputs;
        }

        @Override
        public void setDriveVoltage(double volts) {
            driveVolts = volts;
        }

        @Override
        public void setTurnVoltage(double volts) {
            turnVolts = volts;
        }

        @Override
        public void stop() {
            stopCalled = true;
            driveVolts = 0.0;
            turnVolts = 0.0;
        }
    }
}
