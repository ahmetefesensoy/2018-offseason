package frc.robot.subsystems.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.SwerveConstants;
import java.util.Objects;

/** Closed-loop speed and azimuth controller for one swerve module. */
public final class SwerveModule implements AutoCloseable {
    private static final double STOP_SPEED_THRESHOLD_MPS =
        SwerveConstants.MAX_SPEED_MPS * 0.01;

    private final SwerveModuleIO io;
    private final PIDController driveController;
    private final SimpleMotorFeedforward driveFeedforward;
    private final ProfiledPIDController turnController;

    private SwerveModuleIO.Inputs inputs = new SwerveModuleIO.Inputs(
        0.0,
        0.0,
        Rotation2d.kZero,
        0.0);
    private SwerveModuleState lastDesiredState = new SwerveModuleState();
    private boolean initialized;

    public SwerveModule(SwerveModuleIO io) {
        this.io = Objects.requireNonNull(io);
        driveController = new PIDController(
            SwerveConstants.DRIVE_KP,
            SwerveConstants.DRIVE_KI,
            SwerveConstants.DRIVE_KD,
            SwerveConstants.LOOP_PERIOD_SECONDS);
        driveFeedforward = new SimpleMotorFeedforward(
            SwerveConstants.DRIVE_KS_VOLTS,
            SwerveConstants.DRIVE_KV_VOLT_SECONDS_PER_METER,
            SwerveConstants.DRIVE_KA_VOLT_SECONDS_SQUARED_PER_METER,
            SwerveConstants.LOOP_PERIOD_SECONDS);
        turnController = new ProfiledPIDController(
            SwerveConstants.TURN_KP,
            SwerveConstants.TURN_KI,
            SwerveConstants.TURN_KD,
            new TrapezoidProfile.Constraints(
                SwerveConstants.TURN_MAX_VELOCITY_RAD_PER_SEC,
                SwerveConstants.TURN_MAX_ACCELERATION_RAD_PER_SEC_SQUARED),
            SwerveConstants.LOOP_PERIOD_SECONDS);
        turnController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public void updateInputs() {
        inputs = Objects.requireNonNull(io.readInputs());
        if (!initialized) {
            turnController.reset(inputs.turnAngle().getRadians());
            lastDesiredState = new SwerveModuleState(0.0, inputs.turnAngle());
            initialized = true;
        }
    }

    SwerveModuleState optimizeState(SwerveModuleState desiredState) {
        SwerveModuleState optimized = new SwerveModuleState(
            desiredState.speedMetersPerSecond,
            desiredState.angle);
        optimized.optimize(inputs.turnAngle());
        return optimized;
    }

    SwerveModuleState applyCosineCompensation(SwerveModuleState optimizedState) {
        SwerveModuleState compensated = new SwerveModuleState(
            optimizedState.speedMetersPerSecond,
            optimizedState.angle);
        compensated.cosineScale(inputs.turnAngle());
        return compensated;
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        SwerveModuleState target;
        if (Math.abs(desiredState.speedMetersPerSecond) < STOP_SPEED_THRESHOLD_MPS) {
            target = new SwerveModuleState(0.0, lastDesiredState.angle);
        } else {
            target = applyCosineCompensation(optimizeState(desiredState));
        }
        lastDesiredState = new SwerveModuleState(target.speedMetersPerSecond, target.angle);

        double driveVolts = target.speedMetersPerSecond == 0.0
            ? 0.0
            : driveController.calculate(
                inputs.driveVelocityMetersPerSecond(),
                target.speedMetersPerSecond)
                + driveFeedforward.calculate(target.speedMetersPerSecond);
        double turnVolts = turnController.calculate(
            inputs.turnAngle().getRadians(),
            target.angle.getRadians());

        io.setDriveVoltage(MathUtil.clamp(
            driveVolts,
            -SwerveConstants.NOMINAL_VOLTAGE,
            SwerveConstants.NOMINAL_VOLTAGE));
        io.setTurnVoltage(MathUtil.clamp(
            turnVolts,
            -SwerveConstants.NOMINAL_VOLTAGE,
            SwerveConstants.NOMINAL_VOLTAGE));
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
            inputs.driveVelocityMetersPerSecond(),
            inputs.turnAngle());
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            inputs.drivePositionMeters(),
            inputs.turnAngle());
    }

    public SwerveModuleState getLastDesiredState() {
        return new SwerveModuleState(
            lastDesiredState.speedMetersPerSecond,
            lastDesiredState.angle);
    }

    public double getRawAbsolutePositionRotations() {
        return inputs.rawAbsolutePositionRotations();
    }

    public void stop() {
        lastDesiredState = new SwerveModuleState(0.0, lastDesiredState.angle);
        io.stop();
    }

    @Override
    public void close() {
        io.close();
    }
}
