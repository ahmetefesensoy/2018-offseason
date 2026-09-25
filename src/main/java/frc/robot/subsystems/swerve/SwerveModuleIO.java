package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;

/** Hardware boundary for one drive/turn swerve module. */
public interface SwerveModuleIO extends AutoCloseable {
    record Inputs(
        double drivePositionMeters,
        double driveVelocityMetersPerSecond,
        Rotation2d turnAngle,
        double rawAbsolutePositionRotations) {}

    Inputs readInputs();

    void setDriveVoltage(double volts);

    void setTurnVoltage(double volts);

    void stop();

    @Override
    default void close() {}
}
