package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;

/** Heading-sensor boundary used by the drivetrain and desktop tests. */
public interface GyroIO extends AutoCloseable {
    Rotation2d getHeading();

    boolean isConnected();

    boolean isCalibrating();

    void zeroYaw();

    @Override
    default void close() {}
}
