package frc.robot.subsystems.swerve;

import com.studica.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;

/** NavX connected to the roboRIO MXP SPI bus at a 100 Hz update rate. */
public final class GyroIONavX implements GyroIO {
    private final AHRS navX;

    public GyroIONavX() {
        navX = new AHRS(AHRS.NavXComType.kMXP_SPI, 100);
    }

    static Rotation2d toHeading(double navXYawDegrees) {
        return Rotation2d.fromDegrees(-navXYawDegrees);
    }

    @Override
    public Rotation2d getHeading() {
        return toHeading(navX.getYaw());
    }

    @Override
    public boolean isConnected() {
        return navX.isConnected();
    }

    @Override
    public boolean isCalibrating() {
        return navX.isCalibrating();
    }

    @Override
    public void zeroYaw() {
        navX.zeroYaw();
    }

    @Override
    public void close() {
        navX.close();
    }
}
