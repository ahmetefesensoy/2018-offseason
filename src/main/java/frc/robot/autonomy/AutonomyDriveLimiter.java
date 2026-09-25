package frc.robot.autonomy;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.AutonomyConstants;

/** Vector-preserving translation and rotation slew limiter for autonomy motion. */
final class AutonomyDriveLimiter {
    private double vxMetersPerSecond;
    private double vyMetersPerSecond;
    private double omegaRadiansPerSecond;

    ChassisSpeeds calculate(AutonomyCommandFrame target) {
        double deltaX = target.vxMetersPerSecond() - vxMetersPerSecond;
        double deltaY = target.vyMetersPerSecond() - vyMetersPerSecond;
        double translationDelta = Math.hypot(deltaX, deltaY);
        double maxTranslationDelta = AutonomyConstants.MAX_TRANSLATION_ACCEL_MPS2
            * AutonomyConstants.CONTROL_PERIOD_SECONDS;
        if (translationDelta > maxTranslationDelta) {
            double scale = maxTranslationDelta / translationDelta;
            deltaX *= scale;
            deltaY *= scale;
        }

        double maxRotationDelta = AutonomyConstants.MAX_ROTATION_ACCEL_RAD_PER_SEC2
            * AutonomyConstants.CONTROL_PERIOD_SECONDS;
        double rotationDelta = clamp(
            target.omegaRadiansPerSecond() - omegaRadiansPerSecond,
            -maxRotationDelta,
            maxRotationDelta);

        vxMetersPerSecond += deltaX;
        vyMetersPerSecond += deltaY;
        omegaRadiansPerSecond += rotationDelta;
        return new ChassisSpeeds(
            vxMetersPerSecond,
            vyMetersPerSecond,
            omegaRadiansPerSecond);
    }

    void reset() {
        vxMetersPerSecond = 0.0;
        vyMetersPerSecond = 0.0;
        omegaRadiansPerSecond = 0.0;
    }

    private static double clamp(double value, double minimum, double maximum) {
        return Math.max(minimum, Math.min(maximum, value));
    }
}
