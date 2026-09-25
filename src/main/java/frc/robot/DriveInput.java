package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;

/** Pure joystick shaping for normalized swerve translation and rotation inputs. */
public final class DriveInput {
    private DriveInput() {}

    public static double squareWithSign(double value) {
        return Math.copySign(value * value, value);
    }

    public static double shapeAxis(double value, double deadband) {
        double clamped = MathUtil.clamp(value, -1.0, 1.0);
        return squareWithSign(MathUtil.applyDeadband(clamped, deadband));
    }

    public static Translation2d shapeTranslation(
            double xInput,
            double yInput,
            double deadband) {
        Translation2d raw = new Translation2d(xInput, yInput);
        double magnitude = Math.min(raw.getNorm(), 1.0);
        if (magnitude <= deadband) {
            return Translation2d.kZero;
        }

        double shapedMagnitude = squareWithSign(MathUtil.applyDeadband(magnitude, deadband));
        return new Translation2d(shapedMagnitude, raw.getAngle());
    }
}
