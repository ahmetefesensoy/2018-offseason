package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;

/** Hardware map and conservative bring-up values for the four-module swerve. */
public final class SwerveConstants {
    private SwerveConstants() {}

    public record ModuleConfig(
        String name,
        int driveCanId,
        int turnCanId,
        boolean driveInverted,
        boolean turnInverted,
        boolean absoluteEncoderInverted,
        double absoluteOffsetRadians) {}

    public static final double WHEEL_DIAMETER_METERS = 0.1016;
    public static final double DRIVE_REDUCTION = 6.75;
    public static final double WHEEL_BASE_METERS = 0.60;
    public static final double TRACK_WIDTH_METERS = 0.60;

    public static final double DRIVE_POSITION_FACTOR_METERS =
        Math.PI * WHEEL_DIAMETER_METERS / DRIVE_REDUCTION;
    public static final double DRIVE_VELOCITY_FACTOR_MPS =
        DRIVE_POSITION_FACTOR_METERS / 60.0;
    public static final double TURN_POSITION_FACTOR_RADIANS = 2.0 * Math.PI;
    public static final double TURN_VELOCITY_FACTOR_RAD_PER_SEC = 2.0 * Math.PI / 60.0;

    public static final double MAX_SPEED_MPS = 1.0;
    public static final double MAX_ANGULAR_SPEED_RAD_PER_SEC = 2.0;
    public static final double LOOP_PERIOD_SECONDS = 0.02;
    public static final double NOMINAL_VOLTAGE = 12.0;

    public static final int DRIVE_CURRENT_LIMIT_AMPS = 50;
    public static final int TURN_CURRENT_LIMIT_AMPS = 30;

    public static final double DRIVE_KP = 0.5;
    public static final double DRIVE_KI = 0.0;
    public static final double DRIVE_KD = 0.0;
    public static final double DRIVE_KS_VOLTS = 0.20;
    public static final double DRIVE_KV_VOLT_SECONDS_PER_METER = 2.4;
    public static final double DRIVE_KA_VOLT_SECONDS_SQUARED_PER_METER = 0.0;

    public static final double TURN_KP = 4.0;
    public static final double TURN_KI = 0.0;
    public static final double TURN_KD = 0.0;
    public static final double TURN_MAX_VELOCITY_RAD_PER_SEC = 8.0;
    public static final double TURN_MAX_ACCELERATION_RAD_PER_SEC_SQUARED = 20.0;

    public static final Translation2d FRONT_LEFT_LOCATION = new Translation2d(
        WHEEL_BASE_METERS / 2.0,
        TRACK_WIDTH_METERS / 2.0);
    public static final Translation2d FRONT_RIGHT_LOCATION = new Translation2d(
        WHEEL_BASE_METERS / 2.0,
        -TRACK_WIDTH_METERS / 2.0);
    public static final Translation2d BACK_LEFT_LOCATION = new Translation2d(
        -WHEEL_BASE_METERS / 2.0,
        TRACK_WIDTH_METERS / 2.0);
    public static final Translation2d BACK_RIGHT_LOCATION = new Translation2d(
        -WHEEL_BASE_METERS / 2.0,
        -TRACK_WIDTH_METERS / 2.0);

    public static final List<ModuleConfig> MODULE_CONFIGS = List.of(
        new ModuleConfig("FL", 10, 11, false, false, false, 0.0),
        new ModuleConfig("FR", 12, 13, false, false, false, 0.0),
        new ModuleConfig("BL", 14, 15, false, false, false, 0.0),
        new ModuleConfig("BR", 16, 17, false, false, false, 0.0));
}
