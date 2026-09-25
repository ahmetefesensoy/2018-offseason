package frc.robot;

/** Conservative limits for the experimental Jetson autonomy link. */
public final class AutonomyConstants {
    public static final String NT_ROOT = "/frc/autonomy/v1";

    public static final double MAX_TRANSLATION_SPEED_MPS = 0.75;
    public static final double MAX_ROTATION_SPEED_RAD_PER_SEC = 1.5;

    public static final long MAX_COMMAND_AGE_US = 100_000L;
    public static final long MAX_FUTURE_SKEW_US = 20_000L;
    public static final long MAX_VALIDITY_HORIZON_US = 150_000L;
    public static final long WATCHDOG_TIMEOUT_US = 120_000L;

    public static final double NT_PERIOD_SECONDS = 0.02;

    private AutonomyConstants() {}
}
