package frc.robot.autonomy;

/** Stable reason codes published back to the Jetson and recorded in rosbag. */
public enum AutonomyRejectReason {
    NONE,
    NO_FRAME,
    NOT_COMMITTED,
    DISARMED,
    ROBOT_DISABLED,
    WRONG_MODE,
    SESSION_MISMATCH,
    NON_MONOTONIC_SEQUENCE,
    INVALID_TIMESTAMP,
    STALE_COMMAND,
    EXPIRED_COMMAND,
    VALIDITY_TOO_LONG,
    NON_FINITE_COMMAND,
    SPEED_LIMIT,
    GYRO_UNHEALTHY,
    DRIVETRAIN_UNHEALTHY,
    WATCHDOG_EXPIRED
}
