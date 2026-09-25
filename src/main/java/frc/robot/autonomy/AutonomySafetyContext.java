package frc.robot.autonomy;

/** roboRIO-owned facts used to decide whether a Jetson command may move the robot. */
public record AutonomySafetyContext(
        long nowMicros,
        boolean enabled,
        boolean autonomyAllowed,
        boolean gyroHealthy,
        boolean drivetrainHealthy,
        String expectedSessionId) {}
