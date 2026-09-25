package frc.robot.autonomy;

/** One robot-relative chassis command received from the Jetson. */
public record AutonomyCommandFrame(
        String sessionId,
        boolean armed,
        long sentAtMicros,
        long validUntilMicros,
        double vxMetersPerSecond,
        double vyMetersPerSecond,
        double omegaRadiansPerSecond,
        long sequence,
        long commitSequence) {}
