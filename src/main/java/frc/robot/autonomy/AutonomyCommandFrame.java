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
        long commitSequence,
        boolean mechanismEnabled,
        AutonomyIntakeAction intakeAction,
        double elevatorTargetMeters) {

    public AutonomyCommandFrame(
            String sessionId,
            boolean armed,
            long sentAtMicros,
            long validUntilMicros,
            double vxMetersPerSecond,
            double vyMetersPerSecond,
            double omegaRadiansPerSecond,
            long sequence,
            long commitSequence) {
        this(
            sessionId, armed, sentAtMicros, validUntilMicros,
            vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond,
            sequence, commitSequence, false, AutonomyIntakeAction.STOP, 0.0);
    }
}
