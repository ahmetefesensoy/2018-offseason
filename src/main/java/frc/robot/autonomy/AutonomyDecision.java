package frc.robot.autonomy;

/** Result of the roboRIO safety gate. */
public record AutonomyDecision(
        boolean accepted,
        AutonomyRejectReason reason,
        AutonomyCommandFrame frame) {
    public static AutonomyDecision accept(AutonomyCommandFrame frame) {
        return new AutonomyDecision(true, AutonomyRejectReason.NONE, frame);
    }

    public static AutonomyDecision reject(
            AutonomyRejectReason reason,
            AutonomyCommandFrame frame) {
        return new AutonomyDecision(false, reason, frame);
    }
}
