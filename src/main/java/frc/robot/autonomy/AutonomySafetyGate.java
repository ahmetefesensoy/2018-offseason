package frc.robot.autonomy;

import frc.robot.AutonomyConstants;
import frc.robot.Constants;
import java.util.Objects;

/** Stateful, fail-closed validation boundary between the Jetson and motor control. */
public final class AutonomySafetyGate {
    private long lastAcceptedSequence = -1L;
    private long lastAcceptedAtMicros;
    private boolean hasAcceptedCommand;

    public AutonomyDecision evaluate(
            AutonomyCommandFrame frame,
            AutonomySafetyContext context) {
        Objects.requireNonNull(context, "context");
        if (frame == null) {
            return reject(AutonomyRejectReason.NO_FRAME, null);
        }
        if (frame.sequence() <= 0 || frame.commitSequence() != frame.sequence()) {
            return reject(AutonomyRejectReason.NOT_COMMITTED, frame);
        }
        if (!frame.armed()) {
            return reject(AutonomyRejectReason.DISARMED, frame);
        }
        if (!context.enabled()) {
            return reject(AutonomyRejectReason.ROBOT_DISABLED, frame);
        }
        if (!context.autonomyAllowed()) {
            return reject(AutonomyRejectReason.WRONG_MODE, frame);
        }
        if (frame.sessionId() == null
                || context.expectedSessionId() == null
                || !context.expectedSessionId().equals(frame.sessionId())) {
            return reject(AutonomyRejectReason.SESSION_MISMATCH, frame);
        }
        if (frame.sequence() <= lastAcceptedSequence) {
            return reject(AutonomyRejectReason.NON_MONOTONIC_SEQUENCE, frame);
        }

        long commandAgeMicros = context.nowMicros() - frame.sentAtMicros();
        if (commandAgeMicros < -AutonomyConstants.MAX_FUTURE_SKEW_US) {
            return reject(AutonomyRejectReason.INVALID_TIMESTAMP, frame);
        }
        if (commandAgeMicros > AutonomyConstants.MAX_COMMAND_AGE_US) {
            return reject(AutonomyRejectReason.STALE_COMMAND, frame);
        }
        if (frame.validUntilMicros() < context.nowMicros()) {
            return reject(AutonomyRejectReason.EXPIRED_COMMAND, frame);
        }
        long validityHorizonMicros = frame.validUntilMicros() - frame.sentAtMicros();
        if (validityHorizonMicros < 0) {
            return reject(AutonomyRejectReason.INVALID_TIMESTAMP, frame);
        }
        if (validityHorizonMicros > AutonomyConstants.MAX_VALIDITY_HORIZON_US) {
            return reject(AutonomyRejectReason.VALIDITY_TOO_LONG, frame);
        }

        if (!Double.isFinite(frame.vxMetersPerSecond())
                || !Double.isFinite(frame.vyMetersPerSecond())
                || !Double.isFinite(frame.omegaRadiansPerSecond())) {
            return reject(AutonomyRejectReason.NON_FINITE_COMMAND, frame);
        }
        if (Math.hypot(frame.vxMetersPerSecond(), frame.vyMetersPerSecond())
                    > AutonomyConstants.MAX_TRANSLATION_SPEED_MPS
                || Math.abs(frame.omegaRadiansPerSecond())
                    > AutonomyConstants.MAX_ROTATION_SPEED_RAD_PER_SEC) {
            return reject(AutonomyRejectReason.SPEED_LIMIT, frame);
        }
        if (frame.intakeAction() == null
                || !Double.isFinite(frame.elevatorTargetMeters())
                || frame.elevatorTargetMeters() < Constants.ELEVATOR_GROUND_METERS
                || frame.elevatorTargetMeters() > Constants.ELEVATOR_SCALE_METERS
                || (!frame.mechanismEnabled()
                    && (frame.intakeAction() != AutonomyIntakeAction.STOP
                        || frame.elevatorTargetMeters() != Constants.ELEVATOR_GROUND_METERS))) {
            return reject(AutonomyRejectReason.INVALID_MECHANISM_COMMAND, frame);
        }
        if (!context.gyroHealthy()) {
            return reject(AutonomyRejectReason.GYRO_UNHEALTHY, frame);
        }
        if (!context.drivetrainHealthy()) {
            return reject(AutonomyRejectReason.DRIVETRAIN_UNHEALTHY, frame);
        }

        lastAcceptedSequence = frame.sequence();
        lastAcceptedAtMicros = context.nowMicros();
        hasAcceptedCommand = true;
        return AutonomyDecision.accept(frame);
    }

    public boolean isWatchdogExpired(long nowMicros) {
        return !hasAcceptedCommand
            || nowMicros - lastAcceptedAtMicros >= AutonomyConstants.WATCHDOG_TIMEOUT_US;
    }

    public long lastAcceptedSequence() {
        return lastAcceptedSequence;
    }

    public long lastAcceptedAtMicros() {
        return lastAcceptedAtMicros;
    }

    private static AutonomyDecision reject(
            AutonomyRejectReason reason,
            AutonomyCommandFrame frame) {
        return AutonomyDecision.reject(reason, frame);
    }
}
