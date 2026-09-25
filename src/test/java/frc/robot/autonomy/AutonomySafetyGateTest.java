package frc.robot.autonomy;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.AutonomyConstants;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class AutonomySafetyGateTest {
    private static final long NOW_US = 1_000_000L;
    private static final String SESSION = "boot-session";

    private AutonomySafetyGate gate;

    @BeforeEach
    void setup() {
        gate = new AutonomySafetyGate();
    }

    @Test
    void acceptsFreshCommittedCommandWhenEveryInterlockIsHealthy() {
        AutonomyDecision decision = gate.evaluate(validFrame(1), healthyContext());

        assertTrue(decision.accepted());
        assertEquals(AutonomyRejectReason.NONE, decision.reason());
        assertEquals(1L, gate.lastAcceptedSequence());
        assertFalse(gate.isWatchdogExpired(NOW_US + AutonomyConstants.WATCHDOG_TIMEOUT_US - 1));
    }

    @Test
    void rejectsUncommittedSnapshot() {
        assertRejected(
            withCommit(validFrame(1), 0),
            healthyContext(),
            AutonomyRejectReason.NOT_COMMITTED);
    }

    @Test
    void rejectsDisabledDisarmedAndWrongModeCommands() {
        assertRejected(validFrame(1), withEnabled(healthyContext(), false), AutonomyRejectReason.ROBOT_DISABLED);
        assertRejected(withArmed(validFrame(1), false), healthyContext(), AutonomyRejectReason.DISARMED);
        assertRejected(validFrame(1), withAutonomyAllowed(healthyContext(), false), AutonomyRejectReason.WRONG_MODE);
    }

    @Test
    void rejectsWrongBootSessionAndNonIncreasingSequence() {
        assertRejected(withSession(validFrame(1), "old-boot"), healthyContext(), AutonomyRejectReason.SESSION_MISMATCH);

        assertTrue(gate.evaluate(validFrame(4), healthyContext()).accepted());
        assertRejected(validFrame(4), healthyContext(), AutonomyRejectReason.NON_MONOTONIC_SEQUENCE);
        assertRejected(validFrame(3), healthyContext(), AutonomyRejectReason.NON_MONOTONIC_SEQUENCE);
    }

    @Test
    void rejectsStaleFutureExpiredAndOverlongValidityWindows() {
        assertRejected(
            withTimes(validFrame(1), NOW_US - AutonomyConstants.MAX_COMMAND_AGE_US - 1, NOW_US + 1),
            healthyContext(),
            AutonomyRejectReason.STALE_COMMAND);
        assertRejected(
            withTimes(validFrame(1), NOW_US + AutonomyConstants.MAX_FUTURE_SKEW_US + 1, NOW_US + 100_000),
            healthyContext(),
            AutonomyRejectReason.INVALID_TIMESTAMP);
        assertRejected(
            withTimes(validFrame(1), NOW_US - 50_000, NOW_US - 1),
            healthyContext(),
            AutonomyRejectReason.EXPIRED_COMMAND);
        assertRejected(
            withTimes(
                validFrame(1),
                NOW_US - 1,
                NOW_US - 1 + AutonomyConstants.MAX_VALIDITY_HORIZON_US + 1),
            healthyContext(),
            AutonomyRejectReason.VALIDITY_TOO_LONG);
    }

    @Test
    void rejectsNonFiniteAndOutOfRangeVelocity() {
        assertRejected(
            withVelocity(validFrame(1), Double.NaN, 0.0, 0.0),
            healthyContext(),
            AutonomyRejectReason.NON_FINITE_COMMAND);
        assertRejected(
            withVelocity(validFrame(1), AutonomyConstants.MAX_TRANSLATION_SPEED_MPS, 0.01, 0.0),
            healthyContext(),
            AutonomyRejectReason.SPEED_LIMIT);
        assertRejected(
            withVelocity(validFrame(1), 0.0, 0.0, AutonomyConstants.MAX_ROTATION_SPEED_RAD_PER_SEC + 0.01),
            healthyContext(),
            AutonomyRejectReason.SPEED_LIMIT);
    }

    @Test
    void rejectsUnhealthyGyroOrDrivetrain() {
        assertRejected(validFrame(1), withGyroHealthy(healthyContext(), false), AutonomyRejectReason.GYRO_UNHEALTHY);
        assertRejected(
            validFrame(1),
            withDrivetrainHealthy(healthyContext(), false),
            AutonomyRejectReason.DRIVETRAIN_UNHEALTHY);
    }

    @Test
    void watchdogIsExpiredUntilAcceptanceAndAtConfiguredDeadline() {
        assertTrue(gate.isWatchdogExpired(NOW_US));
        assertTrue(gate.evaluate(validFrame(1), healthyContext()).accepted());
        assertFalse(gate.isWatchdogExpired(NOW_US + AutonomyConstants.WATCHDOG_TIMEOUT_US - 1));
        assertTrue(gate.isWatchdogExpired(NOW_US + AutonomyConstants.WATCHDOG_TIMEOUT_US));
    }

    private void assertRejected(
            AutonomyCommandFrame frame,
            AutonomySafetyContext context,
            AutonomyRejectReason expectedReason) {
        AutonomyDecision decision = gate.evaluate(frame, context);
        assertFalse(decision.accepted());
        assertEquals(expectedReason, decision.reason());
    }

    private static AutonomyCommandFrame validFrame(long sequence) {
        return new AutonomyCommandFrame(
            SESSION,
            true,
            NOW_US - 10_000,
            NOW_US + 80_000,
            0.25,
            -0.10,
            0.4,
            sequence,
            sequence);
    }

    private static AutonomySafetyContext healthyContext() {
        return new AutonomySafetyContext(NOW_US, true, true, true, true, SESSION);
    }

    private static AutonomyCommandFrame withCommit(AutonomyCommandFrame frame, long commit) {
        return new AutonomyCommandFrame(
            frame.sessionId(), frame.armed(), frame.sentAtMicros(), frame.validUntilMicros(),
            frame.vxMetersPerSecond(), frame.vyMetersPerSecond(), frame.omegaRadiansPerSecond(),
            frame.sequence(), commit);
    }

    private static AutonomyCommandFrame withArmed(AutonomyCommandFrame frame, boolean armed) {
        return new AutonomyCommandFrame(
            frame.sessionId(), armed, frame.sentAtMicros(), frame.validUntilMicros(),
            frame.vxMetersPerSecond(), frame.vyMetersPerSecond(), frame.omegaRadiansPerSecond(),
            frame.sequence(), frame.commitSequence());
    }

    private static AutonomyCommandFrame withSession(AutonomyCommandFrame frame, String session) {
        return new AutonomyCommandFrame(
            session, frame.armed(), frame.sentAtMicros(), frame.validUntilMicros(),
            frame.vxMetersPerSecond(), frame.vyMetersPerSecond(), frame.omegaRadiansPerSecond(),
            frame.sequence(), frame.commitSequence());
    }

    private static AutonomyCommandFrame withTimes(AutonomyCommandFrame frame, long sentAt, long validUntil) {
        return new AutonomyCommandFrame(
            frame.sessionId(), frame.armed(), sentAt, validUntil,
            frame.vxMetersPerSecond(), frame.vyMetersPerSecond(), frame.omegaRadiansPerSecond(),
            frame.sequence(), frame.commitSequence());
    }

    private static AutonomyCommandFrame withVelocity(
            AutonomyCommandFrame frame,
            double vx,
            double vy,
            double omega) {
        return new AutonomyCommandFrame(
            frame.sessionId(), frame.armed(), frame.sentAtMicros(), frame.validUntilMicros(),
            vx, vy, omega, frame.sequence(), frame.commitSequence());
    }

    private static AutonomySafetyContext withEnabled(AutonomySafetyContext context, boolean enabled) {
        return new AutonomySafetyContext(
            context.nowMicros(), enabled, context.autonomyAllowed(), context.gyroHealthy(),
            context.drivetrainHealthy(), context.expectedSessionId());
    }

    private static AutonomySafetyContext withAutonomyAllowed(AutonomySafetyContext context, boolean allowed) {
        return new AutonomySafetyContext(
            context.nowMicros(), context.enabled(), allowed, context.gyroHealthy(),
            context.drivetrainHealthy(), context.expectedSessionId());
    }

    private static AutonomySafetyContext withGyroHealthy(AutonomySafetyContext context, boolean healthy) {
        return new AutonomySafetyContext(
            context.nowMicros(), context.enabled(), context.autonomyAllowed(), healthy,
            context.drivetrainHealthy(), context.expectedSessionId());
    }

    private static AutonomySafetyContext withDrivetrainHealthy(AutonomySafetyContext context, boolean healthy) {
        return new AutonomySafetyContext(
            context.nowMicros(), context.enabled(), context.autonomyAllowed(), context.gyroHealthy(),
            healthy, context.expectedSessionId());
    }
}
