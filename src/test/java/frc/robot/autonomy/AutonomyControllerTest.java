package frc.robot.autonomy;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.AutonomyConstants;
import java.util.ArrayDeque;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicLong;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class AutonomyControllerTest {
    private static final String SESSION = "boot-test";

    private FakeLink link;
    private AtomicLong now;
    private AtomicBoolean enabled;
    private AtomicBoolean allowed;
    private AutonomyController controller;

    @BeforeEach
    void setup() {
        link = new FakeLink();
        now = new AtomicLong(1_000_000L);
        enabled = new AtomicBoolean(true);
        allowed = new AtomicBoolean(true);
        controller = new AutonomyController(
            link,
            new AutonomySafetyGate(),
            SESSION,
            now::get,
            enabled::get,
            allowed::get,
            () -> "AUTONOMOUS",
            () -> new AutonomyRobotState(Pose2d.kZero, new ChassisSpeeds(), true, true));
    }

    @Test
    void acceptedFrameBecomesActiveAndPublishesStatus() {
        link.commands.add(validFrame(1));

        Optional<AutonomyCommandFrame> command = controller.update();

        assertTrue(command.isPresent());
        assertEquals(0.25, command.orElseThrow().vxMetersPerSecond(), 1e-9);
        assertTrue(link.lastStatus.commandActive());
        assertEquals(1L, link.lastStatus.acceptedSequence());
        assertEquals(AutonomyRejectReason.NONE, link.lastStatus.rejectReason());
        assertEquals(10_000L, link.lastStatus.commandAgeMicros());
    }

    @Test
    void rejectedFrameClearsActiveCommandAndReportsReason() {
        link.commands.add(validFrame(1));
        assertTrue(controller.update().isPresent());

        link.commands.add(withSession(validFrame(2), "previous-boot"));
        assertTrue(controller.update().isEmpty());
        assertFalse(link.lastStatus.commandActive());
        assertEquals(AutonomyRejectReason.SESSION_MISMATCH, link.lastStatus.rejectReason());
    }

    @Test
    void repeatsLastAcceptedVelocityUntilWatchdogThenStops() {
        link.commands.add(validFrame(1));
        assertTrue(controller.update().isPresent());

        now.addAndGet(AutonomyConstants.WATCHDOG_TIMEOUT_US - 1);
        assertTrue(controller.update().isPresent());

        now.incrementAndGet();
        assertTrue(controller.update().isEmpty());
        assertEquals(AutonomyRejectReason.WATCHDOG_EXPIRED, link.lastStatus.rejectReason());
    }

    @Test
    void currentRobotModeIsRecheckedEvenWithoutANewFrame() {
        link.commands.add(validFrame(1));
        assertTrue(controller.update().isPresent());

        enabled.set(false);
        assertTrue(controller.update().isEmpty());
        assertEquals(AutonomyRejectReason.ROBOT_DISABLED, link.lastStatus.rejectReason());
    }

    @Test
    void cancelImmediatelyClearsLatchedCommand() {
        link.commands.add(validFrame(1));
        assertTrue(controller.update().isPresent());

        controller.cancel();

        assertTrue(controller.activeCommand().isEmpty());
        assertEquals(AutonomyRejectReason.DISARMED, controller.lastDecision().reason());
    }

    private AutonomyCommandFrame validFrame(long sequence) {
        return new AutonomyCommandFrame(
            SESSION,
            true,
            now.get() - 10_000,
            now.get() + AutonomyConstants.MAX_VALIDITY_HORIZON_US - 10_000,
            0.25,
            0.0,
            0.2,
            sequence,
            sequence);
    }

    private static AutonomyCommandFrame withSession(AutonomyCommandFrame frame, String session) {
        return new AutonomyCommandFrame(
            session, frame.armed(), frame.sentAtMicros(), frame.validUntilMicros(),
            frame.vxMetersPerSecond(), frame.vyMetersPerSecond(), frame.omegaRadiansPerSecond(),
            frame.sequence(), frame.commitSequence());
    }

    private static final class FakeLink implements AutonomyLinkIO {
        final ArrayDeque<AutonomyCommandFrame> commands = new ArrayDeque<>();
        AutonomyStatus lastStatus;

        @Override
        public Optional<AutonomyCommandFrame> readNewCommand() {
            return Optional.ofNullable(commands.poll());
        }

        @Override
        public void publishStatus(AutonomyStatus status) {
            lastStatus = status;
        }
    }
}
