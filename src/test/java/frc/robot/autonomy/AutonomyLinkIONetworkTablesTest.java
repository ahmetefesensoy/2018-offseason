package frc.robot.autonomy;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import frc.robot.AutonomyConstants;
import java.util.Optional;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class AutonomyLinkIONetworkTablesTest {
    private NetworkTableInstance nt;
    private AutonomyLinkIONetworkTables link;

    @BeforeEach
    void setup() {
        nt = NetworkTableInstance.create();
        nt.startLocal();
        link = new AutonomyLinkIONetworkTables(nt);
    }

    @AfterEach
    void tearDown() {
        link.close();
        nt.close();
    }

    @Test
    void ignoresIncompleteFrameThenReturnsFullyCommittedFrameExactlyOnce() {
        try (CommandPublishers publishers = new CommandPublishers(nt)) {
            publishers.sequence.set(0);
            publishers.commitSequence.set(1);
            nt.flushLocal();
            assertTrue(link.readNewCommand().isEmpty());

            publishers.sessionId.set("boot-123");
            publishers.armed.set(true);
            publishers.sentAt.set(1_000_000);
            publishers.validUntil.set(1_100_000);
            publishers.vx.set(0.2);
            publishers.vy.set(-0.1);
            publishers.omega.set(0.3);
            publishers.sequence.set(2);
            publishers.commitSequence.set(2);
            nt.flushLocal();

            Optional<AutonomyCommandFrame> result = link.readNewCommand();
            assertTrue(result.isPresent());
            AutonomyCommandFrame frame = result.orElseThrow();
            assertEquals("boot-123", frame.sessionId());
            assertTrue(frame.armed());
            assertEquals(1_000_000L, frame.sentAtMicros());
            assertEquals(1_100_000L, frame.validUntilMicros());
            assertEquals(0.2, frame.vxMetersPerSecond(), 1e-9);
            assertEquals(-0.1, frame.vyMetersPerSecond(), 1e-9);
            assertEquals(0.3, frame.omegaRadiansPerSecond(), 1e-9);
            assertEquals(2L, frame.sequence());
            assertEquals(2L, frame.commitSequence());
            assertTrue(link.readNewCommand().isEmpty());
        }
    }

    @Test
    void publishesCompleteStatusSnapshot() {
        var session = nt.getStringTopic(path("status/session_id")).subscribe("");
        var mode = nt.getStringTopic(path("status/mode")).subscribe("");
        var rejectReason = nt.getStringTopic(path("status/reject_reason")).subscribe("");
        var acceptedSequence = nt.getIntegerTopic(path("status/accepted_sequence")).subscribe(-1);
        var robotTime = nt.getIntegerTopic(path("status/roborio_time_us")).subscribe(-1);
        var commandAge = nt.getIntegerTopic(path("status/command_age_us")).subscribe(-1);
        var active = nt.getBooleanTopic(path("status/command_active")).subscribe(false);
        var gyroHealthy = nt.getBooleanTopic(path("status/gyro_healthy")).subscribe(false);
        var drivetrainHealthy = nt.getBooleanTopic(path("status/drivetrain_healthy")).subscribe(false);
        var pose = nt.getDoubleArrayTopic(path("status/pose")).subscribe(new double[0]);
        var measured = nt.getDoubleArrayTopic(path("status/measured_chassis")).subscribe(new double[0]);

        try {
            link.publishStatus(new AutonomyStatus(
                "boot-xyz",
                "AUTONOMOUS",
                42,
                5_000_000,
                25_000,
                true,
                true,
                true,
                AutonomyRejectReason.NONE,
                new Pose2d(2.0, 3.0, Rotation2d.fromDegrees(90.0)),
                new ChassisSpeeds(0.4, -0.2, 0.1)));
            nt.flushLocal();

            assertEquals("boot-xyz", session.get());
            assertEquals("AUTONOMOUS", mode.get());
            assertEquals("NONE", rejectReason.get());
            assertEquals(42L, acceptedSequence.get());
            assertEquals(5_000_000L, robotTime.get());
            assertEquals(25_000L, commandAge.get());
            assertTrue(active.get());
            assertTrue(gyroHealthy.get());
            assertTrue(drivetrainHealthy.get());
            assertArrayEquals(new double[] {2.0, 3.0, Math.PI / 2.0}, pose.get(), 1e-9);
            assertArrayEquals(new double[] {0.4, -0.2, 0.1}, measured.get(), 1e-9);
        } finally {
            session.close();
            mode.close();
            rejectReason.close();
            acceptedSequence.close();
            robotTime.close();
            commandAge.close();
            active.close();
            gyroHealthy.close();
            drivetrainHealthy.close();
            pose.close();
            measured.close();
        }
    }

    private static String path(String suffix) {
        return AutonomyConstants.NT_ROOT + "/" + suffix;
    }

    private static final class CommandPublishers implements AutoCloseable {
        final StringPublisher sessionId;
        final BooleanPublisher armed;
        final IntegerPublisher sentAt;
        final IntegerPublisher validUntil;
        final DoublePublisher vx;
        final DoublePublisher vy;
        final DoublePublisher omega;
        final IntegerPublisher sequence;
        final IntegerPublisher commitSequence;

        CommandPublishers(NetworkTableInstance nt) {
            sessionId = nt.getStringTopic(path("command/session_id")).publish();
            armed = nt.getBooleanTopic(path("command/armed")).publish();
            sentAt = nt.getIntegerTopic(path("command/sent_at_us")).publish();
            validUntil = nt.getIntegerTopic(path("command/valid_until_us")).publish();
            vx = nt.getDoubleTopic(path("command/vx_mps")).publish();
            vy = nt.getDoubleTopic(path("command/vy_mps")).publish();
            omega = nt.getDoubleTopic(path("command/omega_radps")).publish();
            sequence = nt.getIntegerTopic(path("command/sequence")).publish();
            commitSequence = nt.getIntegerTopic(path("command/commit_sequence")).publish();
        }

        @Override
        public void close() {
            sessionId.close();
            armed.close();
            sentAt.close();
            validUntil.close();
            vx.close();
            vy.close();
            omega.close();
            sequence.close();
            commitSequence.close();
        }
    }
}
