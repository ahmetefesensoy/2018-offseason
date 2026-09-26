package frc.robot.autonomy;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringSubscriber;
import frc.robot.AutonomyConstants;
import java.util.Objects;
import java.util.Optional;

/** NT4 implementation using a final commit sequence as the frame snapshot barrier. */
public final class AutonomyLinkIONetworkTables implements AutonomyLinkIO {
    private static final PubSubOption PERIODIC =
        PubSubOption.periodic(AutonomyConstants.NT_PERIOD_SECONDS);
    private static final PubSubOption KEEP_DUPLICATES = PubSubOption.keepDuplicates(true);

    private final StringSubscriber sessionIdSubscriber;
    private final BooleanSubscriber armedSubscriber;
    private final IntegerSubscriber sentAtSubscriber;
    private final IntegerSubscriber validUntilSubscriber;
    private final DoubleSubscriber vxSubscriber;
    private final DoubleSubscriber vySubscriber;
    private final DoubleSubscriber omegaSubscriber;
    private final BooleanSubscriber mechanismEnabledSubscriber;
    private final IntegerSubscriber intakeActionSubscriber;
    private final DoubleSubscriber elevatorTargetSubscriber;
    private final IntegerSubscriber sequenceSubscriber;
    private final IntegerSubscriber commitSequenceSubscriber;

    private final StringPublisher statusSessionIdPublisher;
    private final StringPublisher statusModePublisher;
    private final StringPublisher statusRejectReasonPublisher;
    private final IntegerPublisher statusAcceptedSequencePublisher;
    private final IntegerPublisher statusRobotTimePublisher;
    private final IntegerPublisher statusCommandAgePublisher;
    private final BooleanPublisher statusCommandActivePublisher;
    private final BooleanPublisher statusGyroHealthyPublisher;
    private final BooleanPublisher statusDrivetrainHealthyPublisher;
    private final DoubleArrayPublisher statusPosePublisher;
    private final DoubleArrayPublisher statusMeasuredChassisPublisher;

    private long lastObservedCommitSequence = -1L;

    public AutonomyLinkIONetworkTables(NetworkTableInstance instance) {
        Objects.requireNonNull(instance, "instance");

        sessionIdSubscriber = instance.getStringTopic(path("command/session_id"))
            .subscribe("", PERIODIC, KEEP_DUPLICATES);
        armedSubscriber = instance.getBooleanTopic(path("command/armed"))
            .subscribe(false, PERIODIC, KEEP_DUPLICATES);
        sentAtSubscriber = instance.getIntegerTopic(path("command/sent_at_us"))
            .subscribe(0L, PERIODIC, KEEP_DUPLICATES);
        validUntilSubscriber = instance.getIntegerTopic(path("command/valid_until_us"))
            .subscribe(0L, PERIODIC, KEEP_DUPLICATES);
        vxSubscriber = instance.getDoubleTopic(path("command/vx_mps"))
            .subscribe(0.0, PERIODIC, KEEP_DUPLICATES);
        vySubscriber = instance.getDoubleTopic(path("command/vy_mps"))
            .subscribe(0.0, PERIODIC, KEEP_DUPLICATES);
        omegaSubscriber = instance.getDoubleTopic(path("command/omega_radps"))
            .subscribe(0.0, PERIODIC, KEEP_DUPLICATES);
        mechanismEnabledSubscriber = instance.getBooleanTopic(path("command/mechanism_enabled"))
            .subscribe(false, PERIODIC, KEEP_DUPLICATES);
        intakeActionSubscriber = instance.getIntegerTopic(path("command/intake_action"))
            .subscribe(0L, PERIODIC, KEEP_DUPLICATES);
        elevatorTargetSubscriber = instance.getDoubleTopic(path("command/elevator_target_m"))
            .subscribe(0.0, PERIODIC, KEEP_DUPLICATES);
        sequenceSubscriber = instance.getIntegerTopic(path("command/sequence"))
            .subscribe(0L, PERIODIC, KEEP_DUPLICATES);
        commitSequenceSubscriber = instance.getIntegerTopic(path("command/commit_sequence"))
            .subscribe(0L, PERIODIC, KEEP_DUPLICATES);

        statusSessionIdPublisher = instance.getStringTopic(path("status/session_id")).publish(PERIODIC);
        statusModePublisher = instance.getStringTopic(path("status/mode")).publish(PERIODIC);
        statusRejectReasonPublisher = instance.getStringTopic(path("status/reject_reason")).publish(PERIODIC);
        statusAcceptedSequencePublisher = instance.getIntegerTopic(path("status/accepted_sequence")).publish(PERIODIC);
        statusRobotTimePublisher = instance.getIntegerTopic(path("status/roborio_time_us")).publish(PERIODIC);
        statusCommandAgePublisher = instance.getIntegerTopic(path("status/command_age_us")).publish(PERIODIC);
        statusCommandActivePublisher = instance.getBooleanTopic(path("status/command_active")).publish(PERIODIC);
        statusGyroHealthyPublisher = instance.getBooleanTopic(path("status/gyro_healthy")).publish(PERIODIC);
        statusDrivetrainHealthyPublisher = instance.getBooleanTopic(path("status/drivetrain_healthy")).publish(PERIODIC);
        statusPosePublisher = instance.getDoubleArrayTopic(path("status/pose")).publish(PERIODIC);
        statusMeasuredChassisPublisher = instance.getDoubleArrayTopic(path("status/measured_chassis")).publish(PERIODIC);
    }

    @Override
    public Optional<AutonomyCommandFrame> readNewCommand() {
        long commitSequence = commitSequenceSubscriber.get();
        if (commitSequence <= 0 || commitSequence == lastObservedCommitSequence) {
            return Optional.empty();
        }

        lastObservedCommitSequence = commitSequence;
        long sequence = sequenceSubscriber.get();
        if (sequence != commitSequence) {
            return Optional.empty();
        }

        return Optional.of(new AutonomyCommandFrame(
            sessionIdSubscriber.get(),
            armedSubscriber.get(),
            sentAtSubscriber.get(),
            validUntilSubscriber.get(),
            vxSubscriber.get(),
            vySubscriber.get(),
            omegaSubscriber.get(),
            sequence,
            commitSequence,
            mechanismEnabledSubscriber.get(),
            AutonomyIntakeAction.fromCode(intakeActionSubscriber.get()),
            elevatorTargetSubscriber.get()));
    }

    @Override
    public void publishStatus(AutonomyStatus status) {
        Objects.requireNonNull(status, "status");
        statusSessionIdPublisher.set(status.sessionId());
        statusModePublisher.set(status.mode());
        statusRejectReasonPublisher.set(status.rejectReason().name());
        statusAcceptedSequencePublisher.set(status.acceptedSequence());
        statusRobotTimePublisher.set(status.robotTimeMicros());
        statusCommandAgePublisher.set(status.commandAgeMicros());
        statusCommandActivePublisher.set(status.commandActive());
        statusGyroHealthyPublisher.set(status.gyroHealthy());
        statusDrivetrainHealthyPublisher.set(status.drivetrainHealthy());
        statusPosePublisher.set(new double[] {
            status.pose().getX(),
            status.pose().getY(),
            status.pose().getRotation().getRadians()
        });
        statusMeasuredChassisPublisher.set(new double[] {
            status.measuredChassisSpeeds().vxMetersPerSecond,
            status.measuredChassisSpeeds().vyMetersPerSecond,
            status.measuredChassisSpeeds().omegaRadiansPerSecond
        });
    }

    private static String path(String suffix) {
        return AutonomyConstants.NT_ROOT + "/" + suffix;
    }

    @Override
    public void close() {
        sessionIdSubscriber.close();
        armedSubscriber.close();
        sentAtSubscriber.close();
        validUntilSubscriber.close();
        vxSubscriber.close();
        vySubscriber.close();
        omegaSubscriber.close();
        mechanismEnabledSubscriber.close();
        intakeActionSubscriber.close();
        elevatorTargetSubscriber.close();
        sequenceSubscriber.close();
        commitSequenceSubscriber.close();

        statusSessionIdPublisher.close();
        statusModePublisher.close();
        statusRejectReasonPublisher.close();
        statusAcceptedSequencePublisher.close();
        statusRobotTimePublisher.close();
        statusCommandAgePublisher.close();
        statusCommandActivePublisher.close();
        statusGyroHealthyPublisher.close();
        statusDrivetrainHealthyPublisher.close();
        statusPosePublisher.close();
        statusMeasuredChassisPublisher.close();
    }
}
