package frc.robot.autonomy;

import java.util.Objects;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.LongSupplier;
import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Polls commands, applies roboRIO safety authority, and publishes feedback. */
public final class AutonomyController extends SubsystemBase implements AutonomyCommandSource {
    private final AutonomyLinkIO link;
    private final AutonomySafetyGate gate;
    private final String sessionId;
    private final LongSupplier clockMicros;
    private final BooleanSupplier enabledSupplier;
    private final BooleanSupplier autonomyAllowedSupplier;
    private final Supplier<String> modeSupplier;
    private final Supplier<AutonomyRobotState> robotStateSupplier;

    private AutonomyCommandFrame activeCommand;
    private AutonomyCommandFrame lastSeenCommand;
    private AutonomyDecision lastDecision =
        AutonomyDecision.reject(AutonomyRejectReason.NO_FRAME, null);

    public AutonomyController(
            AutonomyLinkIO link,
            AutonomySafetyGate gate,
            String sessionId,
            LongSupplier clockMicros,
            BooleanSupplier enabledSupplier,
            BooleanSupplier autonomyAllowedSupplier,
            Supplier<String> modeSupplier,
            Supplier<AutonomyRobotState> robotStateSupplier) {
        this.link = Objects.requireNonNull(link);
        this.gate = Objects.requireNonNull(gate);
        this.sessionId = Objects.requireNonNull(sessionId);
        this.clockMicros = Objects.requireNonNull(clockMicros);
        this.enabledSupplier = Objects.requireNonNull(enabledSupplier);
        this.autonomyAllowedSupplier = Objects.requireNonNull(autonomyAllowedSupplier);
        this.modeSupplier = Objects.requireNonNull(modeSupplier);
        this.robotStateSupplier = Objects.requireNonNull(robotStateSupplier);
    }

    public Optional<AutonomyCommandFrame> update() {
        long nowMicros = clockMicros.getAsLong();
        AutonomyRobotState robotState = Objects.requireNonNull(robotStateSupplier.get());
        AutonomySafetyContext context = context(nowMicros, robotState);

        Optional<AutonomyCommandFrame> received = link.readNewCommand();
        if (received.isPresent()) {
            lastSeenCommand = received.orElseThrow();
            lastDecision = gate.evaluate(lastSeenCommand, context);
            activeCommand = lastDecision.accepted() ? lastSeenCommand : null;
        }

        if (activeCommand != null) {
            AutonomyRejectReason runtimeFailure = runtimeFailure(context, activeCommand);
            if (runtimeFailure != AutonomyRejectReason.NONE) {
                lastDecision = AutonomyDecision.reject(runtimeFailure, activeCommand);
                activeCommand = null;
            }
        }

        publishStatus(nowMicros, robotState);
        return Optional.ofNullable(activeCommand);
    }

    @Override
    public void periodic() {
        update();
    }

    @Override
    public Optional<AutonomyCommandFrame> currentCommand() {
        return Optional.ofNullable(activeCommand);
    }

    @Override
    public void cancel() {
        AutonomyCommandFrame cancelled = activeCommand;
        activeCommand = null;
        lastDecision = AutonomyDecision.reject(AutonomyRejectReason.DISARMED, cancelled);
    }

    public Optional<AutonomyCommandFrame> activeCommand() {
        return Optional.ofNullable(activeCommand);
    }

    public AutonomyDecision lastDecision() {
        return lastDecision;
    }

    public String sessionId() {
        return sessionId;
    }

    private AutonomySafetyContext context(long nowMicros, AutonomyRobotState robotState) {
        return new AutonomySafetyContext(
            nowMicros,
            enabledSupplier.getAsBoolean(),
            autonomyAllowedSupplier.getAsBoolean(),
            robotState.gyroHealthy(),
            robotState.drivetrainHealthy(),
            sessionId);
    }

    private AutonomyRejectReason runtimeFailure(
            AutonomySafetyContext context,
            AutonomyCommandFrame command) {
        if (!context.enabled()) {
            return AutonomyRejectReason.ROBOT_DISABLED;
        }
        if (!context.autonomyAllowed()) {
            return AutonomyRejectReason.WRONG_MODE;
        }
        if (!context.gyroHealthy()) {
            return AutonomyRejectReason.GYRO_UNHEALTHY;
        }
        if (!context.drivetrainHealthy()) {
            return AutonomyRejectReason.DRIVETRAIN_UNHEALTHY;
        }
        if (context.nowMicros() > command.validUntilMicros()) {
            return AutonomyRejectReason.EXPIRED_COMMAND;
        }
        if (gate.isWatchdogExpired(context.nowMicros())) {
            return AutonomyRejectReason.WATCHDOG_EXPIRED;
        }
        return AutonomyRejectReason.NONE;
    }

    private void publishStatus(long nowMicros, AutonomyRobotState robotState) {
        long commandAgeMicros = lastSeenCommand == null
            ? -1L
            : nowMicros - lastSeenCommand.sentAtMicros();
        link.publishStatus(new AutonomyStatus(
            sessionId,
            modeSupplier.get(),
            gate.lastAcceptedSequence(),
            nowMicros,
            commandAgeMicros,
            activeCommand != null,
            robotState.gyroHealthy(),
            robotState.drivetrainHealthy(),
            lastDecision.reason(),
            robotState.pose(),
            robotState.measuredChassisSpeeds()));
    }
}
