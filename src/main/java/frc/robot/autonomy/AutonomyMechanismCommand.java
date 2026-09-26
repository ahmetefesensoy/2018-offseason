package frc.robot.autonomy;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import java.util.Objects;
import java.util.Optional;

/** Applies mechanism intent only while the same accepted Jetson frame is live. */
public final class AutonomyMechanismCommand extends Command {
    interface Output {
        void apply(AutonomyIntakeAction action, double elevatorTargetMeters);
        void stop();
    }

    private final Output output;
    private final AutonomyCommandSource commandSource;

    public AutonomyMechanismCommand(
            ElevatorSubsystem elevator,
            IntakeSubsystem intake,
            AutonomyCommandSource commandSource) {
        this(new RobotOutput(elevator, intake), commandSource);
        addRequirements(elevator, intake);
    }

    AutonomyMechanismCommand(Output output, AutonomyCommandSource commandSource) {
        this.output = Objects.requireNonNull(output);
        this.commandSource = Objects.requireNonNull(commandSource);
    }

    @Override
    public void execute() {
        Optional<AutonomyCommandFrame> frame = commandSource.currentCommand();
        if (frame.isEmpty() || !frame.orElseThrow().mechanismEnabled()) {
            output.stop();
            return;
        }
        AutonomyCommandFrame command = frame.orElseThrow();
        output.apply(command.intakeAction(), command.elevatorTargetMeters());
    }

    @Override
    public void end(boolean interrupted) {
        output.stop();
        commandSource.cancel();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    private static final class RobotOutput implements Output {
        private final ElevatorSubsystem elevator;
        private final IntakeSubsystem intake;

        RobotOutput(ElevatorSubsystem elevator, IntakeSubsystem intake) {
            this.elevator = Objects.requireNonNull(elevator);
            this.intake = Objects.requireNonNull(intake);
        }

        @Override
        public void apply(AutonomyIntakeAction action, double elevatorTargetMeters) {
            elevator.setTargetHeight(elevatorTargetMeters);
            switch (action) {
                case INTAKE -> intake.autoIntake();
                case HOLD -> intake.close();
                case EJECT -> intake.open();
                case STOP -> intake.stop();
            }
        }

        @Override
        public void stop() {
            intake.stop();
            elevator.stop();
        }
    }
}
