package frc.robot.autonomy;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Optional;
import org.junit.jupiter.api.Test;

class AutonomyMechanismCommandTest {
    @Test
    void appliesAcceptedMechanismIntentAndStopsWhenFrameDisappears() {
        StubSource source = new StubSource();
        FakeOutput output = new FakeOutput();
        AutonomyMechanismCommand command = new AutonomyMechanismCommand(output, source);
        source.frame = new AutonomyCommandFrame(
            "session", true, 1, 100_000, 0.0, 0.0, 0.0, 1, 1,
            true, AutonomyIntakeAction.EJECT, 0.55);

        command.execute();

        assertEquals(AutonomyIntakeAction.EJECT, output.action);
        assertEquals(0.55, output.elevatorTarget, 1e-9);

        source.frame = null;
        command.execute();
        command.end(true);

        assertTrue(output.stopped);
        assertTrue(source.cancelled);
    }

    private static final class FakeOutput implements AutonomyMechanismCommand.Output {
        AutonomyIntakeAction action = AutonomyIntakeAction.STOP;
        double elevatorTarget;
        boolean stopped;

        @Override
        public void apply(AutonomyIntakeAction action, double elevatorTargetMeters) {
            this.action = action;
            this.elevatorTarget = elevatorTargetMeters;
            this.stopped = false;
        }

        @Override
        public void stop() {
            stopped = true;
            action = AutonomyIntakeAction.STOP;
        }
    }

    private static final class StubSource implements AutonomyCommandSource {
        AutonomyCommandFrame frame;
        boolean cancelled;

        @Override
        public Optional<AutonomyCommandFrame> currentCommand() {
            return Optional.ofNullable(frame);
        }

        @Override
        public void cancel() {
            cancelled = true;
        }
    }
}
