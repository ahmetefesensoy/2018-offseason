package frc.robot.autonomy;

import java.util.Optional;

/** Supplies a currently safe velocity request to the drivetrain command. */
public interface AutonomyCommandSource {
    Optional<AutonomyCommandFrame> currentCommand();

    void cancel();
}
