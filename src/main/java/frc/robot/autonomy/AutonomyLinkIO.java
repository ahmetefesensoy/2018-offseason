package frc.robot.autonomy;

import java.util.Optional;

/** Transport boundary for Jetson commands and roboRIO status. */
public interface AutonomyLinkIO extends AutoCloseable {
    Optional<AutonomyCommandFrame> readNewCommand();

    void publishStatus(AutonomyStatus status);

    @Override
    default void close() {}
}
