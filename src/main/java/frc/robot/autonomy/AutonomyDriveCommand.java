package frc.robot.autonomy;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.SwerveDriveSubsystem;
import java.util.Objects;
import java.util.Optional;

/** Sole command that may turn an accepted Jetson request into swerve motion. */
public final class AutonomyDriveCommand extends Command {
    private final SwerveDriveSubsystem drivetrain;
    private final AutonomyCommandSource commandSource;
    private final AutonomyDriveLimiter limiter = new AutonomyDriveLimiter();

    public AutonomyDriveCommand(
            SwerveDriveSubsystem drivetrain,
            AutonomyCommandSource commandSource) {
        this.drivetrain = Objects.requireNonNull(drivetrain);
        this.commandSource = Objects.requireNonNull(commandSource);
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        Optional<AutonomyCommandFrame> frame = commandSource.currentCommand();
        if (frame.isEmpty()) {
            limiter.reset();
            drivetrain.stop();
            return;
        }

        AutonomyCommandFrame command = frame.orElseThrow();
        ChassisSpeeds limited = limiter.calculate(command);
        drivetrain.drive(
            limited.vxMetersPerSecond,
            limited.vyMetersPerSecond,
            limited.omegaRadiansPerSecond,
            false);
    }

    @Override
    public void end(boolean interrupted) {
        commandSource.cancel();
        limiter.reset();
        drivetrain.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
