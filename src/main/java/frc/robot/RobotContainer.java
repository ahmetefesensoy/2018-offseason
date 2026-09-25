package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.autonomy.AutonomyCommandFrame;
import frc.robot.autonomy.AutonomyCommandSource;
import frc.robot.autonomy.AutonomyController;
import frc.robot.autonomy.AutonomyDriveCommand;
import frc.robot.autonomy.AutonomyLinkIONetworkTables;
import frc.robot.autonomy.AutonomyRobotState;
import frc.robot.autonomy.AutonomySafetyGate;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import java.util.Objects;
import java.util.Optional;
import java.util.UUID;

/** Owns every subsystem and wires joystick buttons to their commands. */
public class RobotContainer {
    private static final double DRIVE_DEADBAND = 0.08;

    private final Joystick joystick = new Joystick(Constants.JOYSTICK_PORT);

    private final ElevatorSubsystem elevator = new ElevatorSubsystem();
    private final IntakeSubsystem intake = new IntakeSubsystem();
    private final ClimbSubsystem climb = new ClimbSubsystem();
    private final SwerveDriveSubsystem drivetrain;
    private final AutonomyCommandSource autonomyCommandSource;
    private final Command autonomousCommand;

    public RobotContainer() {
        drivetrain = SwerveDriveSubsystem.createReal();
        autonomyCommandSource = createRealAutonomyController(drivetrain);
        autonomousCommand = new AutonomyDriveCommand(drivetrain, autonomyCommandSource);
        configureDriveCommand();
        configureButtonBindings();
    }

    RobotContainer(SwerveDriveSubsystem drivetrain) {
        this(drivetrain, new InactiveAutonomyCommandSource());
    }

    RobotContainer(
            SwerveDriveSubsystem drivetrain,
            AutonomyCommandSource autonomyCommandSource) {
        this.drivetrain = Objects.requireNonNull(drivetrain);
        this.autonomyCommandSource = Objects.requireNonNull(autonomyCommandSource);
        autonomousCommand = new AutonomyDriveCommand(drivetrain, autonomyCommandSource);
        configureDriveCommand();
        configureButtonBindings();
    }

    private static AutonomyController createRealAutonomyController(
            SwerveDriveSubsystem drivetrain) {
        return new AutonomyController(
            new AutonomyLinkIONetworkTables(NetworkTableInstance.getDefault()),
            new AutonomySafetyGate(),
            UUID.randomUUID().toString(),
            RobotController::getFPGATime,
            DriverStation::isEnabled,
            () -> DriverStation.isAutonomous() || DriverStation.isTest(),
            RobotContainer::currentModeName,
            () -> new AutonomyRobotState(
                drivetrain.getPose(),
                drivetrain.getMeasuredChassisSpeeds(),
                drivetrain.isGyroConnected() && !drivetrain.isGyroCalibrating(),
                drivetrain.isDrivetrainHealthy()));
    }

    private static String currentModeName() {
        if (!DriverStation.isEnabled()) {
            return "DISABLED";
        }
        if (DriverStation.isAutonomous()) {
            return "AUTONOMOUS";
        }
        if (DriverStation.isTest()) {
            return "TEST";
        }
        return "TELEOP";
    }

    private void configureDriveCommand() {
        drivetrain.setDefaultCommand(Commands.runEnd(() -> {
            Translation2d translation = DriveInput.shapeTranslation(
                -joystick.getRawAxis(1),
                -joystick.getRawAxis(0),
                DRIVE_DEADBAND);
            double rotation = DriveInput.shapeAxis(
                -joystick.getRawAxis(4),
                DRIVE_DEADBAND);

            drivetrain.drive(
                translation.getX() * SwerveConstants.MAX_SPEED_MPS,
                translation.getY() * SwerveConstants.MAX_SPEED_MPS,
                rotation * SwerveConstants.MAX_ANGULAR_SPEED_RAD_PER_SEC,
                true);
        }, drivetrain::stop, drivetrain));
    }

    private void configureButtonBindings() {
        // whileTrue + startEnd: the mechanism runs only while the button is
        // held and stops the instant it's released. onTrue(RunCommand) would
        // run forever once triggered - nothing would ever call stop().
        new JoystickButton(joystick, 1).whileTrue(
            Commands.startEnd(intake::autoIntake, intake::stop, intake));
        new JoystickButton(joystick, 2).whileTrue(
            Commands.startEnd(intake::open, intake::stop, intake));
        new JoystickButton(joystick, 3).whileTrue(
            Commands.startEnd(intake::close, intake::stop, intake));

        new JoystickButton(joystick, 4).onTrue(
            Commands.runOnce(drivetrain::zeroHeading, drivetrain));

        new JoystickButton(joystick, 5).onTrue(
            Commands.runOnce(() -> elevator.setTargetHeight(Constants.ELEVATOR_SWITCH_METERS), elevator));
        new JoystickButton(joystick, 6).onTrue(
            Commands.runOnce(() -> elevator.setTargetHeight(Constants.ELEVATOR_SCALE_METERS), elevator));
        new JoystickButton(joystick, 7).onTrue(
            Commands.runOnce(() -> elevator.setTargetHeight(Constants.ELEVATOR_GROUND_METERS), elevator));

        new JoystickButton(joystick, 8).whileTrue(
            Commands.startEnd(climb::extend, climb::stop, climb));
        new JoystickButton(joystick, 9).whileTrue(
            Commands.startEnd(climb::retract, climb::stop, climb));
    }

    public SwerveDriveSubsystem getDrivetrain() {
        return drivetrain;
    }

    public Command getAutonomousCommand() {
        return autonomousCommand;
    }

    public void stopAutonomy() {
        autonomyCommandSource.cancel();
        drivetrain.stop();
    }

    public void stopIntake() {
        intake.stop();
    }

    private static final class InactiveAutonomyCommandSource
            implements AutonomyCommandSource {
        @Override
        public Optional<AutonomyCommandFrame> currentCommand() {
            return Optional.empty();
        }

        @Override
        public void cancel() {}
    }
}
