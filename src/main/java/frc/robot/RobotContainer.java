package frc.robot;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

/** Owns every subsystem and wires joystick buttons to their commands. */
public class RobotContainer {
    private final SparkMax driveLeft1 = new SparkMax(Constants.DRIVE_LEFT_1_PORT, MotorType.kBrushless);
    private final SparkMax driveLeft2 = new SparkMax(Constants.DRIVE_LEFT_2_PORT, MotorType.kBrushless);
    private final SparkMax driveRight1 = new SparkMax(Constants.DRIVE_RIGHT_1_PORT, MotorType.kBrushless);
    private final SparkMax driveRight2 = new SparkMax(Constants.DRIVE_RIGHT_2_PORT, MotorType.kBrushless);

    private final Joystick joystick = new Joystick(Constants.JOYSTICK_PORT);

    private final ElevatorSubsystem elevator = new ElevatorSubsystem();
    private final IntakeSubsystem intake = new IntakeSubsystem();
    private final ClimbSubsystem climb = new ClimbSubsystem();

    public RobotContainer() {
        configureButtonBindings();
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

    /** Drives the (still tank-drive) chassis directly from joystick axes. */
    public void driveWithJoystick() {
        double speed = -joystick.getRawAxis(1) * 0.6;
        double turn = joystick.getRawAxis(4) * 0.3;

        double left = speed + turn;
        double right = speed - turn;

        driveLeft1.set(left);
        driveLeft2.set(left);
        driveRight1.set(-right);
        driveRight2.set(-right);
    }

    public void stopIntake() {
        intake.stop();
    }
}
