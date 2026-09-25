package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
    private Command autonomousCommand;
    private RobotContainer robotContainer;

    @Override
    public void robotInit() {
        robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void autonomousInit() {
        autonomousCommand = robotContainer.getAutonomousCommand();
        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
    }

    @Override
    public void disabledInit() {
        cancelAndStopAutonomy();
    }

    @Override
    public void teleopInit() {
        cancelAndStopAutonomy();
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
        cancelAndStopAutonomy();
    }

    private void cancelAndStopAutonomy() {
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
            autonomousCommand = null;
        }
        if (robotContainer != null) {
            robotContainer.stopAutonomy();
        }
    }

}
