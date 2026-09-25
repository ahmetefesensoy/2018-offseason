package frc.robot;

import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.swerve.GyroIO;
import frc.robot.subsystems.swerve.SwerveModule;
import frc.robot.subsystems.swerve.SwerveModuleIO;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class RobotContainerDriveCommandTest {
    private final CommandScheduler scheduler = CommandScheduler.getInstance();

    @BeforeEach
    void setup() throws InterruptedException {
        assertTrue(HAL.initialize(500, 0));
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
        Thread.sleep(100);
    }

    @AfterEach
    void cleanup() {
        scheduler.cancelAll();
    }

    @Test
    void interruptedDefaultDriveCommandStopsEveryModule() {
        FakeSwerveModuleIO[] moduleIOs = {
            new FakeSwerveModuleIO(),
            new FakeSwerveModuleIO(),
            new FakeSwerveModuleIO(),
            new FakeSwerveModuleIO()
        };
        SwerveDriveSubsystem drivetrain = new SwerveDriveSubsystem(
            new FakeGyroIO(),
            new SwerveModule(moduleIOs[0]),
            new SwerveModule(moduleIOs[1]),
            new SwerveModule(moduleIOs[2]),
            new SwerveModule(moduleIOs[3]));
        new RobotContainer(drivetrain);

        scheduler.run();
        scheduler.run();
        scheduler.cancelAll();

        for (FakeSwerveModuleIO io : moduleIOs) {
            assertTrue(io.stopCalled);
        }
    }

    private static final class FakeGyroIO implements GyroIO {
        @Override
        public Rotation2d getHeading() {
            return Rotation2d.kZero;
        }

        @Override
        public boolean isConnected() {
            return true;
        }

        @Override
        public boolean isCalibrating() {
            return false;
        }

        @Override
        public void zeroYaw() {}
    }

    private static final class FakeSwerveModuleIO implements SwerveModuleIO {
        boolean stopCalled;

        @Override
        public Inputs readInputs() {
            return new Inputs(0.0, 0.0, Rotation2d.kZero, 0.0);
        }

        @Override
        public void setDriveVoltage(double volts) {}

        @Override
        public void setTurnVoltage(double volts) {}

        @Override
        public void stop() {
            stopCalled = true;
        }
    }
}
