package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class IntakeSubsystemTest {
    private IntakeSubsystem intake;
    private SparkMax rightMotor;
    private SparkMax leftMotor;

    @BeforeEach
    void setup() throws InterruptedException {
        assertTrue(HAL.initialize(500, 0));
        // Motor controllers force their output to zero while the robot is
        // disabled; simulated tests must explicitly enable it first.
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
        Thread.sleep(100);

        intake = new IntakeSubsystem();
        rightMotor = intake.getRightMotorForTest();
        leftMotor = intake.getLeftMotorForTest();
    }

    @AfterEach
    void teardown() {
        intake.close_();
    }

    @Test
    void closeDrivesMotorsInwardWhenNotYetClosed() {
        rightMotor.getEncoder().setPosition(0.05); // > 0: not yet closed
        leftMotor.getEncoder().setPosition(-0.05); // < 0: not yet closed
        intake.close();
        assertEquals(0.2, rightMotor.get(), 0.001);
        assertEquals(-0.2, leftMotor.get(), 0.001);
    }

    @Test
    void closeStopsMotorsOnceFullyClosed() {
        rightMotor.getEncoder().setPosition(-0.01); // <= 0: fully closed
        leftMotor.getEncoder().setPosition(0.01);   // >= 0: fully closed
        intake.close();
        assertEquals(0.0, rightMotor.get(), 0.001);
        assertEquals(0.0, leftMotor.get(), 0.001);
    }

    @Test
    void openStopsMotorsOncePastOpenLimit() {
        rightMotor.getEncoder().setPosition(-0.25); // <= -0.20: fully open
        leftMotor.getEncoder().setPosition(0.25);   // >= 0.20: fully open
        intake.open();
        assertEquals(0.0, rightMotor.get(), 0.001);
        assertEquals(0.0, leftMotor.get(), 0.001);
    }

    @Test
    void stopZeroesBothMotors() {
        intake.stop();
        assertEquals(0.0, rightMotor.get(), 0.001);
        assertEquals(0.0, leftMotor.get(), 0.001);
    }
}
