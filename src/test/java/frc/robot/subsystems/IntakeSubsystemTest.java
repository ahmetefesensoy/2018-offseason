package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
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
        // Open position is right=-0.20/left=+0.20; closed position is 0 for
        // both. "Not yet closed" means still on the open side of 0.
        rightMotor.getEncoder().setPosition(-0.05); // < 0: not yet closed
        leftMotor.getEncoder().setPosition(0.05);   // > 0: not yet closed
        intake.close();
        assertEquals(0.2, rightMotor.get(), 0.001);
        assertEquals(-0.2, leftMotor.get(), 0.001);
    }

    @Test
    void closeStopsMotorsOnceFullyClosed() {
        rightMotor.getEncoder().setPosition(0.01);  // >= 0: fully closed
        leftMotor.getEncoder().setPosition(-0.01);  // <= 0: fully closed
        intake.close();
        assertEquals(0.0, rightMotor.get(), 0.001);
        assertEquals(0.0, leftMotor.get(), 0.001);
    }

    @Test
    void closeFromFullyOpenPositionActuallyMoves() {
        // Regression test: close() must be able to drive the gripper shut
        // starting from open()'s resting position, not just hold still.
        rightMotor.getEncoder().setPosition(-0.20); // fully open
        leftMotor.getEncoder().setPosition(0.20);   // fully open
        intake.close();
        assertEquals(0.2, rightMotor.get(), 0.001);
        assertEquals(-0.2, leftMotor.get(), 0.001);
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

    @Test
    void cubeDetectedFalseWhenRangeInvalid() {
        // Ultrasonic.getRangeInches() returns 0 when no echo has been
        // measured yet (isRangeValid() == false). 0 is below CUBE_RANGE_CM,
        // so without a validity check this would false-positive as "cube
        // present" on every disabled/just-booted robot.
        assertFalse(IntakeSubsystem.cubeDetected(false, 0.0));
    }

    @Test
    void cubeDetectedFalseWhenValidButFar() {
        assertFalse(IntakeSubsystem.cubeDetected(true, 15.0));
    }

    @Test
    void cubeDetectedTrueWhenValidAndClose() {
        assertTrue(IntakeSubsystem.cubeDetected(true, 1.0));
    }
}
