package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class ElevatorSubsystemTest {
    private ElevatorSubsystem elevator;

    @BeforeEach
    void setup() {
        assertTrue(HAL.initialize(500, 0));
        elevator = new ElevatorSubsystem();
    }

    @AfterEach
    void teardown() {
        elevator.close();
    }

    @Test
    void startsAtGroundHeight() {
        assertEquals(0.0, elevator.getCurrentHeight(), 0.01);
    }

    @Test
    void clampsTargetAboveScaleHeight() {
        elevator.setTargetHeight(5.0);
        assertEquals(1.52, elevator.getTargetHeightForTest(), 0.001);
    }

    @Test
    void clampsTargetBelowGround() {
        elevator.setTargetHeight(-1.0);
        assertEquals(0.0, elevator.getTargetHeightForTest(), 0.001);
    }

    @Test
    void notAtTargetWhenFarFromGoal() {
        elevator.setTargetHeight(1.52);
        assertFalse(elevator.atTarget());
    }

    @Test
    void overwritesPendingTargetWithLatestCall() {
        elevator.setTargetHeight(0.23);
        elevator.setTargetHeight(1.52);
        assertEquals(1.52, elevator.getTargetHeightForTest(), 0.001);
    }

    @Test
    void stopStaysStoppedThroughNextPeriodicCall() {
        // Regression test: periodic() used to unconditionally call
        // motor.setVoltage(pid+ff) every cycle, so a stop() commanded by a
        // released button would be overwritten by the very next 20ms tick.
        elevator.setTargetHeight(1.52);
        elevator.periodic(); // let the PID loop start driving toward target
        elevator.stop();
        elevator.periodic(); // simulate the next scheduler tick after stop()
        assertEquals(0.0, elevator.getMotorForTest().get(), 0.001);
    }

    @Test
    void atTargetTrueWhenEncoderMatchesGoalInMeters() {
        // Regression coverage for a rotations/meters unit mixup: with
        // ELEVATOR_ROTATIONS_TO_METERS != 1.0 this only passes if
        // getCurrentHeight() actually applies the conversion factor.
        elevator.setTargetHeight(0.23);
        elevator.getMotorForTest().getEncoder().setPosition(0.23 / frc.robot.Constants.ELEVATOR_ROTATIONS_TO_METERS);
        assertTrue(elevator.atTarget());
    }

    @Test
    void atTargetFalseWhenEncoderJustOutsideTolerance() {
        elevator.setTargetHeight(0.23);
        double justOutside = (0.23 + 0.03) / frc.robot.Constants.ELEVATOR_ROTATIONS_TO_METERS;
        elevator.getMotorForTest().getEncoder().setPosition(justOutside);
        assertFalse(elevator.atTarget());
    }
}
