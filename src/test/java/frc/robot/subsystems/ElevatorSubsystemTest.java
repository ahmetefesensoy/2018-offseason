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
}
