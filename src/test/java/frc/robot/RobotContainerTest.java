package frc.robot;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class RobotContainerTest {
    @BeforeEach
    void setup() throws InterruptedException {
        assertTrue(HAL.initialize(500, 0));
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
        Thread.sleep(100);
    }

    @Test
    void constructsWithoutThrowing() {
        // Regression test: JOYSTICK_PORT used to be 8, an invalid DS slot
        // (valid range is 0-5). JoystickButton's Trigger polls the button
        // when the binding is created, so an out-of-range port throws
        // IllegalArgumentException straight out of the constructor - the
        // robot program would crash in robotInit() every single boot.
        assertDoesNotThrow(RobotContainer::new);
    }
}
