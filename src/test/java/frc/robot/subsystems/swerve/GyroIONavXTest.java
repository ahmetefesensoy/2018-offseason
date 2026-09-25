package frc.robot.subsystems.swerve;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

class GyroIONavXTest {
    @Test
    void clockwisePositiveNavXYawBecomesCounterClockwisePositiveHeading() {
        assertEquals(-90.0, GyroIONavX.toHeading(90.0).getDegrees(), 1e-9);
        assertEquals(45.0, GyroIONavX.toHeading(-45.0).getDegrees(), 1e-9);
    }
}
