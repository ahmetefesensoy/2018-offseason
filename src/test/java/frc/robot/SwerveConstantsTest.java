package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Collections;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import org.junit.jupiter.api.Test;

class SwerveConstantsTest {
    @Test
    void driveEncoderRotationsConvertToWheelMeters() {
        assertEquals(
            Math.PI * 0.1016 / 6.75,
            SwerveConstants.DRIVE_POSITION_FACTOR_METERS,
            1e-12);
    }

    @Test
    void driveEncoderRpmConvertsToMetersPerSecond() {
        assertEquals(
            SwerveConstants.DRIVE_POSITION_FACTOR_METERS / 60.0,
            SwerveConstants.DRIVE_VELOCITY_FACTOR_MPS,
            1e-12);
    }

    @Test
    void moduleOrderAndCanIdsMatchTheWiringContract() {
        assertEquals(
            List.of("FL", "FR", "BL", "BR"),
            SwerveConstants.MODULE_CONFIGS.stream()
                .map(SwerveConstants.ModuleConfig::name)
                .toList());

        Set<Integer> ids = new HashSet<>();
        for (var module : SwerveConstants.MODULE_CONFIGS) {
            assertTrue(ids.add(module.driveCanId()));
            assertTrue(ids.add(module.turnCanId()));
        }

        assertEquals(Set.of(10, 11, 12, 13, 14, 15, 16, 17), ids);
        assertTrue(Collections.disjoint(ids, Set.of(1, 2, 3, 9)));
    }

    @Test
    void moduleTranslationsUseHalfWheelbaseAndTrackWidth() {
        assertEquals(0.30, SwerveConstants.FRONT_LEFT_LOCATION.getX(), 1e-12);
        assertEquals(0.30, SwerveConstants.FRONT_LEFT_LOCATION.getY(), 1e-12);
        assertEquals(0.30, SwerveConstants.FRONT_RIGHT_LOCATION.getX(), 1e-12);
        assertEquals(-0.30, SwerveConstants.FRONT_RIGHT_LOCATION.getY(), 1e-12);
        assertEquals(-0.30, SwerveConstants.BACK_LEFT_LOCATION.getX(), 1e-12);
        assertEquals(0.30, SwerveConstants.BACK_LEFT_LOCATION.getY(), 1e-12);
        assertEquals(-0.30, SwerveConstants.BACK_RIGHT_LOCATION.getX(), 1e-12);
        assertEquals(-0.30, SwerveConstants.BACK_RIGHT_LOCATION.getY(), 1e-12);
    }
}
