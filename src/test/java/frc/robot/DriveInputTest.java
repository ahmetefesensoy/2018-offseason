package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Translation2d;
import org.junit.jupiter.api.Test;

class DriveInputTest {
    @Test
    void deadbandRejectsStickNoise() {
        assertEquals(0.0, DriveInput.shapeAxis(0.04, 0.08), 1e-9);
        assertEquals(0.0, DriveInput.shapeTranslation(0.04, 0.03, 0.08).getNorm(), 1e-9);
    }

    @Test
    void shapedAxisPreservesSignAndSquaresMagnitude() {
        assertEquals(-0.25, DriveInput.squareWithSign(-0.5), 1e-9);
        assertEquals(0.25, DriveInput.squareWithSign(0.5), 1e-9);
    }

    @Test
    void translationVectorIsLimitedToUnitCircle() {
        Translation2d value = DriveInput.shapeTranslation(1.0, 1.0, 0.08);

        assertEquals(1.0, value.getNorm(), 1e-9);
        assertEquals(45.0, value.getAngle().getDegrees(), 1e-9);
    }

    @Test
    void radialShapingPreservesTranslationDirection() {
        Translation2d value = DriveInput.shapeTranslation(0.3, -0.4, 0.08);

        assertEquals(-53.13010235415598, value.getAngle().getDegrees(), 1e-9);
    }
}
