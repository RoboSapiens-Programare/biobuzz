package org.firstinspires.ftc.teamcode.pedro;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Vector2D;
import org.junit.jupiter.api.Test;

class ManualDriveVectorMathTest {
    @Test
    void vectorMagnitudeAndPolarConversion() {
        Vector2D vector = Vector2D.cartesian(3.0, 4.0);

        assertEquals(5.0, vector.magnitude(), 1e-12);
        assertEquals(25.0, vector.magnitudeSquared(), 1e-12);

        Vector2D polar = Vector2D.polar(1.0, Math.PI / 2.0);
        assertEquals(0.0, polar.x(), 1e-12);
        assertEquals(1.0, polar.y(), 1e-12);
    }

    @Test
    void vectorRotationAndNormalization() {
        Vector2D rotated = Vector2D.cartesian(1.0, 0.0).rotate(Math.PI / 2.0);

        assertEquals(0.0, rotated.x(), 1e-12);
        assertEquals(1.0, rotated.y(), 1e-12);
        assertEquals(1.0, rotated.normalized().magnitude(), 1e-12);
        assertThrows(IllegalArgumentException.class, () -> Vector2D.zero().normalized());
    }

    @Test
    void fieldCentricHeadingZeroPreservesPowers() {
        DrivePowers result = ManualDrive.fieldCentric(new DrivePowers(1.0, 0.0, 0.5), 0.0);

        assertEquals(1.0, result.forward(), 1e-12);
        assertEquals(0.0, result.strafe(), 1e-12);
        assertEquals(0.5, result.turn(), 1e-12);
    }

    @Test
    void fieldCentricNinetyDegreeHeadingRotatesForwardToLeft() {
        DrivePowers result = ManualDrive.fieldCentric(new DrivePowers(1.0, 0.0, 0.0), Math.PI / 2.0);

        assertEquals(0.0, result.forward(), 1e-12);
        assertEquals(-1.0, result.strafe(), 1e-12);
    }
}
