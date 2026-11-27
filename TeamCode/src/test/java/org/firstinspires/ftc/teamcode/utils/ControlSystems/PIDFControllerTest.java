package org.firstinspires.ftc.teamcode.utils.ControlSystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.lang.reflect.Field;
import org.junit.jupiter.api.Test;

class PIDFControllerTest {
    @Test
    void pOnlyOutputIsClipped() {
        PIDFController controller = new PIDFController(2.0, 0.0, 0.0, 0.0, -1.0, 1.0);
        controller.setSetpoint(100.0);

        assertEquals(1.0, controller.update(0.0), 1e-12);
    }

    @Test
    void targetReachedRespectsTolerance() {
        PIDFController controller = new PIDFController(2.0, 0.0, 0.0, 0.0, -1.0, 1.0);
        controller.setTolerance(10.0);
        controller.setSetpoint(100.0);

        controller.update(95.0);

        assertTrue(controller.targetReached());
    }

    @Test
    void resetClearsIntegralAndError() {
        PIDFController controller = new PIDFController(2.0, 1.0, 0.0, 0.0, -1.0, 1.0);
        controller.setSetpoint(100.0);
        controller.update(0.0);

        controller.reset();

        assertEquals(1.0, controller.update(0.0), 1e-12);
    }

    @Test
    void derivativeTermIsClamped() {
        PIDFController controller = new PIDFController(0.0, 0.0, 1e-3, 0.0, -10.0, 10.0);
        controller.setSetpoint(1_000_000.0);

        assertEquals(0.1, controller.update(0.0), 1e-12);
    }

    @Test
    void feedforwardTermUsesToleranceBoundary() {
        PIDFController controller = new PIDFController(0.0, 0.0, 0.0, 2.5, -10.0, 10.0);
        controller.setTolerance(1.0);
        controller.setSetpoint(10.0);

        controller.update(9.0);
        assertTrue(controller.targetReached());

        controller = new PIDFController(0.0, 0.0, 0.0, 2.5, -10.0, 10.0);
        controller.setTolerance(1.0);
        controller.setSetpoint(10.0);

        assertEquals(2.5, controller.update(0.0), 1e-12);
    }

    @Test
    void integralStateIsBoundedAndResetByBand() throws Exception {
        PIDFController controller = new PIDFController(0.0, 0.5, 0.0, 0.0, -100.0, 100.0);
        Field integralField = PIDFController.class.getDeclaredField("integral");
        integralField.setAccessible(true);
        integralField.set(controller, 1_000_000.0);

        controller.setSetpoint(1000.0);
        assertEquals(1.0, controller.update(0.0), 1e-12);

        controller.setSetpoint(0.0);
        controller.update(0.0);
        assertEquals(0.0, controller.update(0.0), 1e-12);
    }
}
