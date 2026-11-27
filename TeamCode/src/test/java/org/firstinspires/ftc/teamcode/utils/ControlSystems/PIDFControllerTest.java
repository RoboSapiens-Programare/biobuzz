package org.firstinspires.ftc.teamcode.utils.ControlSystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

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
}
