package org.firstinspires.ftc.teamcode.utils.ControlSystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class FFControllerTest {
    @Test
    void feedforwardOnlyOutputMatchesVelocityAndAcceleration() {
        FFController controller = new FFController(0.0, 0.0, 0.0, 0.0, 1.5, 2.0, -10.0, 10.0);
        controller.setTargetVelocity(2.0);
        controller.setTargetAcceleration(3.0);

        assertEquals(9.0, controller.update(10.0), 1e-12);
    }

    @Test
    void staticFrictionTermUsesErrorSign() {
        FFController controller = new FFController(0.0, 0.0, 0.0, 0.5, 0.0, 0.0, -10.0, 10.0);
        controller.setTolerance(0.1);
        controller.setTargetVelocity(10.0);

        assertEquals(0.5, controller.update(0.0), 1e-12);
        assertFalse(controller.velocityReached());
    }

    @Test
    void velocityReachedUsesTolerance() {
        FFController controller = new FFController(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -10.0, 10.0);
        controller.setTolerance(1.0);
        controller.setTargetVelocity(10.0);

        controller.update(9.5);

        assertTrue(controller.velocityReached());
    }
}
