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

    @Test
    void outputIsClippedForLargeFeedforward() {
        FFController controller = new FFController(0.0, 0.0, 0.0, 0.0, 100.0, 50.0, -10.0, 10.0);
        controller.setTargetVelocity(2.0);
        controller.setTargetAcceleration(3.0);

        assertEquals(10.0, controller.update(0.0), 1e-12);

        FFController negativeController = new FFController(0.0, 0.0, 0.0, 0.0, 100.0, 50.0, -10.0, 10.0);
        negativeController.setTargetVelocity(-2.0);
        negativeController.setTargetAcceleration(-3.0);

        assertEquals(-10.0, negativeController.update(0.0), 1e-12);
    }

    @Test
    void staticFrictionUsesStrictToleranceBoundary() {
        FFController controller = new FFController(0.0, 0.0, 0.0, 0.5, 0.0, 0.0, -10.0, 10.0);
        controller.setTolerance(1.0);
        controller.setTargetVelocity(10.0);

        controller.update(9.0);
        assertTrue(controller.velocityReached());

        controller = new FFController(0.0, 0.0, 0.0, 0.5, 0.0, 0.0, -10.0, 10.0);
        controller.setTolerance(1.0);
        controller.setTargetVelocity(10.0);

        assertEquals(0.5, controller.update(8.0), 1e-12);
        assertFalse(controller.velocityReached());
    }

    @Test
    void staticFrictionUsesNegativeErrorSign() {
        FFController controller = new FFController(0.0, 0.0, 0.0, 0.5, 0.0, 0.0, -10.0, 10.0);
        controller.setTolerance(0.1);
        controller.setTargetVelocity(-10.0);

        assertEquals(-0.5, controller.update(0.0), 1e-12);
    }

    @Test
    void filteredVelocityUsesExponentialMovingAverage() {
        FFController controller = new FFController(1.0, 0.0, 0.0, 0.0, 0.0, 0.0, -100.0, 100.0);
        controller.setTargetVelocity(100.0);

        assertEquals(50.0, controller.update(100.0), 1e-12);
        assertEquals(25.0, controller.update(100.0), 1e-12);
    }

    @Test
    void resetDoesNotClearFilteredVelocity() {
        FFController fresh = new FFController(1.0, 0.0, 0.0, 0.0, 0.0, 0.0, -100.0, 100.0);
        FFController resetController = new FFController(1.0, 0.0, 0.0, 0.0, 0.0, 0.0, -100.0, 100.0);
        resetController.setTargetVelocity(100.0);
        resetController.update(100.0);
        resetController.update(100.0);
        resetController.reset();
        resetController.setTargetVelocity(100.0);

        assertEquals(12.5, resetController.update(100.0), 1e-12);
    }
}
