package org.firstinspires.ftc.teamcode.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

class ModelsTest {
    private final double originalVelocityC0 = Models.velocityC0;
    private final double originalVelocityC1 = Models.velocityC1;
    private final double originalVelocityC2 = Models.velocityC2;
    private final double originalHoodC0 = Models.hoodC0;
    private final double originalHoodC1 = Models.hoodC1;
    private final double originalHoodC2 = Models.hoodC2;
    private final double originalTofC0 = Models.tofC0;
    private final double originalTofC1 = Models.tofC1;
    private final double originalTofC2 = Models.tofC2;

    @AfterEach
    void restoreCoefficients() {
        Models.velocityC0 = originalVelocityC0;
        Models.velocityC1 = originalVelocityC1;
        Models.velocityC2 = originalVelocityC2;
        Models.hoodC0 = originalHoodC0;
        Models.hoodC1 = originalHoodC1;
        Models.hoodC2 = originalHoodC2;
        Models.tofC0 = originalTofC0;
        Models.tofC1 = originalTofC1;
        Models.tofC2 = originalTofC2;
    }

    @Test
    void velocityUsesHornerForm() {
        assertEquals(1505.58369314, Models.velocity(0.0), 1e-9);
        assertEquals(1504.47507524352, Models.velocity(1.0), 1e-9);
        assertEquals(1498.3362505782, Models.velocity(10.0), 1e-9);
    }

    @Test
    void hoodClampsToUnitInterval() {
        Models.hoodC0 = 2.0;
        Models.hoodC1 = 0.0;
        Models.hoodC2 = 0.0;
        assertEquals(1.0, Models.hood(0.0), 1e-12);

        Models.hoodC0 = -2.0;
        assertEquals(0.0, Models.hood(0.0), 1e-12);
    }

    @Test
    void tofUsesHornerForm() {
        Models.tofC0 = 1.0;
        Models.tofC1 = 2.0;
        Models.tofC2 = 3.0;
        assertEquals(1.0 + 2.0 * 4.0 + 3.0 * 4.0 * 4.0, Models.tof(4.0), 1e-12);
    }
}
