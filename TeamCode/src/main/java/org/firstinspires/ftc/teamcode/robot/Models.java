package org.firstinspires.ftc.teamcode.robot;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class Models {

    public static double velocityC0 = 1505.58369314;
    public static double velocityC1 = -1.15127052318;
    public static double velocityC2 = 0.0426526267;

    public static double hoodC0 = 0.05;
    public static double hoodC1 = 0;
    public static double hoodC2 = 0;

    public static double tofC0 = 0;
    public static double tofC1 = 0;
    public static double tofC2 = 0;

    public static double velocity(double dist) {
        return velocityC0 + dist * (velocityC1 + dist * velocityC2);
    }

    public static double hood(double dist) {
        double hood = hoodC0 + dist * (hoodC1 + dist * hoodC2);
        return Math.max(0, Math.min(1, hood));
    }

    public static double tof(double dist) {
        return tofC0 + dist * (tofC1 + dist * tofC2);
    }
}
