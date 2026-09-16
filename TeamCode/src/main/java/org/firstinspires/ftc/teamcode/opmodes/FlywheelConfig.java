package org.firstinspires.ftc.teamcode.opmodes;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class FlywheelConfig {
    public static double kp = 0.0016;
    public static double ki = 0.0002;
    public static double kd = 0.0050;

    // Feedforward (identified by FlywheelTuner)
    public static double kS = 0.0;
    public static double kV = 0.0;
    public static double kA = 0.0;

    public static double idleVelocity = 700;
    public static double target = 2000;
}