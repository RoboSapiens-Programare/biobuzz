package org.firstinspires.ftc.teamcode.opmodes;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class FlywheelConfig {
    public static double kp = 0.0016;
    public static double ki = 0.0002;
    public static double kd = 0.0050;
    public static double target = 2000;
}