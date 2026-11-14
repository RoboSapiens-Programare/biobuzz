package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.util.ElapsedTime;

public class FrequencyMonitor {
    private static final double ALPHA = 0.1;

    private final ElapsedTime timer = new ElapsedTime();
    private double instantHz = 0;
    private double smoothedHz = 0;
    private double ms = 0;

    public void update() {
        double dt = timer.seconds();
        timer.reset();

        if (dt <= 0) {
            return;
        }

        ms = dt * 1000.0;
        instantHz = 1.0 / dt;
        smoothedHz = smoothedHz == 0 ? instantHz : ALPHA * instantHz + (1 - ALPHA) * smoothedHz;
    }

    public double getInstantHz() {
        return instantHz;
    }

    public double getSmoothedHz() {
        return smoothedHz;
    }

    public double getMs() {
        return ms;
    }
}
