package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.util.ElapsedTime;

public class FrequencyMonitor {
    private static final double ALPHA = 0.1;

    private final ElapsedTime timer = new ElapsedTime();
    private double instantHz = 0;
    private double smoothedHz = 0;
    private double ms = 0;

    private long stageStart;
    private double onUpdateMs = 0;
    private double schedulerMs = 0;
    private double robotMs = 0;
    private double telemetryMs = 0;

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

    public void beginStage() {
        stageStart = System.nanoTime();
    }

    public void endOnUpdate() {
        onUpdateMs = toMs(System.nanoTime() - stageStart);
    }

    public void endScheduler() {
        schedulerMs = toMs(System.nanoTime() - stageStart);
    }

    public void endRobot() {
        robotMs = toMs(System.nanoTime() - stageStart);
    }

    public void endTelemetry() {
        telemetryMs = toMs(System.nanoTime() - stageStart);
    }

    private static double toMs(long nanos) {
        return nanos / 1e6;
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

    public double getOnUpdateMs() {
        return onUpdateMs;
    }

    public double getSchedulerMs() {
        return schedulerMs;
    }

    public double getRobotMs() {
        return robotMs;
    }

    public double getTelemetryMs() {
        return telemetryMs;
    }
}
