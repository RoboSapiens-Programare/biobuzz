package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.robot.Robot;

public class Tracker implements Subsystem {
    public static Pose trackPose = new Pose(0, 0);

    private static final ElapsedTime timer = new ElapsedTime();
    private static double periodSec = 0;
    private static double lastTrackAngle = 0;
    private static double lastVelocity = 0;
    private static double lastHood = 0;
    private static boolean hasComputed = false;
    private static boolean enabled = true;

    private static boolean sotmEnabled = true;
    private static double sotmOffset = 0;

    public static void setTrackingFrequency(double hz) {
        periodSec = hz > 0 ? 1.0 / hz : 0;
    }

    public static void setEnabled(boolean e) {
        enabled = e;
    }

    public static boolean isEnabled() {
        return enabled;
    }

    public static void setSOTM(double offset) {
        sotmOffset = offset;
    }

    public static void setSOTMEnabled(boolean e) {
        sotmEnabled = e;
    }

    public static boolean isSOTMEnabled() {
        return sotmEnabled;
    }

    public double getTrackAngle() {
        return lastTrackAngle;
    }

    public double getVelocity() {
        return lastVelocity;
    }

    public double getHood() {
        return lastHood;
    }

    public void applySOTM() {
        if (sotmEnabled) {
            lastTrackAngle += sotmOffset;
        }
    }

    private double computeVelocity(double dist) {
        // tune regression
        return Math.pow(dist, 3) * 0 + Math.pow(dist, 2) * 0 + dist * 0 + 0;
    }

    private double computeHood(double dist) {
        // tune regression
        return Math.pow(dist, 3) * 0 + Math.pow(dist, 2) * 0 + dist * 0 + 0;
    }

    @Override
    public void init() {}

    @Override
    public void update() {
        if (!enabled) {
            return;
        }

        if (!hasComputed || (periodSec > 0 && timer.seconds() >= periodSec)) {
            Pose robotPose = Robot.follower.pose();
            lastTrackAngle =
                    Math.atan2(trackPose.x() - robotPose.x(), trackPose.y() - robotPose.y()) - robotPose.heading();

            double dist = robotPose.distance(trackPose);
            lastVelocity = computeVelocity(dist);
            lastHood = computeHood(dist);
            hasComputed = true;
            timer.reset();
        }
    }

    @Override
    public void reset() {
        hasComputed = false;
    }

    @Override
    public void stop() {}
}
