package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.pedropathing.math.Pose;
import com.pedropathing.utils.Angle;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.robot.HivePoses;
import org.firstinspires.ftc.teamcode.robot.Robot;

public class Tracker implements Subsystem {
    private static Pose trackPose = new Pose(0, 0);

    private static final ElapsedTime timer = new ElapsedTime();
    private static double periodSec = 0;
    private static double lastTrackAngle = 0;
    private static double lastVelocity = 0;
    private static boolean hasComputed = false;
    private static boolean enabled = false;
    private static final double SHOOTER_CENTER_OFFSET = Math.PI; // turret zero points at robot rear

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

    public static Pose getTrackPose() {
        return trackPose;
    }

    public static void setTrackPose(Pose target) {
        if (target == null) {
            return;
        }
        if (trackPose.x() == target.x() && trackPose.y() == target.y()) {
            return;
        }
        trackPose = target;
        hasComputed = false;
    }

    public static void setTrackPose(double x, double y) {
        setTrackPose(new Pose(x, y));
    }

    private static Pose selectTrackPose(Pose robotPose) {
        boolean audienceSide = robotPose.y() < HivePoses.FIELD_CENTER;
        if (Robot.alliance == Robot.Alliance.RED) {
            return audienceSide ? HivePoses.RED_AUDIENCE : HivePoses.RED_FAR;
        }
        return audienceSide ? HivePoses.BLUE_AUDIENCE : HivePoses.BLUE_FAR;
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

    private double computeVelocity(double dist) {
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

        Pose robotPose = Robot.follower.pose();
        setTrackPose(selectTrackPose(robotPose));

        if (hasComputed && periodSec > 0 && timer.seconds() < periodSec) {
            return;
        }

        double dx = trackPose.x() - robotPose.x();
        double dy = trackPose.y() - robotPose.y();

        double bearing = Angle.normalizeSigned(Math.atan2(dy, dx) - robotPose.heading());
        double angle = Angle.normalizeSigned(SHOOTER_CENTER_OFFSET - bearing);
        if (sotmEnabled) {
            angle = Angle.normalizeSigned(angle + sotmOffset);
        }
        lastTrackAngle = angle;

        double dist = robotPose.distance(trackPose);
        lastVelocity = computeVelocity(dist);
        hasComputed = true;
        timer.reset();
    }

    @Override
    public void reset() {
        hasComputed = false;
    }

    @Override
    public void stop() {}
}
