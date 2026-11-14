package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.robot.HivePoses;
import org.firstinspires.ftc.teamcode.robot.Robot;

public class Tracker implements Subsystem {
    private static Pose trackPose = new Pose(0, 0);

    private static final ElapsedTime timer = new ElapsedTime();
    private static double periodSec = 1.0 / 30;
    private static double lastTrackAngle = 0;
    private static double lastVelocity = 0;
    private static int angleVersion = 0;
    private static boolean hasComputed = false;
    private static boolean enabled = false;
    private static boolean audienceTarget = true;
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
        if (audienceTarget) {
            if (robotPose.y() > 73) {
                audienceTarget = false;
            }
        } else if (robotPose.y() < 71) {
            audienceTarget = true;
        }

        if (Robot.alliance == Robot.Alliance.RED) {
            return audienceTarget ? HivePoses.RED_AUDIENCE : HivePoses.RED_FAR;
        }
        return audienceTarget ? HivePoses.BLUE_AUDIENCE : HivePoses.BLUE_FAR;
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

    public static int getAngleVersion() {
        return angleVersion;
    }

    private double computeVelocity(double dist) {
        // tune regression: v = c0 + c1*d + c2*d^2 + c3*d^3
        final double c0 = 0;
        final double c1 = 0;
        final double c2 = 0;
        final double c3 = 0;
        //        return c0 + dist * (c1 + dist * (c2 + dist * c3));
        return 1000;
    }

    private static double wrapAngle(double angle) {
        angle %= 2 * Math.PI;
        if (angle > Math.PI) {
            angle -= 2 * Math.PI;
        } else if (angle < -Math.PI) {
            angle += 2 * Math.PI;
        }
        return angle;
    }

    @Override
    public void init() {
        enabled = false;
        hasComputed = false;
        audienceTarget = true;
    }

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

        double bearing = wrapAngle(Math.atan2(dy, dx) - robotPose.heading());
        double angle = SHOOTER_CENTER_OFFSET - bearing;

        if (angle > Math.PI) {
            angle -= 2 * Math.PI;
        }
        if (sotmEnabled && sotmOffset != 0) {
            angle = wrapAngle(angle + sotmOffset);
        }

        angle = Math.max(-Math.PI / 2, Math.min(Math.PI / 2, angle));

        lastTrackAngle = angle;

        double dist = Math.hypot(dx, dy);
        lastVelocity = computeVelocity(dist);
        angleVersion++;
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
