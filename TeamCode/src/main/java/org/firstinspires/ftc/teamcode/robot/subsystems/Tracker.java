package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.robot.HivePoses;
import org.firstinspires.ftc.teamcode.robot.Models;
import org.firstinspires.ftc.teamcode.robot.Robot;

@Configurable
public class Tracker implements Subsystem {
    private static Pose trackPose = new Pose(0, 0);

    private static final ElapsedTime timer = new ElapsedTime();
    private static double periodSec = 1.0 / 30;
    private static double lastTrackAngle = 0;
    private static double lastVelocity = 0;
    private static double lastHood = 0.05;
    private static int angleVersion = 0;
    private static boolean hasComputed = false;
    private static boolean enabled = false;
    private static boolean rightHive = true;
    private static final double SHOOTER_CENTER_OFFSET = Math.PI; // turret zero points at robot rear
    private static final double MIN_TRACK_DISTANCE = 5;

    // Hysteresis on the hive switch so the aim does not flap near the midline.
    private static final double SWITCH_TO_LEFT_Y = 73;
    private static final double SWITCH_TO_RIGHT_Y = 71;
    private static final double MAX_RANGE = Math.toRadians((double) (360 * 3) / 4 - 30);
    private static double lastTof = 0;

    private static boolean sotmEnabled = false;

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

    private static Pose selectHivePose(Pose robotPose) {
        if (rightHive) {
            if (robotPose.y() > SWITCH_TO_LEFT_Y) {
                rightHive = false;
            }
        } else if (robotPose.y() < SWITCH_TO_RIGHT_Y) {
            rightHive = true;
        }

        return rightHive ? HivePoses.RIGHT : HivePoses.LEFT;
    }

    public static void setSOTMEnabled(boolean e) {
        sotmEnabled = e;
    }

    public static boolean isSOTMEnabled() {
        return sotmEnabled;
    }

    public static void enableSOTM() {
        sotmEnabled = true;
    }

    public static void disableSOTM() {
        sotmEnabled = false;
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

    public double getTof() {
        return lastTof;
    }

    public static int getAngleVersion() {
        return angleVersion;
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
        sotmEnabled = false;
        hasComputed = false;
        rightHive = true;
        lastTrackAngle = 0;
        lastVelocity = 0;
        lastHood = Models.hood(0);
        lastTof = Models.tof(0);
    }

    @Override
    public void update() {
        if (!enabled) {
            return;
        }

        if (hasComputed && periodSec > 0 && timer.seconds() < periodSec) {
            return;
        }

        Pose robotPose = Robot.follower.pose();
        setTrackPose(selectHivePose(robotPose));

        double dx = trackPose.x() - robotPose.x();
        double dy = trackPose.y() - robotPose.y();

        double dist = Math.hypot(dx, dy);
        hasComputed = true;
        timer.reset();

        if (dist <= MIN_TRACK_DISTANCE) {
            return;
        }

        double tof = Models.tof(dist);
        if (sotmEnabled) {
            Velocity v = Robot.follower.velocity();

            dx -= v.vx * tof;
            dy -= v.vy * tof;
            dist = Math.hypot(dx, dy);
        }

        double bearing = wrapAngle(Math.atan2(dy, dx) - robotPose.heading());
        double angle = SHOOTER_CENTER_OFFSET - bearing;

        if (angle > Math.PI) {
            angle -= 2 * Math.PI;
        }
        angle = Math.max(-MAX_RANGE / 2, Math.min(MAX_RANGE / 2, angle));

        lastTrackAngle = angle;
        lastVelocity = Models.velocity(dist);
        lastHood = Models.hood(dist);
        lastTof = tof;
        angleVersion++;
    }

    @Override
    public void reset() {
        sotmEnabled = false;
        hasComputed = false;
        rightHive = true;
        lastTrackAngle = 0;
        lastVelocity = 0;
        lastHood = Models.hood(0);
        lastTof = Models.tof(0);
    }

    @Override
    public void stop() {
        sotmEnabled = false;
    }
}
