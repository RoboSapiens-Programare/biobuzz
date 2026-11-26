package org.firstinspires.ftc.teamcode.robot;

import android.os.Environment;
import com.pedropathing.math.Velocity;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import java.io.File;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;
import org.firstinspires.ftc.teamcode.robot.FileTelemetry.FileTelemetry;
import org.firstinspires.ftc.teamcode.robot.subsystems.Tracker;

public final class ActionLogger {
    private static HardwareMap hardwareMap;
    private static Robot robot;
    private static com.qualcomm.robotcore.util.ElapsedTime runtime;
    private static FileTelemetry telemetryLog;
    private static boolean started;

    private ActionLogger() {}

    public static void init(HardwareMap hwMap, Robot owner) {
        hardwareMap = hwMap;
        robot = owner;
        runtime = new com.qualcomm.robotcore.util.ElapsedTime();
        telemetryLog = null;
        started = false;
    }

    public static void start() {
        runtime.reset();
        started = true;
        if (Robot.VERBOSE && telemetryLog == null) {
            File file = new File(
                    new File(Environment.getExternalStorageDirectory(), "FIRST"),
                    "telemetry-" + new SimpleDateFormat("dd-MM-yyyy", Locale.US).format(new Date()) + ".json");
            telemetryLog = new FileTelemetry(FileTelemetry.Type.JSON_ARRAY, file);
            telemetryLog.addField("action");
            telemetryLog.addField("distance");
            telemetryLog.addField("velocity");
            telemetryLog.addField("shooter_velocity");
            telemetryLog.addField("x_velocity");
            telemetryLog.addField("y_velocity");
            telemetryLog.addField("millis");
            telemetryLog.addField("battery_voltage");
            telemetryLog.addField("hood_angle");
            telemetryLog.addField("roller1_current");
            telemetryLog.addField("roller2_current");
        }
    }

    public static void log(String action) {
        if (!Robot.VERBOSE || !started || telemetryLog == null || robot == null || hardwareMap == null) {
            return;
        }
        double distance = 0.0;
        try {
            double trackPoseDistance = Robot.follower.pose().distance(Tracker.getTrackPose());
            if (!Double.isNaN(trackPoseDistance) && !Double.isInfinite(trackPoseDistance)) {
                distance = trackPoseDistance;
            }
        } catch (Exception ignored) {
        }
        Velocity v = Robot.follower.velocity();
        double speed = Math.hypot(v.vx, v.vy);
        double xVelocity = v.vx;
        double yVelocity = v.vy;
        double shooterVelocity = robot.outtake.getCurrentSpeed();
        double hoodAngle = robot.outtake.hoodPos;
        double rollerOneCurrent = robot.intake.getRollerOneCurrent();
        double rollerTwoCurrent = robot.intake.getRollerTwoCurrent();
        double millis = runtime.milliseconds();
        double batteryVoltage = 0.0;
        double minVoltage = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0.0 && voltage < minVoltage) {
                minVoltage = voltage;
            }
        }
        if (minVoltage != Double.POSITIVE_INFINITY && !Double.isNaN(minVoltage) && !Double.isInfinite(minVoltage)) {
            batteryVoltage = minVoltage;
        }
        telemetryLog.takeSnapshot(
                action,
                distance,
                speed,
                shooterVelocity,
                xVelocity,
                yVelocity,
                millis,
                batteryVoltage,
                hoodAngle,
                rollerOneCurrent,
                rollerTwoCurrent);
    }

    public static void close() {
        if (telemetryLog != null) {
            telemetryLog.close();
            telemetryLog = null;
        }
        started = false;
    }
}
