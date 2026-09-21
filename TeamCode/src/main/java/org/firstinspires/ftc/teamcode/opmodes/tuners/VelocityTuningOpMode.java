package org.firstinspires.ftc.teamcode.opmodes.tuners;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Scanner;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.opmode.RobotOpMode;
import org.firstinspires.ftc.teamcode.robot.subsystems.Tracker;
import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;


// this is just for a test commit
@TeleOp(name = "Velocity Tuning", group = "Tuners")
public class VelocityTuningOpMode extends RobotOpMode {
    private static final double STEP = 10;

    private double velocity = 1100;
    private JSONArray points = new JSONArray();
    private File logFile;

    private boolean prevDpadLeft = false;
    private boolean prevDpadRight = false;
    private boolean prevTouchpad = false;

    @Override
    protected void onInit() {
        robot.outtake.stopIdling();
        robot.outtake.setVelocity(velocity);

        logFile = new File(hardwareMap.appContext.getFilesDir(), "shooter_tuning_points.json");

        if (logFile.exists()) {
            try (Scanner scanner = new Scanner(logFile)) {
                StringBuilder sb = new StringBuilder();
                while (scanner.hasNextLine()) {
                    sb.append(scanner.nextLine());
                }
                if (sb.length() > 0) {
                    points = new JSONArray(sb.toString());
                }
            } catch (Exception e) {
                telemetry.addData("Load error", e.getMessage());
            }
        }
    }

    @Override
    protected void onUpdate() {
        if (gamepad1.dpad_left && !prevDpadLeft) {
            velocity = Math.max(0, velocity - STEP);
            robot.outtake.setVelocity(velocity);
        }

        if (gamepad1.dpad_right && !prevDpadRight) {
            velocity += STEP;
            robot.outtake.setVelocity(velocity);
        }

        prevDpadLeft = gamepad1.dpad_left;
        prevDpadRight = gamepad1.dpad_right;

        if (gamepad1.touchpad && !prevTouchpad) {
            logPoint();
        }
        prevTouchpad = gamepad1.touchpad;

        telemetry.addData("Target velocity", velocity);
        telemetry.addData("Current speed", robot.outtake.getCurrentSpeed());
        telemetry.addData("Distance to track pose", Robot.follower.pose().distance(Tracker.trackPose));
        telemetry.addData("Points", points.length());
        telemetry.addData("Log file", logFile.getAbsolutePath());
    }

    @Override
    protected void onStop() {
        robot.outtake.startIdling();
        writeFile();
    }

    private void logPoint() {
        double dist = Robot.follower.pose().distance(Tracker.trackPose);
        try {
            points.put(new JSONObject().put("dist", dist).put("vel", velocity));
        } catch (JSONException e) {
            telemetry.addData("Log error", e.getMessage());
        }
        writeFile();
    }

    private void writeFile() {
        try (FileWriter writer = new FileWriter(logFile, false)) {
            writer.write(points.toString(2));
        } catch (IOException | JSONException e) {
            telemetry.addData("Log error", e.getMessage());
        }
    }
}
