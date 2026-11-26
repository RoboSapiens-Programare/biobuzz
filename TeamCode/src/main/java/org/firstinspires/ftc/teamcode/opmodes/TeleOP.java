package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.robot.Robot.follower;

import android.os.Environment;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;
// import dev.frozenmilk.dairy.pasteurized.Pasteurized;
import java.io.File;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.commands.RobotCommands;
import org.firstinspires.ftc.teamcode.robot.ActionLogger;
import org.firstinspires.ftc.teamcode.robot.FileTelemetry.FileTelemetry;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.opmode.RobotOpMode;
import org.firstinspires.ftc.teamcode.robot.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.robot.subsystems.Tracker;

@TeleOp
@Configurable
public class TeleOP extends RobotOpMode {
    private enum ManualMode {
        FIELD_CENTRIC,
        ROBOT_CENTRIC
    }

    private ManualMode manualMode = ManualMode.FIELD_CENTRIC;

    public static final boolean COMPETITION = false;

    private static final double TRACKING_HZ_INTAKE = 30;
    private static final double TRACKING_HZ_OUTTAKE = 70;

    {
        panelsTelemetry = !COMPETITION;
        telemetryHz = 10;
    }

    private final ElapsedTime debounceTimer = new ElapsedTime();

    private CachingDcMotorEx front_left, front_right, rear_left, rear_right, roller1, roller2, flywheel;
    private FileTelemetry tuningLog;
    private String tuningError = "";

    private boolean lastFire, lastPull, lastPush, lastFlower, lastReset, lastManual;
    private boolean lastDriverMoving;
    private boolean rightHeld, leftHeld;
    private int lastBallCount = 0;
    private String driveCurrentFLFR = "",
            driveCurrentRLRR = "",
            roller1Current = "",
            roller2Current = "",
            flywheelCurrent = "",
            intakeMax = "";

    private double tuningVelocity = 1100;

    private static boolean onRise(boolean now, boolean last) {
        return now && !last;
    }

    private static boolean onDrop(boolean now, boolean last) {
        return !now && last;
    }

    @Override
    protected void onInit() {
        front_left = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "front_left"));
        front_right = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "front_right"));
        rear_left = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "rear_left"));
        rear_right = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "rear_right"));
        roller1 = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "roller_motor"));
        roller2 = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "roller_motor_2"));
        flywheel = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "flywheel"));

        // Set initial state actions safely on init
        //        follower.setPose(new Pose(7.2, 8, Math.PI / 2));

        if (Robot.TUNING) {
            robot.outtake.setVelocity(tuningVelocity);
            ensureTuningLog();
        }

        robot.outtake.stopIdling();
        robot.ledDriver.playSweep();
    }

    @Override
    protected void onStart() {
        Tracker.setEnabled(true);
        Tracker.setTrackingFrequency(TRACKING_HZ_INTAKE);
        debounceTimer.reset();
    }

    @Override
    protected void onStop() {
        if (tuningLog != null) {
            tuningLog.close();
        }
    }

    @Override
    protected void onUpdate() {
        Pose robotPose = follower.pose();
        DrivePowers powers = null;
        switch (manualMode) {
            case FIELD_CENTRIC:
                powers = ManualDrive.fieldCentric(
                        -gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, robotPose.heading());
                break;
            case ROBOT_CENTRIC:
                powers = new DrivePowers(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
                break;
        }

        boolean driverMoving = Math.abs(powers.forward()) + Math.abs(powers.strafe()) + Math.abs(powers.turn()) > 0.05;
        if (driverMoving && !lastDriverMoving) {
            ActionLogger.log("drive");
        }
        lastDriverMoving = driverMoving;

        follower.manual(powers);

        if (Robot.TUNING) {
            ensureTuningLog();

            if (gamepad1.dpadUpWasPressed()) {
                tuningVelocity = Math.max(0, tuningVelocity + 10);
                robot.outtake.setVelocity(tuningVelocity);
            }
            if (gamepad1.dpadDownWasPressed()) {
                tuningVelocity = Math.max(0, tuningVelocity - 10);
                robot.outtake.setVelocity(tuningVelocity);
            }
            if (gamepad1.dpadRightWasPressed()) {
                tuningVelocity = Math.max(0, tuningVelocity + 5);
                robot.outtake.setVelocity(tuningVelocity);
            }
            if (gamepad1.dpadLeftWasPressed()) {
                tuningVelocity = Math.max(0, tuningVelocity - 5);
                robot.outtake.setVelocity(tuningVelocity);
            }
            if (gamepad1.triangleWasPressed()) {
                Outtake.hoodAngle = Math.min(1, Outtake.hoodAngle + 0.01);
            }
            if (gamepad1.circleWasPressed()) {
                Outtake.hoodAngle = Math.max(0, Outtake.hoodAngle - 0.01);
            }
            if (gamepad1.squareWasPressed()) {
                saveTuningSnapshot();
            }
        }

        rightHeld = gamepad1.right_trigger > 0.1;
        leftHeld = gamepad1.left_trigger > 0.1;
        boolean fire = gamepad1.right_bumper;
        boolean flower = gamepad1.left_bumper;
        boolean reset = gamepad1.options;
        boolean manual = gamepad1.share;
        int ballCount = robot.intake.getBallCount();

        boolean pullHeld = rightHeld && !leftHeld;
        boolean pushHeld = leftHeld && !rightHeld;

        if (onRise(pullHeld, lastPull)) {
            RobotCommands.pullBalls(robot.intake).schedule();

        } else if (onRise(pushHeld, lastPush)) {
            RobotCommands.pushBalls(robot.intake).schedule();

        } else if ((onDrop(pullHeld, lastPull) || onDrop(pushHeld, lastPush)) && !pullHeld && !pushHeld) {
            RobotCommands.idleIntake(robot.intake).schedule();

        } else if (onRise(fire, lastFire)) {
            RobotCommands.fireWhileHeld(robot.outtake, robot.intake, robot.ledDriver, () -> gamepad1.right_bumper)
                    .schedule();
        } else if (onRise(flower, lastFlower)) {
            RobotCommands.toggleFlower(robot.intake).schedule();

        } else if (onRise(reset, lastReset)) {
            follower.setPose(new Pose(40.4, 61.8, 0));
            manualMode = ManualMode.FIELD_CENTRIC;

        } else if (onRise(manual, lastManual)) {
            if (manualMode == ManualMode.FIELD_CENTRIC) {
                manualMode = ManualMode.ROBOT_CENTRIC;
            } else {
                manualMode = ManualMode.FIELD_CENTRIC;
            }
        }

        if (onRise(ballCount >= 2, lastBallCount >= 2)) {
            Tracker.setTrackingFrequency(TRACKING_HZ_OUTTAKE);
            robot.ledDriver.playSweepGreen();

        } else if (onDrop(ballCount >= 2, lastBallCount >= 2)) {
            Tracker.setTrackingFrequency(TRACKING_HZ_INTAKE);
            robot.ledDriver.playSweep();
        }

        lastFire = fire;
        lastPull = pullHeld;
        lastPush = pushHeld;
        lastBallCount = ballCount;
        lastFlower = flower;
        lastReset = reset;
        lastManual = manual;
    }

    @Override
    protected void onTelemetry(Telemetry t) {
        Pose robotPose = follower.pose();

        t.addLine("== State ==");
        t.addData(
                "Pose",
                String.format(
                        "(%.1f, %.1f) @ %.1f deg", robotPose.x(), robotPose.y(), Math.toDegrees(robotPose.heading())));

        t.addLine("== Shooter ==");
        t.addData(
                "Speed (cur/target)",
                String.format("%.0f / %.0f", robot.outtake.getCurrentSpeed(), robot.outtake.getTarget()));
        t.addData("Ready", robot.outtake.shootReady());
        t.addData("Pivot pos", String.format("%.3f", robot.outtake.pos));
        t.addData("Hood pos", String.format("%.3f", robot.outtake.hoodPos));

        if (diagnosticsDue()) {
            driveCurrentFLFR = String.format(
                    "%.2f / %.2f", front_left.getCurrent(CurrentUnit.AMPS), front_right.getCurrent(CurrentUnit.AMPS));
            driveCurrentRLRR = String.format(
                    "%.2f / %.2f", rear_left.getCurrent(CurrentUnit.AMPS), rear_right.getCurrent(CurrentUnit.AMPS));
            roller1Current = String.format("%.2f", roller1.getCurrent(CurrentUnit.AMPS));
            roller2Current = String.format("%.2f", roller2.getCurrent(CurrentUnit.AMPS));
            flywheelCurrent = String.format("%.2f", flywheel.getCurrent(CurrentUnit.AMPS));
            intakeMax = String.format("%.2f", robot.intake.getMaxCurrent());
        }

        t.addLine("== Drive currents ==");
        t.addData("FL / FR (A)", driveCurrentFLFR);
        t.addData("RL / RR (A)", driveCurrentRLRR);

        t.addLine("== Diagnostics ==");
        t.addData("Roller 1 current (A)", roller1Current);
        t.addData("Roller 2 current (A)", roller2Current);
        t.addData("Flywheel current (A)", flywheelCurrent);
        t.addData("Intake max (A)", intakeMax);

        t.addLine("== Gamepad ==");
        t.addData("Right Held", rightHeld);
        t.addData("Left Held", leftHeld);
        t.addData("Drive mode", manualMode);

        if (Robot.TUNING) {
            t.addLine("== Tuning ==");
            t.addData("Target velocity", String.format("%.0f", robot.outtake.getTarget()));
            t.addData("Hood angle", String.format("%.2f", Outtake.hoodAngle));
            t.addData("Distance to goal", String.format("%.2f", robotPose.distance(Tracker.getTrackPose())));
            t.addData("Snapshots", tuningLog == null ? 0 : tuningLog.getSnapshotCount());
            t.addData(
                    "File", tuningLog == null ? "-" : tuningLog.getOutputFile().getAbsolutePath());
            if (!tuningError.isEmpty()) {
                t.addData("Save error", tuningError);
            }
        }
    }

    private void ensureTuningLog() {
        if (tuningLog != null) {
            return;
        }

        tuningLog = new FileTelemetry(FileTelemetry.Type.JSON_ARRAY, tuningFileForToday());
        tuningLog.addField("dist");
        tuningLog.addField("vel");
        tuningLog.addField("hood");
    }

    private void saveTuningSnapshot() {
        ensureTuningLog();

        double distance = Robot.follower.pose().distance(Tracker.getTrackPose());
        double velocity = robot.outtake.getTarget();
        double hood = robot.outtake.hoodPos;

        try {
            tuningLog.takeSnapshot(distance, velocity, hood);
            tuningError = "";
        } catch (RuntimeException e) {
            tuningError = e.toString();
        }
    }

    private File tuningFileForToday() {
        String date = new SimpleDateFormat("dd_MM_yyyy", Locale.US).format(new Date());
        File dir = new File(Environment.getExternalStorageDirectory(), "FIRST");
        if (!dir.exists()) {
            dir.mkdirs();
        }
        return new File(dir, date + "_tuning.json");
    }
}
