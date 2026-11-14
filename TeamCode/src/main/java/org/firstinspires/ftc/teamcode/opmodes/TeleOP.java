package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.robot.Robot.follower;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;
// import dev.frozenmilk.dairy.pasteurized.Pasteurized;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;
import java.util.Scanner;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.opmode.RobotOpMode;
import org.firstinspires.ftc.teamcode.robot.subsystems.Tracker;
import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

@TeleOp
@Configurable
public class TeleOP extends RobotOpMode {
    public static final boolean COMPETITION = false;

    private static final double TRACKING_HZ_INTAKE = 30;
    private static final double TRACKING_HZ_OUTTAKE = 70;

    {
        panelsTelemetry = !COMPETITION;
        telemetryHz = COMPETITION ? 10 : 20;
    }

    private enum States {
        INTAKE,
        OUTTAKE
    };

    // Initialize state to avoid NullPointerException
    private States state = States.INTAKE;

    private final ElapsedTime debounceTimer = new ElapsedTime();

    private CachingDcMotorEx front_left, front_right, rear_left, rear_right, roller, flywheel;
    private VoltageSensor myExpansionHubVoltageSensor;

    private boolean firing;
    private boolean lastFire, lastPull, lastPush;
    private String driveCurrentFLFR = "",
            driveCurrentRLRR = "",
            rollerCurrent = "",
            flywheelCurrent = "",
            intakeMax = "";

    private double tuningVelocity = 1100;
    private int tuningSnapshotCount;
    private File tuningFile;
    private String tuningError = "";

    private static boolean onRise(boolean now, boolean last) {
        return now && !last;
    }

    private static boolean onDrop(boolean now, boolean last) {
        return !now && last;
    }

    private void changeState(States newState) {
        state = newState;

        Tracker.setTrackingFrequency(state == States.OUTTAKE ? TRACKING_HZ_OUTTAKE : TRACKING_HZ_INTAKE);

        switch (state) {
            case INTAKE:
                robot.outtake.setFiring(false);
                robot.outtake.startIdling();
                robot.intake.idle();
                break;
            case OUTTAKE:
                robot.outtake.stopIdling();
                break;
        }

        // Require the triggers to be released and pressed again after a state change.
        boolean rightHeld = gamepad1.right_trigger > 0.1;
        boolean leftHeld = gamepad1.left_trigger > 0.1;
        lastFire = rightHeld;
        lastPull = rightHeld && !leftHeld;
        lastPush = leftHeld && !rightHeld;
        firing = false;

        debounceTimer.reset();
    }

    private void toggleState() {
        if (state == States.INTAKE) changeState(States.OUTTAKE);
        else changeState(States.INTAKE);
    }

    @Override
    protected void onInit() {
        front_left = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "front_left"));
        front_right = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "front_right"));
        rear_left = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "rear_left"));
        rear_right = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "rear_right"));
        roller = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "roller_motor"));
        flywheel = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "flywheel"));

        myExpansionHubVoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");

        follower.setHeading(0);
        robot.setLLLocatorEnabled(false);

        // Set initial state actions safely on init
        changeState(States.INTAKE);

        if (Robot.TUNING) {
            robot.outtake.setVelocity(tuningVelocity);
            tuningFile = tuningFileForToday();
            tuningSnapshotCount = loadTuningPoints(tuningFile).length();
        }
    }

    @Override
    protected void onInitLoop() {
        if (gamepad1.touchpadWasPressed()) {
            Robot.alliance = Robot.alliance == Robot.Alliance.BLUE ? Robot.Alliance.RED : Robot.Alliance.BLUE;
            gamepad1.rumble(10);
            gamepad1.setLedColor(
                    Robot.alliance == Robot.Alliance.RED ? 255 : 0,
                    0,
                    Robot.alliance == Robot.Alliance.BLUE ? 255 : 0,
                    0);
        }
    }

    @Override
    protected void onStart() {
        Tracker.setEnabled(true);
        Tracker.setTrackingFrequency(state == States.OUTTAKE ? TRACKING_HZ_OUTTAKE : TRACKING_HZ_INTAKE);
        debounceTimer.reset();

        follower.setPose(new Pose(7.2, 8, Math.PI / 2));
    }

    @Override
    protected boolean shouldExecuteScheduler() {
        return false;
    }

    @Override
    protected void onUpdate() {
        Pose robotPose = follower.pose();

        DrivePowers powers = ManualDrive.fieldCentric(
                -gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, robotPose.heading());

        follower.manual(powers);

        if (Robot.TUNING) {
            if (gamepad1.dpadUpWasPressed()) {
                tuningVelocity = Math.max(0, tuningVelocity + 10);
                robot.outtake.setVelocity(tuningVelocity);
            }
            if (gamepad1.dpadDownWasPressed()) {
                tuningVelocity = Math.max(0, tuningVelocity - 10);
                robot.outtake.setVelocity(tuningVelocity);
            }
            if (gamepad1.squareWasPressed()) {
                saveTuningSnapshot();
            }
        }

        if (gamepad1.crossWasPressed()) {
            toggleState();
        }

        boolean rightHeld = gamepad1.right_trigger > 0.1;
        boolean leftHeld = gamepad1.left_trigger > 0.1;

        switch (state) {
            case INTAKE: {
                boolean pullHeld = rightHeld && !leftHeld;
                boolean pushHeld = leftHeld && !rightHeld;

                if (onRise(pullHeld, lastPull)) {
                    robot.intake.pullBalls();
                }
                if (onRise(pushHeld, lastPush)) {
                    robot.intake.pushBalls();
                }
                if ((onDrop(pullHeld, lastPull) || onDrop(pushHeld, lastPush)) && !pullHeld && !pushHeld) {
                    robot.intake.idle();
                }

                lastPull = pullHeld;
                lastPush = pushHeld;
                break;
            }

            case OUTTAKE: {
                if (onRise(rightHeld, lastFire)) {
                    robot.outtake.setFiring(true);
                    firing = true;
                } else if (onDrop(rightHeld, lastFire) && firing) {
                    robot.outtake.setFiring(false);
                    robot.outtake.startIdling();
                    robot.intake.idle();
                    state = States.INTAKE;
                    firing = false;
                }

                if (firing) {
                    if (robot.outtake.shootReady()) {
                        robot.intake.pullBalls();
                    } else {
                        robot.intake.idle();
                    }
                }

                lastFire = rightHeld;
                break;
            }
        }

        // TELEMETRY
        if (telemetryDue()) {
            pTelemetry.addLine("== State ==");
            pTelemetry.addData("Mode", state);
            pTelemetry.addData(
                    "Pose",
                    String.format(
                            "(%.1f, %.1f) @ %.1f deg",
                            robotPose.x(), robotPose.y(), Math.toDegrees(robotPose.heading())));

            pTelemetry.addLine("== Shooter ==");
            pTelemetry.addData(
                    "Speed (cur/target)",
                    String.format("%.0f / %.0f", robot.outtake.getCurrentSpeed(), robot.outtake.getTarget()));
            pTelemetry.addData("Ready", robot.outtake.shootReady());
            pTelemetry.addData("Pivot pos", String.format("%.3f", robot.outtake.pos));

            driveCurrentFLFR = String.format(
                    "%.2f / %.2f", front_left.getCurrent(CurrentUnit.AMPS), front_right.getCurrent(CurrentUnit.AMPS));
            driveCurrentRLRR = String.format(
                    "%.2f / %.2f", rear_left.getCurrent(CurrentUnit.AMPS), rear_right.getCurrent(CurrentUnit.AMPS));
            rollerCurrent = String.format("%.2f", roller.getCurrent(CurrentUnit.AMPS));
            flywheelCurrent = String.format("%.2f", flywheel.getCurrent(CurrentUnit.AMPS));
            intakeMax = String.format("%.2f", robot.intake.getMaxCurrent());

            pTelemetry.addLine("== Drive currents ==");
            pTelemetry.addData("FL / FR (A)", driveCurrentFLFR);
            pTelemetry.addData("RL / RR (A)", driveCurrentRLRR);

            pTelemetry.addLine("== Diagnostics ==");
            pTelemetry.addData("Roller current (A)", rollerCurrent);
            pTelemetry.addData("Flywheel current (A)", flywheelCurrent);
            pTelemetry.addData("Intake max (A)", intakeMax);

            if (Robot.TUNING) {
                pTelemetry.addLine("== Tuning ==");
                pTelemetry.addData("Target velocity", String.format("%.0f", robot.outtake.getTarget()));
                pTelemetry.addData(
                        "Distance to goal", String.format("%.2f", robotPose.distance(Tracker.getTrackPose())));
                pTelemetry.addData("Snapshots", tuningSnapshotCount);
                pTelemetry.addData("File", tuningFile == null ? "-" : tuningFile.getAbsolutePath());
                if (!tuningError.isEmpty()) {
                    pTelemetry.addData("Save error", tuningError);
                }
            }
        }

        // Omega limit block disabled until it is fed into the shooter.
        // final double BASE_OMEGA_FREE = 6000.0;
        // final double BASE_ISTALL = 9.2;
        // final double BASE_INOLOAD = 0.25;
        //
        // double vActual = myExpansionHubVoltageSensor.getVoltage();
        // double vNominal = 12.0;
        //
        // double vRatio = vActual / vNominal;
        // double omegaFreeAdj = BASE_OMEGA_FREE * vRatio;
        // double iStallAdj = BASE_ISTALL * vRatio;
        // double iNoLoadAdj = BASE_INOLOAD * vRatio; // (Or use your measured live I_no_load)
        //
        // double iMeasured = roller.getCurrent(CurrentUnit.AMPS);
        //
        // double omegaLimit = omegaFreeAdj * (1.0 - (2.0 * (iMeasured - iNoLoadAdj)) / iStallAdj);
        //
        // omegaLimit = Math.max(0.0, omegaLimit);
        // pTelemetry.addData("Omega limit", String.format("%.1f", omegaLimit));
    }

    private void saveTuningSnapshot() {
        tuningFile = tuningFileForToday();
        JSONArray points = loadTuningPoints(tuningFile);

        double distance = Robot.follower.pose().distance(Tracker.getTrackPose());
        double velocity = robot.outtake.getTarget();

        try {
            points.put(new JSONObject().put("dist", distance).put("vel", velocity));
            try (FileWriter writer = new FileWriter(tuningFile, false)) {
                writer.write(points.toString(2));
            }
            tuningSnapshotCount = points.length();
            tuningError = "";
        } catch (JSONException | IOException e) {
            tuningError = e.toString();
        }
    }

    private File tuningFileForToday() {
        String date = new SimpleDateFormat("dd_MM_yyyy", Locale.US).format(new Date());
        return new File(hardwareMap.appContext.getFilesDir(), date + "_tuning.json");
    }

    private JSONArray loadTuningPoints(File file) {
        if (!file.exists()) {
            return new JSONArray();
        }

        try (Scanner scanner = new Scanner(file)) {
            StringBuilder contents = new StringBuilder();
            while (scanner.hasNextLine()) {
                contents.append(scanner.nextLine());
            }
            if (contents.length() > 0) {
                return new JSONArray(contents.toString());
            }
        } catch (Exception e) {
            tuningError = e.toString();
        }
        return new JSONArray();
    }
}
