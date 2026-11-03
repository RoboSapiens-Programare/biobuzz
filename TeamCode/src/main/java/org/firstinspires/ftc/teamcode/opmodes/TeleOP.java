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
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.commands.RobotCommands;
import org.firstinspires.ftc.teamcode.robot.opmode.RobotOpMode;
import org.firstinspires.ftc.teamcode.robot.subsystems.Outtake;

@TeleOp
@Configurable
public class TeleOP extends RobotOpMode {
    public static final boolean COMPETITION = false;

    {
        panelsTelemetry = !COMPETITION;
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

    private ElapsedTime sweepTimer = new ElapsedTime();
    private double sweepTarget = -Outtake.PIVOT_RANGE / 2;
    private double sweepDelta = Outtake.PIVOT_RANGE / 2;
    private int sweepStep = 0;

    private void changeState(States newState) {
        state = newState;

        switch (state) {
            case INTAKE:
                RobotCommands.idleShooter(robot.outtake).schedule();
                break;
            case OUTTAKE:
                RobotCommands.spoolShooter(robot.outtake).schedule();
                break;
        }

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

        // Set initial state actions safely on init
        changeState(States.INTAKE);
    }

    @Override
    protected void onStart() {
        debounceTimer.reset();
    }

    @Override
    protected void onUpdate() {
        DrivePowers powers = ManualDrive.fieldCentric(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                follower.pose().heading());

        follower.manual(powers);

        if (gamepad1.crossWasPressed()) {
            toggleState();
        }

        switch (state) {
            case INTAKE:
                if (gamepad1.right_trigger_pressed) {
                    RobotCommands.pullBalls(robot.intake).schedule();
                } else if (gamepad1.left_trigger_pressed) {
                    RobotCommands.pushBalls(robot.intake).schedule();
                } else {
                    RobotCommands.idleIntake(robot.intake).schedule();
                }
                break;

            case OUTTAKE:
                if (gamepad1.rightTriggerWasPressed()) {
                    RobotCommands.fireShooter(robot.outtake, robot.intake).schedule();
                } else if (gamepad1.rightTriggerWasReleased()) {
                    RobotCommands.stopFiring(robot.outtake).schedule();
                }
                break;
        }

        if (sweepTimer.milliseconds() > 400) {
            sweepStep = (sweepStep + 1) % 4;
            robot.outtake.aimAt(sweepTarget);
            sweepTarget += sweepDelta;
            if (sweepStep == 2 || sweepStep == 0) {
                sweepDelta *= -1;
            }
            sweepTimer.reset();
        }

        Pose robotPose = follower.pose();

        final double BASE_OMEGA_FREE = 6000.0;
        final double BASE_ISTALL = 9.2;
        final double BASE_INOLOAD = 0.25;

        double vActual = myExpansionHubVoltageSensor.getVoltage();
        double vNominal = 12.0;

        double vRatio = vActual / vNominal;
        double omegaFreeAdj = BASE_OMEGA_FREE * vRatio;
        double iStallAdj = BASE_ISTALL * vRatio;
        double iNoLoadAdj = BASE_INOLOAD * vRatio; // (Or use your measured live I_no_load)

        double iMeasured = roller.getCurrent(CurrentUnit.AMPS);

        double omegaLimit = omegaFreeAdj * (1.0 - (2.0 * (iMeasured - iNoLoadAdj)) / iStallAdj);

        omegaLimit = Math.max(0.0, omegaLimit);

        // TELEMETRY
        pTelemetry.addLine("== State ==");
        pTelemetry.addData("Mode", state);
        pTelemetry.addData(
                "Pose",
                String.format(
                        "(%.1f, %.1f) @ %.1f deg", robotPose.x(), robotPose.y(), Math.toDegrees(robotPose.heading())));

        pTelemetry.addLine("== Shooter ==");
        pTelemetry.addData(
                "Speed (cur/target)",
                String.format("%.0f / %.0f", robot.outtake.getCurrentSpeed(), robot.outtake.getTarget()));
        pTelemetry.addData("Ready", robot.outtake.shootReady());
        pTelemetry.addData("Pivot pos", String.format("%.3f", robot.outtake.pos));

        pTelemetry.addLine("== Drive currents ==");
        pTelemetry.addData(
                "FL / FR (A)",
                String.format(
                        "%.2f / %.2f",
                        front_left.getCurrent(CurrentUnit.AMPS), front_right.getCurrent(CurrentUnit.AMPS)));
        pTelemetry.addData(
                "RL / RR (A)",
                String.format(
                        "%.2f / %.2f",
                        rear_left.getCurrent(CurrentUnit.AMPS), rear_right.getCurrent(CurrentUnit.AMPS)));

        pTelemetry.addLine("== Diagnostics ==");
        pTelemetry.addData("Roller current (A)", String.format("%.2f", roller.getCurrent(CurrentUnit.AMPS)));
        pTelemetry.addData("Flywheel current (A)", String.format("%.2f", flywheel.getCurrent(CurrentUnit.AMPS)));
        pTelemetry.addData("Omega limit", String.format("%.1f", omegaLimit));
        pTelemetry.addData("Intake max (A)", String.format("%.2f", robot.intake.getMaxCurrent()));
    }
}
