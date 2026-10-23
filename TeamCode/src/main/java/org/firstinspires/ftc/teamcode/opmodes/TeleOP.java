package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.robot.Robot.follower;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
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

@TeleOp
@Configurable
public class TeleOP extends RobotOpMode {

    private enum States {
        INTAKE,
        OUTTAKE
    };

    // Initialize state to avoid NullPointerException
    private States state = States.INTAKE;

    private final ElapsedTime debounceTimer = new ElapsedTime();

    private CachingDcMotorEx front_left, front_right, rear_left, rear_right, roller, flywheel;
    private VoltageSensor myExpansionHubVoltageSensor;

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

        Pose robotPose = follower.pose();

        // Constants from the 6000 RPM goBILDA datasheet @ 12V
        final double BASE_OMEGA_FREE = 6000.0;
        final double BASE_ISTALL = 9.2;
        final double BASE_INOLOAD = 0.25;

// Read live battery voltage from the Control Hub
        double vActual = myExpansionHubVoltageSensor.getVoltage();
        double vNominal = 12.0;

// Step 1: Scale constants to live voltage
        double vRatio = vActual / vNominal;
        double omegaFreeAdj = BASE_OMEGA_FREE * vRatio;
        double iStallAdj = BASE_ISTALL * vRatio;
        double iNoLoadAdj = BASE_INOLOAD * vRatio; // (Or use your measured live I_no_load)

// Step 2: Read current sensor from your intake motor
        double iMeasured = roller.getCurrent(CurrentUnit.AMPS);

// Step 3: Calculate safe RPM limit with 2x safety factor
        double omegaLimit = omegaFreeAdj * (1.0 - (2.0 * (iMeasured - iNoLoadAdj)) / iStallAdj);

// Ensure limit doesn't drop below zero
        omegaLimit = Math.max(0.0, omegaLimit);

// Compare your actual output RPM against omegaLimit (accounting for your gearbox ratio)
        // TELEMETRY
        pTelemetry.addData("State", state);
        pTelemetry.addData("Robot X", robotPose.x());
        pTelemetry.addData("Robot Y", robotPose.y());
        pTelemetry.addData("Robot Heading", Math.toDegrees(robotPose.heading()));
        pTelemetry.addData("Flywheel Powers", front_left.getPower() + " | " + front_right.getPower());
        pTelemetry.addData("Shooter speed", robot.outtake.getCurrentSpeed());
        pTelemetry.addData("Shooter target", robot.outtake.getTarget());
        pTelemetry.addData("Front left current", front_left.getCurrent(CurrentUnit.AMPS));
        pTelemetry.addData("Front right current", front_right.getCurrent(CurrentUnit.AMPS));
        pTelemetry.addData("Rear left current", rear_left.getCurrent(CurrentUnit.AMPS));
        pTelemetry.addData("Rear right current", rear_right.getCurrent(CurrentUnit.AMPS));
        pTelemetry.addData("Roller current", roller.getCurrent(CurrentUnit.AMPS));
        pTelemetry.addData("Omega Limit", omegaLimit);
        pTelemetry.addData("Current flywheel", flywheel.getCurrent(CurrentUnit.AMPS));
        pTelemetry.addData("INTAKE MAX CURRENT", this.robot.intake.getMaxCurrent());

        pTelemetry.update();
    }
}
