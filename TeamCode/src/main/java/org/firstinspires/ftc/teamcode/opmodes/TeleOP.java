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
import com.qualcomm.robotcore.util.ElapsedTime;
import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;
// import dev.frozenmilk.dairy.pasteurized.Pasteurized;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.commands.RobotCommands;
import org.firstinspires.ftc.teamcode.robot.opmode.RobotOpMode;

@TeleOp
@Configurable
public class TeleOP extends RobotOpMode {
    TelemetryManager.TelemetryWrapper pTelemetry;

    private enum States {
        INTAKE,
        OUTTAKE
    };

    // Initialize state to avoid NullPointerException
    private States state = States.INTAKE;

    private final ElapsedTime debounceTimer = new ElapsedTime();

    private CachingDcMotorEx front_left, front_right, rear_left, rear_right;

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
        front_left = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "flywheel_left"));
        front_right = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "flywheel_right"));
        rear_left = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "rear_left"));
        rear_right = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "rear_right"));

        follower.setHeading(0);

        // Set initial state actions safely on init
        changeState(States.INTAKE);

        pTelemetry = PanelsTelemetry.INSTANCE.getTelemetry().getWrapper();
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

        pTelemetry.update();
    }
}
