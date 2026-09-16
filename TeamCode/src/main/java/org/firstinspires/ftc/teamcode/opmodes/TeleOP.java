package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.robot.Robot.follower;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.commands.RobotCommands;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.opmode.RobotOpMode;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;

@TeleOp
@Configurable
public class TeleOP extends RobotOpMode {

    private enum States {
        INTAKE,
        OUTTAKE
    };

    private States state;


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

    @Override
    protected void onInit() {
        front_left = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "front_left"));
        front_right = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "front_right"));
        rear_left = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "rear_left"));
        rear_right = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "rear_right"));

        changeState(States.INTAKE);

        follower.setHeading(0);
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
                follower.pose().heading()
        );

        follower.manual(powers);

        switch (state) {
            case INTAKE:
                if (gamepad1.right_trigger_pressed) {
                    RobotCommands.pullBalls(robot.intake).schedule();
                } else if (gamepad1.left_trigger_pressed) {
                    RobotCommands.pushBalls(robot.intake).schedule();
                } else {
                    RobotCommands.idleIntake(robot.intake).schedule();
                }

                if (gamepad1.cross && debounceTimer.milliseconds() > 20) {
                    changeState(States.OUTTAKE);
                }
                break;

            case OUTTAKE:
                if (gamepad1.right_trigger_pressed) {
                    RobotCommands.fireShooter(robot.outtake).schedule();
                } else {
                    RobotCommands.stopFiring(robot.outtake).schedule();
                }

                if (gamepad1.cross && debounceTimer.milliseconds() > 20) {
                    changeState(States.INTAKE);
                }
                break;
        }

        Pose robotPose = follower.pose();

        // TELEMETRY
        telemetry.addData("State", state);
        telemetry.addData("Robot X", robotPose.x());
        telemetry.addData("Robot Y", robotPose.y());
        telemetry.addData("Robot Heading", Math.toDegrees(robotPose.heading()));

        telemetry.addData("Shooter speed", robot.outtake.getCurrentSpeed());
        telemetry.addData("Shooter target", robot.outtake.getTarget());
        telemetry.addData("Front left current", front_left.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Front right current", front_right.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Rear left current", rear_left.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Rear right current", rear_right.getCurrent(CurrentUnit.AMPS));

    }
}