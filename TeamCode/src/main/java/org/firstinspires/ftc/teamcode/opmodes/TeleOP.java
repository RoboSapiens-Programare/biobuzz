package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.robot.Robot.follower;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.commands.RobotCommands;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.opmode.RobotOpMode;
import org.firstinspires.ftc.teamcode.utils.controlSystems.PIDFController;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;

@TeleOp
@Configurable
public class TeleOP extends RobotOpMode {

    private enum States {
        INTAKE,
        OUTTAKE
    };

    private States state;

    public static boolean TAIE_DEGETE_LU_COCO = true;

    private final ElapsedTime debounceTimer = new ElapsedTime();

    private CachingDcMotorEx front_left, front_right, rear_left, rear_right;

    private final PIDFController pidf = new PIDFController(FlywheelConfig.kp, FlywheelConfig.ki, FlywheelConfig.kd, 0.1, -1, 1);
    private CachingDcMotorEx flywheelLeft, flywheelRight;

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
    }

    @Override
    protected void onInit() {
        changeState(States.INTAKE);

        front_left = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "front_left"));
        front_right = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "front_right"));
        rear_left = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "rear_left"));
        rear_right = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "rear_right"));

        flywheelLeft = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "flywheel_left"));
        flywheelRight = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "flywheel_right"));
        flywheelRight.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheelLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheelRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.outtake.setManualFlywheel(true);

        follower.setHeading(0);
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
                if (gamepad1.right_trigger > 0.1 && TAIE_DEGETE_LU_COCO) {
                    RobotCommands.pullBalls(robot.intake).schedule();
                } else if (gamepad1.right_bumper) {
                    RobotCommands.pushBalls(robot.intake).schedule();
                } else {
                    RobotCommands.idleIntake(robot.intake).schedule();
                }

                if (gamepad1.cross && debounceTimer.milliseconds() > 100) {
                    changeState(States.OUTTAKE);

                    debounceTimer.reset();
                }
                break;

            case OUTTAKE:
                if (gamepad1.right_trigger > 0.1) {
                    RobotCommands.pullBalls(robot.intake).schedule();
//                    RobotCommands.fireShooter(robot.outtake).schedule();
                } else {
//                    RobotCommands.stopFiring(robot.outtake).schedule();
                    RobotCommands.idleIntake(robot.intake).schedule();
                }

                if (gamepad1.cross && debounceTimer.milliseconds() > 100) {
                    changeState(States.INTAKE);

                    debounceTimer.reset();
                }
                break;
        }

        pidf.kP = FlywheelConfig.kp;
        pidf.kI = FlywheelConfig.ki;
        pidf.kD = FlywheelConfig.kd;

        if (gamepad1.left_trigger_pressed)
            pidf.setSetpoint(FlywheelConfig.target);
        else pidf.setSetpoint(-800);


        double p = pidf.update(-flywheelLeft.getVelocity());
        flywheelLeft.setPower(p);
        flywheelRight.setPower(p);

        Pose robotPose = follower.pose();

        // TELEMETRY
        telemetry.addData("State", state);
        telemetry.addData("Robot X", robotPose.x());
        telemetry.addData("Robot Y", robotPose.y());
        telemetry.addData("Robot Heading", Math.toDegrees(robotPose.heading()));

        telemetry.addData("current speed", flywheelLeft.getVelocity());
        telemetry.addData("target speed", FlywheelConfig.target);
        telemetry.addData("p", p);
        telemetry.addData("kp", FlywheelConfig.kp);
        telemetry.addData("ki", FlywheelConfig.ki);
        telemetry.addData("kd", FlywheelConfig.kd);

        telemetry.addData("Front left current", front_left.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Front right current", front_right.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Rear left current", rear_left.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Rear right current", rear_right.getCurrent(CurrentUnit.AMPS));

    }
}