package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.robot.Robot.follower;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.commands.RobotCommands;
import org.firstinspires.ftc.teamcode.robot.opmode.RobotOpMode;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;

@TeleOp
@Configurable
public class test_motoare_powah_1 extends OpMode {
    private DcMotorEx left, right, roller;

    @Override
    public void init() {
        left = hardwareMap.get(DcMotorEx.class, "flywheel_left");
        right = hardwareMap.get(DcMotorEx.class, "flywheel_right");

        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        right.setDirection(DcMotorSimple.Direction.REVERSE);

        roller = hardwareMap.get(DcMotorEx.class, "roller_motor");
        roller.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        telemetry.addData("v", left.getVelocity());
        telemetry.update();
        left.setPower(0.65);
        right.setPower(0.65);

        roller.setPower(0.7);
    }
}