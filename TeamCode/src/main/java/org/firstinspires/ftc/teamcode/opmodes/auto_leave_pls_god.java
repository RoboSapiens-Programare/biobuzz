package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.robot.Robot.follower;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.commands.RobotCommands;
import org.firstinspires.ftc.teamcode.robot.opmode.RobotOpMode;
import org.firstinspires.ftc.teamcode.utils.controlSystems.PIDFController;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;

@Autonomous
@Configurable
public class auto_leave_pls_god extends RobotOpMode {
    ElapsedTime moveTimer = new ElapsedTime();
        @Override
    protected void onInit() {
        follower.setHeading(0);
    }

    @Override
    protected void onStart() {
        moveTimer.reset();
    }

    @Override
    protected void onUpdate() {
        DrivePowers powers_fwd = ManualDrive.fieldCentric(
                1,
                0,
                0,
                follower.pose().heading()
        );

        DrivePowers powers_stop = ManualDrive.fieldCentric(
                0,
                0,
                0,
                follower.pose().heading()
        );

        if (moveTimer.milliseconds() < 2 * 1000) {
            follower.manual(powers_fwd);
        } else {
            follower.manual(powers_stop);
            requestOpModeStop();
        }

    }
}