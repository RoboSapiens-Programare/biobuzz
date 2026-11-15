package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.opmode.RobotOpMode;

@Autonomous
public class PrettyLocalizer extends RobotOpMode {
    @Override
    protected void onUpdate() {
        pTelemetry.addData("Pose", Robot.follower.pose());
        pTelemetry.addData("X", Robot.follower.pose().x());
        pTelemetry.addData("Y", Robot.follower.pose().y());
        pTelemetry.addData("H", Math.toDegrees(Robot.follower.pose().heading()));

    }
}
