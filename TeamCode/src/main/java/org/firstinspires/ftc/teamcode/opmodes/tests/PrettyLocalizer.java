package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.opmode.RobotOpMode;

@Autonomous
public class PrettyLocalizer extends RobotOpMode {
    @Override
    protected void onInit() {
        Robot.follower.setPose(new Pose(54.5, 8, Math.toRadians(-90)));
    }

    @Override
    protected void onUpdate() {
        Pose p = Robot.follower.pose();

        pTelemetry.addData("Pose", p);
        pTelemetry.addData("X", p.x());
        pTelemetry.addData("Y", p.y());
        pTelemetry.addData("H", Math.toDegrees(p.heading()));
        pTelemetry.addLine("  == PEDRO == ");
        pTelemetry.addData(
                "Pose", String.format("poseFactory.of(%.3f, %.3f, %.3f);", p.x(), p.y(), Math.toDegrees(p.heading())));

        telemetry.addData("Pose", p);
        telemetry.addData("X", p.x());
        telemetry.addData("Y", p.y());
        telemetry.addData("H", Math.toDegrees(p.heading()));
    }
}
