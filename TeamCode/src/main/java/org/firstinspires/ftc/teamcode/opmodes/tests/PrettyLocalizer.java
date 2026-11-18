package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.opmode.RobotOpMode;

@Autonomous
public class PrettyLocalizer extends RobotOpMode {
    @Override
    protected void onInit() {
        Robot.follower.setPose(new Pose(54.5, 8, Math.toRadians(-90)));
    }

    @Override
    protected void onUpdate() {}

    @Override
    protected void onTelemetry(Telemetry t) {
        Pose p = Robot.follower.pose();

        t.addData("Pose", p);
        t.addData("X", p.x());
        t.addData("Y", p.y());
        t.addData("H", Math.toDegrees(p.heading()));
        t.addLine("  == PEDRO == ");
        t.addData(
                "Pose", String.format("poseFactory.of(%.3f, %.3f, %.3f);", p.x(), p.y(), Math.toDegrees(p.heading())));
    }
}
