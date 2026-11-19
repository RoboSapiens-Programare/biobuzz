package org.firstinspires.ftc.teamcode.opmodes.tests;

import static org.firstinspires.ftc.teamcode.robot.Robot.follower;

import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.opmode.RobotOpMode;
import org.firstinspires.ftc.teamcode.robot.subsystems.LLLocator;

@Autonomous(name = "PollenFollow", group = "Tests")
public class PollenFollowOpmode extends RobotOpMode {
    LLLocator llLocator;

    Path p = null;

    private boolean following;

    @Override
    protected void onInit() {
        llLocator = robot.llLocator;

        follower.setPose(new Pose(0, 0, 0));
    }

    @Override
    protected void onUpdate() {
        Pose robotPose = follower.pose();
        Pose pollen = llLocator.getPollenAbsolutePose(robotPose);

        if (!following && llLocator.hasValidTarget()) {
            following = true;
            p = llLocator.buildPathToPollen(robotPose);
            follower.follow(p);
        }

        if (following && !follower.isBusy()) {
            following = false;
        }
    }

    @Override
    protected void onTelemetry(Telemetry t) {
        Pose robotPose = follower.pose();
        Pose pollen = llLocator.getPollenAbsolutePose(robotPose);

        t.addData("Valid target", llLocator.hasValidTarget());
        t.addData("Robot pose", robotPose);
        t.addData("Pollen pose", pollen);
        t.addData("Distance", robotPose.distance(pollen));
        t.addData("Following", following);
        t.addData("Path", p);
    }
}
