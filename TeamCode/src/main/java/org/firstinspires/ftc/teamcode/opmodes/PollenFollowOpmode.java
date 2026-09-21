package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.robot.Robot.follower;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.robot.opmode.RobotOpMode;
import org.firstinspires.ftc.teamcode.robot.subsystems.LLLocator;

@Autonomous(name = "PollenFollow", group = "Tests")
public class PollenFollowOpmode extends RobotOpMode {
    LLLocator llLocator;

    TelemetryManager.TelemetryWrapper pTelemetry;
    Path p = null;

    private boolean following;

    @Override
    protected void onInit() {
        llLocator = robot.llLocator;

        pTelemetry = PanelsTelemetry.INSTANCE.getTelemetry().getWrapper();
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

        pTelemetry.addData("Valid target", llLocator.hasValidTarget());
        pTelemetry.addData("Robot pose", robotPose);
        pTelemetry.addData("Pollen pose", pollen);
        pTelemetry.addData("Distance", robotPose.distance(pollen));
        pTelemetry.addData("Following", following);
        pTelemetry.addData("Path", p);
        pTelemetry.update();
    }
}
