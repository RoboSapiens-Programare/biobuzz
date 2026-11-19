package org.firstinspires.ftc.teamcode;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.opmode.RobotOpMode;
import org.firstinspires.ftc.teamcode.robot.subsystems.Tracker;

@Autonomous(name = "Auto -- v2", group = "Autonomous")
public class AutoV2 extends RobotOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(54.5, 8, 90);
    private final Pose goGrabGarden = poseFactory.of(8, 13, -90);
    private final Pose grabGardenOne = poseFactory.of(8, 8, -90);
    private final Pose grabGardenTwo = poseFactory.of(8, 8, -90);
    private final Pose goGrabFlowerOne = poseFactory.of(17.9215, 46.8422, -104.3288);
    private final Pose grabFlowerOne = poseFactory.of(8.5451, 47.0259, -1.1221);
    private final Pose shootLeftStart = poseFactory.of(8.5451, 47.0259, -90);
    private final Pose shootLeft = poseFactory.of(46.7, 125, 90);
    private final Pose shootLeftControl1 = poseFactory.of(33.4716, 65.5688, 0);
    private final Pose shootLeftControl2 = poseFactory.of(22.5776, 102.0504, 0);
    private final Pose grabFlowerTwo = poseFactory.of(46.7, 130, 90);
    private final Pose parkStart = poseFactory.of(46.7, 130, 90);
    private final Pose park = poseFactory.of(10.6891, 114.5117, 0);

    private String autoStep = "idle";
    private final ElapsedTime stepTimer = new ElapsedTime();

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, goGrabGarden()),
                follow(follower, grabGarden()),
                follow(follower, goGrabFlowerOne()),
                follow(follower, grabFlowerOne()),
                follow(follower, shootLeft()),
                follow(follower, grabFlower()),
                follow(follower, park()));
    }

    @Override
    protected void onInit() {
        Robot.follower.setPose(start);

        Tracker.setEnabled(true);

        robot.outtake.aimAt(0);
        robot.outtake.stopIdling();

        autoStep = "init";
        stepTimer.reset();

        schedule(autoRoutine());
    }

    @Override
    protected void onUpdate() {}

    @Override
    protected void onTelemetry(Telemetry t) {
        t.addData("Auto step", autoStep);
        t.addData("Step seconds", String.format("%.1f", stepTimer.seconds()));
        t.addData("Ball count", robot.intake.getBallCount());
        t.addData("Shooter ready", robot.outtake.shootReady());
        t.addData("Shooter speed", String.format("%.0f", robot.outtake.getCurrentSpeed()));
        t.addData("Shooter target", String.format("%.0f", robot.outtake.getTarget()));
        t.addData("Shooter error", String.format("%.0f", robot.outtake.getTarget() - robot.outtake.getCurrentSpeed()));

        Pose robotPose = Robot.follower.pose();
        Pose trackPose = Tracker.getTrackPose();
        t.addData("Track pose", String.format("%.1f, %.1f", trackPose.x(), trackPose.y()));
        t.addData("Track distance", String.format("%.1f", robotPose.distance(trackPose)));
        t.addData("Tracker velocity", String.format("%.0f", robot.tracker.getVelocity()));

        if (Robot.follower.currentPath() != null) {
            t.addData("Current path distance remaining", Robot.follower.distanceToEndpoint());
            t.addData("Path number", Robot.follower.pathIndex());
        }

        if (autoStep.equals("park") || autoStep.equals("park timeout")) {
            t.addData("Follower mode", Robot.follower.mode());
            t.addData("Following", Robot.follower.following());
            t.addData("Busy", Robot.follower.isBusy());
            t.addData("At para end", Robot.follower.atParametricEnd());
            t.addData("Completion", String.format("%.3f", Robot.follower.completion()));
            t.addData("Remaining", String.format("%.2f", Robot.follower.remainingDistance()));

            Pose pose = Robot.follower.pose();
            t.addData("Pose", String.format("%.2f, %.2f, %.1f", pose.x(), pose.y(), Math.toDegrees(pose.heading())));
        }
    }

    public Path goGrabGarden() {
        return Paths.line(start, goGrabGarden).constant(goGrabGarden);
    }

    public Path grabGarden() {
        return Paths.path(
                Paths.line(goGrabGarden, grabGardenOne).constant(grabGardenOne),
                Paths.line(grabGardenOne, grabGardenTwo).linear(grabGardenTwo, grabGardenOne));
    }

    public Path goGrabFlowerOne() {
        return Paths.line(grabGardenTwo, goGrabFlowerOne).reverseTangent();
    }

    public Path grabFlowerOne() {
        return Paths.line(goGrabFlowerOne, grabFlowerOne).reverseTangent();
    }

    public Path shootLeft() {
        return Paths.curve(shootLeftStart, shootLeftControl1, shootLeftControl2, shootLeft)
                .linear(shootLeftStart, shootLeft);
    }

    public Path grabFlower() {
        return Paths.line(shootLeft, grabFlowerTwo).constant(grabFlowerTwo);
    }

    public Path park() {
        return Paths.line(parkStart, park).linear(parkStart, park);
    }
}
