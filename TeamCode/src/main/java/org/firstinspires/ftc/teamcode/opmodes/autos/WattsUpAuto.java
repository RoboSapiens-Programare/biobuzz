package org.firstinspires.ftc.teamcode.opmodes.autos;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.RobotCommands;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.opmode.RobotOpMode;
import org.firstinspires.ftc.teamcode.robot.subsystems.Tracker;

@Autonomous(name = "WATTs UP auto", group = "Autonomous")
public class WattsUpAuto extends RobotOpMode {

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(54.5, 8, -90);
    private final Pose goGrabGarden = poseFactory.of(20.292, 10, 243);
    private final Pose grabGardenOne = poseFactory.of(10.871, 8.769, 247.500);
    private final Pose grabGardenTwo = poseFactory.of(7.167, 8, 269.687);
    private final Pose shootOneStart = poseFactory.of(7.167, 8, 269.687);
    private final Pose shootOne = poseFactory.of(44.147, 118.913, 90);
    private final Pose shootOneControl1 = poseFactory.of(21.7309, 22.1526, 0);
    private final Pose shootOneControl2 = poseFactory.of(20.3256, 99.6044, 0);
    private final Pose grabFlower = poseFactory.of(44.808, 124.798, 90);
    private final Pose parkStart = poseFactory.of(44.808, 124.798, 90);
    private final Pose park = poseFactory.of(16, 111, 0);
    private final Pose parkTurn = poseFactory.of(
            parkStart.x() + 0.25 * (park.x() - parkStart.x()), parkStart.y() + 0.25 * (park.y() - parkStart.y()), 0);

    private String autoStep = "idle";
    private final ElapsedTime stepTimer = new ElapsedTime();

    private Command step(String name) {
        return instant(() -> {
            autoStep = name;
            stepTimer.reset();
        });
    }

    private Command fireShot() {
        return instant(() -> Tracker.setTrackingFrequency(80))
                .then(waitUntil(robot.outtake::shootReady))
                .then(RobotCommands.fireShooter(robot.outtake, robot.intake, robot.ledDriver)
                        .raceWith(waitMs(500).then(RobotCommands.shooterTimeout(robot.intake, 500))));
    }

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                step("preload shot"),
                fireShot(),
                step("drive to garden"),
                follow(Robot.follower, goGrabGarden()),
                step("pick garden"),
                instant(() -> Tracker.setTrackingFrequency(30)),
                RobotCommands.pullBalls(robot.intake),
                follow(Robot.follower, grabGarden()),
                RobotCommands.intakeTimeout(robot.intake, 1000),
                RobotCommands.idleIntake(robot.intake),
                step("drive to shoot"),
                instant(() -> Tracker.setTrackingFrequency(80)),
                follow(Robot.follower, shootOne()),
                step("shoot one"),
                fireShot(),
                step("go to flower"),
                instant(() -> Tracker.setTrackingFrequency(30)),
                RobotCommands.lowerFlower(robot.intake),
                RobotCommands.pullBalls(robot.intake),
                follow(Robot.follower, grabFlower()),
                step("pick flower"),
                instant(() -> Tracker.setTrackingFrequency(80)),
                RobotCommands.intakeTimeout(robot.intake, 1000),
                RobotCommands.idleIntake(robot.intake),
                step("shoot flower"),
                fireShot(),
                step("park"),
                follow(Robot.follower, park()),
                step("done"));
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
                Paths.line(grabGardenOne, grabGardenTwo).linear(grabGardenTwo, grabGardenOne)
                );
    }

    public Path shootOne() {
        return Paths.curve(shootOneStart, shootOneControl1, shootOneControl2, shootOne)
                .linear(shootOneStart, shootOne);
    }

    public Path grabFlower() {
        return Paths.line(shootOne, grabFlower).constant(grabFlower);
    }

    public Path park() {
        return Paths.path(
                Paths.line(parkStart, parkTurn).constant(parkStart),
                Paths.line(parkTurn, park).constant(park));
    }
}
