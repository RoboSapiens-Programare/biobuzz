package org.firstinspires.ftc.teamcode.opmodes.autos;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.bylazar.configurables.annotations.Configurable;
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

@Configurable
@Autonomous(name = "PARK", group = "0. Autonomous")
public class AutoPark extends RobotOpMode {

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(63.3127, 133.4121, 90);
    private final Pose path1 = poseFactory.of(8.8333, 117.5362, 90);
    private final Pose path1Control1 = poseFactory.of(66.124, 115.3546, 0);
    private String autoStep = "idle";
    private final ElapsedTime stepTimer = new ElapsedTime();
    public static int delayMs = 7000;

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
                step("park"),
                waitMs(delayMs),
                fireShot(),
                RobotCommands.followPath(Robot.follower, path1()),
                step("complete"));
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

    public Path path1() {
        return Paths.curve(start, path1Control1, path1).constant(path1);
    }
}
