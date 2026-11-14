package org.firstinspires.ftc.teamcode.opmodes.autos;

import static com.pedropathing.api.Paths.*;
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
import org.firstinspires.ftc.teamcode.commands.RobotCommands;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.opmode.RobotOpMode;
import org.firstinspires.ftc.teamcode.robot.subsystems.Tracker;

@Autonomous(name = "RED auto", group = "Autonomous")
public class RedAuto extends RobotOpMode {

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(54.5, 8, 90);
    private final Pose goGrabGarden = poseFactory.of(8, 13, -90);
    private final Pose grabGarden = poseFactory.of(8, 8, -90);
    private final Pose shootOneStart = poseFactory.of(8, 8, -90);
    private final Pose shootOne = poseFactory.of(46.7, 125, 90);
    private final Pose shootOneControl1 = poseFactory.of(16.2281, 121.2026, 0);
    private final Pose grabFlower = poseFactory.of(46.7, 130, 90);
    private final Pose endStart = poseFactory.of(46.7, 130, 90);
    private final Pose end = poseFactory.of(60.2544, 120, 0);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                RobotCommands.fireWhileHeld(robot.outtake, robot.intake, () -> robot.intake.getBallCount() > 0),

                follow(Robot.follower, goGrabGarden()),
                instant(() -> Tracker.setTrackingFrequency(30)),
                RobotCommands.pullBalls(robot.intake),

                follow(Robot.follower, grabGarden()),
                RobotCommands.intakeTimeout(robot.intake, 1000),
                RobotCommands.idleIntake(robot.intake),
                instant(() -> Tracker.setTrackingFrequency(80)),

                follow(Robot.follower, shootOne()),
                RobotCommands.fireWhileHeld(robot.outtake, robot.intake, () -> robot.intake.getBallCount() > 0),

                instant(() -> Tracker.setTrackingFrequency(30)),
                RobotCommands.lowerFlower(robot.intake),
                RobotCommands.pullBalls(robot.intake),

                follow(Robot.follower, grabFlower()),

                instant(() -> Tracker.setTrackingFrequency(80)),
                RobotCommands.intakeTimeout(robot.intake, 1000),
                RobotCommands.idleIntake(robot.intake),

                RobotCommands.fireWhileHeld(robot.outtake, robot.intake, () -> robot.intake.getBallCount() > 0),

                follow(Robot.follower, end()),
                RobotCommands.raiseFlower(robot.intake)
        );
    }

    @Override
    protected void onInit() {
        Robot.alliance = Robot.Alliance.RED;

        Robot.follower.setPose(start);

        Tracker.setEnabled(true);
        Tracker.setTrackingFrequency(80);

        schedule(autoRoutine());
    }

    @Override
    protected void onUpdate() {
        if (Robot.follower.currentPath() != null) {
            telemetry.addData("Current path distance remaining", Robot.follower.distanceToEndpoint());
            telemetry.addData("Path number", Robot.follower.pathIndex());
        }
    }

    public Path goGrabGarden() {
        return Paths.line(start, goGrabGarden).constant(goGrabGarden);
    }

    public Path grabGarden() {
        return Paths.line(goGrabGarden, grabGarden).constant(grabGarden);
    }

    public Path shootOne() {
        return Paths.curve(shootOneStart, shootOneControl1, shootOne).linear(shootOneStart, shootOne);
    }

    public Path grabFlower() {
        return Paths.line(shootOne, grabFlower).constant(grabFlower);
    }

    public Path end() {
        return Paths.line(endStart, end).linear(endStart, end);
    }
}
