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
import org.firstinspires.ftc.teamcode.commands.RobotCommands;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.opmode.RobotOpMode;
import org.firstinspires.ftc.teamcode.robot.subsystems.Tracker;

@Autonomous(name = "WATTs UP auto", group = "Autonomous")
public class WattsUpAuto extends RobotOpMode {

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(54.5, 8, 90);
    private final Pose goGrabGarden = poseFactory.of(8, 13, -90);
    private final Pose grabGarden = poseFactory.of(8, 8, 90);
    private final Pose shootOne = poseFactory.of(46.7, 125, -172.8964);
    private final Pose shootOneControl1 = poseFactory.of(16.2281, 121.2026, 0);
    private final Pose grabFlower = poseFactory.of(46.7, 130, -90);
    private final Pose parkStart = poseFactory.of(46.7, 130, 90);
    private final Pose park = poseFactory.of(10.6891, 114.5117, 0);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                RobotCommands.fireWhileHeld(
                        robot.outtake, robot.intake, robot.ledDriver, () -> robot.intake.getBallCount() > 0),
                follow(Robot.follower, goGrabGarden()),
                instant(() -> Tracker.setTrackingFrequency(30)),
                RobotCommands.pullBalls(robot.intake),
                follow(Robot.follower, grabGarden()),
                RobotCommands.intakeTimeout(robot.intake, 1000),
                RobotCommands.idleIntake(robot.intake),
                instant(() -> Tracker.setTrackingFrequency(80)),
                follow(Robot.follower, shootOne()),
                RobotCommands.fireWhileHeld(
                        robot.outtake, robot.intake, robot.ledDriver, () -> robot.intake.getBallCount() > 0),
                instant(() -> Tracker.setTrackingFrequency(30)),
                RobotCommands.lowerFlower(robot.intake),
                RobotCommands.pullBalls(robot.intake),
                follow(Robot.follower, grabFlower()),
                instant(() -> Tracker.setTrackingFrequency(80)),
                RobotCommands.intakeTimeout(robot.intake, 1000),
                RobotCommands.idleIntake(robot.intake),
                RobotCommands.fireWhileHeld(
                        robot.outtake, robot.intake, robot.ledDriver, () -> robot.intake.getBallCount() > 0),
                follow(Robot.follower, park()),
                RobotCommands.raiseFlower(robot.intake));
    }

    @Override
    protected void onInit() {
        Robot.follower.setPose(start);

        Tracker.setEnabled(true);
        Tracker.setTrackingFrequency(80);

        robot.outtake.aimAt(0);

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
        return Paths.line(goGrabGarden, grabGarden).reverseTangent();
    }

    public Path shootOne() {
        return Paths.curve(grabGarden, shootOneControl1, shootOne).reverseTangent();
    }

    public Path grabFlower() {
        return Paths.line(shootOne, grabFlower).reverseTangent();
    }

    public Path park() {
        return Paths.line(parkStart, park).linear(parkStart, park);
    }

}
