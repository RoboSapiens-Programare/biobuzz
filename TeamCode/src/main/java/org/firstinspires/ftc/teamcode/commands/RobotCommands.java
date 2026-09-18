package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;

import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystems.Outtake;

/**
 * Static factories that wrap subsystem actions in Ivy commands with proper requirements.
 * Usage inside an OpMode: RobotCommands.spoolShooter(robot.outtake).schedule();
 */
public final class RobotCommands {
    private RobotCommands() {}

    /** Spools the flywheel to shooting speed. */
    public static Command spoolShooter(Outtake outtake) {
        return Commands.instant(outtake::stopIdling).requiring(outtake);
    }

    /** Returns the shooter to its idle velocity and stops firing. */
    public static Command idleShooter(Outtake outtake) {
        return Commands.instant(outtake::startIdling).requiring(outtake);
    }

    /** Opens the gate and pulls rollers. The flywheel is gated on speed in Outtake.update(). */
    public static Command fireShooter(Outtake outtake, Intake intake) {
        return Command.build()
                .setStart(outtake::computeVelocity)
                .setExecute(() -> {
                    if (outtake.shootReady()) {
                        intake.pullBalls();
                        outtake.update();
                    }
                })
                .setDone(() -> outtake.getBallCount() == 0)
                .setEnd(endCondition -> outtake.startIdling())
                .requiring(outtake, intake);
    }

    /** Closes the gate and stops the rollers. */
    public static Command stopFiring(Outtake outtake) {
        return Commands.instant(outtake::startIdling).requiring(outtake);
    }

    /** Pulls balls into the mechanism. */
    public static Command pullBalls(Intake intake) {
        return Commands.instant(intake::pullBalls).requiring(intake);
    }

    /** Pushes balls back out (unclog). */
    public static Command pushBalls(Intake intake) {
        return Commands.instant(intake::pushBalls).requiring(intake);
    }

    /** Stops the intake rollers. */
    public static Command idleIntake(Intake intake) {
        return Commands.instant(intake::idle).requiring(intake);
    }

}