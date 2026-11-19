package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import java.util.function.BooleanSupplier;
import org.firstinspires.ftc.teamcode.Prism.Color;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystems.LEDDriver;
import org.firstinspires.ftc.teamcode.robot.subsystems.Outtake;

/**
 * Static factories that wrap subsystem actions in Ivy commands with proper requirements.
 * Usage inside an OpMode: RobotCommands.spoolShooter(robot.outtake).schedule();
 */
public final class RobotCommands {
    private static final double UNCLOG_MS = 400;

    private RobotCommands() {}

    /** Spools the flywheel to shooting speed. */
    public static Command spoolShooter(Outtake outtake) {
        return Commands.instant(() -> {
                    outtake.setFiring(false);
                    outtake.stopIdling();
                })
                .requiring(outtake);
    }

    /** Returns the shooter to its idle velocity and stops firing. */
    public static Command idleShooter(Outtake outtake) {
        return Commands.instant(() -> {
                    outtake.setFiring(false);
                    outtake.startIdling();
                })
                .requiring(outtake)
                .setPriority(1);
    }

    /**
     * Fires until interrupted. Waits for the shooter to reach speed, then feeds continuously.
     * Interrupted by {@link #safeStop(Outtake, Intake)}.
     */
    public static Command fireShooter(Outtake outtake, Intake intake, LEDDriver ledDriver) {
        return fireWhileHeld(outtake, intake, ledDriver, () -> true);
    }

    /** Fires while {@code held} is true and cleans up as soon as it is released. */
    public static Command fireWhileHeld(Outtake outtake, Intake intake, LEDDriver ledDriver, BooleanSupplier held) {
        boolean[] firing = {false};
        boolean[] feeding = {false};
        return Command.build()
                .setStart(() -> {
                    firing[0] = false;
                    feeding[0] = false;
                    outtake.setFiring(false);
                    ledDriver.setColor(Color.RED);
                })
                .setExecute(() -> {
                    if (!firing[0] && outtake.shootReady()) {
                        firing[0] = true;
                        outtake.setFiring(true);
                    }

                    if (firing[0]) {
                        feedIfReady(outtake, intake, feeding);
                    } else {
                        intake.idle();
                    }
                })
                .setDone(() -> !held.getAsBoolean())
                .setEnd(endCondition -> {
                    stopFiringNow(outtake, intake);
                    ledDriver.playSweep();
                })
                .requiring(outtake, intake, ledDriver);
    }

    /**
     * Waits for the shooter to reach speed, then feeds continuously until interrupted.
     * Does not control the gate or firing state; pair it with the shooter commands.
     */
    public static Command feedWhenReady(Outtake outtake, Intake intake) {
        boolean[] feeding = {false};
        return Commands.infinite(() -> feedIfReady(outtake, intake, feeding))
                .requiring(intake)
                .setEnd(endCondition -> intake.idle());
    }

    private static void feedIfReady(Outtake outtake, Intake intake, boolean[] feeding) {
        if (feeding[0] || outtake.shootReady()) {
            feeding[0] = true;
            intake.pullBalls(true);
        } else {
            intake.idle();
        }
    }

    /** Stops firing and idles the intake. Preempts a running fire command. */
    public static Command safeStop(Outtake outtake, Intake intake) {
        return Commands.instant(() -> stopFiringNow(outtake, intake))
                .requiring(outtake, intake)
                .setPriority(1);
    }

    private static void stopFiringNow(Outtake outtake, Intake intake) {
        outtake.setFiring(false);
        //        outtake.startIdling();
        intake.idle();
    }

    /** Pulls balls into the mechanism. */
    public static Command pullBalls(Intake intake) {
        return Commands.instant(intake::pullBalls).requiring(intake);
    }

    /** Pushes balls back out (unclog). */
    public static Command pushBalls(Intake intake) {
        return Commands.instant(intake::pushBalls).requiring(intake);
    }

    /** Pushes balls out briefly, then idles the intake. */
    public static Command unclogIntake(Intake intake) {
        return pushBalls(intake).then(Commands.waitMs(UNCLOG_MS)).then(idleIntake(intake));
    }

    /** Stops the intake rollers. */
    public static Command idleIntake(Intake intake) {
        return Commands.instant(intake::idle).requiring(intake);
    }

    /**
     * Drops the flower servo into its lowered (pickup) position.
     * Drives only the servo, so it does not require the intake and will not interrupt a running
     * intake or fire command.
     */
    public static Command lowerFlower(Intake intake) {
        return Commands.instant(intake::lowerFlower);
    }

    /** Returns the flower servo to its raised (stowed) position. */
    public static Command raiseFlower(Intake intake) {
        return Commands.instant(intake::raiseFlower);
    }

    public static Command toggleFlower(Intake intake) {
        return Commands.instant(intake::toggleFlower);
    }

    /**
     * Waits until the intake stages two balls, or until {@code ms} elapses, whichever comes first.
     * Does not run the intake; pair it with {@link #pullBalls(Intake)} and {@link #idleIntake(Intake)}.
     */
    public static Command intakeTimeout(Intake intake, int ms) {
        return Commands.waitUntil(() -> intake.getBallCount() >= 2).raceWith(Commands.waitMs(ms));
    }

    public static Command shooterTimeout(Intake intake, int ms) {
        return Commands.waitUntil(() -> intake.getBallCount() == 0).raceWith(Commands.waitMs(ms));
    }
}
