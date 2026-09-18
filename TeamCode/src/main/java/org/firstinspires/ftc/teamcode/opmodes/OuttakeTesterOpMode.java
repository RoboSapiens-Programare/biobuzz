package org.firstinspires.ftc.teamcode.opmodes;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.RobotCommands;
import org.firstinspires.ftc.teamcode.robot.opmode.RobotOpMode;

@TeleOp(name = "Outtake Auto Test", group = "Test")
@Configurable
public class OuttakeTesterOpMode extends RobotOpMode {

    private enum Step {
        IDLE,
        SPOOL,
        FIRE,
        STOP_FIRING
    }

    private static final double AT_SPEED_TOLERANCE = 15;

    private static final double IDLE_SECONDS = 3;
    private static final double SPOOL_SECONDS = 6;
    private static final double FIRE_SECONDS = 3;
    private static final double STOP_FIRING_SECONDS = 2;

    private final ElapsedTime runTimer = new ElapsedTime();
    private Step currentStep;
    private double stepStart = 0;

    @Override
    protected void onInit() {
        RobotCommands.idleShooter(robot.outtake).schedule();
        currentStep = Step.IDLE;
        stepStart = 0;
    }

    @Override
    protected void onUpdate() {
        double total = runTimer.seconds();

        Step step = stepFor(total);
        if (step != currentStep) {
            stepStart = total;
            currentStep = step;

            switch (currentStep) {
                case IDLE:
                    RobotCommands.idleShooter(robot.outtake).schedule();
                    break;
                case SPOOL:
                    RobotCommands.spoolShooter(robot.outtake).schedule();
                    break;
                case FIRE:
                    RobotCommands.fireShooter(robot.outtake, robot.intake).schedule();
                    break;
                case STOP_FIRING:
                    RobotCommands.stopFiring(robot.outtake).schedule();
                    break;
            }
        }

        double current = robot.outtake.getCurrentSpeed();
        double target = robot.outtake.getTarget();
        boolean atSpeed = Math.abs(target - current) <= AT_SPEED_TOLERANCE;

        telemetry.addData("Step", currentStep);
        telemetry.addData("Step time", String.format("%.1fs / %.1fs", total - stepStart, stepDuration(currentStep)));
        telemetry.addData("Run time", String.format("%.1fs", total));
        telemetry.addLine();
        telemetry.addData("Firing", currentStep == Step.FIRE);
        telemetry.addData("Commanded target", target);
        telemetry.addData("Current speed", String.format("%.1f", current));
        telemetry.addData("Error", String.format("%.1f", target - current));
        telemetry.addData("At speed", atSpeed);
        telemetry.addLine();
        telemetry.addLine("Auto cycle: IDLE(" + IDLE_SECONDS + "s) -> SPOOL(" + SPOOL_SECONDS
                + "s) -> FIRE(" + FIRE_SECONDS + "s) -> STOP(" + STOP_FIRING_SECONDS + "s) -> repeat");
    }

    @Override
    protected void onStop() {
        RobotCommands.stopFiring(robot.outtake).schedule();
        RobotCommands.idleShooter(robot.outtake).schedule();
    }

    private Step stepFor(double total) {
        double cycle = IDLE_SECONDS + SPOOL_SECONDS + FIRE_SECONDS + STOP_FIRING_SECONDS;
        double t = total % cycle;

        if (t < IDLE_SECONDS) {
            return Step.IDLE;
        }
        if (t < IDLE_SECONDS + SPOOL_SECONDS) {
            return Step.SPOOL;
        }
        if (t < IDLE_SECONDS + SPOOL_SECONDS + FIRE_SECONDS) {
            return Step.FIRE;
        }
        return Step.STOP_FIRING;
    }

    private double stepDuration(Step step) {
        switch (step) {
            case IDLE: return IDLE_SECONDS;
            case SPOOL: return SPOOL_SECONDS;
            case FIRE: return FIRE_SECONDS;
            case STOP_FIRING: return STOP_FIRING_SECONDS;
            default: return 0;
        }
    }
}