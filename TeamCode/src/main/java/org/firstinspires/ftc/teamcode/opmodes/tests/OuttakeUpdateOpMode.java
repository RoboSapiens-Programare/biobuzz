package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.robot.opmode.RobotOpMode;
import org.firstinspires.ftc.teamcode.robot.subsystems.Tracker;

@Configurable
@TeleOp(name = "Outtake Update Test", group = "Tests")
public class OuttakeUpdateOpMode extends RobotOpMode {
    public static double TARGET_VELOCITY = 1630;

    private DcMotorEx flywheel;

    @Override
    protected void onInit() {

        flywheel = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "flywheel"));
        robot.outtake.stopIdling();
        robot.outtake.setTracker(null);
        Tracker.setEnabled(true);
    }

    @Override
    protected void onUpdate() {
        robot.outtake.setVelocity(TARGET_VELOCITY);
    }

    @Override
    protected void onTelemetry(Telemetry t) {
        double target = robot.outtake.getTarget();
        double current = robot.outtake.getCurrentSpeed();
        boolean atSpeed = Math.abs(target - current) <= 15;

        t.addData("Target velocity", target);
        t.addData("Current speed", String.format("%.1f", current));
        t.addData("Error", String.format("%.1f", target - current));
        t.addData("At speed", atSpeed);
        t.addData("Gate open", robot.outtake.shootReady());
        t.addData("Track angle", String.format("%.1f deg", Math.toDegrees(robot.tracker.getTrackAngle())));
        t.addData("Pivot pos", String.format("%.3f", robot.outtake.pos));
        t.addData("Distance", String.format("%.1f", robot.follower.pose().distance(Tracker.getTrackPose())));

        t.addData("Motor current", flywheel.getCurrent(CurrentUnit.AMPS));
    }
}
