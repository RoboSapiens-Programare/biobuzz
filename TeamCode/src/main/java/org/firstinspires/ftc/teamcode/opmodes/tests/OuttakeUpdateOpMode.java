package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.robot.opmode.RobotOpMode;
import org.firstinspires.ftc.teamcode.robot.subsystems.Tracker;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;

@Configurable
@TeleOp(name = "Outtake Update Test", group = "Tests")
public class OuttakeUpdateOpMode extends RobotOpMode {
    public static double TARGET_VELOCITY = 1630;

    private DcMotorEx flywheel;

    @Override
    protected void onInit() {

        flywheel = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "flywheel"));
        robot.outtake.stopIdling();
    }

    @Override
    protected void onUpdate() {
        robot.outtake.setVelocity(TARGET_VELOCITY);

        double target = robot.outtake.getTarget();
        double current = robot.outtake.getCurrentSpeed();
        boolean atSpeed = Math.abs(target - current) <= 15;

        pTelemetry.addData("Target velocity", target);
        pTelemetry.addData("Current speed", String.format("%.1f", current));
        pTelemetry.addData("Error", String.format("%.1f", target - current));
        pTelemetry.addData("At speed", atSpeed);
        pTelemetry.addData("Gate open", robot.outtake.shootReady());
        pTelemetry.addData("Track angle", String.format("%.1f deg", Math.toDegrees(robot.tracker.getTrackAngle())));
        pTelemetry.addData("Hood", String.format("%.2f", robot.tracker.getHood()));
        pTelemetry.addData(
                "Distance", String.format("%.1f", robot.follower.pose().distance(Tracker.trackPose)));

        pTelemetry.addData("Motor current", flywheel.getCurrent(CurrentUnit.AMPS));
    }
}
