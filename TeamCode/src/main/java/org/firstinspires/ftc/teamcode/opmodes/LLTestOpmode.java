package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.robot.Robot.follower;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.robot.opmode.RobotOpMode;
import org.firstinspires.ftc.teamcode.robot.subsystems.LLLocator;

@TeleOp(group = "Tests")
public class LLTestOpmode extends RobotOpMode {
    TelemetryManager.TelemetryWrapper pTelemetry;

    @Override
    protected void onInit() {
        pTelemetry = PanelsTelemetry.INSTANCE.getTelemetry().getWrapper();
        follower.setPose(new Pose(0, 0, 0));
    }

    @Override
    protected void onUpdate() {
        Pose robotPose = follower.pose();
        Pose pollen = robot.llLocator.getPollenAbsolutePose(robotPose);
        LLResult result = robot.llLocator.getLatestResult();

        pTelemetry.addData("Valid target", robot.llLocator.hasValidTarget());
        pTelemetry.addData("Connected", robot.llLocator.isConnected());

        if (result != null) {
            pTelemetry.addData("Tx deg", result.getTx());
            pTelemetry.addData("Ty deg", result.getTy());

            addMathTrace(result);
        }

        pTelemetry.addData("Robot (x,y)", robotPose.x() + ", " + robotPose.y());
        pTelemetry.addData("Robot heading deg", Math.toDegrees(robotPose.heading()));
        pTelemetry.addData("Pollen (x,y)", pollen.x() + ", " + pollen.y());
        pTelemetry.addData("Distance", robotPose.distance(pollen));
        pTelemetry.addData("Heading error", robot.llLocator.getHeadingError());

        pTelemetry.update();
    }

    private void addMathTrace(LLResult result) {
        double tx = Math.toRadians(result.getTx());
        double ty = -Math.toRadians(result.getTy());

        double B = ty + LLLocator.LL_A;
        double dllpx = Math.tan(Math.PI / 2 - B) * LLLocator.LL_H;
        double dllpy = Math.tan(tx) * dllpx;

        pTelemetry.addData("LL_H in", LLLocator.LL_H);
        pTelemetry.addData("LL_DX", LLLocator.LL_DX);
        pTelemetry.addData("LL_DY", LLLocator.LL_DY);
        pTelemetry.addData("LL_A deg", Math.toDegrees(LLLocator.LL_A));

        pTelemetry.addData("B deg", Math.toDegrees(B));
        pTelemetry.addData("dllpx", dllpx);
        pTelemetry.addData("dllpy", dllpy);
        pTelemetry.addData("xRel", LLLocator.LL_DX + dllpx);
        pTelemetry.addData("yRel", LLLocator.LL_DY + dllpy);
    }
}
