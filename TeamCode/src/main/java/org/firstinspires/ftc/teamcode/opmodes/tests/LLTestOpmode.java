package org.firstinspires.ftc.teamcode.opmodes.tests;

import static org.firstinspires.ftc.teamcode.robot.Robot.follower;

import com.pedropathing.math.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.opmode.RobotOpMode;
import org.firstinspires.ftc.teamcode.robot.subsystems.LLLocator;

@Disabled
@TeleOp(group = "Tests")
public class LLTestOpmode extends RobotOpMode {
    @Override
    protected void onInit() {
        follower.setPose(new Pose(0, 0, 0));
    }

    @Override
    protected void onUpdate() {}

    @Override
    protected void onTelemetry(Telemetry t) {
        Pose robotPose = follower.pose();
        Pose pollen = robot.llLocator.getPollenAbsolutePose(robotPose);
        LLResult result = robot.llLocator.getLatestResult();

        t.addData("Valid target", robot.llLocator.hasValidTarget());
        t.addData("Connected", robot.llLocator.isConnected());

        if (result != null) {
            t.addData("Tx deg", result.getTx());
            t.addData("Ty deg", result.getTy());

            addMathTrace(t, result);
        }

        t.addData("Robot (x,y)", robotPose.x() + ", " + robotPose.y());
        t.addData("Robot heading deg", Math.toDegrees(robotPose.heading()));
        t.addData("Pollen (x,y)", pollen.x() + ", " + pollen.y());
        t.addData("Distance", robotPose.distance(pollen));
        t.addData("Heading error", robot.llLocator.getHeadingError());
    }

    private void addMathTrace(Telemetry t, LLResult result) {
        double tx = Math.toRadians(result.getTx());
        double ty = -Math.toRadians(result.getTy());

        double B = ty + LLLocator.LL_A;
        double dllpx = Math.tan(Math.PI / 2 - B) * LLLocator.LL_H;
        double dllpy = Math.tan(tx) * dllpx;

        t.addData("LL_H in", LLLocator.LL_H);
        t.addData("LL_DX", LLLocator.LL_DX);
        t.addData("LL_DY", LLLocator.LL_DY);
        t.addData("LL_A deg", Math.toDegrees(LLLocator.LL_A));

        t.addData("B deg", Math.toDegrees(B));
        t.addData("dllpx", dllpx);
        t.addData("dllpy", dllpy);
        t.addData("xRel", LLLocator.LL_DX + dllpx);
        t.addData("yRel", LLLocator.LL_DY + dllpy);
    }
}
