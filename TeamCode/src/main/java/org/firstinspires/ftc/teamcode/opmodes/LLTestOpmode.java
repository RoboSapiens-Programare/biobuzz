package org.firstinspires.ftc.teamcode.opmodes;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.robot.subsystems.LLLocator;

@TeleOp(group = "Tests")
public class LLTestOpmode extends OpMode {
    LLLocator llLocator;
    Follower follower;

    TelemetryManager.TelemetryWrapper pTelemetry;

    @Override
    public void init() {
        llLocator = new LLLocator(hardwareMap);
        follower = Constants.create(hardwareMap);

        pTelemetry = PanelsTelemetry.INSTANCE.getTelemetry().getWrapper();

        follower.setPose(new Pose(0, 0, 0));
    }

    @Override
    public void loop() {
        Pose robot = follower.pose();

        Pose pollen = llLocator.getPollenAbsolutePose(robot);

        pTelemetry.addData("0. Pollen pose", pollen);
        pTelemetry.addData("1. Robot pose", robot);
        pTelemetry.addData("2. Distance", robot.distance(pollen));
        pTelemetry.addData("3. Error angle", llLocator.getHeadingError());

        follower.update();
        llLocator.update();
        pTelemetry.update();
    }
}
