package org.firstinspires.ftc.teamcode.robot.subsystems;

import static com.pedropathing.api.Paths.line;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Configurable
public class LLLocator implements Subsystem {
    private final Limelight3A limelight;

    public static double LL_H = 4.4957;
    public static double LL_DX = -7.086614;
    public static double LL_DY = 0;
    public static double LL_A = Math.toRadians(10);

    private double xRel = 0;
    private double yRel = 0;

    private double headingError = 0;

    private boolean hasValidTarget = false;

    public LLLocator(HardwareMap hwMap) {
        limelight = hwMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); // This tells Limelight to start looking!

        limelight.pipelineSwitch(0); // switch to right index for limelight
    }

    @Override
    public void init() {}

    @Override
    public void update() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            double tx = Math.toRadians(result.getTx()); // How far left or right the target is (radians)
            double ty = -Math.toRadians(result.getTy()); // How far up or down the target is (radians)

            double B = ty + LL_A;

            double dllpx = Math.tan(Math.PI / 2 - B) * LL_H;
            double dllpy = Math.tan(tx) * dllpx;

            headingError = tx;

            xRel = LL_DX + dllpx;
            yRel = LL_DY + dllpy;

            hasValidTarget = true;
        } else {
            hasValidTarget = false;
        }
    }

    public boolean hasValidTarget() {
        return hasValidTarget;
    }

    public LLResult getLatestResult() {
        return limelight.getLatestResult();
    }

    public boolean isConnected() {
        return limelight.isConnected();
    }

    public long getTimeSinceLastUpdate() {
        return limelight.getTimeSinceLastUpdate();
    }

    @Override
    public void reset() {}

    @Override
    public void stop() {
        limelight.stop();
    }

    public Pose getPollenAbsolutePose(double x_offset, double y_offset, Pose robot) {
        double sin = Math.sin(robot.heading());
        double cos = Math.cos(robot.heading());

        double x = robot.x() + (xRel - x_offset) * cos + (yRel - y_offset) * sin;
        double y = robot.y() + (xRel - x_offset) * sin - (yRel - y_offset) * cos;

        return new Pose(x, y, Math.atan2(y - robot.y(), x - robot.x()));
    }

    public Pose getPollenAbsolutePose(Pose robot) {
        return getPollenAbsolutePose(0, 0, robot);
    }

    public Path buildPathToPollen(double x_offset, double y_offset, Pose robot) {
        Pose pollen = getPollenAbsolutePose(x_offset, y_offset, robot);
        return line(robot, pollen).linear(robot, pollen);
    }

    public Path buildPathToPollen(Pose robot) {
        return buildPathToPollen(0, 0, robot);
    }

    public double getHeadingError() {
        return headingError;
    }
}
