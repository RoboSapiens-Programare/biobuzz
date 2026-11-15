package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Prism.Color;
import org.firstinspires.ftc.teamcode.Prism.Direction;
import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.Prism.PrismAnimations;

@TeleOp(name = "LED Snake Test", group = "Tests")
@Configurable
public class LEDSnakeTest extends LinearOpMode {
    public static int SNAKE_LENGTH = 2;
    private static final int SPACING_BETWEEN = 5;
    private static final int REPEAT_AFTER = 20;
    public static float SPEED = 0.24f;
    private static final long PHASE_MS = 3000;

    @Override
    public void runOpMode() {
        GoBildaPrismDriver prism = hardwareMap.get(GoBildaPrismDriver.class, "prism");
        prism.setStripLength(12);

        PrismAnimations.Snakes snake = new PrismAnimations.Snakes(
                SNAKE_LENGTH, SPACING_BETWEEN, REPEAT_AFTER, Color.TRANSPARENT, SPEED, Direction.Forward, Color.PURPLE);
        PrismAnimations.Snakes snake_rev = new PrismAnimations.Snakes(
                SNAKE_LENGTH,
                SPACING_BETWEEN,
                REPEAT_AFTER,
                Color.TRANSPARENT,
                SPEED,
                Direction.Backward,
                Color.PURPLE);

        PrismAnimations.DroidScan scan = new PrismAnimations.DroidScan(new Color(200, 9, 200), new Color(255, 70, 11));
        scan.setSpeed(SPEED);
        scan.setTrailWidth(SNAKE_LENGTH);
        scan.setStopIndex(12);

        telemetry.addData("Device ID", prism.getDeviceID());
        telemetry.addData("Firmware", prism.getFirmwareVersionString());
        telemetry.addData("LEDs", prism.getNumberOfLEDs());
        telemetry.addLine("Purple snake sweeps left to right, then right to left");
        telemetry.update();

        waitForStart();

        ElapsedTime phaseTimer = new ElapsedTime();
        boolean forward = true;

        prism.clearAllAnimations();
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, scan);

        while (opModeIsActive()) {
            telemetry.addData("Direction", forward ? "Forward" : "Reverse");
            telemetry.addData("FPS", prism.getCurrentFPS());
            telemetry.addData("LEDs", prism.getNumberOfLEDs());
            telemetry.update();
        }

        prism.clearAllAnimations();
    }
}
