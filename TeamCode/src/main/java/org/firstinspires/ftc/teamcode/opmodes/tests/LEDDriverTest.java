package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Prism.Color;
import org.firstinspires.ftc.teamcode.robot.subsystems.LEDDriver;

@TeleOp(name = "LED Driver Test", group = "Tests")
public class LEDDriverTest extends LinearOpMode {
    private static final long PHASE_MS = 1000;

    private static final Color[] COLORS = {Color.RED, Color.GREEN, Color.BLUE, Color.WHITE};

    private static final String[] NAMES = {"RED", "GREEN", "BLUE", "WHITE"};

    @Override
    public void runOpMode() {
        LEDDriver leds = new LEDDriver(hardwareMap);
        leds.init();

        ElapsedTime timer = new ElapsedTime();
        int phase = 0;
        leds.setColor(COLORS[phase]);

        telemetry.addLine("Cycle: RED -> GREEN -> BLUE -> WHITE (1s each)");
        telemetry.update();

        waitForStart();
        timer.reset();

        while (opModeIsActive()) {
            if (timer.milliseconds() >= PHASE_MS) {
                timer.reset();
                phase = (phase + 1) % COLORS.length;
                leds.setColor(COLORS[phase]);
            }

            leds.update();

            telemetry.addData("Phase", NAMES[phase]);
            telemetry.addData("Elapsed (ms)", String.format("%.0f", timer.milliseconds()));
            telemetry.update();

            sleep(20);
        }

        leds.stop();
    }
}
