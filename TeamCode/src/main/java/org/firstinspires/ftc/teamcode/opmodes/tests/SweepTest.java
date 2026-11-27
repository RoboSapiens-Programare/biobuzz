package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class SweepTest extends OpMode {
    private static final double RANGE_DEGREES = 300.0;
    private static final double STEP_DEGREES = 60.0;
    private static final double STEP_SECONDS = 0.115;
    private static final double SWEEP_SECONDS = RANGE_DEGREES / STEP_DEGREES * STEP_SECONDS;

    private Servo left, right;
    private final ElapsedTime timer = new ElapsedTime();

    @Override
    public void init() {
        left = hardwareMap.get(Servo.class, "left");
        right = hardwareMap.get(Servo.class, "right");
        timer.reset();
    }

    @Override
    public void loop() {
        double cycle = timer.seconds() % (2.0 * SWEEP_SECONDS);
        double position;
        if (cycle <= SWEEP_SECONDS) {
            position = cycle / SWEEP_SECONDS;
        } else {
            position = 1.0 - (cycle - SWEEP_SECONDS) / SWEEP_SECONDS;
        }

        left.setPosition(position);
        right.setPosition(position);

        telemetry.addData("position", position);
        telemetry.addData("degrees", position * RANGE_DEGREES);
        telemetry.update();
    }
}
