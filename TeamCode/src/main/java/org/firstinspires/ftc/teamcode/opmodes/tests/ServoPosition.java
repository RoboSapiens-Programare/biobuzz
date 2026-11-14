package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous
@Configurable
public class ServoPosition extends OpMode {
    private Servo servo1, servo2;
    public static double pos = 0.5;

    @Override
    public void init() {
        servo1 = hardwareMap.get(Servo.class, "pivot_left");
        servo2 = hardwareMap.get(Servo.class, "pivot_right");
    }

    @Override
    public void loop() {
        servo1.setPosition(pos);
        servo2.setPosition(pos);
    }
}
