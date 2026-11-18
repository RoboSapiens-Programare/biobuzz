package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous
@Configurable
public class ServoPosition extends OpMode {
    private Servo flowerLeft, flowerRight;
    public static double pos = 0.5;

    @Override
    public void init() {
        flowerLeft = hardwareMap.get(Servo.class, "left_flower_servo");
        flowerRight = hardwareMap.get(Servo.class, "right_flower_servo");
        flowerRight.setDirection(Servo.Direction.REVERSE);
    }

    @Override
    public void loop() {
        flowerLeft.setPosition(pos);
        flowerRight.setPosition(pos);
    }
}
