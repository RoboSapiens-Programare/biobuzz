package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous
public class IntakeStressMaxxing extends OpMode {

    private DcMotorEx intake, intake2;
    private Servo gate;

    @Override
    public void init() {
        intake = hardwareMap.get(DcMotorEx.class, "roller_motor");
        intake2 = hardwareMap.get(DcMotorEx.class, "roller_motor_2");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        gate = hardwareMap.get(Servo.class, "gate");

        gate.setPosition(0.63);
    }

    @Override
    public void loop() {
        intake.setPower(1);
        intake2.setPower(1);
    }
}
