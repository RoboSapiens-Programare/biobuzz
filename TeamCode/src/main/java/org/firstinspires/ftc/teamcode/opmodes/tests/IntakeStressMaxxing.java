package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous
public class IntakeStressMaxxing extends OpMode {

    private DcMotorEx intake;

    @Override
    public void init() {
        intake = hardwareMap.get(DcMotorEx.class, "roller_motor");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        intake.setPower(1);
    }
}
