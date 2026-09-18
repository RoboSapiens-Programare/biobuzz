package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.ArrayList;
import java.util.List;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;

public class Roller implements Mechanism {
    private final List<CachingDcMotorEx> motors = new ArrayList<>();
    private double power = 0;

    private double currentLimit = 0;
    private double maxCurrent = 0;

    public Roller addMotor(DcMotorEx motor, boolean reverse) {
        CachingDcMotorEx motorC = new CachingDcMotorEx(motor);

        motorC.setDirection(reverse ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        motorC.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorC.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorC.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        motors.add(motorC);

        return this;
    }

    public Roller addMotor(DcMotorEx motor) {
        return addMotor(motor, false);
    }

    private void setMotorPower(double p) {
        for (CachingDcMotorEx motor : motors) {
            motor.setPower(p);
        }
    }

    public void setPower(double p) {
        this.power = p;
    }

    public void setCurrentLimit(double c) {
        this.currentLimit = c;
    }

    public void pull() {
        setMotorPower(power);
    }

    public void push() {
        setMotorPower(-power);
    }

    public void idle() {
        setMotorPower(0);
    }

    @Override
    public void init() {

    }

    public boolean isOverCurrent() {
        return maxCurrent > currentLimit;
    }

    public double getMaxCurrent() {
        return maxCurrent;
    }

    @Override
    public void update() {
        maxCurrent = 0;

        for (CachingDcMotorEx motor : motors) {
            double current = motor.getCurrent(CurrentUnit.AMPS);
            if (current > maxCurrent) {
                maxCurrent = current;
            }
        }
    }

    @Override
    public void reset() {
    }

    @Override
    public void stop() {
        idle();
    }

}
