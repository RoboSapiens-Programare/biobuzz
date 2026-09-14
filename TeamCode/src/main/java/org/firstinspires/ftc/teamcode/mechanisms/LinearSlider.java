package org.firstinspires.ftc.teamcode.mechanisms;

//import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.utils.controlSystems.PIDFController;

import java.util.ArrayList;
import java.util.List;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;

public class LinearSlider implements Mechanism {
    private final PIDFController pidfController = new PIDFController(0, 0, 0, 0);

    private final List<CachingDcMotorEx> motors = new ArrayList<>();
    private CachingDcMotorEx encoder;
    private double targetPosition;


    public LinearSlider setEncoder(CachingDcMotorEx encoder) {
        this.encoder = encoder;
        return this;
    }

    public LinearSlider addMotor(CachingDcMotorEx motor, boolean reverse) {
        motor.setDirection(reverse ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        motors.add(motor);

        return this;
    }

    public LinearSlider addMotor(CachingDcMotorEx motor) {
        return addMotor(motor, false);
    }

    public LinearSlider setPIDFConstants(double kP, double kI, double kD, double kF) {
        pidfController.kP = kP;
        pidfController.kI = kI;
        pidfController.kD = kD;
        pidfController.kF = kF;

        return this;
    }

    public void setTargetPosition(double position) {
        pidfController.setSetpoint(position);
        targetPosition = position;
    }

    public double getTargetPosition(double position) {
        return targetPosition;
    }

    public double getCurrentPosition(double position) {
        return encoder.getCurrentPosition();
    }

    public void setTolerance(double tolerance) {
        pidfController.setTolerance(tolerance);
    }

    public boolean positionReached() {
        return pidfController.targetReached();
    }

    @Override
    public void init() {
        if (encoder == null && !motors.isEmpty()) {
            encoder = motors.get(0);
        }
    }

    @Override
    public void update() {
        if (encoder == null) return;

        double output = pidfController.update(encoder.getCurrentPosition());

        for (DcMotorEx motor : motors) {
            motor.setPower(output);
        }
    }

    @Override
    public void reset() {
        for (DcMotorEx motor : motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void stop() {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setPower(0);
        }
    }

}
