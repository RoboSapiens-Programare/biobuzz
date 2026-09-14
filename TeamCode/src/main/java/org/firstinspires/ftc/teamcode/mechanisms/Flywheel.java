package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.utils.controlSystems.FFController;
import org.firstinspires.ftc.teamcode.utils.controlSystems.PIDFController;

import java.util.ArrayList;
import java.util.List;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;

public class Flywheel implements Mechanism {
    private final FFController ffController = new FFController(0, 0, 0, 0, 0, 0);

    private final List<CachingDcMotorEx> motors = new ArrayList<>();
    private CachingDcMotorEx encoder;
    private double idlingVelocity = 0;
    private double targetVelocity;


    public Flywheel setEncoder(DcMotorEx encoder) {

        this.encoder = new CachingDcMotorEx(encoder);
        return this;
    }

    public Flywheel addMotor(DcMotorEx motor, boolean reverse) {
        CachingDcMotorEx motorC = new CachingDcMotorEx(motor);

        motorC.setDirection(reverse ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        motorC.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorC.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorC.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        motors.add(motorC);

        return this;
    }

    public Flywheel addMotor(DcMotorEx motor) {
        return addMotor(motor, false);
    }


    public Flywheel setPIDConstants(double kP, double kI, double kD) {
        ffController.setPIDConstants(kP, kI, kD);

        return this;
    }

    public Flywheel setFFConstants(double kS, double kV, double kA) {
        ffController.kS = kS;
        ffController.kV = kV;
        ffController.kA = kA;

        return this;
    }

    public Flywheel setAcceleration(double acceleration) {
        ffController.setTargetAcceleration(acceleration);

        return this;
    }

    public Flywheel setIdlingVelocity(double velocity) {
        idlingVelocity = velocity;

        return this;
    }

    public void setTargetVelocity(double velocity) {
        targetVelocity = velocity;
        ffController.setTargetVelocity(velocity);
    }

    public boolean velocityReached() {
        return ffController.velocityReached();
    }

    public double getCurrentSpeed() {
        return encoder.getVelocity();
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public double getKP() {
        return ffController.kP;
    }

    public double getKI() {
        return ffController.kI;
    }

    public double getKD() {
        return ffController.kD;
    }

    public void setTolerance(double tolerance) {
        ffController.setTolerance(tolerance);
    }

    public void idle() {
        setTargetVelocity(idlingVelocity);
    }

    @Override
    public void init() {

    }

    @Override
    public void update() {
        double output = ffController.update(encoder.getVelocity());

        for (DcMotorEx motor : motors) {
            motor.setPower(output);
        }
    }

    @Override
    public void reset() {
        for (DcMotorEx motor : motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void stop() {
        ffController.setTargetVelocity(0);

        for (DcMotorEx motor : motors) {
            motor.setPower(0);
        }
    }

}
