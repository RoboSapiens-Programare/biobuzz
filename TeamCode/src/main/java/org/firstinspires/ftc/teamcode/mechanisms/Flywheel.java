package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;
import java.util.ArrayList;
import java.util.List;
import org.firstinspires.ftc.teamcode.utils.ControlSystems.FFController;

public class Flywheel implements Mechanism {
    private final FFController ffController = new FFController(0, 0, 0, 0, 0, 0);

    private final List<CachingDcMotorEx> motors = new ArrayList<>();
    private CachingDcMotorEx encoder;
    public double idlingPower = 0;
    private double targetVelocity;
    private double lastPower = Double.NaN;

    public enum FlywheelMode {
        IDLING,
        PRIMED
    }

    public FlywheelMode mode = FlywheelMode.IDLING;

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

    public void setTargetVelocity(double velocity) {
        targetVelocity = velocity;
        ffController.setTargetVelocity(velocity);
    }

    public boolean velocityReached() {
        return ffController.velocityReached();
    }

    public double getCurrentVelocity() {
        return encoder.getVelocity();
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public Flywheel setTolerance(double tolerance) {
        ffController.setTolerance(tolerance);
        return this;
    }

    public Flywheel setIdlingPower(double p) {
        idlingPower = p;
        return this;
    }

    @Override
    public void init() {}

    private void applyPower(double power) {
        if (power == lastPower) {
            return;
        }
        lastPower = power;
        for (int i = 0, n = motors.size(); i < n; i++) {
            motors.get(i).setPower(power);
        }
    }

    @Override
    public void update() {
        switch (mode) {
            case IDLING:
                applyPower(idlingPower);
                break;
            case PRIMED:
                applyPower(ffController.update(encoder.getVelocity()));
                break;
        }
    }

    @Override
    public void reset() {
        for (int i = 0, n = motors.size(); i < n; i++) {
            motors.get(i).setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motors.get(i).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void stop() {
        ffController.setTargetVelocity(0);
        applyPower(0);
    }
}
