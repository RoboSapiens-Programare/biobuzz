package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;
import java.util.List;

import dev.frozenmilk.dairy.cachinghardware.CachingServo;

public class Claw implements Mechanism {
    private final List<CachingServo> servos = new ArrayList<>();
    private double targetPosition;
    private double openPosition;
    private double closedPosition;

    public Claw addServo(CachingServo servo, boolean reverse) {
        servo.setDirection(reverse ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
        servos.add(servo);
        return this;
    }

    public Claw setOpenPosition(double position) {
        openPosition = position;
        return this;
    }

    public Claw setClosedPosition(double position) {
        closedPosition = position;
        return this;
    }

    public void setPosition(double position) { this.targetPosition = position; }

    public void open() { setPosition(openPosition); }
    public void close() { setPosition(closedPosition); }

    @Override
    public void init() {

    }

    @Override
    public void update() {
        for (Servo servo : servos) {
            servo.setPosition(targetPosition);
        }
    }

    @Override
    public void reset() {

    }

    @Override
    public void stop() {

    }
}