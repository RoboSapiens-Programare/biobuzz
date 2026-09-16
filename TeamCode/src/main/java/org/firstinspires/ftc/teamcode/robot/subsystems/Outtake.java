package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.mechanisms.Flywheel;
import org.firstinspires.ftc.teamcode.mechanisms.Roller;

import dev.frozenmilk.dairy.cachinghardware.CachingServo;

public class Outtake implements Subsystem {

    private final Flywheel flywheel = new Flywheel();
    private final Roller rollers = new Roller();

    private final CachingServo gate;

    private final double GATE_OPEN = 0.5;
    private final double GATE_CLOSED = 0;

    private boolean spooled = false;
    private boolean firing = false;

    public Outtake(HardwareMap hwMap) {
        flywheel.addMotor(hwMap.get(DcMotorEx.class, "flywheel_left"), true)
                .addMotor(hwMap.get(DcMotorEx.class, "flywheel_right"), false)
                .setEncoder(hwMap.get(DcMotorEx.class, "flywheel_left"))
                .setPIDConstants(0.0002, 0.0024, 0.0000)
                .setFFConstants(0.0046, 0.0004, 0.0000)
                .setTolerance(40);

        rollers.addMotor(hwMap.get(DcMotorEx.class, "roller_motor"))
               .setPower(1);

        gate = new CachingServo(hwMap.get(Servo.class, "gate"));
    }

    @Override
    public void init() {}

    @Override
    public void update() {
        if (firing && flywheel.velocityReached()) {
            gate.setPosition(GATE_OPEN);
            rollers.pull();
        } else {
            gate.setPosition(GATE_CLOSED);
            rollers.idle();
        }
    }

    public double getTarget() {
        return flywheel.getTargetVelocity();
    }

    public double getCurrentSpeed() {
        return flywheel.getCurrentVelocity();
    }


    public void spool() {
        spooled = true;
    }

    public void unspool() {
        spooled = false;
        firing = false;
    }

    public void fire() {
        firing = true;
    }

    public void stopFiring() {
        firing = false;
    }

    @Override
    public void reset() {}

    @Override
    public void stop() {
        spooled = false;
        firing = false;
        gate.setPosition(GATE_CLOSED);
        rollers.idle();
        flywheel.stop();
    }
}
