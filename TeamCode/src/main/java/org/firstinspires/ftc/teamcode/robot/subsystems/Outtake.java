package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import dev.frozenmilk.dairy.cachinghardware.CachingServo;
import org.firstinspires.ftc.teamcode.mechanisms.Flywheel;
import org.firstinspires.ftc.teamcode.mechanisms.Roller;

public class Outtake implements Subsystem {

    private final Flywheel flywheel = new Flywheel();
    private final Roller rollers = new Roller();

    private final CachingServo gate;

    private final double GATE_OPEN = 0.5;
    private final double GATE_CLOSED = 0;

    private boolean firing = false;

    public Outtake(HardwareMap hwMap) {
        flywheel.addMotor(hwMap.get(DcMotorEx.class, "flywheel_left"), false)
                .addMotor(hwMap.get(DcMotorEx.class, "flywheel_right"), true)
                .setEncoder(hwMap.get(DcMotorEx.class, "flywheel_left"))
                .setPIDConstants(0.0003, 0.0037, 0.0000)
                .setFFConstants(0.0500, 0.0005, 0.0000)
                .setTolerance(40)
                .setIdlingPower(0.2);

        gate = new CachingServo(hwMap.get(Servo.class, "gate"));
    }

    @Override
    public void init() {}

    @Override
    public void update() {
        flywheel.update();

        if (shootReady() && flywheel.mode == Flywheel.FlywheelMode.PRIMED) {
            gate.setPosition(GATE_OPEN);
        } else {
            gate.setPosition(GATE_CLOSED);
        }
    }

    public double getTarget() {
        return flywheel.getTargetVelocity();
    }

    public double getCurrentSpeed() {
        return flywheel.getCurrentVelocity();
    }

    public int getBallCount() {
        // Implement when sensors
        return 0;
    }

    public void computeVelocity() {
        flywheel.setTargetVelocity(800);
    }

    public boolean shootReady() {
        return flywheel.velocityReached();
    }

    public void startIdling() {
        flywheel.mode = Flywheel.FlywheelMode.IDLING;
    }

    public void stopIdling() {
        flywheel.mode = Flywheel.FlywheelMode.PRIMED;
    }

    @Override
    public void reset() {}

    @Override
    public void stop() {
        firing = false;
        gate.setPosition(GATE_CLOSED);
        rollers.idle();
        flywheel.stop();
    }
}
