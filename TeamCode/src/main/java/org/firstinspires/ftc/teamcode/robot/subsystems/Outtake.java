package org.firstinspires.ftc.teamcode.robot.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanisms.Flywheel;
import org.firstinspires.ftc.teamcode.mechanisms.Roller;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;
import dev.frozenmilk.dairy.cachinghardware.CachingServo;

public class Outtake implements Subsystem {

    private final Flywheel flywheel = new Flywheel();
    private final Roller rollers = new Roller();

    private final CachingServo gate;

    private static final double IDLE_VELOCITY = 700;
    private static final double SPOOLED_VELOCITY = 1500;

    private final double GATE_OPEN = 0.5;
    private final double GATE_CLOSED = 0;

    private boolean spooled = false;
    private boolean firing = false;
    private boolean manualFlywheel = false;

    public Outtake(HardwareMap hwMap) {
        flywheel.addMotor(hwMap.get(DcMotorEx.class, "flywheel_left"))
                .addMotor(hwMap.get(DcMotorEx.class, "flywheel_right"), true)
                .setEncoder(hwMap.get(DcMotorEx.class, "flywheel_left"))
//                .setPIDConstants(0.0022, 0.0173, 0.0001)
                .setPIDConstants(0.0027, 0.0290, 0.0003)
                .setTolerance(15);

        rollers.addMotor(hwMap.get(DcMotorEx.class, "roller_motor"))
               .setPower(1);

        gate = new CachingServo(hwMap.get(Servo.class, "gate"));
    }

    @Override
    public void init() {}

    @Override
    public void update() {
        if (!manualFlywheel) {
            flywheel.setPIDConstants(0.0027, 0.0290, 0.0003);
            flywheel.setTargetVelocity(spooled ? SPOOLED_VELOCITY : IDLE_VELOCITY);
            flywheel.update();
        }

        if (firing && flywheel.velocityReached()) {
            gate.setPosition(GATE_OPEN);
            rollers.pull();
        } else {
            gate.setPosition(GATE_CLOSED);
            rollers.idle();
        }
    }

    public void setManualFlywheel(boolean manual) {
        this.manualFlywheel = manual;
    }

    public double getTarget() {
        return flywheel.getTargetVelocity();
    }

    public double getCurrentSpeed() {
        return flywheel.getCurrentSpeed();
    }

    public void setPIDConstants(double kP, double kI, double kD) {
        flywheel.setPIDConstants(kP, kI, kD);
    }

    public double getKP() {
        return flywheel.getKP();
    }

    public double getKI() {
        return flywheel.getKI();
    }

    public double getKD() {
        return flywheel.getKD();
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
