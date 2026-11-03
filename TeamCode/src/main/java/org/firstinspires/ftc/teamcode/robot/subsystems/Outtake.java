package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import dev.frozenmilk.dairy.cachinghardware.CachingServo;
import org.firstinspires.ftc.teamcode.mechanisms.Flywheel;
import org.firstinspires.ftc.teamcode.mechanisms.Roller;

@Configurable
public class Outtake implements Subsystem {

    private final Flywheel flywheel = new Flywheel();
    private final Roller rollers = new Roller();

    private final CachingServo gate;
    private final CachingServo pivot_left, pivot_right;
    public static final double PIVOT_RANGE = Math.toRadians((double) (360 * 3) / 4);
    public static double aimOffset = Math.toRadians(0);

    private Tracker tracker;

    public static double GATE_OPEN = 0.5;
    public static double GATE_CLOSED = 0;

    public static double velocity = 0;

    public static double kP = 0.0172;
    public static double kI = 0.1707;
    public static double kD = 0.0004;
    public static double kS = 0.0199;
    public static double kV = 0.0004;

    public double pos = 0;

    public Outtake(HardwareMap hwMap) {
        flywheel.addMotor(hwMap.get(DcMotorEx.class, "flywheel"), true)
                .setEncoder(hwMap.get(DcMotorEx.class, "flywheel"))
                .setPIDConstants(kP, kI, kD)
                .setFFConstants(kS, kV, 0)
                .setTolerance(40)
                .setIdlingPower(0.2);

        gate = new CachingServo(hwMap.get(Servo.class, "gate"));

        pivot_left = new CachingServo(hwMap.get(Servo.class, "pivot_left"));

        pivot_right = new CachingServo(hwMap.get(Servo.class, "pivot_right"));
    }

    @Override
    public void init() {
        aimAt(0);
    }

    @Override
    public void update() {
        flywheel.setPIDConstants(kP, kI, kD);
        flywheel.setFFConstants(kS, kV, 0);
        flywheel.update();

        if (shootReady() && flywheel.mode == Flywheel.FlywheelMode.PRIMED) {
            gate.setPosition(GATE_OPEN);
        } else {
            gate.setPosition(GATE_CLOSED);
        }

        if (tracker != null && Tracker.isEnabled()) {
            aimAt(tracker.getTrackAngle());
        }
    }

    public void setTracker(Tracker tracker) {
        this.tracker = tracker;
    }

    public double getTarget() {
        return flywheel.getTargetVelocity();
    }

    public double getCurrentSpeed() {
        return flywheel.getCurrentVelocity();
    }

    public void computeVelocity() {
        flywheel.setTargetVelocity(velocity);
    }

    public void setVelocity(double velocity) {
        flywheel.setTargetVelocity(velocity);
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

    private double angleToPosition(double rad) {
        rad = Math.max(Math.min(rad, PIVOT_RANGE / 2), -PIVOT_RANGE / 2); // clip
        rad += PIVOT_RANGE / 2;
        rad += aimOffset;
        return Math.max(0, Math.min(1, rad / PIVOT_RANGE));
    }

    public void aimAt(double angleRad) {

        pos = angleToPosition(angleRad);
        pivot_left.setPosition(pos);
        pivot_right.setPosition(pos);
    }

    @Override
    public void reset() {}

    @Override
    public void stop() {
        gate.setPosition(GATE_CLOSED);
        rollers.idle();
        flywheel.stop();
    }
}
