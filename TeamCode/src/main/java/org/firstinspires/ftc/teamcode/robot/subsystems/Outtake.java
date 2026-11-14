package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import dev.frozenmilk.dairy.cachinghardware.CachingServo;
import org.firstinspires.ftc.teamcode.mechanisms.Flywheel;
import org.firstinspires.ftc.teamcode.mechanisms.Roller;
import org.firstinspires.ftc.teamcode.robot.Robot;

@Configurable
public class Outtake implements Subsystem {

    private final Flywheel flywheel = new Flywheel();
    private final Roller rollers = new Roller();

    private final CachingServo gate;
    private final CachingServo pivot_left, pivot_right;
    public static final double PIVOT_RANGE = Math.toRadians((double) (360 * 3) / 4);
    private static final double PIVOT_TOLERANCE = 0.002;
    public static double aimOffset = Math.toRadians(0);

    private Tracker tracker;
    private boolean firing = false;

    private double lastGatePosition = Double.NaN;
    private int lastAimVersion = Integer.MIN_VALUE;

    private double lastKP = Double.NaN;
    private double lastKI = Double.NaN;
    private double lastKD = Double.NaN;
    private double lastKS = Double.NaN;
    private double lastKV = Double.NaN;

    public static double GATE_OPEN = 0.76;
    public static double GATE_CLOSED = 0.63;

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

        pivot_left.setCachingTolerance(PIVOT_TOLERANCE);
        pivot_right.setCachingTolerance(PIVOT_TOLERANCE);

        pivot_right.getController().pwmDisable();
        pivot_left.getController().pwmDisable();
    }

    @Override
    public void init() {
        aimAt(0);
        setGate(GATE_CLOSED);
    }

    @Override
    public void update() {
        if (kP != lastKP || kI != lastKI || kD != lastKD) {
            lastKP = kP;
            lastKI = kI;
            lastKD = kD;
            flywheel.setPIDConstants(kP, kI, kD);
        }

        if (kS != lastKS || kV != lastKV) {
            lastKS = kS;
            lastKV = kV;
            flywheel.setFFConstants(kS, kV, 0);
        }

        if (tracker != null && Tracker.isEnabled()) {
            if (!Robot.TUNING) {
                double trackerVelocity = tracker.getVelocity();
                if (firing || trackerVelocity > 0) {
                    flywheel.setTargetVelocity(trackerVelocity);
                }
            }

            int angleVersion = Tracker.getAngleVersion();
            if (angleVersion != lastAimVersion) {
                lastAimVersion = angleVersion;
                aimAt(tracker.getTrackAngle());
            }
        }

        flywheel.update();

        if (Robot.TUNING && flywheel.mode == Flywheel.FlywheelMode.PRIMED) {
            setGate(GATE_OPEN);
        } else if (firing && shootReady() && flywheel.mode == Flywheel.FlywheelMode.PRIMED) {
            setGate(GATE_OPEN);
        } else {
            setGate(GATE_CLOSED);
        }
    }

    private void setGate(double position) {
        if (position == lastGatePosition) {
            return;
        }
        lastGatePosition = position;
        gate.setPosition(position);
    }

    public void setTracker(Tracker tracker) {
        this.tracker = tracker;
    }

    public void setFiring(boolean firing) {
        this.firing = firing;
    }

    public boolean isFiring() {
        return firing;
    }

    public double getTarget() {
        return flywheel.getTargetVelocity();
    }

    public double getCurrentSpeed() {
        return flywheel.getCurrentVelocity();
    }

    public void setVelocity(double velocity) {
        flywheel.setTargetVelocity(velocity);
    }

    public boolean shootReady() {
        //        return flywheel.velocityReached();
        return true;
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
        firing = false;
        setGate(GATE_CLOSED);
        rollers.idle();
        flywheel.stop();
    }
}
