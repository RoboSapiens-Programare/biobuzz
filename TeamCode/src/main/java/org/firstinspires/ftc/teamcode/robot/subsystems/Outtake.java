package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;
import dev.frozenmilk.dairy.cachinghardware.CachingServo;
import org.firstinspires.ftc.teamcode.mechanisms.Flywheel;
import org.firstinspires.ftc.teamcode.mechanisms.Roller;
import org.firstinspires.ftc.teamcode.utils.ControlSystems.PIDFController;

@Configurable
public class Outtake implements Subsystem {

    private final Flywheel flywheel = new Flywheel();
    private final Roller rollers = new Roller();

    private final CachingServo gate;
    private final CachingServo hood;
    private final CachingDcMotorEx pivot;
    private final PIDFController pivotController = new PIDFController(0, 0, 0, 0);
    private Tracker tracker;
    public static int CORRESPONDING_180_CLOCKWISE_TICKS = 0;

    public static double GATE_OPEN = 0.5;
    public static double GATE_CLOSED = 0;

    public static double HOOD_LOW = 0;
    public static double HOOD_HIGH = 1;

    public static double velocity = 0;

    public static double kP = 0.0172;
    public static double kI = 0.4035;
    public static double kD = 0.0002;
    public static double kS = 0.0133;
    public static double kV = 0.0004;

    public Outtake(HardwareMap hwMap) {
        flywheel.addMotor(hwMap.get(DcMotorEx.class, "flywheel"), false)
                .setEncoder(hwMap.get(DcMotorEx.class, "flywheel"))
                .setPIDConstants(kP, kI, kD)
                .setFFConstants(kS, kV, 0)
                .setTolerance(40)
                .setIdlingPower(0.2);

        gate = new CachingServo(hwMap.get(Servo.class, "gate"));

        hood = new CachingServo(hwMap.get(Servo.class, "hood"));

        pivot = new CachingDcMotorEx(hwMap.get(DcMotorEx.class, "pivot"));
    }

    @Override
    public void init() {
        hood.setPosition(HOOD_LOW);
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
            setHood(tracker.getHood());
        }

        double pow = pivotController.update(pivot.getCurrentPosition());
        pivot.setPower(pow);
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

    private int angleToTicks(double rad) {
        return Math.toIntExact(Math.round((-rad / Math.PI) * CORRESPONDING_180_CLOCKWISE_TICKS));
    }

    public void aimAt(double angleRad) {
        pivotController.setSetpoint(angleToTicks(angleRad));
    }

    public void setHood(double position) {
        hood.setPosition(Math.max(0, Math.min(1, position)));
    }

    @Override
    public void reset() {}

    @Override
    public void stop() {
        gate.setPosition(GATE_CLOSED);
        hood.setPosition(HOOD_LOW);
        rollers.idle();
        flywheel.stop();
    }
}
