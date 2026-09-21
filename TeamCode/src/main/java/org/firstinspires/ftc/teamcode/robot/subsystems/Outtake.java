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

    public Outtake(HardwareMap hwMap) {
        flywheel.addMotor(hwMap.get(DcMotorEx.class, "flywheel_left"), false)
                .addMotor(hwMap.get(DcMotorEx.class, "flywheel_right"), true)
                .setEncoder(hwMap.get(DcMotorEx.class, "flywheel_left"))
                .setPIDConstants(0.0003, 0.0037, 0.0000)
                .setFFConstants(0.0500, 0.0005, 0.0000)
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
        flywheel.setTargetVelocity(800);
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
