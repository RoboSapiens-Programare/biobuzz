package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.mechanisms.Roller;

@Configurable
public class Intake implements Subsystem {
    private enum Mode {
        PULL,
        PUSH,
        IDLE
    }

    private final Roller rollerOne = new Roller();
    private final Roller rollerTwo = new Roller();

    private final DistanceSensor pollenOneSensor;
    private final DistanceSensor pollenThreeSensor;

    private static final long SENSOR_POLL_MS = 200;

    private final ElapsedTime sensorTimer = new ElapsedTime();

    private double pollenOneDistance = Double.POSITIVE_INFINITY;
    private double pollenThreeDistance = Double.POSITIVE_INFINITY;
    private Mode mode = Mode.IDLE;

    public static double MIN_DISTANCE = 5;
    public static double PULL_POWER = 0.86;
    private boolean force = false;

    private final Servo flower;

    public Intake(HardwareMap hwMap) {
        rollerOne.addMotor(hwMap.get(DcMotorEx.class, "roller_motor"), true);
        rollerTwo.addMotor(hwMap.get(DcMotorEx.class, "roller_motor_2"));

        rollerOne.setPower(PULL_POWER);
        rollerTwo.setPower(PULL_POWER);

        pollenOneSensor = hwMap.get(DistanceSensor.class, "pollen_one_sensor");
        pollenThreeSensor = hwMap.get(DistanceSensor.class, "pollen_three_sensor");

        flower = hwMap.get(Servo.class, "flower_servo");
    }

    public void pullBalls() {
        pullBalls(false);
    }

    public void pullBalls(boolean force) {
        mode = Mode.PULL;
        this.force = force;
    }

    public void pushBalls() {
        mode = Mode.PUSH;
        force = false;
    }

    public void idle() {
        mode = Mode.IDLE;
        force = false;
    }

    public void lowerFlower() {
        flower.setPosition(0.3);
    }

    public void raiseFlower() {
        flower.setPosition(0.9);
    }

    public int getBallCount() {
        int count = 0;
        count += pollenOneDistance < MIN_DISTANCE ? 1 : 0;
        count += pollenThreeDistance < MIN_DISTANCE ? 1 : 0;

        return count;
    }

    @Override
    public void init() {
        mode = Mode.IDLE;
        sensorTimer.reset();

        raiseFlower();
    }

    @Override
    public void update() {
        if (sensorTimer.milliseconds() > SENSOR_POLL_MS) {
            pollenOneDistance = pollenOneSensor.getDistance(DistanceUnit.CM);
            pollenThreeDistance = pollenThreeSensor.getDistance(DistanceUnit.CM);

            sensorTimer.reset();
        }

        boolean staged = pollenOneDistance < MIN_DISTANCE && pollenThreeDistance < MIN_DISTANCE;
        applyMode(staged);

        rollerOne.update();
        rollerTwo.update();
    }

    private void applyMode(boolean staged) {
        switch (mode) {
            case PULL:
                rollerOne.pull();
                if (staged && !force) {
                    rollerTwo.idle();
                } else {
                    rollerTwo.pull();
                }
                break;
            case PUSH:
                rollerOne.push();
                rollerTwo.push();
                break;
            case IDLE:
            default:
                rollerOne.idle();
                rollerTwo.idle();
                break;
        }
    }

    @Override
    public void reset() {
        mode = Mode.IDLE;
        pollenOneDistance = Double.POSITIVE_INFINITY;
        pollenThreeDistance = Double.POSITIVE_INFINITY;
        sensorTimer.reset();
    }

    @Override
    public void stop() {
        idle();
        rollerOne.stop();
        rollerTwo.stop();
    }

    public double getMaxCurrent() {
        return Math.max(rollerOne.getMaxCurrent(), rollerTwo.getMaxCurrent());
    }
}
