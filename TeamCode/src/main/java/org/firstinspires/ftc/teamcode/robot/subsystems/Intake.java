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

    private enum FlowerPosition {
        RAISED,
        LOWERED
    }

    private final Roller rollerOne = new Roller();
    private final Roller rollerTwo = new Roller();

    private final DistanceSensor pollenOneSensor;
    private final DistanceSensor pollenThreeSensor;

    private static final long SENSOR_POLL_MS = 300;

    private final ElapsedTime sensorTimer = new ElapsedTime();

    private double pollenOneDistance = Double.POSITIVE_INFINITY;
    private double pollenThreeDistance = Double.POSITIVE_INFINITY;
    private Mode mode = Mode.IDLE;

    public static double MIN_DISTANCE = 5;
    public static double PULL_POWER = 0.86;
    private boolean force = false;
    private int ballCount = 0;

    private final Servo flowerLeft, flowerRight;
    private FlowerPosition flowerPosition;

    public Intake(HardwareMap hwMap) {
        rollerOne.addMotor(hwMap.get(DcMotorEx.class, "roller_motor"), true);
        rollerTwo.addMotor(hwMap.get(DcMotorEx.class, "roller_motor_2"));

        rollerOne.setPower(0.86);
        rollerTwo.setPower(0.41);

        pollenOneSensor = hwMap.get(DistanceSensor.class, "pollen_one_sensor");
        pollenThreeSensor = hwMap.get(DistanceSensor.class, "pollen_three_sensor");

        flowerLeft = hwMap.get(Servo.class, "left_flower_servo");
        flowerRight = hwMap.get(Servo.class, "right_flower_servo");
        flowerRight.setDirection(Servo.Direction.REVERSE);
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
        flowerLeft.setPosition(0.4);
        flowerRight.setPosition(0.4);
        flowerPosition = FlowerPosition.LOWERED;
    }

    public void raiseFlower() {
        flowerRight.setPosition(0.78);
        flowerLeft.setPosition(0.78);
        flowerPosition = FlowerPosition.RAISED;
    }

    public void toggleFlower() {
        if (flowerPosition == FlowerPosition.RAISED) {
            lowerFlower();
        } else {
            raiseFlower();
        }
    }

    public int getBallCount() {
        return ballCount;
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

            ballCount = 0;
            ballCount += pollenOneDistance < MIN_DISTANCE ? 1 : 0;
            ballCount += pollenThreeDistance < MIN_DISTANCE ? 1 : 0;

            sensorTimer.reset();
        }

        boolean staged = pollenOneDistance < MIN_DISTANCE && pollenThreeDistance < MIN_DISTANCE;
        applyMode(staged);

        //        if (rollerTwo.getMaxCurrent() > 6) {
        //            rollerTwoDisabled = true;
        //        }

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
