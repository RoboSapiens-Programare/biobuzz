package org.firstinspires.ftc.teamcode.opmodes.tuners;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.utils.ControlSystems.Tuners.FFTuner;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;

/**
 * Tool OpMode (bypasses RobotOpMode on purpose) that fully autotunes the flywheel's
 * FFController in one run:
 *
 * Stage 1 (FF_RAMP): ramps from 0 to MAX_POWER, fits kS / kV via least squares.
 * Stage 2 (PID_RELAY): runs a Ziegler-Nichols relay test on top of the identified feedforward,
 * fits kP / kI / kD.
 *
 * Copy the reported gains into setPIDConstants(...).
 *
 * Press START to run.
 */
@TeleOp(name = "FF + PID Tuner", group = "Tuners")
public class FFTunerOpMode extends OpMode {
    private static final String MOTOR_NAME = "flywheel_left";
    private static final String MOTOR_NAME_2 = "flywheel_right";
    private static final double MAX_POWER = 0.8;
    private static final double POWER_STEP = 0.05;
    private static final double STEADY_STATE_TIME = 0.6;
    private static final double MOTION_THRESHOLD = 3.0;
    private static final double TIMEOUT_SECONDS = 30.0;

    // PID relay stage (must be within the range MAX_POWER can sustain)
    private static final double PID_SETPOINT = 1000;
    private static final double RELAY_POWER = 0.2;
    private static final int PID_CYCLES = 6;
    private static final double PID_TIMEOUT_SECONDS = 20.0;

    private CachingDcMotorEx motor, motor2;
    private FFTuner tuner;

    @Override
    public void init() {
        motor = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, MOTOR_NAME));
//        motor2 = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, MOTOR_NAME_2));
//        motor2.setDirection(DcMotorSimple.Direction.REVERSE);
        tuner = new FFTuner(MAX_POWER, POWER_STEP, STEADY_STATE_TIME, MOTION_THRESHOLD, TIMEOUT_SECONDS,
                PID_SETPOINT, RELAY_POWER, PID_CYCLES, PID_TIMEOUT_SECONDS);
    }

    @Override
    public void start() {
        tuner.start();
    }

    @Override
    public void loop() {
        Robot.resetCache();

        if (tuner.getState() == FFTuner.TuningStates.COMPLETE
                || tuner.getState() == FFTuner.TuningStates.FAILED) {
            motor.setPower(0);
//            motor2.setPower(0);
        } else {
            double d = tuner.update(motor.getVelocity());
            motor.setPower(d);
//            motor2.setPower(d);
        }

        telemetry.addData("State", tuner.getState());
        telemetry.addData("Stage", tuner.getStage());
        telemetry.addData("Power", motor.getPower());
        telemetry.addData("Velocity", motor.getVelocity());

        if (tuner.getStage() == FFTuner.TuningStages.PID_RELAY) {
            telemetry.addData("Setpoint", PID_SETPOINT);
        }

        if (tuner.getState() == FFTuner.TuningStates.COMPLETE) {
            telemetry.addLine();
            telemetry.addData("kP", String.format("%.4f", tuner.kP));
            telemetry.addData("kI", String.format("%.4f", tuner.kI));
            telemetry.addData("kD", String.format("%.4f", tuner.kD));
            telemetry.addLine("Copy into setPIDConstants(kP, kI, kD)");
            telemetry.addLine();
            telemetry.addData("kS", String.format("%.4f", tuner.kS));
            telemetry.addData("kV", String.format("%.4f", tuner.kV));
            telemetry.addLine("Feedforward is disabled in control; use PIDTuner for PID-only gains.");
        }
    }

    @Override
    public void stop() {
        if (motor != null) {
            motor.setPower(0);
        }

        if (motor2 != null) {
            motor2.setPower(0);
        }
    }
}