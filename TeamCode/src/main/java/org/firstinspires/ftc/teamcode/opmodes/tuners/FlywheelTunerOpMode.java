package org.firstinspires.ftc.teamcode.opmodes.tuners;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.utils.controlSystems.tuners.FFTuner;

import java.util.ArrayList;
import java.util.List;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;

/**
 * Tool OpMode (bypasses RobotOpMode on purpose) that autotunes a flywheel with FF + PID in one run.
 *
 * Edit the MOTORS table below to match your robot: list every motor that drives the flywheel and
 * whether it must be reversed (so all wheels spin the same physical direction under the same power).
 * The first motor is used as the velocity encoder source.
 *
 * Stage 1 (FF_RAMP): ramps from 0 to MAX_POWER, fits kS / kV via least squares.
 * Stage 2 (PID_RELAY): runs a Ziegler-Nichols relay test on top of the identified feedforward,
 * fits kP / kI / kD.
 *
 * Copy the reported gains into setPIDConstants(...) / setFFConstants(...).
 *
 * Press START to run.
 */
@TeleOp(name = "Flywheel Tuner", group = "Tuners")
public class FlywheelTunerOpMode extends OpMode {
    /** One entry per flywheel motor. The first entry is the velocity encoder. */
    private static final MotorSpec[] MOTORS = {
            new MotorSpec("flywheel_left", false),
            new MotorSpec("flywheel_right", true),
    };

    // FF ramp stage
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

    private final List<CachingDcMotorEx> motors = new ArrayList<>();
    private CachingDcMotorEx encoder;
    private boolean encoderReversed;

    private FFTuner tuner;

    @Override
    public void init() {
        if (MOTORS.length == 0) {
            throw new IllegalStateException("Configure at least one motor in MOTORS[]");
        }

        for (MotorSpec spec : MOTORS) {
            CachingDcMotorEx motor = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, spec.name));
            motor.setDirection(spec.reversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motors.add(motor);
        }

        encoder = motors.get(0);
        encoderReversed = MOTORS[0].reversed;

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
            setAllPower(0);
        } else {
            double d = tuner.update(getVelocity());
            setAllPower(d);
        }

        telemetry.addData("State", tuner.getState());
        telemetry.addData("Stage", tuner.getStage());
        telemetry.addData("Power", encoder.getPower());
        telemetry.addData("Velocity", getVelocity());

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
            telemetry.addLine("Copy into setFFConstants(kS, kV, 0) and feed target velocity.");
        }
    }

    @Override
    public void stop() {
        setAllPower(0);
    }

    /** Reports the encoder velocity with a consistent sign regardless of the encoder motor's direction. */
    private double getVelocity() {
        double raw = encoder.getVelocity();
        return encoderReversed ? -raw : raw;
    }

    private void setAllPower(double power) {
        for (CachingDcMotorEx motor : motors) {
            motor.setPower(power);
        }
    }

    private static class MotorSpec {
        private final String name;
        private final boolean reversed;

        private MotorSpec(String name, boolean reversed) {
            this.name = name;
            this.reversed = reversed;
        }
    }
}