package org.firstinspires.ftc.teamcode.opmodes.tuners;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanisms.Flywheel;
import org.firstinspires.ftc.teamcode.utils.ControlSystems.Tuners.FFTuner;

import java.util.ArrayList;
import java.util.List;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;

/**
 * Tool OpMode that tunes the flywheel's FF + PID with a configurable motor table.
 *
 * Edit the MOTORS table below to match your robot: list every motor that drives the flywheel and
 * whether it must be reversed (so all wheels spin the same physical direction under the same power).
 * The first motor is used as the velocity encoder source.
 *
 * Stage 1 (FF_RAMP): ramps power from 0 to MAX_POWER, fits kS / kV via least squares.
 * Stage 2 (PID_RELAY): runs a Ziegler-Nichols relay test on top of the identified feedforward,
 * fits kP / kI / kD.
 * Stage 3 (VERIFY): writes the tuned gains into FlywheelConfig + a Flywheel mechanism, spools at
 * target and reports how close the flywheel actually holds.
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
    private static final double RELAY_POWER = 0.5;
    private static final int PID_CYCLES = 10;
    private static final double PID_TIMEOUT_SECONDS = 20.0;

    // Verify stage
    private static final double VERIFY_TIMEOUT_SECONDS = 5.0;

    private final List<CachingDcMotorEx> motors = new ArrayList<>();
    private CachingDcMotorEx encoder;

    private Flywheel flywheel;
    private FFTuner tuner;
    private boolean verifyActive = false;
    private final ElapsedTime verifyTimer = new ElapsedTime();

    private List<LynxModule> hubs;

    private double target = 2000;

    @Override
    public void init() {
        if (MOTORS.length == 0) {
            throw new IllegalStateException("Configure at least one motor in MOTORS[]");
        }

        // Manually manage bulk caching (Robot is not constructed in this OpMode).
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        Flywheel built = new Flywheel();
        for (MotorSpec spec : MOTORS) {
            CachingDcMotorEx motor = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, spec.name));
            motor.setDirection(spec.reversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motors.add(motor);
            built.addMotor(motor, spec.reversed);
        }

        encoder = motors.get(0);
        built.setEncoder(encoder);

        flywheel = built;

        tuner = new FFTuner(MAX_POWER, POWER_STEP, STEADY_STATE_TIME, MOTION_THRESHOLD, TIMEOUT_SECONDS,
                PID_SETPOINT, RELAY_POWER, PID_CYCLES, PID_TIMEOUT_SECONDS);
    }

    @Override
    public void start() {
        tuner.start();
    }

    @Override
    public void loop() {
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }

        FFTuner.TuningStates state = tuner.getState();

        if (state == FFTuner.TuningStates.FAILED) {
            setAllPower(0);
        } else if (state == FFTuner.TuningStates.COMPLETE) {
            if (!verifyActive) {
                applyGainsAndStartVerify();
            }
            runVerify();
        } else {
            setAllPower(tuner.update(getVelocity()));
        }

        telemetry.addData("State", tuneSummary());
        telemetry.addData("Stage", tuner.getStage());
        telemetry.addData("Velocity", getVelocity());

        if (tuner.getStage() == FFTuner.TuningStages.PID_RELAY) {
            telemetry.addData("Setpoint", PID_SETPOINT);
        }

        if (state == FFTuner.TuningStates.COMPLETE) {
            telemetry.addLine();
            telemetry.addData("kP", String.format("%.4f", tuner.kP));
            telemetry.addData("kI", String.format("%.4f", tuner.kI));
            telemetry.addData("kD", String.format("%.4f", tuner.kD));
            telemetry.addData("kS", String.format("%.4f", tuner.kS));
            telemetry.addData("kV", String.format("%.4f", tuner.kV));
            telemetry.addLine("Written to FlywheelConfig for TeleOP.");
        }
    }

    @Override
    public void stop() {
        setAllPower(0);
    }

    private void applyGainsAndStartVerify() {
        flywheel.setPIDConstants(tuner.kP, tuner.kI, tuner.kD);
        flywheel.setFFConstants(tuner.kS, tuner.kV, 0);
        flywheel.setTargetVelocity(target);

        verifyActive = true;
        verifyTimer.reset();
    }

    private void runVerify() {
        flywheel.update();

        telemetry.addData("Verify target", target);
        telemetry.addData("Verify error", String.format("%.1f", target - getVelocity()));

        if (verifyTimer.seconds() > VERIFY_TIMEOUT_SECONDS) {
            telemetry.addLine(getVelocity() >= target - 15
                    ? "HOLDS target to within tolerance."
                    : "Does NOT reach target - lower target or re-tune.");
        }
    }

    private String tuneSummary() {
        if (verifyActive) {
            return "VERIFY";
        }
        switch (tuner.getState()) {
            case IDLE: return "IDLE - press START";
            case TUNING: return "TUNING";
            case COMPLETE: return "COMPLETE";
            case FAILED: return "FAILED";
            default: return tuner.getState().name();
        }
    }

    /**
     * Raw encoder velocity. MOTORS[0] must read POSITIVE when the flywheel spins forward
     * (i.e. launches the ring), matching the Outtake subsystem convention.
     */
    private double getVelocity() {
        return encoder.getVelocity();
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