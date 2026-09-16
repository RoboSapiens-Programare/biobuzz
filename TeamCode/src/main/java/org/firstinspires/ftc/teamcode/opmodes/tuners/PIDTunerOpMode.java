package org.firstinspires.ftc.teamcode.opmodes.tuners;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.utils.controlSystems.tuners.PIDTuner;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;

/**
 * Tool OpMode (bypasses RobotOpMode on purpose) that tunes the flywheel's velocity PID loop
 * with a Ziegler-Nichols relay test. The relay drives power around VELOCITY_SETPOINT, and the
 * PIDTuner reports kP / kI / kD. Copy the reported gains into setPIDConstants(...).
 *
 * Press START to run.
 */
@TeleOp(name = "PIDTuner", group = "Tuners")
public class PIDTunerOpMode extends OpMode {
    private static final String MOTOR_NAME = "flywheel_left";
    private static final double VELOCITY_SETPOINT = 1000;
    private static final double RELAY_POWER = 0.5;
    private static final int CYCLES = 6;
    private static final double HYSTERESIS = 2.0;
    private static final double TIMEOUT_SECONDS = 20.0;

    private CachingDcMotorEx motor, motor2;
    private PIDTuner tuner;

    @Override
    public void init() {
        motor = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, MOTOR_NAME));
//        motor2 = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "flywheel_right"));
//        motor2.setDirection(DcMotorSimple.Direction.REVERSE);
        tuner = new PIDTuner(VELOCITY_SETPOINT, RELAY_POWER, CYCLES);
    }

    @Override
    public void start() {
        tuner.start(motor.getVelocity());
    }

    @Override
    public void loop() {
        Robot.resetCache();

        if (tuner.getState() == PIDTuner.TuningStates.COMPLETE
                || tuner.getState() == PIDTuner.TuningStates.FAILED) {
            motor.setPower(0);
//            motor2.setPower(0);
        } else {
            double t = tuner.update(motor.getVelocity());
            motor.setPower(t);
//            motor2.setPower(t);
        }

        telemetry.addData("State", tuner.getState());
        telemetry.addData("Power", motor.getPower());
        telemetry.addData("Velocity", motor.getVelocity());
        telemetry.addData("Setpoint", tuner.setpoint);

        if (tuner.getState() == PIDTuner.TuningStates.COMPLETE) {
            telemetry.addLine();
            telemetry.addData("kP", String.format("%.4f", tuner.kP));
            telemetry.addData("kI", String.format("%.4f", tuner.kI));
            telemetry.addData("kD", String.format("%.4f", tuner.kD));
            telemetry.addLine("Copy into setPIDConstants(kP, kI, kD)");
        }
    }

    @Override
    public void stop() {
        if (motor != null) {
            motor.setPower(0);
        }
    }
}