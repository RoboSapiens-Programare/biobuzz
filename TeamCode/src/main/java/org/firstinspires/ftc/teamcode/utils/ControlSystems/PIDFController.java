package org.firstinspires.ftc.teamcode.utils.ControlSystems;

// import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class PIDFController {
    // 1. TUNING VARIABLES (Not static, so you can have multiple PIDs)
    public double kP;
    public double kI;
    public double kD;
    public double kF;

    // 2. STATE VARIABLES
    public double error = 0;
    private double previousError = 0;
    private double integral = 0;
    private double setpoint = 0;
    private double maxIntegral = 10.0; // Cap for the "I" term
    private double tolerance = 1;
    private static final double MAX_D_OUTPUT = 0.1; // Clamp for derivative term to kill noise kick

    public double maxOutput;
    public double minOutput;

    private ElapsedTime timer = new ElapsedTime();

    public PIDFController(double kP, double kI, double kD, double kF) {
        this(kP, kI, kD, kF, -1.0, 1.0);
    }

    public PIDFController(double kP, double kI, double kD, double kF, double minOutput, double maxOutput) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
        timer.reset();

        this.minOutput = minOutput;
        this.maxOutput = maxOutput;
    }

    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    public double update(double currentValue) {
        double timeChange = timer.seconds();

        if (timeChange <= 0) {
            timeChange = 1e-3;
        }

        // Calculate Error
        error = setpoint - currentValue;

        // Calculate Derivative (clamped so velocity noise can't flip the output sign)
        double derivative = (error - previousError) / timeChange;
        previousError = error;

        // Proportional + clamped Derivative term (before I, to check windup)
        double output = (kP * error) + Range.clip(kD * derivative, -MAX_D_OUTPUT, MAX_D_OUTPUT);

        // Anti-windup: only accumulate integral while the raw output isn't saturated
        if (output > minOutput && output < maxOutput) {
            if (Math.abs(error) <= tolerance) {
                integral = 0;
            } else {
                integral += error * timeChange;
            }
        }
        integral = Range.clip(integral, -maxIntegral, maxIntegral);

        // Calculate PID Output
        output += (kI * integral);

        if (Math.abs(error) > tolerance) {
            output += kF * Math.signum(error);
        }

        timer.reset();

        return Range.clip(output, minOutput, maxOutput);
    }

    public void reset() {
        integral = 0;
        previousError = 0;
        timer.reset();
    }

    public void setTolerance(double tolerance) {
        this.tolerance = tolerance;
    }

    public boolean targetReached() {
        return Math.abs(error) <= tolerance;
    }
}
