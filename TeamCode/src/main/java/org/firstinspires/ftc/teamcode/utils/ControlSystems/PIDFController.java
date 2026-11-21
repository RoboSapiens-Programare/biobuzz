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
    private double maxIntegral = 1.0; // Cap for the I-term contribution to the output
    private double tolerance = 1;
    private double integralTolerance = 1;
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

        double pTerm = kP * error;
        double dTerm = Range.clip(kD * derivative, -MAX_D_OUTPUT, MAX_D_OUTPUT);
        double ffTerm = (Math.abs(error) > tolerance) ? (kF * Math.signum(error)) : 0.0;

        // Cap the accumulated integral so the I contribution stays within maxIntegral.
        double integralLimit = (Math.abs(kI) > 1e-9) ? (maxIntegral / Math.abs(kI)) : maxIntegral;

        // Reset the integral once we are inside the integral tolerance band.
        if (Math.abs(error) <= integralTolerance) {
            integral = 0;
        } else {
            // Anti-windup: only integrate when doing so does not push the
            // (unsaturated) output further into saturation. Unwinding is always
            // allowed, so a large setpoint step can recover instead of latching.
            double tentative = pTerm + dTerm + (kI * integral) + ffTerm;
            boolean windupHigh = tentative >= maxOutput && error > 0;
            boolean windupLow = tentative <= minOutput && error < 0;
            if (!windupHigh && !windupLow) {
                integral += error * timeChange;
            }
        }
        integral = Range.clip(integral, -integralLimit, integralLimit);

        // Calculate PID Output
        double output = pTerm + dTerm + (kI * integral) + ffTerm;

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

    public void setIntegralTolerance(double integralTolerance) {
        this.integralTolerance = integralTolerance;
    }

    public boolean targetReached() {
        return Math.abs(error) <= tolerance;
    }
}
