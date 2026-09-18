package org.firstinspires.ftc.teamcode.utils.ControlSystems;

import com.qualcomm.robotcore.util.Range;

public class FFController {
    // 1. TUNING VARIABLES
    public double kP;
    public double kI;
    public double kD;

    public double kS;
    public double kV;
    public double kA;

    public double maxOutput;
    public double minOutput;

    // 2. STATE VARIABLES
    private double targetVelocity = 0;
    private double targetAcceleration = 0;
    private double error = 0;
    private double tolerance = 0;
    private double filteredVelocity = 0;

    private static final double VELOCITY_FILTER_ALPHA = 0.5;

    public PIDFController pidfController;

    public FFController(double kP, double kI, double kD, double kS, double kV, double kA) {
        this(kP, kI, kD, kS, kV, kA, -1.0, 1.0);
    }

    public FFController(
            double kP, double kI, double kD, double kS, double kV, double kA, double minOutput, double maxOutput) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;

        this.kS = kS;
        this.kV = kV;
        this.kA = kA;

        this.minOutput = minOutput;
        this.maxOutput = maxOutput;

        // Pass 0 for feedforward so PIDFController purely computes PID feedback
        this.pidfController = new PIDFController(kP, kI, kD, 0.0, minOutput, maxOutput);
    }

    public void setTargetVelocity(double velocity) {
        this.targetVelocity = velocity;
        if (pidfController != null) {
            pidfController.setSetpoint(velocity);
        }
    }

    public void setTargetAcceleration(double acceleration) {
        this.targetAcceleration = acceleration;
    }

    public void setPIDConstants(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;

        if (pidfController != null) {
            pidfController.kP = kP;
            pidfController.kI = kI;
            pidfController.kD = kD;
        }
    }

    public double update(double currentVelocity) {
        this.error = targetVelocity - currentVelocity;

        // EMA-filter the reading so derivative noise can't destabilize the hold
        filteredVelocity += VELOCITY_FILTER_ALPHA * (currentVelocity - filteredVelocity);

        // PID only: feedback correction based on actual error
        double pidOutput = pidfController.update(filteredVelocity);

        // Static-friction feedforward fights zero-crossing stiction without riding the noise
        if (Math.abs(error) > tolerance) {
            pidOutput += kS * Math.signum(error);
        }

        // Velocity / acceleration feedforward (identified by FFTuner)
        pidOutput += kV * targetVelocity + kA * targetAcceleration;

        return Range.clip(pidOutput, minOutput, maxOutput);
    }

    public void reset() {
        error = 0;
        if (pidfController != null) {
            pidfController.reset();
        }
    }

    public void setTolerance(double tolerance) {
        this.tolerance = tolerance;
        if (pidfController != null) {
            pidfController.setTolerance(tolerance);
        }
    }

    public boolean velocityReached() {
        return Math.abs(error) <= tolerance;
    }
}
