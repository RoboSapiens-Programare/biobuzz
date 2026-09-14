package org.firstinspires.ftc.teamcode.utils.controlSystems.tuners;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDTuner {
    public double setpoint;
    public double power;
    
    private final int requiredCycles;
    private int currentCycle = 0;
    private boolean lastAbove = false;

    private final ElapsedTime cycleTimer = new ElapsedTime();

    private double minVal = Double.POSITIVE_INFINITY;
    private double maxVal = Double.NEGATIVE_INFINITY;

    private double totalAmplitude = 0;
    private double totalPeriod = 0;

    public double kP, kI, kD;

    public enum TuningStates {
        IDLE,
        TUNING,
        COMPLETE,
        FAILED
    }

    private TuningStates state;

    public PIDTuner(double setpoint, double power, int cycles) {
        this.setpoint = setpoint;
        this.power = Math.abs(power);
        this.state = TuningStates.IDLE;
        this.requiredCycles = cycles;
    }

    public void start(double currentValue) {
        if (state != TuningStates.IDLE) {
            return;
        }

        this.state = TuningStates.TUNING;
        this.lastAbove = currentValue >= setpoint;
        this.minVal = currentValue;
        this.maxVal = currentValue;
        
        cycleTimer.reset();
    }

    public double update(double currentValue) {
        if (state != TuningStates.TUNING) {
            return 0;
        }

        if (currentCycle >= requiredCycles) {
            computeCoefficients();
            return 0;
        }

        // PEAK DETECTION
        minVal = Math.min(minVal, currentValue);
        maxVal = Math.max(maxVal, currentValue);

        // Detect point crossing
        boolean isAbove = currentValue >= setpoint;

        if (lastAbove != isAbove) {
            // Going from under to above => 1 full oscillation cycle completed
            if (isAbove) {
                double period = cycleTimer.seconds();
                double amplitude = (maxVal - minVal) / 2.0;

                // Ignore cycle 0 to discard startup transient motion
                if (currentCycle > 0) {
                    totalPeriod += period;
                    totalAmplitude += amplitude;
                }

                currentCycle++;

                // Reset peak detectors for next cycle
                minVal = currentValue;
                maxVal = currentValue;
                cycleTimer.reset();
            }

            lastAbove = isAbove;
        }

        // BANG-BANG RELAY OUTPUT
        return isAbove ? -power : power;
    }

    public void computeCoefficients() {
        int recordedCycles = currentCycle - 1;

        if (recordedCycles <= 0 || totalAmplitude == 0) {
            state = TuningStates.FAILED;
            return;
        }

        double tU = totalPeriod / recordedCycles;
        double avgAmplitude = totalAmplitude / recordedCycles;

        double kU = (4.0 * power) / (Math.PI * avgAmplitude);

        kP = 0.6 * kU;
        kI = (1.2 * kU) / tU;
        kD = 0.075 * kU * tU;

        state = TuningStates.COMPLETE;
    }

    public TuningStates getState() {
        return state;
    }
}