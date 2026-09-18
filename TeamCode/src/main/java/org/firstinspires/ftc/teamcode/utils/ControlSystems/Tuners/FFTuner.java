package org.firstinspires.ftc.teamcode.utils.ControlSystems.Tuners;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;

/**
 * Combined feedforward + PID autotuner for velocity plants (e.g. flywheels).
 *
 * Stage 1 (FF_RAMP): ramps applied power from 0 to maxPower in steps, waits for steady state at
 * each level, then least-squares fits the steady-state data to the model
 * power = kS + kV * velocity. Only points above the motion threshold are fitted, so the stall
 * region (power below kS) cannot skew the slope. Gains are clamped to non-negative; if the fit
 * implies negative breakaway, the measured first-motion power is used for kS instead. kA is
 * intentionally not identified (acceleration feedforward needs a separate ramp test).
 *
 * Stage 2 (PID_RELAY, optional): runs a Ziegler-Nichols relay test on top of the identified
 * feedforward (kS + kV * setpoint) so the flywheel stays energized and oscillates cleanly around
 * the setpoint. Reports kP / kI / kD.
 *
 * Usage: feed update(measuredVelocity) every loop and apply the returned power to the plant.
 */
public class FFTuner {
    private final double maxPower;
    private final double powerStep;
    private final double settleTime;
    private final double thresholdVelocity;
    private final double timeoutSeconds;

    private final boolean pidEnabled;
    private final double pidSetpoint;
    private final double relayPower;
    private final int pidCycles;
    private final double pidTimeoutSeconds;

    private final ElapsedTime settleTimer = new ElapsedTime();
    private final ElapsedTime startTimer = new ElapsedTime();
    private final ElapsedTime pidStageTimer = new ElapsedTime();

    private final List<Double> powers = new ArrayList<>();
    private final List<Double> velocities = new ArrayList<>();

    private double currentPower = 0;
    private boolean rampComplete = false;
    private boolean firstMotionFound = false;
    private double firstMotionPower = 0;

    private PIDTuner pidTuner;

    public double kS;
    public double kV;
    public double kP;
    public double kI;
    public double kD;

    public enum TuningStates {
        IDLE,
        TUNING,
        COMPLETE,
        FAILED
    }

    public enum TuningStages {
        FF_RAMP,
        PID_RELAY
    }

    private TuningStates state;
    private TuningStages stage;

    /**
     * FF-only tuner (no PID relay stage).
     */
    public FFTuner(double maxPower, double powerStep, double settleTime, double thresholdVelocity) {
        this(maxPower, powerStep, settleTime, thresholdVelocity, 60.0);
    }

    /**
     * FF-only tuner (no PID relay stage).
     */
    public FFTuner(double maxPower, double powerStep, double settleTime, double thresholdVelocity, double timeoutSeconds) {
        this(maxPower, powerStep, settleTime, thresholdVelocity, timeoutSeconds,
                0, 0, 0, 0, false);
    }

    /**
     * Combined FF + PID tuner. After the ramp fit completes, a relay test is run on top of the
     * identified feedforward around pidSetpoint.
     */
    public FFTuner(double maxPower, double powerStep, double settleTime, double thresholdVelocity,
                   double timeoutSeconds,
                   double pidSetpoint, double relayPower, int pidCycles, double pidTimeoutSeconds) {
        this(maxPower, powerStep, settleTime, thresholdVelocity, timeoutSeconds,
                pidSetpoint, relayPower, pidCycles, pidTimeoutSeconds, true);
    }

    private FFTuner(double maxPower, double powerStep, double settleTime, double thresholdVelocity,
                    double timeoutSeconds,
                    double pidSetpoint, double relayPower, int pidCycles, double pidTimeoutSeconds,
                    boolean pidEnabled) {
        if (maxPower <= 0 || powerStep <= 0) {
            throw new IllegalArgumentException("maxPower and powerStep must be positive");
        }

        this.maxPower = maxPower;
        this.powerStep = powerStep;
        this.settleTime = settleTime;
        this.thresholdVelocity = thresholdVelocity;
        this.timeoutSeconds = timeoutSeconds;

        this.pidEnabled = pidEnabled;
        this.pidSetpoint = pidSetpoint;
        this.relayPower = relayPower;
        this.pidCycles = pidCycles;
        this.pidTimeoutSeconds = pidTimeoutSeconds;

        this.state = TuningStates.IDLE;
        this.stage = TuningStages.FF_RAMP;
    }

    public void start() {
        if (state != TuningStates.IDLE) {
            return;
        }

        state = TuningStates.TUNING;
        stage = TuningStages.FF_RAMP;
        currentPower = 0;
        rampComplete = false;
        firstMotionFound = false;
        firstMotionPower = 0;
        powers.clear();
        velocities.clear();
        pidTuner = null;

        kS = 0;
        kV = 0;
        kP = 0;
        kI = 0;
        kD = 0;

        settleTimer.reset();
        startTimer.reset();
        pidStageTimer.reset();
    }

    public double update(double measuredVelocity) {
        if (state == TuningStates.COMPLETE || state == TuningStates.FAILED) {
            return 0;
        }
        if (state != TuningStates.TUNING) {
            return 0;
        }

        if (stage == TuningStages.PID_RELAY) {
            return updatePIDStage(measuredVelocity);
        }
        return updateFFStage(measuredVelocity);
    }

    private double updateFFStage(double measuredVelocity) {
        if (startTimer.seconds() > timeoutSeconds) {
            state = TuningStates.FAILED;
            return 0;
        }

        // Record first-motion breakaway power as a kS guard.
        if (!firstMotionFound && measuredVelocity > thresholdVelocity) {
            firstMotionFound = true;
            firstMotionPower = currentPower;
        }

        if (settleTimer.seconds() >= settleTime) {
            if (measuredVelocity > thresholdVelocity) {
                powers.add(currentPower);
                velocities.add(measuredVelocity);
            }

            if (rampComplete) {
                if (powers.size() < 2) {
                    state = TuningStates.FAILED;
                    return 0;
                }

                computeCoefficients();
                if (state == TuningStates.FAILED) {
                    return 0;
                }

                if (pidEnabled) {
                    stage = TuningStages.PID_RELAY;
                    pidTuner = new PIDTuner(pidSetpoint, relayPower, pidCycles);
                    pidTuner.start(measuredVelocity);
                    pidStageTimer.reset();
                    return ffPower(pidSetpoint);
                }

                state = TuningStates.COMPLETE;
                return 0;
            }

            currentPower = Math.min(currentPower + powerStep, maxPower);
            settleTimer.reset();

            if (currentPower >= maxPower) {
                rampComplete = true;
            }
        }

        return currentPower;
    }

    private double updatePIDStage(double measuredVelocity) {
        if (pidStageTimer.seconds() > pidTimeoutSeconds) {
            state = TuningStates.FAILED;
            return 0;
        }

        PIDTuner.TuningStates pidState = pidTuner.getState();

        if (pidState == PIDTuner.TuningStates.FAILED) {
            state = TuningStates.FAILED;
            return 0;
        }

        if (pidState == PIDTuner.TuningStates.COMPLETE) {
            kP = pidTuner.kP;
            kI = pidTuner.kI;
            kD = pidTuner.kD;
            state = TuningStates.COMPLETE;
            return 0;
        }

        double relay = pidTuner.update(measuredVelocity);
        return ffPower(pidSetpoint) + relay;
    }

    private double ffPower(double velocity) {
        return kS + kV * velocity;
    }

    private void computeCoefficients() {
        // Least-squares fit: power ~= kS + kV * velocity.
        double meanV = 0;
        for (double v : velocities) {
            meanV += v;
        }
        meanV /= velocities.size();

        double meanP = 0;
        for (double p : powers) {
            meanP += p;
        }
        meanP /= powers.size();

        double num = 0;
        double den = 0;
        for (int i = 0; i < velocities.size(); i++) {
            double dv = velocities.get(i) - meanV;
            num += dv * (powers.get(i) - meanP);
            den += dv * dv;
        }

        if (den == 0) {
            state = TuningStates.FAILED;
            return;
        }

        double slope = num / den;
        double intercept = meanP - slope * meanV;

        kV = Math.max(slope, 0);
        kS = intercept;

        // kS is physically non-negative; trust the measured breakaway power when the fit lies.
        if (kS < 0) {
            kS = firstMotionFound ? firstMotionPower : 0;
        }
    }

    public TuningStates getState() {
        return state;
    }

    public TuningStages getStage() {
        return stage;
    }
}