package org.firstinspires.ftc.teamcode.mechanisms;

public interface Mechanism {
    /** Called once on init */
    void init();

    /** Called every loop iteration to run PID/FF logic and update hardware */
    void update();

    /** Resets sensors or homing positions safely */
    void reset();

    /** Stop all outputs for safety */
    void stop();
}