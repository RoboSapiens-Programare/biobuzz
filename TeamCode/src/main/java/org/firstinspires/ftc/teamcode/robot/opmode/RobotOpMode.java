package org.firstinspires.ftc.teamcode.robot.opmode;

import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.robot.Robot;

/**
 * Base class for every OpMode. Wires up the Robot and the Ivy Scheduler so concrete
 * OpModes only implement game logic.
 *
 * Iterative OpMode lifecycle:
 *   init()      -> Scheduler.reset(), build Robot      -> onInit()
 *   init_loop() -> every loop before Start             -> onInitLoop()
 *   start()     -> START is pressed                    -> onStart()
 *   loop()      -> every loop while active             -> onUpdate() -> Scheduler.execute()
 *                                                       -> robot.update() (follower + mechanisms)
 *                                                       -> telemetry.update()
 *   stop()      -> OpMode ends                         -> robot.stop() -> onStop()
 */
public abstract class RobotOpMode extends OpMode {
    protected Robot robot;

    /** Called once in init(). Override for setup. */
    protected void onInit() {}

    /** Called every loop before START is pressed. Override for homing / sensor warm-up. */
    protected void onInitLoop() {}

    /** Called once right after START is pressed. Override to schedule commands. */
    protected void onStart() {}

    /** Called every loop while active. Map gamepads / triggers to commands here. */
    protected abstract void onUpdate();

    /** Called once after the OpMode stops. Override for cleanup. */
    protected void onStop() {}

    @Override
    public final void init() {
        // IvY's scheduler is static, so clear anything carried over from a previous OpMode.
        Scheduler.reset();

        robot = new Robot(hardwareMap);
        robot.init();

        onInit();
    }

    @Override
    public final void init_loop() {
        onInitLoop();
    }

    @Override
    public final void start() {
        onStart();
    }

    @Override
    public final void loop() {
        Robot.resetCache();

        onUpdate();

        Scheduler.execute();

        robot.update();
        telemetry.update();
    }

    @Override
    public final void stop() {
        robot.stop();
        onStop();
    }
}
