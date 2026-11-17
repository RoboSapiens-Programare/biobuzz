package org.firstinspires.ftc.teamcode.robot.opmode;

import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.utils.FrequencyMonitor;

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
    protected Telemetry pTelemetry;
    protected boolean panelsTelemetry = true;
    protected final FrequencyMonitor frequencyMonitor = new FrequencyMonitor();

    /** Telemetry update rate in Hz. 0 keeps the original every-loop behavior. */
    protected double telemetryHz = 0;

    /** Diagnostic hardware reads (motor currents, etc.) rate in Hz. */
    protected double diagnosticsHz = 4;

    private boolean telemetryDue = true;
    private long lastTelemetryNanos;
    private boolean diagnosticsDue = true;
    private long lastDiagnosticsNanos;

    protected boolean telemetryDue() {
        return telemetryDue;
    }

    protected boolean diagnosticsDue() {
        return diagnosticsDue;
    }

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
        pTelemetry = panelsTelemetry ? PanelsTelemetry.INSTANCE.getTelemetry().getWrapper() : telemetry;

        // IvY's scheduler is static, so clear anything carried over from a previous OpMode.
        Scheduler.reset();

        robot = new Robot(hardwareMap);
        robot.init();

        onInit();
    }

    @Override
    public final void init_loop() {
        onInitLoop();

        telemetry.update();
        if (pTelemetry != telemetry) {
            pTelemetry.update();
        }
    }

    @Override
    public final void start() {
        onStart();
    }

    @Override
    public final void loop() {
        frequencyMonitor.update();

        Robot.resetCache();

        long now = System.nanoTime();
        telemetryDue = telemetryHz <= 0 || now - lastTelemetryNanos >= (long) (1e9 / telemetryHz);
        if (telemetryDue) {
            lastTelemetryNanos = now;
        }

        diagnosticsDue = diagnosticsHz <= 0 || now - lastDiagnosticsNanos >= (long) (1e9 / diagnosticsHz);
        if (diagnosticsDue) {
            lastDiagnosticsNanos = now;
        }

        frequencyMonitor.beginStage();
        onUpdate();
        frequencyMonitor.endOnUpdate();

        frequencyMonitor.beginStage();
        Scheduler.execute();
        frequencyMonitor.endScheduler();

        frequencyMonitor.beginStage();
        robot.update();
        frequencyMonitor.endRobot();

        if (telemetryDue) {
            frequencyMonitor.beginStage();
            addLoopTelemetry(telemetry);
            if (pTelemetry != telemetry) {
                addLoopTelemetry(pTelemetry);
            }

            telemetry.update();
            if (pTelemetry != telemetry) {
                pTelemetry.update();
            }
            frequencyMonitor.endTelemetry();
        }
    }

    private void addLoopTelemetry(Telemetry telemetry) {
        telemetry.addData("Loop Hz (inst)", String.format("%.1f", frequencyMonitor.getInstantHz()));
        telemetry.addData("Loop Hz (avg)", String.format("%.1f", frequencyMonitor.getSmoothedHz()));
        telemetry.addData("Loop ms", String.format("%.2f", frequencyMonitor.getMs()));
        telemetry.addData("Update ms", String.format("%.2f", frequencyMonitor.getOnUpdateMs()));
        telemetry.addData("Scheduler ms", String.format("%.2f", frequencyMonitor.getSchedulerMs()));
        telemetry.addData("Robot ms", String.format("%.2f", frequencyMonitor.getRobotMs()));
        telemetry.addData("Telemetry ms", String.format("%.2f", frequencyMonitor.getTelemetryMs()));
        Pose pose = Robot.follower.pose();
        telemetry.addLine("== Pose ==");
        telemetry.addData("X", pose.x());
        telemetry.addData("Y", pose.y());
        telemetry.addData("Heading", pose.heading());
    }

    @Override
    public final void stop() {
        robot.stop();
        onStop();
    }
}
