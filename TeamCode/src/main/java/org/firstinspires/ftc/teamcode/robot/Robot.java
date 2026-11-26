package org.firstinspires.ftc.teamcode.robot;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.Collections;
import java.util.List;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystems.LEDDriver;
import org.firstinspires.ftc.teamcode.robot.subsystems.LLLocator;
import org.firstinspires.ftc.teamcode.robot.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.robot.subsystems.Tracker;

@Configurable
public class Robot {
    public Outtake outtake;
    public Intake intake;
    public LLLocator llLocator;
    public Tracker tracker;
    public LEDDriver ledDriver;
    private final ElapsedTime headlightTimer = new ElapsedTime();

    public static Follower follower;

    private static List<LynxModule> allHubs = Collections.emptyList();

    public static boolean TUNING = false;
    public static boolean USE_LIMELIGHT = false;
    public static boolean VERBOSE = false;

    public Robot(HardwareMap hwMap) {
        allHubs = hwMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        outtake = new Outtake(hwMap);

        intake = new Intake(hwMap);

        tracker = new Tracker();
        outtake.setTracker(tracker);

        if (USE_LIMELIGHT) llLocator = new LLLocator(hwMap);

        ledDriver = new LEDDriver(hwMap);

        follower = Constants.create(hwMap);

        ActionLogger.init(hwMap, this);
    }

    public static void resetCache() {
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
    }

    public void update() {
        follower.update();
        tracker.update();
        intake.update();
        outtake.update();
        ledDriver.update();

        if (USE_LIMELIGHT) llLocator.update();
    }

    public void init() {
        intake.init();
        outtake.init();
        tracker.init();
        ledDriver.init();

        if (USE_LIMELIGHT) llLocator.init();

        outtake.setTracker(tracker);
    }

    public void stop() {
        intake.stop();
        outtake.stop();
        tracker.stop();
        ledDriver.stop();

        if (USE_LIMELIGHT) llLocator.stop();

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }
}
