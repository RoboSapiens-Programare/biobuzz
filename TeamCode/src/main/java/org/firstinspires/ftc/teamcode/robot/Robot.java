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

    private boolean llLocatorEnabled = true;

    private static List<LynxModule> allHubs = Collections.emptyList();

    public static boolean TUNING = false;
    //    public static Pose transitionPose = new Pose(63, 9, Math.PI / 2);

    public Robot(HardwareMap hwMap) {
        allHubs = hwMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        outtake = new Outtake(hwMap);

        intake = new Intake(hwMap);

        llLocator = new LLLocator(hwMap);

        tracker = new Tracker();
        outtake.setTracker(tracker);

        ledDriver = new LEDDriver(hwMap);

        follower = Constants.create(hwMap);
    }

    public static void resetCache() {
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
    }

    public void setLLLocatorEnabled(boolean enabled) {
        llLocatorEnabled = enabled;
    }

    public boolean isLLLocatorEnabled() {
        return llLocatorEnabled;
    }

    public void update() {
        follower.update();
        tracker.update();
        intake.update();
        outtake.update();
        ledDriver.update();

        if (llLocatorEnabled) {
            llLocator.update();
        }
    }

    public void init() {
        intake.init();
        outtake.init();
        llLocator.init();
        tracker.init();
        ledDriver.init();

        outtake.setTracker(tracker);
    }

    public void stop() {
        intake.stop();
        outtake.stop();
        llLocator.stop();
        tracker.stop();
        ledDriver.stop();
    }
}
