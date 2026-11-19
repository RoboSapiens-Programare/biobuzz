package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Prism.Color;
import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.Prism.PrismAnimations;

public class LEDDriver implements Subsystem {
    private static final int DEFAULT_BRIGHTNESS = 100;

    private final GoBildaPrismDriver driver;
    private final PrismAnimations.Solid solid = new PrismAnimations.Solid(Color.WHITE, DEFAULT_BRIGHTNESS);

    private Color color = Color.WHITE;
    private int brightness = DEFAULT_BRIGHTNESS;
    private boolean dirty = true;

    public LEDDriver(HardwareMap hwMap) {
        driver = hwMap.get(GoBildaPrismDriver.class, "prism");
        driver.setStripLength(24);
    }

    public void setColor(Color color) {
        this.color = color;
        dirty = true;
    }

    public void setBrightness(int brightness) {
        this.brightness = Math.max(0, Math.min(brightness, 100));
        dirty = true;
    }

    private void apply() {
        solid.setPrimaryColor(color);
        solid.setBrightness(brightness);
        driver.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, solid);
        dirty = false;
    }

    public void playSweep() {
        PrismAnimations.DroidScan scan = new PrismAnimations.DroidScan(new Color(200, 9, 200), new Color(255, 70, 11));
        scan.setSpeed(0.24f);
        scan.setTrailWidth(2);
        scan.setStopIndex(24);

        driver.clearAllAnimations();
        driver.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, scan);
    }

    @Override
    public void init() {
        dirty = true;
        apply();
    }

    @Override
    public void update() {
        if (dirty) {
            apply();
        }
    }

    @Override
    public void reset() {
        color = Color.WHITE;
        brightness = DEFAULT_BRIGHTNESS;
        dirty = true;
    }

    @Override
    public void stop() {
        driver.clearAllAnimations();
    }
}
