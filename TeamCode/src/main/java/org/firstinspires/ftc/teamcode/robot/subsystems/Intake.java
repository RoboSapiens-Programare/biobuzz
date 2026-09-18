package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.mechanisms.Roller;

@Configurable
public class Intake implements Subsystem {
    private final Roller rollerOne = new Roller();

    public Intake(HardwareMap hwMap) {
        // Example usages
        rollerOne.addMotor(hwMap.get(DcMotorEx.class, "roller_motor"), true);

        rollerOne.setPower(0.72);
    }

    public void pullBalls() {
        rollerOne.pull();
    }

    public void pushBalls() {
        rollerOne.push();
    }

    public void idle() {
        rollerOne.idle();
    }

    @Override
    public void init() {}

    @Override
    public void update() {
        rollerOne.update();
    }

    @Override
    public void reset() {}

    @Override
    public void stop() {
        idle();
    }
}
