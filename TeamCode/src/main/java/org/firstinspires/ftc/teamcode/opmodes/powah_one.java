package org.firstinspires.ftc.teamcode.opmodes;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.RobotCommands;
import org.firstinspires.ftc.teamcode.robot.opmode.RobotOpMode;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;

@TeleOp(name = "powah Test", group = "Test")
@Configurable
public class powah_one extends OpMode {
    CachingDcMotorEx left, right;
    @Override
    public void init() {
        left = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "flywheel_left"));
        right = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "flywheel_right"));
        right.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    @Override
    public void loop() {
        left.setPower(1);
        right.setPower(1);
    }
}