package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("front_left");
        c.frontRightName.set("front_right");
        c.backLeftName.set("rear_left");
        c.backRightName.set("rear_right");
        c.frontLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.frontRightDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backRightDirection.set(DcMotorSimple.Direction.REVERSE);
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(-5.817655428188054);
        c.yPodOffset.set(-0.0020312215751550327);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.20365671210795497);
                Controller secondaryTranslationalForward = Controller.proportional(0.07524571331631863);
                Controller primaryTranslationalLateral = Controller.proportional(0.3600999475801474);
                Controller secondaryTranslationalLateral = Controller.proportional(0.13304730858305333);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.009076054596787534));
                c.brake.set(Controller.proportionalFeedforward(0.007714646407269404));

                c.headingFeedback.set(Controller.proportional(3.158648092740405));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.047996849837815825, 0.009595621101145655));

                c.linearBrakeCoefficients.set(Matrix.diag(0.0380978462240754, 0.058231370334258285));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.002166016764032193, 0.0018643855329281568));

                c.maxAchievableForwardVelocity.set(99.97447551507753);
                c.maxAchievableStrafeVelocity.set(56.653609731766856);
                c.naturalForwardDeceleration.set(21.308668564904426);
                c.naturalStrafeDeceleration.set(46.980432578939144);
            }
    );

    public static Follower create(HardwareMap h) {
        return new Follower(new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig));
    }
}