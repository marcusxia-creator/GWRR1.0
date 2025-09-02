package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.Locale;

public class PinPoint {
    GoBildaPinpointDriver pinPoint;

    public PinPoint(GoBildaPinpointDriver pinPoint) {
        this.pinPoint = pinPoint;
    }

    public void initPinPoint() {
        pinPoint.setOffsets(-149.225, -165.1); //these are tuned for 3110-0002-0001 Product Insight #1
        pinPoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinPoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        pinPoint.resetPosAndIMU();
        pinPoint.update();
    }

    public String getPosition() {
        pinPoint.update();
        Pose2D pos = pinPoint.getPosition();

        return String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.INCH), pos.getY(DistanceUnit.INCH), pos.getHeading(AngleUnit.DEGREES));
    }
}
