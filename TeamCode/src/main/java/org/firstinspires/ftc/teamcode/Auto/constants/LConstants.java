package org.firstinspires.ftc.teamcode.Auto.constants;

import com.pedropathing.localization.Encoder;
import com.pedropathing.localization.constants.TwoWheelConstants;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

public class LConstants {
    static {
        TwoWheelConstants.forwardTicksToInches = 0.0010631306669932295;
        TwoWheelConstants.strafeTicksToInches =  0.0010627260933279423;
        TwoWheelConstants.forwardY = -13.9/2.54;
        TwoWheelConstants.strafeX = -6.0/2.54;
        TwoWheelConstants.forwardEncoder_HardwareMapName = "rear_right";
        TwoWheelConstants.strafeEncoder_HardwareMapName = "front_left";
        TwoWheelConstants.forwardEncoderDirection = Encoder.REVERSE;
        TwoWheelConstants.strafeEncoderDirection = Encoder.FORWARD;
        TwoWheelConstants.IMU_HardwareMapName = "external_imu";
        TwoWheelConstants.IMU_Orientation =
            new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                                         RevHubOrientationOnRobot.UsbFacingDirection.DOWN);
    }
}