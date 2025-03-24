package org.firstinspires.ftc.teamcode.Auto.constants;

import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.localization.Localizers;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class FConstants {
    static {
        FollowerConstants.localizers = Localizers.TWO_WHEEL;

        FollowerConstants.leftFrontMotorName = "front_left";
        FollowerConstants.leftRearMotorName = "rear_left";
        FollowerConstants.rightFrontMotorName = "front_right";
        FollowerConstants.rightRearMotorName = "rear_right";

        FollowerConstants.leftFrontMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.leftRearMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.rightFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
        FollowerConstants.rightRearMotorDirection = DcMotorSimple.Direction.FORWARD;

        FollowerConstants.mass = 14;

        FollowerConstants.xMovement = 67.25945500141216;
        FollowerConstants.yMovement = 45.65808666767382;

        FollowerConstants.forwardZeroPowerAcceleration = -41.33983885200747;
        FollowerConstants.lateralZeroPowerAcceleration = -86.18019311136716;

//        FollowerConstants.translationalPIDFCoefficients.setCoefficients(0.2,0,0.02,0);
//        FollowerConstants.useSecondaryTranslationalPID = true;
//        FollowerConstants.secondaryTranslationalPIDFCoefficients.setCoefficients(0.08,0,0.03,0);

        FollowerConstants.translationalPIDFCoefficients.setCoefficients(0.1,0,0.02,0);
        FollowerConstants.useSecondaryTranslationalPID = true;
        FollowerConstants.secondaryTranslationalPIDFCoefficients.setCoefficients(0.03,0,0.03,0);
        // Not being used, @see useSecondaryTranslationalPID

//        FollowerConstants.headingPIDFCoefficients.setCoefficients(6,0,0.6,0);
//        FollowerConstants.useSecondaryHeadingPID = true;
//        FollowerConstants.secondaryHeadingPIDFCoefficients.setCoefficients(2,0,0.05,0); // Not being used, @see useSecondaryHeadingPID

        FollowerConstants.headingPIDFCoefficients.setCoefficients(1.5,0,0.6,0);
        FollowerConstants.useSecondaryHeadingPID = true;
        FollowerConstants.secondaryHeadingPIDFCoefficients.setCoefficients(0.2,0,0.05,0); // Not
        // being used, @see useSecondaryHeadingPID

//        FollowerConstants.drivePIDFCoefficients.setCoefficients(0.015,0,0.00002,0.6,0);
//        FollowerConstants.useSecondaryDrivePID = true;
//        FollowerConstants.secondaryDrivePIDFCoefficients.setCoefficients(0.002,0,0.00001,0.6,0); // Not being used, @see useSecondaryDrivePID

        FollowerConstants.drivePIDFCoefficients.setCoefficients(0.01,0,0.00002,0.6,0);
        FollowerConstants.useSecondaryDrivePID = true;
        FollowerConstants.secondaryDrivePIDFCoefficients.setCoefficients(0.0001,0,0.000015,0.6,
                                                                         0); // Not being used, @see useSecondaryDrivePID

        FollowerConstants.zeroPowerAccelerationMultiplier = 4;
        FollowerConstants.centripetalScaling = 0.001;

        FollowerConstants.pathEndTimeoutConstraint = 500;
        FollowerConstants.pathEndTValueConstraint = 0.995;

        FollowerConstants.pathEndVelocityConstraint = 0.1;
        FollowerConstants.pathEndTranslationalConstraint = 0.1;
        FollowerConstants.pathEndHeadingConstraint = 0.007;

        FollowerConstants.drivePIDFSwitch = 30;
    }
}
