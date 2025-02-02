package org.firstinspires.ftc.teamcode.IntoTheDeepRobot.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.RobotMap;
import org.inventors.ftc.robotbase.hardware.MotorExEx;

public class HangingSubsystem extends SubsystemBase {
    private MotorExEx coupler1, coupler2, coupler3;
    private ServoImplEx releaseServo1, releaseServo2;

    public HangingSubsystem(RobotMap robotMap) {
        coupler1 = robotMap.getFrontLeftMotor();
        coupler2 = robotMap.getFrontRightMotor();
        coupler3 = robotMap.getRearRightMotor();
        releaseServo1 = robotMap.getHangingReleaseServo1();
        releaseServo2 = robotMap.getHangingReleaseServo2();

        secure();
    }

    public void secure() {
        releaseServo1.setPosition(0.0);
        releaseServo2.setPosition(1.0);
    }

    public void release() {
        releaseServo1.setPosition(0.65);
        releaseServo2.setPosition(0.35);
    }
}
