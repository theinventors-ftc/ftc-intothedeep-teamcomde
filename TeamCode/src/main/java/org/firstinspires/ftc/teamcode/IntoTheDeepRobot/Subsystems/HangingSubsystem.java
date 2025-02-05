package org.firstinspires.ftc.teamcode.IntoTheDeepRobot.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotMap;
import org.inventors.ftc.robotbase.hardware.MotorExEx;

public class HangingSubsystem extends SubsystemBase {
    private MotorExEx coupler1, coupler2, coupler3;
    private ServoImplEx releaseServo1, releaseServo2;

    private double power = 1.0;
    private boolean is_hanging = false;
    private int hanging_pos = 3050;

    private Telemetry telemetry;

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
        coupler2.resetEncoder();
        releaseServo1.setPosition(0.65);
        releaseServo2.setPosition(0.35);
    }

    public void ascend() {
        coupler1.set(-power);
        coupler2.set(power);
        coupler3.set(power);
    }

    public void stop() {
        coupler1.set(0);
        coupler2.set(0);
        coupler3.set(0);
    }

    public void descend() {
        coupler1.set(power/3);
        coupler2.set(-power/3);
        coupler3.set(-power/3);
    }

    public boolean reached_hanging_pos() {
        return coupler2.getCurrentPosition() >= hanging_pos;
    }
}
