package org.firstinspires.ftc.teamcode.IntoTheDeepRobot.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.RobotMap;

public class CouplersSubsystem extends SubsystemBase {
    private ServoImplEx coupler_servo;

    // ------------------------------------------ States ---------------------------------------- //
    public enum CouplerState {
        ENGAGED,
        DISENGAGED
    }

    private CouplerState state;
    private static double engaged_position = 0.05, disengaged_position = 0.65;

    public CouplersSubsystem(RobotMap robotMap) {
        this.coupler_servo = robotMap.getCouplerServo();

        disengage();
    }

    // ----------------------------------------- Actuators -------------------------------------- //

    public void engage() {
        coupler_servo.setPosition(engaged_position);
        state = CouplerState.ENGAGED;
    }

    public void disengage() {
        coupler_servo.setPosition(disengaged_position);
        state = CouplerState.DISENGAGED;
    }

    // ------------------------------------------- Getters -------------------------------------- //

    public CouplerState getState() {
        return state;
    }
}
